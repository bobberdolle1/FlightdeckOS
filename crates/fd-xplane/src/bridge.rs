//! Host-side client for the fd-xplm-bridge plugin (Task 7 §7-8).
//!
//! The plugin (in-simulator) publishes versioned, line-delimited JSON
//! FMS snapshots on loopback TCP. This client connects, validates the
//! handshake, and maps wire snapshots into the canonical
//! [`fd_core::fplan::FmsSnapshot`].
//!
//! Contract (§8):
//! - loopback only; the client NEVER sends application data (the plugin
//!   does not read);
//! - reconnectable: every error is surfaced as [`BridgeError`] and the
//!   caller may reconnect; a dead bridge degrades to "FMS unavailable"
//!   (§13: honest absence), never to a guessed plan;
//! - bounded: one line = one snapshot; oversized lines are rejected.

use std::io::{BufRead, BufReader};
use std::net::TcpStream;
use std::time::Duration;

use fd_core::fplan::{FmsDeviceKind, FmsEntry, FmsEntryKind, FmsPlan, FmsSnapshot, PlanKind};
use serde::Deserialize;

/// Wire protocol version this client speaks.
pub const PROTO_VERSION: u32 = 1;

/// Default bridge port (matches fd-xplm-bridge DEFAULT_PORT).
pub const DEFAULT_PORT: u16 = 57501;

/// Maximum accepted line length (§8: bounded messages).
const MAX_LINE_BYTES: usize = 256 * 1024;

/// Bridge client errors (typed, never a bare String).
#[derive(Debug, thiserror::Error)]
pub enum BridgeError {
    #[error("bridge connect failed: {0}")]
    Connect(std::io::Error),
    #[error("bridge io: {0}")]
    Io(#[from] std::io::Error),
    #[error("bridge protocol: {0}")]
    Protocol(String),
    #[error("bridge snapshot too large: {0} bytes")]
    Oversized(usize),
}

/// Handshake sent by the plugin on connect.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct BridgeHello {
    pub proto: u32,
    pub kind: String,
    #[serde(default)]
    pub xplane: Option<i64>,
    #[serde(default)]
    pub xplm: Option<i64>,
}

/// One plan on the wire.
#[derive(Debug, Clone, PartialEq, Deserialize)]
struct WirePlan {
    #[serde(default)]
    entries: Vec<WireEntry>,
    #[serde(default)]
    dest: Option<i64>,
    #[serde(default)]
    disp: Option<i64>,
    #[serde(default)]
    count: Option<i64>,
}

/// One entry on the wire. X-Plane nav-type bitmask passes through
/// UNTRANSLATED (plugin contract) and is mapped here into the canonical
/// [`FmsEntryKind`].
#[derive(Debug, Clone, PartialEq, Deserialize)]
struct WireEntry {
    #[serde(default)]
    ty: i64,
    #[serde(default)]
    id: Option<String>,
    #[serde(default)]
    lat: Option<f64>,
    #[serde(default)]
    lon: Option<f64>,
    #[serde(default)]
    alt: Option<i64>,
    #[serde(default)]
    nav: bool,
}

/// Full snapshot on the wire.
#[derive(Debug, Clone, PartialEq, Deserialize)]
struct WireSnapshot {
    proto: u32,
    kind: String,
    plans: std::collections::BTreeMap<String, Option<WirePlan>>,
}

/// Map an X-Plane nav-type bitmask to the canonical entry kind.
/// Unknown bitmasks stay Unknown — never guessed (§50).
fn map_entry_kind(ty: i64) -> FmsEntryKind {
    match ty {
        1 => FmsEntryKind::Airport,
        2 => FmsEntryKind::Ndb,
        4 => FmsEntryKind::Vor,
        512 => FmsEntryKind::Fix,
        2048 => FmsEntryKind::LatLon,
        _ => FmsEntryKind::Unknown,
    }
}

fn map_plan(w: &WirePlan) -> FmsPlan {
    FmsPlan {
        entries: w
            .entries
            .iter()
            .map(|e| FmsEntry {
                kind: map_entry_kind(e.ty),
                id: e.id.clone(),
                lat_deg: e.lat,
                lon_deg: e.lon,
                altitude_constraint_ft: e.alt.map(|a| a as i32),
                nav_ref_resolved: e.nav,
            })
            .collect(),
        destination_entry: w.dest.and_then(|d| usize::try_from(d).ok()),
        displayed_entry: w.disp.and_then(|d| usize::try_from(d).ok()),
    }
}

/// Map a wire snapshot to the canonical model. The device kind is
/// inferred honestly: a GPS-style surface (primary+approach, no
/// temporary) on a stock aircraft is a stock GPS; anything else stays
/// Unknown rather than guessed (§13).
fn map_snapshot(w: WireSnapshot) -> Result<FmsSnapshot, BridgeError> {
    if w.proto != PROTO_VERSION {
        return Err(BridgeError::Protocol(format!(
            "proto version {}: host speaks {}",
            w.proto, PROTO_VERSION
        )));
    }
    if w.kind != "fms" {
        return Err(BridgeError::Protocol(format!(
            "unexpected kind {:?}",
            w.kind
        )));
    }
    // Fixed processing order: the XPLM410 primary wins over the legacy
    // surface when both are present (they describe the same active plan
    // on stock devices; 410 is the richer contract). Plan kinds outside
    // the known set are protocol drift and rejected.
    for name in w.plans.keys() {
        if !matches!(
            name.as_str(),
            "primary" | "approach" | "temporary" | "legacy"
        ) {
            return Err(BridgeError::Protocol(format!("unknown plan kind {name:?}")));
        }
    }
    let mut plans = std::collections::BTreeMap::new();
    for name in ["primary", "approach", "temporary", "legacy"] {
        let Some(Some(plan)) = w.plans.get(name) else {
            continue;
        };
        let kind = match name {
            "primary" | "legacy" => PlanKind::Primary,
            "approach" => PlanKind::Approach,
            _ => PlanKind::Temporary,
        };
        plans.entry(kind).or_insert_with(|| map_plan(plan));
    }
    let device = match (
        &plans.get(&PlanKind::Temporary),
        plans.get(&PlanKind::Primary),
    ) {
        (Some(_), _) => FmsDeviceKind::StockFms,
        (None, Some(_)) => FmsDeviceKind::StockGps,
        (None, None) => FmsDeviceKind::Unknown,
    };
    let evidence = format!(
        "xplane-fms-bridge proto={} plans={:?}",
        w.proto,
        w.plans.keys().collect::<Vec<_>>()
    );
    Ok(FmsSnapshot::new(device, plans, evidence))
}

/// Reconnecting bridge client.
pub struct FmsBridgeClient {
    reader: BufReader<TcpStream>,
    pub hello: BridgeHello,
}

impl FmsBridgeClient {
    /// Connect to the bridge and validate the handshake.
    pub fn connect(port: u16) -> Result<Self, BridgeError> {
        let stream = TcpStream::connect(("127.0.0.1", port)).map_err(BridgeError::Connect)?;
        stream
            .set_read_timeout(Some(Duration::from_millis(200)))
            .ok();
        stream.set_nodelay(true).ok();
        let mut reader = BufReader::with_capacity(64 * 1024, stream);
        let mut line = String::new();
        reader.read_line(&mut line)?;
        let hello: BridgeHello = serde_json::from_str(line.trim())
            .map_err(|e| BridgeError::Protocol(format!("bad handshake: {e}")))?;
        if hello.proto != PROTO_VERSION || hello.kind != "hello" {
            return Err(BridgeError::Protocol(format!(
                "handshake mismatch: proto={} kind={}",
                hello.proto, hello.kind
            )));
        }
        Ok(Self { reader, hello })
    }

    /// Try to read one snapshot line. `Ok(None)` = nothing new within
    /// the read timeout. Any error means the connection is unusable and
    /// the caller should reconnect.
    pub fn poll(&mut self) -> Result<Option<FmsSnapshot>, BridgeError> {
        let mut line = String::new();
        match self.reader.read_line(&mut line) {
            Ok(0) => Err(BridgeError::Io(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "bridge closed",
            ))),
            Ok(_) => {
                if line.len() > MAX_LINE_BYTES {
                    return Err(BridgeError::Oversized(line.len()));
                }
                let wire: WireSnapshot = serde_json::from_str(line.trim())
                    .map_err(|e| BridgeError::Protocol(format!("bad snapshot: {e}")))?;
                map_snapshot(wire).map(Some)
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                Ok(None)
            }
            Err(e) => Err(BridgeError::Io(e)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const HELLO: &str = r#"{"proto":1,"kind":"hello","xplane":124033,"xplm":430}"#;

    fn snap(plans: &str) -> String {
        format!(r#"{{"proto":1,"kind":"fms","xplane":124033,"xplm":430,"plans":{plans}}}"#)
    }

    fn parse(plans: &str) -> FmsSnapshot {
        let wire: WireSnapshot = serde_json::from_str(&snap(plans)).unwrap();
        map_snapshot(wire).unwrap()
    }

    #[test]
    fn hello_parses() {
        let h: BridgeHello = serde_json::from_str(HELLO).unwrap();
        assert_eq!(h.proto, 1);
        assert_eq!(h.kind, "hello");
        assert_eq!(h.xplane, Some(124033));
    }

    #[test]
    fn empty_plans_map_to_unknown_device() {
        let s = parse("{}");
        assert_eq!(s.device, FmsDeviceKind::Unknown);
        assert!(s.primary().is_none());
    }

    #[test]
    fn primary_and_approach_map_to_gps_device() {
        let s = parse(
            r#"{"primary":{"entries":[{"ty":512,"id":"SEALS","lat":33.9,"lon":-118.4,"alt":null,"nav":true}],"dest":0,"disp":0,"count":1},
                "approach":{"entries":[{"ty":1,"id":"KSNA","lat":33.67,"lon":-117.86,"alt":56,"nav":true}],"dest":0,"disp":0,"count":1}}"#,
        );
        assert_eq!(s.device, FmsDeviceKind::StockGps);
        let p = s.primary().unwrap();
        assert_eq!(p.entries.len(), 1);
        assert_eq!(p.entries[0].id.as_deref(), Some("SEALS"));
        assert_eq!(p.entries[0].kind, FmsEntryKind::Fix);
        assert_eq!(s.approach_loaded(), Some(true));
        assert_eq!(
            p.destination().and_then(|e| e.id.clone()),
            Some("SEALS".into())
        );
    }

    #[test]
    fn temporary_plan_implies_fms_device() {
        let s = parse(
            r#"{"primary":{"entries":[],"dest":null,"disp":null,"count":0},"temporary":{"entries":[{"ty":2048,"id":"--","lat":1.0,"lon":2.0,"alt":null,"nav":false}],"dest":null,"disp":null,"count":1}}"#,
        );
        assert_eq!(s.device, FmsDeviceKind::StockFms);
        assert_eq!(s.primary().unwrap().entries.len(), 0);
    }

    #[test]
    fn legacy_is_shadowed_by_primary() {
        let s = parse(
            r#"{"legacy":{"entries":[{"ty":512,"id":"LEG","lat":1.0,"lon":1.0,"alt":null,"nav":false}],"dest":0,"disp":0,"count":1},
                "primary":{"entries":[{"ty":512,"id":"PRI","lat":2.0,"lon":2.0,"alt":null,"nav":true}],"dest":0,"disp":0,"count":1}}"#,
        );
        let p = s.primary().unwrap();
        assert_eq!(p.entries[0].id.as_deref(), Some("PRI"));
    }

    #[test]
    fn unknown_nav_type_stays_unknown() {
        let s = parse(
            r#"{"primary":{"entries":[{"ty":999999,"id":"X","lat":null,"lon":null,"alt":null,"nav":false}],"dest":null,"disp":null,"count":1}}"#,
        );
        assert_eq!(s.primary().unwrap().entries[0].kind, FmsEntryKind::Unknown);
    }

    #[test]
    fn wrong_proto_is_protocol_error() {
        let wire: WireSnapshot =
            serde_json::from_str(&snap("{}").replace("\"proto\":1", "\"proto\":2")).unwrap();
        assert!(map_snapshot(wire).is_err());
    }

    #[test]
    fn null_plan_is_absent_not_empty() {
        let s = parse(
            r#"{"primary":{"entries":[],"dest":null,"disp":null,"count":0},"approach":null}"#,
        );
        assert_eq!(
            s.approach_loaded(),
            None,
            "absent plan must not read as empty approach"
        );
        assert_eq!(s.plans.get(&PlanKind::Approach), None);
    }
}
