//! Canonical telemetry state: a minimal generic core plus an opaque,
//! numerically-keyed extension map for aircraft-specific values.
//!
//! Task 2 moved all A32NX-specific typed state out of this crate into the
//! aircraft layer (`fd-aircraft`). `fd-core` knows nothing about A32NX.
//!
//! Aircraft-specific values travel in
//! [`TelemetrySnapshot::aircraft_values`]: an opaque map keyed by stable
//! numeric ids assigned by trusted adapter code (fd-simconnect binding
//! table); their meaning is owned by the aircraft layer. Missing data is
//! represented as `None` / absent keys — FlightdeckOS never fabricates
//! values.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

use crate::units::{
    AltitudeAglFt, AltitudeFt, AngleDeg, LatDeg, LonDeg, SpeedKt, VerticalSpeedFpm,
};

/// Serde helpers: JSON object with string keys (`{"1": 0.0}`) mapped onto
/// `BTreeMap<u16, f64>` (serde_json cannot key maps by raw integers).
mod ext_values_serde {
    use super::BTreeMap;
    use serde::{Deserializer, Serializer};

    pub fn serialize<S: Serializer>(m: &BTreeMap<u16, f64>, ser: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeMap;
        let mut map = ser.serialize_map(Some(m.len()))?;
        for (k, v) in m {
            map.serialize_entry(&k.to_string(), v)?;
        }
        map.end()
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(de: D) -> Result<BTreeMap<u16, f64>, D::Error> {
        struct V;
        impl<'de> serde::de::Visitor<'de> for V {
            type Value = BTreeMap<u16, f64>;
            fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                f.write_str("a map of numeric-id -> value")
            }
            fn visit_map<A: serde::de::MapAccess<'de>>(
                self,
                mut access: A,
            ) -> Result<Self::Value, A::Error> {
                let mut m = BTreeMap::new();
                while let Some(key) = access.next_key::<String>()? {
                    let id: u16 = key.parse().map_err(serde::de::Error::custom)?;
                    let value = access.next_value::<f64>()?;
                    m.insert(id, value);
                }
                Ok(m)
            }
        }
        de.deserialize_map(V)
    }
}

/// Logical simulation timestamp in milliseconds.
///
/// Sourced from the simulator clock (`ABSOLUTE TIME`) when available; in
/// replay mode it is injected by the fixture. Ordering MUST rely on this
/// value plus the monotonic event sequence — never on wall clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct SimTimestamp {
    pub ms: u64,
}

impl SimTimestamp {
    pub const fn new(ms: u64) -> Self {
        Self { ms }
    }
}

/// Geographic position.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Position {
    pub lat: LatDeg,
    pub lon: LonDeg,
}

/// Coarse simulator execution state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimState {
    Running,
    Paused,
    #[default]
    Unknown,
}

/// Simulator timing awareness: pause / sim-rate / slew.
///
/// First-class runtime concerns (Task 1 §12): during pause no derived events
/// may be generated; slew/teleport must not produce false phase sequences.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SimTiming {
    pub state: SimState,
    /// Simulation rate (1.0 = real time). `None` = not reported by simulator.
    pub sim_rate: Option<f64>,
    /// Slew mode active flag. `None` = not reported.
    pub slew_active: Option<bool>,
}

impl Default for SimTiming {
    fn default() -> Self {
        Self {
            state: SimState::Unknown,
            sim_rate: None,
            slew_active: None,
        }
    }
}

/// Tri-state light switch (e.g. the NAV/LOGO light switch on Airbus-style
/// overhead panels: Off / System 1 / System 2). Parameter of the closed
/// [`crate::actions::CockpitAction::SetNavLogo`] action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NavLogoMode {
    Off,
    Sys1,
    Sys2,
}

impl NavLogoMode {
    pub const fn raw(self) -> f64 {
        match self {
            Self::Off => 0.0,
            Self::Sys1 => 1.0,
            Self::Sys2 => 2.0,
        }
    }

    /// Fail-closed decode: non-finite raw values decode as `None`
    /// (unknown), never as a valid switch position.
    pub fn from_raw(v: f64) -> Option<Self> {
        if !v.is_finite() {
            return None;
        }
        match v as u8 {
            0 => Some(Self::Off),
            1 => Some(Self::Sys1),
            2 => Some(Self::Sys2),
            _ => None,
        }
    }
}

/// Per-channel data quality (spec §21): `missing`, `stale`, and
/// `invalid` are different facts, and a stale value must never pass as
/// fresh evidence for a flight-control post-condition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DataQuality {
    /// Recent, plausible observation.
    Fresh,
    /// Present but older than the channel freshness window.
    Stale,
    /// Never received in this session.
    Missing,
    /// Received but non-finite / unrepresentable.
    Invalid,
}

/// A single canonical snapshot of aircraft + simulator state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TelemetrySnapshot {
    pub timestamp: SimTimestamp,

    // -- generic core -------------------------------------------------------
    pub position: Option<Position>,
    pub altitude_msl: Option<AltitudeFt>,
    pub altitude_agl: Option<AltitudeAglFt>,
    pub groundspeed: Option<SpeedKt>,
    pub indicated_airspeed: Option<SpeedKt>,
    pub vertical_speed: Option<VerticalSpeedFpm>,
    pub heading_true: Option<AngleDeg>,
    pub pitch: Option<AngleDeg>,
    pub bank: Option<AngleDeg>,
    pub on_ground: Option<bool>,
    /// Gear lever selected down (true = down).
    pub gear_handle_down: Option<bool>,
    /// Generic flaps handle index (0 = up).
    pub flaps_handle_index: Option<u8>,
    /// Engine combustion per engine slot (1..=4); `None` = unknown engine.
    pub engine_combustion: Option<[Option<bool>; 4]>,
    pub autopilot_master: Option<bool>,
    pub autothrottle_arm: Option<bool>,
    /// Beacon light state (standard `LIGHT BEACON` simvar).
    pub beacon_light: Option<bool>,

    // -- simulator timing ---------------------------------------------------
    pub sim_timing: SimTiming,

    // -- aircraft-specific extension -----------------------------------------
    /// Opaque aircraft-specific normalized values, keyed by a stable numeric
    /// id assigned by trusted adapter code (fd-simconnect binding table).
    /// `fd-core` deliberately attaches NO semantics to these ids; the
    /// aircraft layer (fd-aircraft) owns their meaning. Absent key = unknown.
    #[serde(with = "ext_values_serde", default)]
    pub aircraft_values: BTreeMap<u16, f64>,
    /// Quality annotations for channels that are NOT fresh (exception
    /// list; absent = fresh). Serde-default so old recordings load.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub channel_quality: BTreeMap<u16, DataQuality>,
}

impl TelemetrySnapshot {
    /// Empty (all-unknown) snapshot at a timestamp.
    pub const fn empty(timestamp: SimTimestamp) -> Self {
        Self {
            timestamp,
            channel_quality: BTreeMap::new(),
            position: None,
            altitude_msl: None,
            altitude_agl: None,
            groundspeed: None,
            indicated_airspeed: None,
            vertical_speed: None,
            heading_true: None,
            pitch: None,
            bank: None,
            on_ground: None,
            gear_handle_down: None,
            flaps_handle_index: None,
            engine_combustion: None,
            autopilot_master: None,
            autothrottle_arm: None,
            beacon_light: None,
            sim_timing: SimTiming {
                state: SimState::Unknown,
                sim_rate: None,
                slew_active: None,
            },
            aircraft_values: BTreeMap::new(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_snapshot_is_all_unknown() {
        let s = TelemetrySnapshot::empty(SimTimestamp::new(0));
        assert!(s.altitude_msl.is_none());
        assert!(s.on_ground.is_none());
        assert_eq!(s.sim_timing.state, SimState::Unknown);
        assert!(s.aircraft_values.is_empty());
    }

    #[test]
    fn nav_logo_mode_raw_mapping_is_total_and_injective() {
        for m in [NavLogoMode::Off, NavLogoMode::Sys1, NavLogoMode::Sys2] {
            assert_eq!(NavLogoMode::from_raw(m.raw()), Some(m));
        }
        assert_eq!(NavLogoMode::from_raw(7.0), None);
    }

    #[test]
    fn timestamp_ordering_is_by_ms() {
        assert!(SimTimestamp::new(1) < SimTimestamp::new(2));
        assert_eq!(SimTimestamp::new(5), SimTimestamp::new(5));
    }

    #[test]
    fn ext_values_serde_roundtrip_via_string_keys() {
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(3));
        s.aircraft_values.insert(1, 95.5);
        s.aircraft_values.insert(4, 2.0);
        let text = serde_json::to_string(&s).unwrap();
        assert!(text.contains("\"aircraft_values\":{\"1\":95.5,\"4\":2.0}"));
        let back: TelemetrySnapshot = serde_json::from_str(&text).unwrap();
        assert_eq!(back.aircraft_values.get(&1), Some(&95.5));
        assert_eq!(back.aircraft_values.get(&4), Some(&2.0));
    }
}
