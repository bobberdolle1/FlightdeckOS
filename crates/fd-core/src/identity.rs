//! Aircraft identity: what aircraft is this, and how do we know?
//!
//! The stock X-Plane UDP interface streams FLOAT datarefs only. The identity
//! datarefs (`sim/aircraft/view/acf_ICAO`, `acf_author`, `acf_tailnum`, ...)
//! are byte-array refs and therefore unreachable over RREF. Until a native
//! plugin bridge exists, identity arrives as `UserProvided` (an operator
//! claim) or stays `Unknown`. Nothing in FlightdeckOS may infer identity from
//! telemetry vibes: `Unknown` never silently becomes known.

use serde::{Deserialize, Serialize};

/// How this identity claim was obtained. Drives trust decisions.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IdentitySource {
    /// Nothing observed or provided.
    #[default]
    Unknown,
    /// Operator-supplied claim (CLI flag, config). A hint, never proof.
    UserProvided,
    /// Read from the simulator through a trusted transport (SDK plugin).
    Adapter,
}

/// Best-effort aircraft identity snapshot. Every field independently
/// unknown; `source` records the provenance of the whole claim.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AircraftIdentity {
    pub icao: Option<String>,
    pub tail_number: Option<String>,
    pub author: Option<String>,
    pub description: Option<String>,
    /// ACF file name (e.g. `B738.acf`), when the transport exposes it.
    pub acf_name: Option<String>,
    pub source: IdentitySource,
}

impl AircraftIdentity {
    /// Fully unknown identity — the honest default for generic mode.
    pub fn unknown() -> Self {
        Self::default()
    }

    /// Operator-claimed identity. Fields stay `None` when not supplied.
    pub fn user_provided(icao: Option<String>) -> Self {
        Self {
            icao,
            source: IdentitySource::UserProvided,
            ..Self::default()
        }
    }

    /// True only when identity was read by trusted adapter code.
    pub fn is_trusted(&self) -> bool {
        self.source == IdentitySource::Adapter
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unknown_is_default_and_untrusted() {
        let id = AircraftIdentity::unknown();
        assert_eq!(id.icao, None);
        assert_eq!(id.source, IdentitySource::Unknown);
        assert!(!id.is_trusted());
    }

    #[test]
    fn user_provided_is_a_claim_not_proof() {
        let id = AircraftIdentity::user_provided(Some("IL76".into()));
        assert_eq!(id.icao.as_deref(), Some("IL76"));
        assert_eq!(id.source, IdentitySource::UserProvided);
        assert!(!id.is_trusted(), "user claims are never trusted reads");
    }

    #[test]
    fn serde_roundtrip() {
        let id = AircraftIdentity::user_provided(Some("A320".into()));
        let json = serde_json::to_string(&id).unwrap();
        let back: AircraftIdentity = serde_json::from_str(&json).unwrap();
        assert_eq!(back, id);
    }
}
