//! Read-only crew context view (Task 7 §33-34).
//!
//! A BOUNDED, serializable summary of everything a future crew model may
//! see, and nothing else. This is the contract between the observation
//! runtime and a future LLM/tool layer:
//!
//! - No raw DataRef dumps, no 5000-item lists (§34): every section is a
//!   fixed-size summary.
//! - Unknown stays unknown (`None`), never a guess (§50).
//! - Read-only by construction: the view carries no action surface.
//!
//! The builder consumes ALREADY-COMPUTED primitives (phase label, plan
//! summary, route status...) so this crate stays decoupled from the
//! analytics crates (dependency direction preserved).

use fd_core::fplan::ProcedureContext;
use fd_core::identity::AircraftIdentity;
use serde::{Deserialize, Serialize};

/// Bumped on incompatible view schema changes.
pub const CREW_VIEW_FORMAT_VERSION: u32 = 1;

/// Aircraft summary line.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AircraftSummary {
    /// ICAO type when known.
    pub icao: Option<String>,
    /// Identity provenance ("UserProvided"/"Adapter"/"Unknown").
    pub identity_source: String,
    /// Active aircraft package id, when one is loaded and validated.
    pub package: Option<String>,
}

/// Navigation/procedure context line (§16: navigation context, NOT
/// mission phase).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct NavigationSummary {
    pub origin: Option<String>,
    pub destination: Option<String>,
    /// "Enroute"/"SID"/"STAR"/"APPROACH"/"MissedApproach"/"Unknown".
    pub procedure_phase: Option<String>,
    /// Best correlated procedure, when deterministically supported.
    pub procedure: Option<ProcedureContext>,
}

/// Flight-plan summary line (§11 answers).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct FlightPlanSummaryLine {
    /// Plan exists (FMS observed a non-empty primary plan).
    pub observed: bool,
    pub device: Option<String>,
    pub entry_count: Option<usize>,
    /// Identifier of the entry the FMS is flying to.
    pub active_waypoint: Option<String>,
    pub destination_waypoint: Option<String>,
    pub approach_loaded: Option<bool>,
    /// Plan revision counter (meaningful changes, §12).
    pub revision: u64,
}

/// Route-monitoring status line.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RouteStatusLine {
    /// Route source ("xplane_fms"/"openairac"/"operator"/...).
    pub source: Option<String>,
    pub waypoint_count: Option<usize>,
    pub active_leg: Option<usize>,
    pub next_waypoint: Option<String>,
    pub distance_to_destination_nm: Option<f64>,
    /// Cross-track error, nm (signed; None = unknown).
    pub cross_track_nm: Option<f64>,
}

/// Systems snapshot line (from the latest telemetry sample).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SystemsSummary {
    pub on_ground: Option<bool>,
    pub gear_down: Option<bool>,
    pub any_engine_running: Option<bool>,
    pub autopilot_master: Option<bool>,
    pub altitude_msl_ft: Option<f64>,
    pub indicated_airspeed_kt: Option<f64>,
    pub vertical_speed_fpm: Option<f64>,
}

/// Data-quality line (§24: gaps are reported, never hidden).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct DataQualityLine {
    pub samples: u64,
    /// Channels with annotated (non-fresh) samples: id -> occurrences.
    pub non_fresh_channels: std::collections::BTreeMap<u16, u64>,
    pub max_gap_ms: u64,
    pub gaps_over_threshold: u64,
}

/// The bounded crew view (§34).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct CrewView {
    pub format_version: u32,
    pub aircraft: AircraftSummary,
    /// Current flight/mission phase label from the runtime phase engine.
    pub phase: Option<String>,
    pub navigation: NavigationSummary,
    pub flight_plan: FlightPlanSummaryLine,
    pub route: RouteStatusLine,
    pub systems: SystemsSummary,
    /// Capability id -> status string ("Available"/"Unavailable"/...).
    pub capabilities: Vec<(String, String)>,
    /// Active FDM event descriptions (bounded by the FDM episode model).
    pub fdm_active_events: Vec<String>,
    /// One-line shadow state ("armed: N intents, M matches...") when the
    /// shadow is armed; None = not armed.
    pub shadow: Option<String>,
    pub data_quality: DataQualityLine,
}

impl CrewView {
    /// A fresh, empty view for one session.
    pub fn new() -> Self {
        Self {
            format_version: CREW_VIEW_FORMAT_VERSION,
            ..Default::default()
        }
    }

    /// Serialize for a future tool surface (file or IPC payload).
    pub fn to_json_pretty(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }
}

/// Build the aircraft summary from an identity.
pub fn aircraft_summary(identity: &AircraftIdentity, package: Option<&str>) -> AircraftSummary {
    AircraftSummary {
        icao: identity.icao.clone(),
        identity_source: format!("{:?}", identity.source),
        package: package.map(|s| s.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn view_is_bounded_and_serializable() {
        let mut view = CrewView::new();
        view.aircraft = AircraftSummary {
            icao: Some("C172".into()),
            identity_source: "UserProvided".into(),
            package: None,
        };
        view.phase = Some("Cruise".into());
        view.flight_plan = FlightPlanSummaryLine {
            observed: true,
            device: Some("StockGps".into()),
            entry_count: Some(5),
            active_waypoint: Some("SEALS".into()),
            destination_waypoint: Some("KSNA".into()),
            approach_loaded: Some(false),
            revision: 1,
        };
        // Bounded: sections are fixed-size; capabilities/events are small
        // by contract.
        view.capabilities
            .push(("fms.plan".into(), "Available".into()));
        let json = view.to_json_pretty().unwrap();
        assert!(json.contains("\"SEALS\""));
        assert!(json.contains("\"format_version\": 1"));
    }

    #[test]
    fn unknown_stays_none() {
        let view = CrewView::new();
        assert_eq!(view.phase, None);
        assert!(!view.flight_plan.observed);
        assert_eq!(view.route.cross_track_nm, None);
    }

    #[test]
    fn aircraft_summary_maps_identity() {
        let identity = AircraftIdentity {
            icao: Some("C172".into()),
            tail_number: None,
            author: None,
            description: None,
            acf_name: None,
            source: fd_core::identity::IdentitySource::UserProvided,
        };
        let s = aircraft_summary(&identity, None);
        assert_eq!(s.icao.as_deref(), Some("C172"));
        assert_eq!(s.package, None);
    }
}
