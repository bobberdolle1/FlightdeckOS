//! Scenario specification (TOML, typed).
//!
//! Example:
//! ```toml
//! [scenario]
//! id = "uuee-ulli-headless"
//! package = "aircraft/a32nx"          # optional: omit for generic mode
//! flow = "before_start"               # optional: flow to start
//!
//! [origin]
//! id = "UUEE"
//! lat_deg = 55.972642
//! lon_deg = 37.414589
//! elevation_ft = 622.0
//!
//! [destination]
//! id = "ULLI"
//! lat_deg = 59.800278
//! lon_deg = 30.2625
//! elevation_ft = 79.0
//!
//! [initial_conditions]
//! heading_deg = 330.0
//!
//! [mission]
//! cruise_altitude_ft = 34000.0
//!
//! [simulation]
//! dt_ms = 100
//! max_sim_seconds = 7200
//! ```

use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, thiserror::Error)]
pub enum ScenarioSpecError {
    #[error("scenario io error: {0}")]
    Io(String),
    #[error("scenario parse error: {0}")]
    Toml(String),
    #[error("invalid scenario: {0}")]
    Invalid(String),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioSpec {
    pub scenario: ScenarioHeader,
    pub origin: AirportSpec,
    pub destination: AirportSpec,
    #[serde(default)]
    pub initial_conditions: InitialConditions,
    #[serde(default)]
    pub mission: MissionSpec,
    pub simulation: SimulationSpec,
    /// Deterministic fault injection (spec §41); omit for nominal runs.
    #[serde(default)]
    pub faults: Option<fd_virtual::faults::FaultConfig>,
    /// Optional explicit route (Task 6 §18: operator/scenario route source).
    /// Waypoints are carried into the report for route-monitor wiring.
    #[serde(default)]
    pub route: Option<RouteSpec>,
}

/// Scenario-declared route (Task 6 §17-18): an ordered waypoint list with
/// scenario provenance. This is TEST vocabulary — the production route
/// source is OpenAIRAC/operator input.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RouteSpec {
    /// At least 2 waypoints; validated.
    pub waypoints: Vec<WaypointSpec>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WaypointSpec {
    pub id: String,
    pub lat_deg: f64,
    pub lon_deg: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioHeader {
    pub id: String,
    /// Optional aircraft package directory (relative to repo root).
    #[serde(default)]
    pub package: Option<String>,
    /// Optional flow id to start from the package.
    #[serde(default)]
    pub flow: Option<String>,
    /// Negative scenario: the run PASSES only when the SPECIFIC expected
    /// trigger fires. A nominal PASS, a different trigger, or no failure
    /// at all is a scenario failure — an unrelated error can never
    /// satisfy the expectation.
    #[serde(default)]
    pub expected_failure: Option<ExpectedTrigger>,
}

/// Which deterministic failure trigger a negative scenario expects
/// (spec §3A: precise expectation, not "any failure means success").
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExpectedTrigger {
    /// Mission controller reached `Failed`.
    MissionFailed,
    /// Tick budget expired before the mission completed.
    TickTimeout,
    /// One or more dispatched actions failed post-condition verification.
    ActionFailed,
}

impl ExpectedTrigger {
    /// Human-readable name used in report reasons.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::MissionFailed => "mission_failed",
            Self::TickTimeout => "tick_timeout",
            Self::ActionFailed => "action_failed",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AirportSpec {
    pub id: String,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub elevation_ft: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct InitialConditions {
    pub heading_deg: f64,
    pub engines_running: bool,
    pub on_ground: bool,
    /// Initial MSL altitude; defaults to origin elevation (on ground).
    /// Setting it above elevation starts the scenario airborne.
    pub altitude_ft: Option<f64>,
}

impl Default for InitialConditions {
    fn default() -> Self {
        Self {
            heading_deg: 0.0,
            engines_running: false,
            on_ground: true,
            altitude_ft: None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct MissionSpec {
    pub cruise_altitude_ft: f64,
}

impl Default for MissionSpec {
    fn default() -> Self {
        Self {
            cruise_altitude_ft: 34_000.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimulationSpec {
    /// Fixed simulated timestep (ms). Deterministic clock quantum.
    pub dt_ms: u64,
    /// Hard cap of simulated time (s) — prevents infinite runs.
    pub max_sim_seconds: u64,
}

/// Load a scenario spec from TOML.
pub fn load_spec(path: &Path) -> Result<ScenarioSpec, ScenarioSpecError> {
    let text = std::fs::read_to_string(path).map_err(|e| ScenarioSpecError::Io(e.to_string()))?;
    parse_spec(&text)
}

pub fn parse_spec(text: &str) -> Result<ScenarioSpec, ScenarioSpecError> {
    let spec: ScenarioSpec =
        toml::from_str(text).map_err(|e| ScenarioSpecError::Toml(e.to_string()))?;
    spec.validate()?;
    Ok(spec)
}

impl ScenarioSpec {
    fn validate(&self) -> Result<(), ScenarioSpecError> {
        if self.scenario.id.trim().is_empty() {
            return Err(ScenarioSpecError::Invalid("scenario id is empty".into()));
        }
        if self.simulation.dt_ms == 0 {
            return Err(ScenarioSpecError::Invalid("dt_ms must be > 0".into()));
        }
        if self.simulation.max_sim_seconds == 0 {
            return Err(ScenarioSpecError::Invalid(
                "max_sim_seconds must be > 0".into(),
            ));
        }
        if self.origin.id == self.destination.id {
            return Err(ScenarioSpecError::Invalid(
                "origin and destination are the same airport".into(),
            ));
        }
        if let Some(route) = &self.route {
            if route.waypoints.len() < 2 {
                return Err(ScenarioSpecError::Invalid(
                    "route needs at least 2 waypoints".into(),
                ));
            }
            for (i, w) in route.waypoints.iter().enumerate() {
                if !w.id.trim().is_empty() && !w.lat_deg.is_finite() && !w.lon_deg.is_finite() {
                    return Err(ScenarioSpecError::Invalid(format!(
                        "route waypoint {i} invalid"
                    )));
                }
            }
        }
        Ok(())
    }
}
