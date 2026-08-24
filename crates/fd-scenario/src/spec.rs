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
    /// Negative scenario: the run PASSES only when the mission FAILS
    /// (or times out) as expected. A nominal PASS is reported as a
    /// scenario failure ("expected failure did not occur").
    #[serde(default)]
    pub expected_failure: bool,
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
        Ok(())
    }
}
