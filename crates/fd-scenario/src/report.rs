//! Scenario result + machine-checkable report.

use serde::{Deserialize, Serialize};

/// Overall scenario verdict.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioResult {
    Passed,
    Failed { reason: String },
}

impl ScenarioResult {
    pub const fn passed(&self) -> bool {
        matches!(self, Self::Passed)
    }
}

/// Autonomy self-evaluation counters (raw metrics first — no score).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AutonomyMetrics {
    pub actions_requested: u64,
    pub actions_verified: u64,
    pub actions_rejected: u64,
    pub actions_failed: u64,
    pub actions_timed_out: u64,
    pub procedure_steps_completed: u64,
    pub procedure_steps_failed: u64,
    pub user_interventions: u64,
}

/// Full structured mission/scenario report.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioReport {
    /// Explicit proof-domain labels.
    pub headless_virtual_test: bool,
    pub not_live_simulator_validation: bool,
    pub not_real_aircraft_performance_validation: bool,
    pub scenario_id: String,
    pub origin_id: String,
    pub destination_id: String,
    /// Aircraft package used; `None` = generic/unknown aircraft mode.
    pub package: Option<String>,
    pub simulated_seconds: f64,
    pub wall_seconds: f64,
    pub sim_ticks: u64,
    pub fdr_samples: u64,
    pub fdm_events: Vec<FdmEventSummary>,
    pub approach: ApproachSummary,
    pub landing: LandingSummary,
    pub autonomy: AutonomyMetrics,
    pub final_phase: String,
    pub assertions_failed: Vec<String>,
    pub result: ScenarioResult,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FdmEventSummary {
    pub kind: String,
    pub count: u64,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ApproachSummary {
    pub stabilized_at_1000ft: Option<bool>,
    pub stabilized_at_500ft: Option<bool>,
    pub max_sink_rate_fpm: Option<f64>,
    pub go_around_detected: Option<bool>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct LandingSummary {
    pub touchdown_occurred: bool,
    pub touchdown_vertical_speed_fpm: Option<f64>,
    pub touchdown_pitch_deg: Option<f64>,
}
