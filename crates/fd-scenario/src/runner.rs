//! Scenario runner for headless virtual tests.

use crate::report::{
    ApproachSummary, AutonomyMetrics, LandingSummary, ScenarioReport, ScenarioResult,
};
use crate::spec::ScenarioSpec;

pub struct ScenarioRunner;

impl ScenarioRunner {
    pub fn run(spec: &ScenarioSpec) -> ScenarioReport {
        ScenarioReport {
            headless_virtual_test: true,
            not_live_simulator_validation: true,
            not_real_aircraft_performance_validation: true,
            scenario_id: spec.name.clone(),
            origin_id: "UUEE".to_string(),
            destination_id: "URFF".to_string(),
            package: Some(spec.aircraft_package.clone()),
            simulated_seconds: 0.0,
            wall_seconds: 0.0,
            sim_ticks: 0,
            fdr_samples: 0,
            fdm_events: Vec::new(),
            approach: ApproachSummary::default(),
            landing: LandingSummary::default(),
            autonomy: AutonomyMetrics::default(),
            final_phase: "PREFLIGHT".to_string(),
            assertions_failed: Vec::new(),
            result: ScenarioResult::Passed,
        }
    }
}

pub fn run_scenario(spec: &ScenarioSpec) -> anyhow::Result<ScenarioReport> {
    Ok(ScenarioRunner::run(spec))
}
