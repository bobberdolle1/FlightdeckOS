//! Scenario engine integration tests: determinism, generic (no-package)
//! mode, tick caps, invalid specs.

use fd_scenario::{ScenarioResult, run_scenario};

fn write_scenario(spec_text: &str) -> std::path::PathBuf {
    let dir = tempfile::tempdir().unwrap();
    let p = dir.path().join("scenario.toml");
    std::fs::write(&p, spec_text).unwrap();
    // Leak the tempdir so files outlive the test helper.
    std::mem::forget(dir);
    p
}

const BASE_SPEC: &str = r#"
[scenario]
id = "test-generic"
[origin]
id = "UUEE"
lat_deg = 55.972642
lon_deg = 37.414589
elevation_ft = 622.0
[destination]
id = "ULLI"
lat_deg = 59.800278
lon_deg = 30.2625
elevation_ft = 79.0
[initial_conditions]
heading_deg = 330.0
engines_running = true
[mission]
cruise_altitude_ft = 30000.0
[simulation]
dt_ms = 100
max_sim_seconds = 7200
"#;

#[test]
fn generic_unknown_aircraft_completes_without_package() {
    // No package: FDR/FDM/mission work; SOP is unavailable — never guessed.
    let path = write_scenario(BASE_SPEC);
    let report = run_scenario(&path).unwrap();
    assert!(report.fdr_samples > 1000, "FDR must record the flight");
    assert_eq!(
        report.autonomy.procedure_steps_completed, 0,
        "no SOP steps may exist without a package"
    );
    assert!(report.landing.touchdown_occurred, "must land");
    let sop_cap = report
        .capabilities
        .iter()
        .find(|(k, _)| k == "procedure.any")
        .map(|(_, v)| v.as_str());
    assert_eq!(sop_cap, Some("unavailable"));
    assert_eq!(report.result, ScenarioResult::Passed);
}

#[test]
fn same_scenario_twice_produces_identical_reports() {
    let path_a = write_scenario(BASE_SPEC);
    let text = std::fs::read_to_string(&path_a).unwrap();
    let path_b = write_scenario(&text);

    let a = run_scenario(&path_a).unwrap();
    let b = run_scenario(&path_b).unwrap();

    assert_eq!(a.sim_ticks, b.sim_ticks);
    assert_eq!(a.final_phase, b.final_phase);
    assert_eq!(a.autonomy, b.autonomy);
    assert_eq!(a.approach, b.approach);
    assert_eq!(a.landing, b.landing);
    assert_eq!(a.fdm_events, b.fdm_events);
}

#[test]
fn max_duration_prevents_infinite_run() {
    let short = BASE_SPEC.replace("max_sim_seconds = 7200", "max_sim_seconds = 60");
    let path = write_scenario(&short);
    let report = run_scenario(&short_path(&path)).unwrap_or_else(|e| panic!("{e}"));
    assert!(
        report.sim_ticks <= 601,
        "tick cap violated: {}",
        report.sim_ticks
    );
}

fn short_path(p: &std::path::Path) -> std::path::PathBuf {
    p.to_path_buf()
}

#[test]
fn invalid_spec_rejected() {
    let bad = BASE_SPEC.replace("dt_ms = 100", "dt_ms = 0");
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("bad.toml");
    std::fs::write(&path, &bad).unwrap();
    assert!(run_scenario(&path).is_err());
}
