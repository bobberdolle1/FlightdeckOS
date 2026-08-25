//! Scenario runner implementation.
//!
//! Wires: package (optional) + VirtualSimulator + Runtime + MissionController
//! + FDR/FDM into a deterministic headless execution loop.

use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;

use fd_core::adapter::{FlightControlTargets, SimulatorAdapter};
use fd_core::events::EventSource;
use fd_core::telemetry::TelemetrySnapshot;
use fd_fdm::fdm::FdmAnalyzer;
use fd_fdm::fdr::{FdrEvent, FlightRecording, Recorder};
use fd_fdm::qoa::{ApproachAnalyzer, GateClassification, StabilizationCriteria};
use fd_fdm::qol;
use fd_mission::controller::{MissionContext, MissionController, MissionParameters, MissionPhase};
use fd_mission::route::RouteFollower;
use fd_runtime::{DeadlineTicks, Runtime, SessionId, TraceWriter};
use fd_virtual::VirtualSimulator;

use crate::report::{
    ApproachSummary, AutonomyMetrics, FdmEventSummary, LandingSummary, ScenarioReport,
    ScenarioResult,
};

/// Adapter handle delegating to the shared virtual world.
struct VirtualAdapter {
    world: Rc<RefCell<VirtualSimulator>>,
}

impl SimulatorAdapter for VirtualAdapter {
    fn connect(&mut self) -> Result<(), fd_core::adapter::AdapterError> {
        Ok(())
    }
    fn disconnect(&mut self) {}
    fn is_connected(&self) -> bool {
        self.world.borrow().is_connected()
    }
    fn poll(&mut self) -> Result<Vec<TelemetrySnapshot>, fd_core::adapter::AdapterError> {
        if !self.world.borrow().is_connected() {
            return Err(fd_core::adapter::AdapterError::NotConnected);
        }
        Ok(vec![self.world.borrow().snapshot()])
    }
    fn capability(&self, action: CockpitAction) -> fd_core::adapter::Capability {
        self.world.borrow().capability(action)
    }
    fn execute(&mut self, action: CockpitAction) -> Result<(), fd_core::adapter::AdapterError> {
        self.world.borrow_mut().execute(action)
    }
}

use fd_core::actions::CockpitAction;

/// Run a headless scenario end-to-end. Deterministic.
pub fn run_scenario(spec_path: &std::path::Path) -> Result<ScenarioReport, String> {
    let spec = crate::spec::load_spec(spec_path).map_err(|e| e.to_string())?;
    let wall_start = std::time::Instant::now();

    let package_dir: Option<PathBuf> = spec.scenario.package.as_ref().map(|p| {
        let p = PathBuf::from(p);
        if p.is_absolute() {
            p
        } else {
            PathBuf::from(".").join(p)
        }
    });
    let validated_package = match &package_dir {
        Some(dir) => Some(fd_sop::load_package(dir).map_err(|e| e.to_string())?),
        None => None,
    };

    let world = Rc::new(RefCell::new(VirtualSimulator::new(
        spec.origin.lat_deg,
        spec.origin.lon_deg,
        spec.origin.elevation_ft,
        spec.initial_conditions.heading_deg,
        spec.simulation.dt_ms,
    )));
    if let Some(faults) = &spec.faults {
        faults
            .validate()
            .map_err(|e| format!("invalid [faults] in scenario: {e}"))?;
        world.borrow_mut().set_faults(faults.clone());
    }
    {
        let mut w = world.borrow_mut();
        w.systems_mut()
            .set_engines_running(spec.initial_conditions.engines_running);
        if let Some(alt) = spec.initial_conditions.altitude_ft {
            w.kinematics_mut().start_airborne_at(alt);
        }
    }

    let trace_path = std::env::temp_dir().join(format!(
        "fd_scenario_trace_{}_{}.jsonl",
        spec.scenario.id.replace(['/', '\\'], "_"),
        std::process::id()
    ));
    let _ = std::fs::remove_file(&trace_path);
    let trace_writer =
        TraceWriter::create(&trace_path).map_err(|e| format!("trace create failed: {e}"))?;
    let catalog = validated_package
        .as_ref()
        .map(|_| fd_aircraft::catalog::a32nx_default_catalog())
        .unwrap_or_default();
    let mut runtime = Runtime::new(
        Box::new(VirtualAdapter {
            world: world.clone(),
        }),
        trace_writer,
        SessionId(0),
        catalog,
        DeadlineTicks(200),
    );
    runtime.start().map_err(|e| e.to_string())?;

    if let (Some(flow_id), Some(pkg)) = (&spec.scenario.flow, &validated_package) {
        let def = pkg
            .flows
            .iter()
            .find(|f| &f.id == flow_id)
            .ok_or_else(|| format!("flow `{flow_id}` not found in package"))?
            .clone();
        runtime.start_flow(def).map_err(|e| e.to_string())?;
    }

    let route_wpts = vec![
        fd_mission::Waypoint {
            id: spec.origin.id.clone(),
            lat_deg: spec.origin.lat_deg,
            lon_deg: spec.origin.lon_deg,
        },
        fd_mission::Waypoint {
            id: spec.destination.id.clone(),
            lat_deg: spec.destination.lat_deg,
            lon_deg: spec.destination.lon_deg,
        },
    ];
    let mut route = RouteFollower::new(route_wpts, 5.0);
    let mission_params = MissionParameters {
        cruise_altitude_ft: spec.mission.cruise_altitude_ft,
        ..MissionParameters::default()
    };
    let mut mission = MissionController::new(mission_params);
    let mut recorder = Recorder::new();
    let mut recording = FlightRecording::default();
    let mut fdm = FdmAnalyzer::new_development_default();
    let mut qoa_analyzer: Option<ApproachAnalyzer> = None;
    let mut fdr_event_seq: u64 = 0;

    let max_ticks = spec.simulation.max_sim_seconds * 1000 / spec.simulation.dt_ms;
    let mut ticks: u64 = 0;

    while ticks < max_ticks {
        world.borrow_mut().advance_tick();
        ticks += 1;

        match runtime.tick(EventSource::Simulator) {
            Ok(_) => {}
            // Deterministic disconnect window: the runtime gets no telemetry
            // this tick, but the world keeps evolving and the scenario keeps
            // running — recovery is exercised through the production
            // adapter boundary, not special-cased.
            Err(fd_runtime::RuntimeError::Adapter(
                fd_core::adapter::AdapterError::NotConnected,
            )) => {}
            Err(e) => return Err(e.to_string()),
        }

        let snapshot_now: TelemetrySnapshot = world.borrow().snapshot();
        let phase_label = format!("{:?}", mission.phase());
        let sample = recorder.record(&snapshot_now, &phase_label);
        recording.push_sample(sample.clone());

        for ev in fdm.process(&sample) {
            fdr_event_seq += 1;
            recording.push_event(FdrEvent {
                seq: fdr_event_seq,
                timestamp: sample.timestamp,
                kind: "fdm".into(),
                detail: format!("{:?} measured={:.0}", ev.kind, ev.measured),
                payload: None,
            });
        }

        let lat = snapshot_now
            .position
            .as_ref()
            .map(|p| p.lat.value())
            .unwrap_or(0.0);
        let lon = snapshot_now
            .position
            .as_ref()
            .map(|p| p.lon.value())
            .unwrap_or(0.0);
        let dist_dest = route.distance_to_destination(lat, lon);

        if matches!(
            mission.phase(),
            MissionPhase::Descent | MissionPhase::Approach
        ) {
            qoa_analyzer
                .get_or_insert_with(|| {
                    ApproachAnalyzer::new(StabilizationCriteria::default(), Some(160.0))
                })
                .push(sample);
        }

        let (bearing_to_wp, _dist_wp) = route.guidance(lat, lon);
        let ctx = MissionContext {
            snapshot: &snapshot_now,
            distance_to_destination_nm: dist_dest,
            bearing_to_waypoint_deg: bearing_to_wp,
        };
        let pre_phase = mission.phase();
        {
            let mut world_mut = world.borrow_mut();
            let targets: &mut dyn FlightControlTargets = &mut *world_mut;
            mission.step(&ctx, targets, &mut route);
        }
        if mission.phase() != pre_phase {
            fdr_event_seq += 1;
            recording.push_event(FdrEvent {
                seq: fdr_event_seq,
                timestamp: snapshot_now.timestamp,
                kind: "mission".into(),
                detail: format!("{:?} -> {:?}", pre_phase, mission.phase()),
                payload: None,
            });
        }

        if matches!(
            mission.phase(),
            MissionPhase::Completed | MissionPhase::Failed
        ) {
            break;
        }
        if std::env::var("FD_SCENARIO_DEBUG").is_ok() && ticks.is_multiple_of(3600) {
            let snap = world.borrow().snapshot();
            eprintln!(
                "DBG t={:>6}s phase={:?} alt={:.0} gs={:.0} ias={:.0} dist_dest={:.1}",
                snap.timestamp.ms / 1000,
                mission.phase(),
                snap.altitude_msl.map(|v| v.value()).unwrap_or(0.0),
                snap.groundspeed.map(|v| v.value()).unwrap_or(0.0),
                snap.indicated_airspeed.map(|v| v.value()).unwrap_or(0.0),
                route.distance_to_destination(
                    snap.position.as_ref().map(|p| p.lat.value()).unwrap_or(0.0),
                    snap.position.as_ref().map(|p| p.lon.value()).unwrap_or(0.0),
                )
            );
        }
    }

    let simulated_seconds = (ticks * spec.simulation.dt_ms) as f64 / 1000.0;
    let wall_seconds = wall_start.elapsed().as_secs_f64();

    let approach_summary: Option<ApproachSummary> = qoa_analyzer.map(|a| {
        let r = a.finish();
        ApproachSummary {
            // Gate classification: Stable=true, Unstable=false,
            // Indeterminate/crossed-unknown=None (never fabricated as
            // stable — Task 6 §25).
            stabilized_at_1000ft: r
                .classifications
                .first()
                .copied()
                .flatten()
                .map(|c| c == GateClassification::Stable),
            stabilized_at_500ft: r
                .classifications
                .get(1)
                .copied()
                .flatten()
                .map(|c| c == GateClassification::Stable),
            max_sink_rate_fpm: r.max_sink_rate_fpm,
            go_around_detected: r.go_around.as_ref().map(|_| true),
        }
    });
    let landing_report = qol::analyze(&recording.samples);

    let trace_text = std::fs::read_to_string(&trace_path).unwrap_or_default();
    let mut autonomy = AutonomyMetrics::default();
    for line in trace_text.lines() {
        if line.contains("\"kind\":\"action_requested\"") {
            autonomy.actions_requested += 1;
        } else if line.contains("\"kind\":\"action_verified\"") {
            autonomy.actions_verified += 1;
        } else if line.contains("\"kind\":\"action_rejected\"") {
            autonomy.actions_rejected += 1;
        } else if line.contains("\"kind\":\"action_failed\"") {
            autonomy.actions_failed += 1;
        } else if line.contains("\"kind\":\"step_verified\"") {
            autonomy.procedure_steps_completed += 1;
        } else if line.contains("\"kind\":\"step_failed\"") {
            autonomy.procedure_steps_failed += 1;
        }
    }

    // Scenario verdict (spec §40): a nominal scenario PASSES only when the
    // mission reached Completed. Mission Failed, or the tick budget
    // exhausting before completion, is a FAILED scenario — never a silent
    // false success. A negative scenario (spec §3A) passes ONLY when the
    // SPECIFIC expected trigger fired: an unrelated failure mode or a
    // nominal pass can never satisfy the expectation.
    let mission_failed = matches!(mission.phase(), fd_mission::MissionPhase::Failed);
    let timed_out = !matches!(mission.phase(), fd_mission::MissionPhase::Completed);
    let actions_failed = autonomy.actions_failed > 0;
    let mut assertions_failed: Vec<String> = Vec::new();
    if actions_failed {
        assertions_failed.push(format!(
            "{} dispatched action(s) failed post-condition verification",
            autonomy.actions_failed
        ));
    }
    // The trigger that actually fired (priority: mission failure, then
    // timeout, then action failures — at most one is reported).
    let fired_trigger = if mission_failed {
        Some(crate::spec::ExpectedTrigger::MissionFailed)
    } else if timed_out {
        Some(crate::spec::ExpectedTrigger::TickTimeout)
    } else if actions_failed {
        Some(crate::spec::ExpectedTrigger::ActionFailed)
    } else {
        None
    };
    let result = match (spec.scenario.expected_failure, fired_trigger) {
        // Nominal scenario, nominal pass.
        (None, None) => ScenarioResult::Passed,
        // Nominal scenario, something failed: report the trigger.
        (None, Some(t)) => {
            let reason = match t {
                crate::spec::ExpectedTrigger::MissionFailed => {
                    assertions_failed
                        .push(format!("mission failed in phase {:?}", mission.phase()));
                    format!("mission failed in phase {:?}", mission.phase())
                }
                crate::spec::ExpectedTrigger::TickTimeout => {
                    assertions_failed.push(format!(
                        "mission did not complete within the tick budget (phase {:?})",
                        mission.phase()
                    ));
                    format!(
                        "mission did not complete within the tick budget (phase {:?})",
                        mission.phase()
                    )
                }
                crate::spec::ExpectedTrigger::ActionFailed => {
                    "dispatched action(s) failed post-condition verification".to_string()
                }
            };
            ScenarioResult::Failed { reason }
        }
        // Negative scenario: pass ONLY on the exact expected trigger.
        (Some(expected), Some(fired)) if fired == expected => ScenarioResult::Passed,
        // Negative scenario: expected trigger did not fire...
        (Some(expected), fired) => {
            let reason = match fired {
                Some(fired) => format!(
                    "expected trigger `{}` did not fire: unrelated trigger `{}` fired instead",
                    expected.as_str(),
                    fired.as_str()
                ),
                None => format!(
                    "expected trigger `{}` did not fire: mission completed nominally",
                    expected.as_str()
                ),
            };
            assertions_failed.push(reason.clone());
            ScenarioResult::Failed { reason }
        }
    };
    let landing_summary = LandingSummary {
        touchdown_occurred: landing_report.touchdown.is_some(),
        touchdown_vertical_speed_fpm: landing_report.touchdown.as_ref().and_then(|t| t.vs_fpm),
        touchdown_pitch_deg: landing_report.touchdown.as_ref().and_then(|t| t.pitch_deg),
    };

    let mut fdm_summary: Vec<FdmEventSummary> = Vec::new();
    for e in fdm.events() {
        let kind = format!("{:?}", e.kind).to_lowercase();
        match fdm_summary.iter_mut().find(|c| c.kind == kind) {
            Some(c) => c.count += 1,
            None => fdm_summary.push(FdmEventSummary { kind, count: 1 }),
        }
    }

    // Capability report composition (generic vs profiled).
    // Task 4 §1-2: headless/virtual proof is NEVER `Verified`. Verified is
    // reserved for capabilities demonstrated through a live adapter.
    let mut caps = fd_core::capability::CapabilityReport::new();
    use fd_core::capability::{CapabilityStatus as Cap, EvidenceSource as Ev};
    let sim_ev = Ev::VirtualSim;
    for cap in [
        "telemetry.position",
        "telemetry.airspeed",
        "telemetry.attitude",
        "telemetry.gear",
        "telemetry.vertical_speed",
        "fdr.recording",
        "fdm.analysis",
        "qoa.approach",
        "autonomy.route_guidance",
    ] {
        caps.set_with_evidence(cap, Cap::Supported, sim_ev);
    }
    for cap in [
        "systems.electrical",
        "systems.pneumatic",
        "action.lights",
        "action.beacon",
        "procedure.before_start",
        "procedure.any",
        "autonomy.sop",
    ] {
        if validated_package.is_some() {
            caps.set_with_evidence(cap, Cap::Supported, sim_ev);
        } else {
            // No package: SOP/system knowledge is UNAVAILABLE — never guessed.
            caps.set_with_evidence(cap, Cap::Unavailable, sim_ev);
        }
    }
    caps.set_with_evidence(
        "action.apu",
        if validated_package.is_some() {
            Cap::Supported
        } else {
            Cap::Unsupported
        },
        sim_ev,
    );
    // Route guidance is implemented and virtual-proven; LIVE flight control
    // remains unproven until demonstrated through a real adapter.
    caps.set_with_evidence("autonomy.flight", Cap::Supported, sim_ev);
    caps.set_with_evidence("autonomy.ground", Cap::Unavailable, sim_ev);

    Ok(ScenarioReport {
        capabilities: caps.entries_sorted(),
        headless_virtual_test: true,
        not_live_simulator_validation: true,
        not_real_aircraft_performance_validation: true,
        scenario_id: spec.scenario.id.clone(),
        origin_id: spec.origin.id.clone(),
        destination_id: spec.destination.id.clone(),
        package: spec.scenario.package.clone(),
        simulated_seconds,
        wall_seconds,
        sim_ticks: ticks,
        fdr_samples: recording.len() as u64,
        fdm_events: fdm_summary,
        approach: approach_summary.unwrap_or_default(),
        landing: landing_summary,
        autonomy,
        route_waypoints: spec.route.as_ref().map(|r| {
            r.waypoints
                .iter()
                .map(|w| crate::report::RouteWaypointReport {
                    id: w.id.clone(),
                    lat_deg: w.lat_deg,
                    lon_deg: w.lon_deg,
                })
                .collect()
        }),
        final_phase: format!("{:?}", mission.phase()),
        assertions_failed,
        result,
    })
}
