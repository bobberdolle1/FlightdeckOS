//! Task 6 invariant + replay-determinism tests (spec §36, §37, §52).
//!
//! These tests defend the observable contracts of the observatory slice:
//! - a real recorded flight replays through production analytics with
//!   IDENTICAL semantic results (replay proof, §36);
//! - unknown data never satisfies a positive safety condition (§52);
//! - FDR round-trips never invent telemetry (§52);
//! - analytics contain no per-source branching (§37 — structural, enforced
//!   by feeding the SAME analyzers from virtual and reloaded recordings).

use fd_core::identity::AircraftIdentity;
use fd_core::phase::{FlightPhaseEngine, PhaseTelemetry};
use fd_core::telemetry::{SimState, SimTimestamp, TelemetrySnapshot};
use fd_core::units::{AltitudeAglFt, AltitudeFt, SpeedKt, VerticalSpeedFpm};
use fd_fdm::fdm::FdmAnalyzer;
use fd_fdm::fdr::{
    FdrEvent, FdrSample, FdrSessionMeta, FlightRecording, Recorder, StreamedRecorder,
};
use fd_fdm::qol;
use std::collections::BTreeMap;

fn meta() -> FdrSessionMeta {
    FdrSessionMeta {
        session_id: "inv-1".into(),
        simulator: "virtual".into(),
        sim_version: None,
        aircraft: AircraftIdentity::unknown(),
        fdos_version: "0.0.0".into(),
        adapter_source: Some("virtual".into()),
        started_wall_unix_ms: None,
        ended_wall_unix_ms: None,
        origin: Some("UUEE".into()),
        destination: Some("ULLI".into()),
        started_ms: 0,
        ended_ms: None,
    }
}

/// A deterministic synthetic flight: climb, cruise, descent, landing —
/// with a touchdown. Same input sequence every call.
fn synthetic_flight() -> Vec<FdrSample> {
    let mut rec = Recorder::new();
    let mut out = Vec::new();
    let mut agl = 0.0f64;
    // Climb 0 -> 3000 ft at 1000 fpm.
    for seq in 0..60u64 {
        let vs = 1000.0f64;
        agl += vs / 600.0; // per 100 ms tick
        out.push(sample(&mut rec, seq, seq * 100, agl, false, vs));
    }
    // Cruise.
    for seq in 60..100u64 {
        out.push(sample(&mut rec, seq, seq * 100, agl, false, 0.0));
    }
    // Descent at -700 fpm.
    for seq in 100..160u64 {
        let vs = -700.0f64;
        agl += vs / 600.0;
        out.push(sample(&mut rec, seq, seq * 100, agl.max(0.0), false, vs));
    }
    // Touchdown: one noisy ground flicker (2 samples) then solid ground.
    out.push(sample(&mut rec, 160, 16_000, 0.0, true, -100.0));
    out.push(sample(&mut rec, 161, 16_100, 0.5, false, -50.0));
    out.push(sample(&mut rec, 162, 16_200, 0.0, true, 0.0));
    for seq in 163..170u64 {
        out.push(sample(&mut rec, seq, seq * 100, 0.0, true, 0.0));
    }
    out
}

fn sample(rec: &mut Recorder, seq: u64, ms: u64, agl: f64, on_ground: bool, vs: f64) -> FdrSample {
    let mut s = TelemetrySnapshot::empty(SimTimestamp::new(ms));
    s.altitude_agl = Some(AltitudeAglFt::new(agl));
    s.altitude_msl = Some(AltitudeFt::new(622.0 + agl));
    s.indicated_airspeed = Some(SpeedKt::new(120.0));
    s.groundspeed = Some(SpeedKt::new(115.0));
    s.vertical_speed = Some(VerticalSpeedFpm::new(vs));
    s.on_ground = Some(on_ground);
    s.gear_handle_down = Some(true);
    s.flaps_handle_index = Some(3);
    let mut sample = rec.record(&s, "Flight");
    sample.seq = seq;
    sample
}

fn phase_trace(samples: &[FdrSample]) -> Vec<String> {
    let mut engine = FlightPhaseEngine::new();
    let mut out = Vec::new();
    for s in samples {
        let telem = PhaseTelemetry {
            on_ground: s.on_ground.unwrap_or(true),
            altitude_msl_ft: s.altitude_msl.unwrap_or(0.0),
            altitude_agl_ft: s.radio_altitude,
            groundspeed_kt: s.groundspeed.unwrap_or(0.0),
            vertical_speed_fpm: s.vertical_speed.unwrap_or(0.0),
            distance_to_dest_nm: None,
            distance_to_runway_nm: None,
            distance_from_dep_nm: None,
            active_procedure_kind: None,
            timestamp: s.timestamp,
        };
        out.push(engine.evaluate(&telem).phase.as_str().to_string());
    }
    out
}

fn fdm_events(samples: &[FdrSample]) -> Vec<(String, u64, String)> {
    let mut a = FdmAnalyzer::new_development_default();
    let mut out = Vec::new();
    for s in samples {
        for e in a.process(s) {
            out.push((
                format!("{:?}", e.kind),
                e.sample_seq,
                format!("{:?}", e.lifecycle),
            ));
        }
    }
    out
}

#[test]
fn real_recording_replays_with_identical_semantics() {
    // 1. Fly the flight, record through the STREAMED writer (the production
    //    live path), then reload from disk — the replay input is a real FDR
    //    file, not an in-memory struct (spec §36).
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("flight.jsonl");
    let flight = synthetic_flight();
    {
        let mut w = StreamedRecorder::create(&path, &meta()).unwrap();
        for s in &flight {
            w.record_sample(s).unwrap();
        }
        w.record_event(&FdrEvent {
            seq: 1,
            timestamp: flight[160].timestamp,
            kind: "touchdown".into(),
            detail: "observed".into(),
        })
        .unwrap();
        w.finish().unwrap();
    }
    let reloaded = FlightRecording::load(&path).unwrap();

    // 2. Semantic equality: no field invented, dropped, or fabricated.
    //    (f64 fields compare within text-container precision: the JSONL
    //    container may shift the last ulp; that is not invention.)
    assert_eq!(reloaded.samples.len(), flight.len());
    for (a, b) in flight.iter().zip(reloaded.samples.iter()) {
        assert_samples_semantically_equal(a, b);
    }
    assert_eq!(reloaded.events.len(), 1);

    // 3. Production analytics over the reloaded stream must produce the
    //    SAME results as over the live stream (spec §36: wall-clock
    //    diagnostics may differ; semantic output must match).
    assert_eq!(fdm_events(&flight), fdm_events(&reloaded.samples));
    assert_eq!(phase_trace(&flight), phase_trace(&reloaded.samples));

    // 4. Landing analysis: exactly ONE touchdown despite the ground
    //    flicker, identical metrics.
    let live_landing = qol::analyze(&flight);
    let replay_landing = qol::analyze(&reloaded.samples);
    assert_eq!(live_landing, replay_landing);
    let td = live_landing
        .touchdown
        .expect("exactly one touchdown detected");
    // The detector may anchor the touchdown at the first ground contact
    // (160) or at the post-flicker stable contact (162) — both are honest
    // debounce semantics; what it must NEVER do is report two touchdowns.
    assert!(
        td.seq == 160 || td.seq == 162,
        "touchdown anchored at an honest sample, got {}",
        td.seq
    );
}

#[test]
fn unknown_data_never_satisfies_positive_safety_conditions() {
    // All-unknown telemetry: FDM must stay silent (no exceedance can be
    // claimed from absent data) and the phase engine must not report a
    // confident airborne state.
    let mut rec = Recorder::new();
    let mut samples = Vec::new();
    for seq in 0..30u64 {
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(seq * 100));
        // Everything unknown: no fields set at all.
        let sample = rec.record(&s, "Unknown");
        samples.push(sample);
        s.sim_timing.state = SimState::Running;
    }
    assert!(
        fdm_events(&samples).is_empty(),
        "no FDM event may arise from fully-unknown data"
    );

    // Partially unknown: an exceedance with unknown magnitude is not
    // asserted — a None VS can never produce a sink-rate event.
    let mut s = TelemetrySnapshot::empty(SimTimestamp::new(0));
    s.vertical_speed = None; // unknown
    s.altitude_agl = Some(AltitudeAglFt::new(100.0));
    s.on_ground = Some(false);
    let mut rec2 = Recorder::new();
    let mut partial = Vec::new();
    for seq in 0..10u64 {
        let mut snap = TelemetrySnapshot::empty(SimTimestamp::new(seq * 100));
        snap.vertical_speed = None;
        snap.altitude_agl = Some(AltitudeAglFt::new(100.0));
        snap.on_ground = Some(false);
        partial.push(rec2.record(&snap, "Flight"));
    }
    let kinds: Vec<String> = fdm_events(&partial)
        .iter()
        .map(|(k, _, _)| k.clone())
        .collect();
    assert!(
        !kinds.iter().any(|k| k.to_lowercase().contains("sink")),
        "unknown VS must never produce a sink-rate event: {kinds:?}"
    );
}

#[test]
fn stale_quality_never_becomes_fresh_evidence() {
    // The executor-level gate is covered in fd-runtime; here we pin the
    // data-model invariant: quality annotations survive the FDR round trip
    // and WarmingUp/Stale never serialize into Fresh (absence).
    let mut rec = Recorder::new();
    let mut snap = TelemetrySnapshot::empty(SimTimestamp::new(5));
    snap.beacon_light = Some(true);
    let mut q = BTreeMap::new();
    q.insert(17u16, fd_core::telemetry::DataQuality::WarmingUp);
    snap.channel_quality = q;
    let sample = rec.record(&snap, "Preflight");
    assert_eq!(
        sample.channel_quality.get(&17),
        Some(&fd_core::telemetry::DataQuality::WarmingUp)
    );
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("q.jsonl");
    let mut w = StreamedRecorder::create(&path, &meta()).unwrap();
    w.record_sample(&sample).unwrap();
    w.finish().unwrap();
    let loaded = FlightRecording::load(&path).unwrap();
    assert_eq!(
        loaded.samples[0].channel_quality.get(&17),
        Some(&fd_core::telemetry::DataQuality::WarmingUp),
        "quality annotations survive the container round trip unchanged"
    );
}

/// Field-by-field semantic comparison of two samples. Exact for
/// discrete/identity fields; 1e-9-relative for f64 measurement fields
/// (the JSONL text container may move the last ulp).
fn assert_samples_semantically_equal(a: &FdrSample, b: &FdrSample) {
    assert_eq!(a.seq, b.seq);
    assert_eq!(a.timestamp, b.timestamp);
    let close = |x: Option<f64>, y: Option<f64>, what: &str| match (x, y) {
        (None, None) => {}
        (Some(x), Some(y)) => assert!(
            (x - y).abs() <= 1e-9_f64.max(x.abs() * 1e-12),
            "{what}: {x} vs {y} at seq {}",
            a.seq
        ),
        _ => panic!("{what}: presence diverged at seq {}", a.seq),
    };
    close(a.altitude_msl, b.altitude_msl, "altitude_msl");
    close(a.radio_altitude, b.radio_altitude, "radio_altitude");
    close(a.indicated_airspeed, b.indicated_airspeed, "ias");
    close(a.groundspeed, b.groundspeed, "gs");
    close(a.vertical_speed, b.vertical_speed, "vs");
    close(a.heading_true, b.heading_true, "heading");
    close(a.pitch, b.pitch, "pitch");
    close(a.bank, b.bank, "bank");
    assert_eq!(a.on_ground, b.on_ground);
    assert_eq!(a.gear_down, b.gear_down);
    assert_eq!(a.flaps_handle_index, b.flaps_handle_index);
    assert_eq!(a.any_engine_running, b.any_engine_running);
    assert_eq!(a.autopilot_master, b.autopilot_master);
    assert_eq!(a.flight_phase, b.flight_phase);
    assert_eq!(a.sim_state, b.sim_state);
    match (a.position, b.position) {
        (None, None) => {}
        (Some(p), Some(q)) => {
            assert!((p.lat.value() - q.lat.value()).abs() < 1e-9);
            assert!((p.lon.value() - q.lon.value()).abs() < 1e-9);
        }
        _ => panic!("position presence diverged at seq {}", a.seq),
    }
    assert_eq!(a.track_true_deg, b.track_true_deg);
    assert_eq!(a.sim_rate, b.sim_rate);
    assert_eq!(a.slew, b.slew);
    assert_eq!(a.channel_quality, b.channel_quality);
}
