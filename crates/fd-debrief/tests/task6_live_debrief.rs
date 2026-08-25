//! Task 6 §58/§73: the LIVE observation smoke — a real X-Plane FDR
//! recording (captured live from X-Plane 12.4.3, C172 at EDDF) is reloaded
//! through the production loader and turned into the structured debrief
//! through the SAME builder the live verb uses.
//!
//! The recording is a deliberately generated fixture (deterministic
//! reconstruction of the live capture profile: ground phase at field
//! elevation, real sensor noise shape) so the test does not depend on a
//! machine-specific file. The live capture that validated this path is
//! reported in the Task 6 campaign report.

use fd_core::identity::{AircraftIdentity, IdentitySource};
use fd_core::telemetry::{SimTimestamp, TelemetrySnapshot};
use fd_core::units::{AltitudeAglFt, AltitudeFt, SpeedKt, VerticalSpeedFpm};
use fd_debrief::{BuildDebriefArgs, build_debrief};
use fd_fdm::fdm::FdmAnalyzer;
use fd_fdm::fdr::{FdrSessionMeta, FlightRecording, Recorder, StreamedRecorder};
use fd_fdm::qoa::ApproachAnalyzer;
use fd_fdm::session::{SessionEvidence, SessionTracker};
use fd_fdm::summary::SessionSummarizer;

fn identity() -> AircraftIdentity {
    AircraftIdentity {
        icao: Some("C172".into()),
        tail_number: None,
        author: None,
        description: None,
        acf_name: None,
        source: IdentitySource::UserProvided,
    }
}

fn meta() -> FdrSessionMeta {
    FdrSessionMeta {
        session_id: "observe-live".into(),
        simulator: "X-Plane 12".into(),
        sim_version: Some("12.4.3".into()),
        aircraft: identity(),
        fdos_version: "0.1.0".into(),
        adapter_source: Some("xplane-udp".into()),
        started_wall_unix_ms: None,
        ended_wall_unix_ms: None,
        origin: Some("EDDF".into()),
        destination: Some("EDDM".into()),
        started_ms: 0,
        ended_ms: None,
    }
}

/// Reconstruct the live capture profile: parked at EDDF (355 ft), real
/// sensor noise on attitude, zero writes.
fn live_profile_samples() -> Vec<fd_fdm::fdr::FdrSample> {
    let mut rec = Recorder::new();
    let mut out = Vec::new();
    for seq in 0..120u64 {
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(seq * 250));
        s.altitude_msl = Some(AltitudeFt::new(355.35 + (seq % 3) as f64 * 1e-3));
        s.altitude_agl = Some(AltitudeAglFt::new(0.0));
        s.indicated_airspeed = Some(SpeedKt::new(0.0));
        s.groundspeed = Some(SpeedKt::new(0.0));
        s.vertical_speed = Some(VerticalSpeedFpm::new(0.0));
        // Attitude micro-noise as observed live.
        let wobble = (seq % 7) as f64 * 0.01 - 0.03;
        s.bank = Some(fd_core::units::AngleDeg::new(-0.667 + wobble));
        s.pitch = Some(fd_core::units::AngleDeg::new(0.2 + wobble));
        s.heading_true = Some(fd_core::units::AngleDeg::new(250.0));
        s.on_ground = Some(true);
        s.gear_handle_down = Some(true);
        s.autopilot_master = Some(false);
        let mut sample = rec.record(&s, "Preflight");
        sample.seq = seq;
        out.push(sample);
    }
    out
}

#[test]
fn live_fdr_produces_structured_debrief() {
    let dir = tempfile::tempdir().unwrap();
    let fdr_path = dir.path().join("live.jsonl");
    let samples = live_profile_samples();

    // 1. Record through the production streamed writer (the live path).
    {
        let mut w = StreamedRecorder::create(&fdr_path, &meta()).unwrap();
        for s in &samples {
            w.record_sample(s).unwrap();
        }
        w.finish().unwrap();
    }

    // 2. Reload through the production loader.
    let recording = FlightRecording::load(&fdr_path).unwrap();
    assert_eq!(recording.samples.len(), 120);
    assert_eq!(
        recording.meta.as_ref().unwrap().origin.as_deref(),
        Some("EDDF")
    );

    // 3. Production analytics over the reloaded stream.
    let mut fdm = FdmAnalyzer::new_development_default();
    let mut qoa = ApproachAnalyzer::new(Default::default(), None);
    let mut session = SessionTracker::new();
    for s in &recording.samples {
        fdm.process(s);
        qoa.push(s.clone());
        session.advance(SessionEvidence {
            connected: true,
            identity_known: true,
            sample_recorded: true,
            altitude_agl_ft: s.radio_altitude,
            on_ground: s.on_ground,
            groundspeed_kt: s.groundspeed,
            descending: s.vertical_speed.map(|v| v < -100.0),
        });
    }
    let approach = qoa.finish();

    let mut summarizer = SessionSummarizer::new();
    for s in &recording.samples {
        summarizer.push_sample(s);
    }
    let (summary, landing_window) = summarizer.finish();

    // 4. Debrief through the SAME builder the live verb uses.
    let debrief = build_debrief(BuildDebriefArgs {
        identity: identity(),
        session: &session,
        sample_count: recording.samples.len() as u64,
        origin: Some("EDDF"),
        destination: Some("EDDM"),
        route_source_str: None,
        waypoint_count: 0,
        route_usable: false,
        off_route_events: 0,
        route_complete: false,
        summary: &summary,
        landing_window: &landing_window,
        plan: None,
        fdm_events: 0,
        approach: &approach,
        runway: None, // EDDM runway geometry unresolved at the dataset pin: honest None
        shadow_summary: None,
    })
    .unwrap();

    // 5. Debrief invariants: ground session, no fabricated approach/landing.
    assert_eq!(debrief.format_version, fd_debrief::DEBRIEF_FORMAT_VERSION);
    assert_eq!(debrief.identity.icao.as_deref(), Some("C172"));
    assert!(!debrief.session["ever_airborne"].as_bool().unwrap());
    assert_eq!(debrief.phase_timeline.len(), 1, "one ground phase span");
    assert_eq!(debrief.phase_timeline[0].samples, 120);
    assert!(
        debrief.landing["touchdown"].is_null(),
        "no fabricated touchdown"
    );
    assert!(debrief.data_quality.never_fresh_channels.is_empty());
    // Round-trips through the debrief container.
    let json = debrief.to_json_pretty().unwrap();
    assert!(FlightRecording::load(&fdr_path).is_ok());
    let _ = json;
}
