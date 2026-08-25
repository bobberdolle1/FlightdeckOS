//! Long-run stability (Task 7 §46): several hours of deterministic
//! simulated time through the observation analytics. No sleeping — the
//! sim clock advances arithmetically.
//!
//! Proves: bounded memory structure (landing window capped, phase spans
//! capped with honest truncation), no sequence overflow assumptions, and
//! exact sample accounting at 6 h × 4 Hz.

use fd_core::telemetry::{Position, SimState, SimTimestamp, SimTiming, TelemetrySnapshot};
use fd_core::units::{
    AltitudeAglFt, AltitudeFt, AngleDeg, LatDeg, LonDeg, SpeedKt, VerticalSpeedFpm,
};
use fd_fdm::fdm::FdmAnalyzer;
use fd_fdm::fdr::Recorder;
use fd_fdm::summary::{LANDING_WINDOW_SAMPLES, MAX_PHASE_SPANS, SessionSummarizer};

fn sample(rec: &mut Recorder, ms: u64, alt_ft: f64, phase: &str) -> fd_fdm::fdr::FdrSample {
    let s = TelemetrySnapshot {
        timestamp: SimTimestamp { ms },
        position: Some(Position {
            lat: LatDeg::new(34.0 + (ms as f64) * 1e-5),
            lon: LonDeg::new(-118.0),
        }),
        altitude_msl: Some(AltitudeFt::new(alt_ft)),
        altitude_agl: Some(AltitudeAglFt::new((alt_ft - 100.0).max(0.0))),
        groundspeed: Some(SpeedKt::new(105.0)),
        indicated_airspeed: Some(SpeedKt::new(100.0)),
        vertical_speed: Some(VerticalSpeedFpm::new(0.0)),
        heading_true: Some(AngleDeg::new(250.0)),
        pitch: Some(AngleDeg::new(2.0)),
        bank: Some(AngleDeg::new(0.0)),
        on_ground: Some(alt_ft <= 100.0),
        gear_handle_down: Some(true),
        flaps_handle_index: Some(0),
        engine_combustion: Some([Some(true), None, None, None]),
        autopilot_master: Some(false),
        autothrottle_arm: None,
        beacon_light: None,
        aircraft_values: std::collections::BTreeMap::new(),
        channel_quality: std::collections::BTreeMap::new(),
        sim_timing: SimTiming {
            state: SimState::Running,
            sim_rate: Some(1.0),
            slew_active: Some(false),
        },
    };
    // ONE shared recorder: sequential seqs 0..N, preserved verbatim by
    // the summarizer (§46: no overflow/renumbering assumptions).
    rec.record(&s, phase)
}

#[test]
fn six_hours_at_4hz_stays_bounded_and_deterministic() {
    let total = 6 * 3600 * 4; // 6 h at 4 Hz = 86 400 samples
    let mut summarizer = SessionSummarizer::new();
    let mut fdm = FdmAnalyzer::new_development_default();
    let mut rec = Recorder::new();
    for i in 0..total {
        let ms = i * 250;
        let s = sample(&mut rec, ms, 5500.0, "Cruise");
        let _ = fdm.process(&s);
        summarizer.push_sample(&s);
        // u64 sequence preserved exactly — no overflow assumptions (§46).
        assert_eq!(summarizer.sample_count(), i + 1);
    }
    let (summary, landing_window) = summarizer.finish();
    assert_eq!(summary.sample_count, total);
    assert_eq!(summary.first_seq, Some(0));
    assert_eq!(summary.last_seq, Some(total - 1));
    // Bounded structures.
    assert_eq!(landing_window.len(), LANDING_WINDOW_SAMPLES);
    assert!(summary.phase_spans.len() <= MAX_PHASE_SPANS);
    assert_eq!(summary.phase_spans.len(), 1, "constant phase = one span");
    assert_eq!(
        summary.gaps.max_gap_ms, 250,
        "no gaps in the synthetic stream"
    );
    assert_eq!(summary.gaps.gaps_over_threshold, 0);
}

#[test]
fn phase_oscillation_caps_with_truncation_counter() {
    let mut summarizer = SessionSummarizer::new();
    let mut rec = Recorder::new();
    let oscillations = MAX_PHASE_SPANS as u64 + 1000;
    for i in 0..oscillations {
        let phase = if i % 2 == 0 { "A" } else { "B" };
        summarizer.push_sample(&sample(&mut rec, i * 250, 5500.0, phase));
    }
    let (summary, _) = summarizer.finish();
    assert_eq!(summary.phase_spans.len(), MAX_PHASE_SPANS);
    assert!(summary.phase_spans_truncated > 0);
    assert_eq!(summary.sample_count, oscillations);
}
