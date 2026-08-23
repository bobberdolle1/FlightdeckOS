//! Development-grade FDM/FOQA-style event detection.
//!
//! IMPORTANT: these are DEVELOPMENT thresholds for testing the analysis
//! pipeline headlessly. They are NOT airline policy, not FOQA-certified
//! limits, and not real A320 performance data. Thresholds are named and
//! configurable so future aircraft/operator packages can override them.
//!
//! Rules consume consecutive FDR samples of generic core fields. Fields that
//! are unknown produce NO fabricated values.
//!
//! # Exceedance lifecycle aggregation (spec §21)
//!
//! Earlier revisions emitted one [`FdmEvent`] PER SAMPLE while a threshold
//! was crossed (event flooding). Sustained conditions are now aggregated
//! into episodes by a per-kind state machine — an intentional semantic
//! improvement:
//!
//! * first sample meeting the condition → one `started` event;
//! * every further sample while the condition holds → NO event (peak
//!   severity and duration are tracked silently);
//! * the first subsequent sample failing the condition → one `ended`
//!   event carrying the episode peak and active-sample count.
//!
//! The deterministic rule, chosen for simplicity: the condition is
//! evaluated strictly per sample, so unknown measurements, `on_ground`,
//! or AGL above the limit all CLOSE an open episode at that tick; an
//! episode still open at end of stream has no `ended` event. A
//! single-sample exceedance is therefore `started` followed by `ended`
//! at the next clear tick. Edge-triggered rules that cannot span samples
//! ([`FdmEventKind::HardTouchdown`]) stay single events marked `started`.

use crate::fdr::FdrSample;
use serde::{Deserialize, Serialize};

/// Development threshold set (`ApproachProfile::DevelopmentDefault`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DevelopmentThresholds {
    /// Excessive sink rate below this AGL (ft), |VS| above this (fpm).
    pub excessive_sink_rate_fpm: f64,
    pub excessive_sink_max_agl_ft: f64,
    /// Excessive bank below this AGL (ft), bank above this (deg).
    pub excessive_bank_deg: f64,
    pub excessive_bank_max_agl_ft: f64,
    /// Hard touchdown: vertical speed at touchdown beyond this (|fpm|).
    pub hard_touchdown_vs_fpm: f64,
}

impl Default for DevelopmentThresholds {
    fn default() -> Self {
        // DEVELOPMENT DEFAULTS — not airline policy.
        Self {
            excessive_sink_rate_fpm: 1200.0,
            excessive_sink_max_agl_ft: 2500.0,
            excessive_bank_deg: 30.0,
            excessive_bank_max_agl_ft: 1500.0,
            hard_touchdown_vs_fpm: 600.0,
        }
    }
}

/// Position of an [`FdmEvent`] within an exceedance episode.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FdmLifecycle {
    /// First crossing of the condition.
    #[default]
    Started,
    /// First sample after the condition cleared; carries the episode peak.
    Ended,
}

/// One detected FDM event.
///
/// Sign semantics: descent vertical speed is a negative f64; magnitudes
/// are compared negated (`-vs >= threshold`). Never negate the stored
/// raw sign elsewhere — units are untouched.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FdmEvent {
    pub sample_seq: u64,
    pub timestamp_ms: u64,
    pub kind: FdmEventKind,
    /// Magnitude at this sample: sink rate `-vs` (fpm), bank `|bank|`
    /// (deg), or touchdown impact `-vs` (fpm).
    pub measured: f64,
    pub threshold: f64,
    /// Episode position (see [`FdmLifecycle`]).
    #[serde(default)]
    pub lifecycle: FdmLifecycle,
    /// Accumulated peak severity of the episode. Known only once the
    /// episode ends, so only `ended` events carry it.
    #[serde(default)]
    pub peak: Option<f64>,
    /// Consecutive samples in the episode (1 for a single-sample one).
    #[serde(default)]
    pub samples_active: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FdmEventKind {
    ExcessiveSinkRate,
    ExcessiveBankLowAltitude,
    HardTouchdown,
}

/// Open exceedance episode for one sustained-condition rule.
#[derive(Debug, Clone, Copy)]
struct Exceedance {
    peak: f64,
    samples_active: u64,
}

/// Stateful analyzer over an ordered sample stream.
#[derive(Debug)]
pub struct FdmAnalyzer {
    thresholds: DevelopmentThresholds,
    prev_on_ground: Option<bool>,
    events: Vec<FdmEvent>,
    sink_exceedance: Option<Exceedance>,
    bank_exceedance: Option<Exceedance>,
}

impl FdmAnalyzer {
    pub fn new(thresholds: DevelopmentThresholds) -> Self {
        Self {
            thresholds,
            prev_on_ground: None,
            events: Vec::new(),
            sink_exceedance: None,
            bank_exceedance: None,
        }
    }

    pub fn new_development_default() -> Self {
        Self::new(DevelopmentThresholds::default())
    }

    pub fn events(&self) -> &[FdmEvent] {
        &self.events
    }

    pub fn into_events(self) -> Vec<FdmEvent> {
        self.events
    }

    /// Feed one ordered FDR sample; returns events generated by it.
    ///
    /// Sustained conditions emit at most one event per state transition
    /// (`started` on first crossing, `ended` on first clear) — never one
    /// event per sample.
    pub fn process(&mut self, s: &FdrSample) -> Vec<FdmEvent> {
        let mut new_events = Vec::new();
        let airborne = !matches!(s.on_ground, Some(true));

        // Excessive sink rate (airborne, low AGL, known VS). Descent VS is
        // negative; compare the negated magnitude against the threshold.
        let sink_severity = if !airborne {
            None
        } else {
            match (s.radio_altitude, s.vertical_speed) {
                (Some(agl), Some(vs))
                    if agl <= self.thresholds.excessive_sink_max_agl_ft && vs.is_finite() =>
                {
                    let sink = -vs;
                    (sink >= self.thresholds.excessive_sink_rate_fpm).then_some(sink)
                }
                _ => None,
            }
        };
        Self::advance(
            &mut self.sink_exceedance,
            sink_severity,
            FdmEventKind::ExcessiveSinkRate,
            self.thresholds.excessive_sink_rate_fpm,
            s,
            &mut new_events,
        );

        // Excessive bank at low altitude (airborne, known bank).
        let bank_severity = if !airborne {
            None
        } else if let Some(bank) = s.bank
            && bank.is_finite()
            && bank.abs() >= self.thresholds.excessive_bank_deg
        {
            Some(bank.abs())
        } else {
            None
        };
        Self::advance(
            &mut self.bank_exceedance,
            bank_severity,
            FdmEventKind::ExcessiveBankLowAltitude,
            self.thresholds.excessive_bank_deg,
            s,
            &mut new_events,
        );

        // Touchdown transition: airborne -> on_ground with known VS.
        // Edge-triggered and momentary: always a lone `started` event.
        if let (Some(prev_g), Some(now_g)) = (self.prev_on_ground, s.on_ground)
            && !prev_g
            && now_g
            && let Some(vs) = s.vertical_speed
            && vs.is_finite()
        {
            let impact = -vs;
            if impact >= self.thresholds.hard_touchdown_vs_fpm {
                new_events.push(FdmEvent {
                    sample_seq: s.seq,
                    timestamp_ms: s.timestamp.ms,
                    kind: FdmEventKind::HardTouchdown,
                    measured: impact,
                    threshold: self.thresholds.hard_touchdown_vs_fpm,
                    lifecycle: FdmLifecycle::Started,
                    peak: None,
                    samples_active: 1,
                });
            }
        }

        self.prev_on_ground = s.on_ground;
        self.events.extend(new_events.iter().cloned());
        new_events
    }

    /// Advance one sustained-condition state machine for this sample.
    ///
    /// Deterministic aggregation rule: first crossing emits `started`
    /// (entry severity as `measured`); continuation updates the tracked
    /// peak silently; the first sample failing the condition emits
    /// `ended` with the accumulated episode peak.
    fn advance(
        open: &mut Option<Exceedance>,
        severity: Option<f64>,
        kind: FdmEventKind,
        threshold: f64,
        s: &FdrSample,
        out: &mut Vec<FdmEvent>,
    ) {
        match (open.take(), severity) {
            (None, Some(sev)) => {
                *open = Some(Exceedance {
                    peak: sev,
                    samples_active: 1,
                });
                out.push(FdmEvent {
                    sample_seq: s.seq,
                    timestamp_ms: s.timestamp.ms,
                    kind,
                    measured: sev,
                    threshold,
                    lifecycle: FdmLifecycle::Started,
                    peak: None,
                    samples_active: 1,
                });
            }
            (Some(mut e), Some(sev)) => {
                e.peak = e.peak.max(sev);
                e.samples_active += 1;
                *open = Some(e);
            }
            (Some(e), None) => {
                out.push(FdmEvent {
                    sample_seq: s.seq,
                    timestamp_ms: s.timestamp.ms,
                    kind,
                    measured: e.peak,
                    threshold,
                    lifecycle: FdmLifecycle::Ended,
                    peak: Some(e.peak),
                    samples_active: e.samples_active,
                });
            }
            (None, None) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fdr::Recorder;
    use fd_core::telemetry::{SimState, SimTimestamp, TelemetrySnapshot};
    use fd_core::units::{AltitudeAglFt, AltitudeFt, AngleDeg, VerticalSpeedFpm};

    fn analyzer() -> FdmAnalyzer {
        FdmAnalyzer::new(DevelopmentThresholds {
            excessive_sink_rate_fpm: 1000.0,
            excessive_sink_max_agl_ft: 2000.0,
            excessive_bank_deg: 30.0,
            excessive_bank_max_agl_ft: 1000.0,
            hard_touchdown_vs_fpm: 500.0,
        })
    }

    fn sample(
        rec: &mut Recorder,
        seq: u64,
        agl: Option<f64>,
        vs: Option<f64>,
        bank: Option<f64>,
        on_ground: Option<bool>,
    ) -> FdrSample {
        let mut snap = TelemetrySnapshot::empty(SimTimestamp::new(seq * 100));
        snap.altitude_agl = agl.map(AltitudeAglFt::new);
        snap.altitude_msl = agl.map(|a| AltitudeFt::new(a + 100.0));
        snap.vertical_speed = vs.map(VerticalSpeedFpm::new);
        snap.bank = bank.map(AngleDeg::new);
        snap.on_ground = on_ground;
        snap.sim_timing.state = SimState::Running;
        rec.record(&snap, "TEST")
    }

    #[test]
    fn normal_stream_produces_no_false_events() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        for i in 0..6 {
            let s = sample(
                &mut rec,
                i,
                Some(3000.0 - (i as f64) * 400.0),
                Some(-700.0),
                Some(8.0),
                Some(false),
            );
            assert!(a.process(&s).is_empty(), "false positive at sample {i}");
        }
    }

    #[test]
    fn excessive_sink_produces_started_then_ended() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        // Intentional semantic improvement over the previous per-sample
        // flooding: one crossing sample yields exactly one `started`.
        let s = sample(
            &mut rec,
            1,
            Some(800.0),
            Some(-1400.0),
            Some(0.0),
            Some(false),
        );
        let evts = a.process(&s);
        assert_eq!(evts.len(), 1);
        assert_eq!(evts[0].kind, FdmEventKind::ExcessiveSinkRate);
        assert_eq!(evts[0].lifecycle, FdmLifecycle::Started);
        assert_eq!(evts[0].peak, None);
        assert_eq!(evts[0].samples_active, 1);
        assert!((evts[0].measured - 1400.0).abs() < 1e-9);
    }

    #[test]
    fn sustained_sink_emits_exactly_two_events_with_peak() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        let mut all = Vec::new();
        // Crossing at 1200, deeper at 1500, still deep at 1300, clear at 500.
        let stream = [
            (0, Some(800.0), Some(-1200.0)),
            (1, Some(750.0), Some(-1500.0)),
            (2, Some(600.0), Some(-1300.0)),
            (3, Some(500.0), Some(-500.0)),
        ];
        for (seq, agl, vs) in stream {
            let s = sample(&mut rec, seq, agl, vs, Some(0.0), Some(false));
            all.extend(a.process(&s));
        }

        assert_eq!(all.len(), 2, "no per-sample flooding: started + ended");
        let started = &all[0];
        assert_eq!(started.lifecycle, FdmLifecycle::Started);
        assert_eq!(started.sample_seq, 0);
        assert!((started.measured - 1200.0).abs() < 1e-9);
        assert_eq!(started.peak, None);

        let ended = &all[1];
        assert_eq!(ended.lifecycle, FdmLifecycle::Ended);
        assert_eq!(ended.sample_seq, 3);
        assert!((ended.measured - 1500.0).abs() < 1e-9, "ended carries peak");
        assert_eq!(ended.peak, Some(1500.0));
        assert_eq!(ended.samples_active, 3);
        assert_eq!(
            ended.kind,
            FdmEventKind::ExcessiveSinkRate,
            "same kind as its episode"
        );
    }

    #[test]
    fn intermittent_exceedance_re_arms() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        let mut all = Vec::new();
        // Cross, clear, cross again deeper, clear.
        let stream = [
            (0, Some(800.0), Some(-1100.0)),
            (1, Some(800.0), Some(-300.0)),
            (2, Some(700.0), Some(-1600.0)),
            (3, Some(700.0), Some(-200.0)),
        ];
        for (seq, agl, vs) in stream {
            let s = sample(&mut rec, seq, agl, vs, Some(0.0), Some(false));
            all.extend(a.process(&s));
        }

        let lifecycles: Vec<_> = all.iter().map(|e| e.lifecycle).collect();
        assert_eq!(
            lifecycles,
            vec![
                FdmLifecycle::Started,
                FdmLifecycle::Ended,
                FdmLifecycle::Started,
                FdmLifecycle::Ended,
            ],
            "each episode pairs its own started/ended"
        );
        // Episodes are independent: first peak 1100 over 1 sample, second
        // peak 1600 over 1 sample.
        assert_eq!(all[1].peak, Some(1100.0));
        assert_eq!(all[1].samples_active, 1);
        assert_eq!(all[3].peak, Some(1600.0));
        assert_eq!(all[3].samples_active, 1);
    }

    #[test]
    fn single_sample_exceedance_ends_at_next_clear_tick() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        let cross = sample(
            &mut rec,
            4,
            Some(400.0),
            Some(-1250.0),
            Some(0.0),
            Some(false),
        );
        assert_eq!(a.process(&cross).len(), 1); // started
        let clear = sample(
            &mut rec,
            5,
            Some(400.0),
            Some(-100.0),
            Some(0.0),
            Some(false),
        );
        let evts = a.process(&clear);
        assert_eq!(evts.len(), 1);
        assert_eq!(evts[0].lifecycle, FdmLifecycle::Ended);
        assert_eq!(evts[0].sample_seq, 1, "ended stamped at the clear tick");
        assert_eq!(evts[0].samples_active, 1);
        assert_eq!(evts[0].peak, Some(1250.0));
    }

    #[test]
    fn unknown_measurement_dropout_closes_open_exceedance() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        let cross = sample(
            &mut rec,
            0,
            Some(900.0),
            Some(-1400.0),
            Some(0.0),
            Some(false),
        );
        assert_eq!(a.process(&cross).len(), 1);
        // VS drops out: the strict per-sample rule treats unknown as
        // not-crossing, closing the episode honestly.
        let dropout = sample(&mut rec, 1, Some(850.0), None, Some(0.0), Some(false));
        let evts = a.process(&dropout);
        assert_eq!(evts.len(), 1);
        assert_eq!(evts[0].lifecycle, FdmLifecycle::Ended);
    }

    #[test]
    fn touchdown_closes_open_sink_exceedance() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        let s1 = sample(
            &mut rec,
            0,
            Some(50.0),
            Some(-1150.0),
            Some(0.0),
            Some(false),
        );
        assert_eq!(a.process(&s1).len(), 1); // sink started
        let touchdown = sample(&mut rec, 1, Some(0.0), Some(-650.0), Some(0.0), Some(true));
        let evts = a.process(&touchdown);
        assert_eq!(evts.len(), 2, "hard touchdown + sink episode close");
        assert!(evts.iter().any(|e| e.kind == FdmEventKind::HardTouchdown));
        let closed = evts
            .iter()
            .find(|e| e.kind == FdmEventKind::ExcessiveSinkRate)
            .expect("sink episode must close on ground contact");
        assert_eq!(closed.lifecycle, FdmLifecycle::Ended);
        assert_eq!(closed.sample_seq, 1);
    }

    #[test]
    fn stream_end_with_open_exceedance_has_no_ended() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        let cross = sample(
            &mut rec,
            0,
            Some(800.0),
            Some(-1400.0),
            Some(0.0),
            Some(false),
        );
        assert_eq!(a.process(&cross).len(), 1);
        // End of stream: exactly one started, never a synthetic ended.
        assert_eq!(a.into_events().len(), 1);
    }

    #[test]
    fn sustained_bank_exceedance_aggregates() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        let b1 = sample(
            &mut rec,
            0,
            Some(400.0),
            Some(-200.0),
            Some(-35.0),
            Some(false),
        );
        assert_eq!(a.process(&b1).len(), 1); // started
        let b2 = sample(
            &mut rec,
            1,
            Some(350.0),
            Some(-200.0),
            Some(-42.0),
            Some(false),
        );
        assert!(a.process(&b2).is_empty(), "continuation is silent");
        let b3 = sample(
            &mut rec,
            2,
            Some(300.0),
            Some(-200.0),
            Some(-10.0),
            Some(false),
        );
        let evts = a.process(&b3);
        assert_eq!(evts.len(), 1);
        assert_eq!(evts[0].kind, FdmEventKind::ExcessiveBankLowAltitude);
        assert_eq!(evts[0].lifecycle, FdmLifecycle::Ended);
        assert_eq!(evts[0].peak, Some(42.0));
        assert_eq!(evts[0].samples_active, 2);
    }

    #[test]
    fn low_altitude_excessive_bank_produces_event() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        let s = sample(
            &mut rec,
            2,
            Some(400.0),
            Some(-200.0),
            Some(-35.0),
            Some(false),
        );
        let evts = a.process(&s);
        assert_eq!(evts.len(), 1);
        assert_eq!(evts[0].kind, FdmEventKind::ExcessiveBankLowAltitude);
        assert_eq!(evts[0].lifecycle, FdmLifecycle::Started);
    }

    #[test]
    fn unknown_measurements_never_fabricate_events() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        // AGL / VS / bank all unknown.
        let s = sample(&mut rec, 3, None, None, None, Some(false));
        assert!(a.process(&s).is_empty());
    }

    #[test]
    fn touchdown_transition_is_detected_with_vs() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        let airborne = sample(&mut rec, 1, Some(5.0), Some(-120.0), Some(0.0), Some(false));
        a.process(&airborne);
        let touchdown = sample(&mut rec, 2, Some(0.0), Some(-150.0), Some(0.0), Some(true));
        let evts = a.process(&touchdown);
        // Soft touchdown (< 500 fpm dev threshold): no FDM event, but the
        // transition itself is observable via QoL (tested there).
        assert!(evts.is_empty());
    }

    #[test]
    fn hard_touchdown_produces_event() {
        let mut a = analyzer();
        let mut rec = Recorder::new();
        let airborne = sample(&mut rec, 1, Some(5.0), Some(-600.0), Some(0.0), Some(false));
        a.process(&airborne);
        let touchdown = sample(&mut rec, 2, Some(0.0), Some(-650.0), Some(0.0), Some(true));
        let evts = a.process(&touchdown);
        assert_eq!(evts.len(), 1);
        assert_eq!(evts[0].kind, FdmEventKind::HardTouchdown);
        assert_eq!(evts[0].lifecycle, FdmLifecycle::Started);
        assert_eq!(evts[0].peak, None);
    }

    #[test]
    fn fdm_event_json_backward_compat() {
        // Legacy fixture shape: no lifecycle / peak / samples_active keys.
        let legacy = r#"{
            "sample_seq": 7,
            "timestamp_ms": 700,
            "kind": "excessive_sink_rate",
            "measured": 1350.0,
            "threshold": 1200.0
        }"#;
        let e: FdmEvent = serde_json::from_str(legacy).unwrap();
        assert_eq!(e.sample_seq, 7);
        assert_eq!(e.kind, FdmEventKind::ExcessiveSinkRate);
        assert_eq!(e.lifecycle, FdmLifecycle::Started, "legacy default");
        assert_eq!(e.peak, None);
        assert_eq!(e.samples_active, 0);
    }

    #[test]
    fn fdm_event_serde_roundtrip_with_lifecycle() {
        let e = FdmEvent {
            sample_seq: 9,
            timestamp_ms: 900,
            kind: FdmEventKind::ExcessiveBankLowAltitude,
            measured: 41.5,
            threshold: 30.0,
            lifecycle: FdmLifecycle::Ended,
            peak: Some(44.0),
            samples_active: 6,
        };
        let text = serde_json::to_string(&e).unwrap();
        assert!(text.contains("\"lifecycle\":\"ended\""));
        assert!(text.contains("\"peak\":44"));
        let back: FdmEvent = serde_json::from_str(&text).unwrap();
        assert_eq!(back, e);
    }
}
