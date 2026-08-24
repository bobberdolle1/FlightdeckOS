//! Quality of Approach V2: MEASUREMENTS over the approach segment with
//! evidence-carrying stabilization gates.
//!
//! Deliberately not a single score (Task 3 §17). Metrics with unknown input
//! are reported as `None` — never zero, never fabricated.
//!
//! # Gate semantics (contract C10)
//!
//! Each stabilization gate produces a [`GateEvidence`] record (what was known
//! at the crossing sample) plus a [`GateClassification`]. Classification is
//! FAIL-CLOSED:
//!
//! * any status that is *known false* → `Unstable` (decisive negative
//!   evidence outweighs unrelated unknowns);
//! * otherwise any *unknown* (`None`) status → `Indeterminate`
//!   (**never** `Stable`);
//! * otherwise (all five statuses known true) → `Stable`.
//!
//! "Stabilized" here means a DEVELOPMENT definition. It is not an airline
//! stabilized-approach gate definition.
//!
//! # Go-around detection (contract C10)
//!
//! A go-around requires ARRESTED DESCENT followed by SUSTAINED positive
//! climb: VS ≥ [`DEV_GO_AROUND_MIN_CLIMB_FPM`] for
//! [`DEV_GO_AROUND_SUSTAIN_SAMPLES`] consecutive airborne samples, after the
//! aircraft descended below [`DEV_GO_AROUND_MAX_AGL_FT`] while airborne.
//! Single-sample VS spikes are ignored by construction.

use crate::fdr::FdrSample;
use fd_core::units::{AngleDeg, SpeedKt, VerticalSpeedFpm};
use serde::{Deserialize, Serialize};

// DEVELOPMENT DEFAULTS — not airline policy. Named so future aircraft or
// operator packages can override them deliberately.
/// Development bank limit while stabilized below the gates (deg, magnitude).
pub const DEV_MAX_BANK_DEG: f64 = 10.0;
/// Development IAS deviation limit vs the approach reference speed (kt).
pub const DEV_MAX_IAS_DEVIATION_KT: f64 = 10.0;
/// Development landing-flap setting: handle index at or above this counts as
/// configured.
pub const DEV_LANDING_FLAPS_INDEX: u8 = 3;
/// Minimum sustained climb rate confirming a go-around (fpm).
pub const DEV_GO_AROUND_MIN_CLIMB_FPM: f64 = 100.0;
/// Consecutive climbing samples required before a go-around is confirmed.
pub const DEV_GO_AROUND_SUSTAIN_SAMPLES: usize = 3;
/// AGL below which descending flight counts as go-around context (ft).
pub const DEV_GO_AROUND_MAX_AGL_FT: f64 = 200.0;
/// AGL where gear configuration is sampled (ft).
const GEAR_CHECK_GATE_FT: f64 = 500.0;

/// Development stabilization criteria assessed at each gate.
///
/// Sign conventions: VS is signed (negative = descent); sink comparisons use
/// the magnitude `-vs`. Bank comparisons use `|bank|`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StabilizationCriteria {
    /// |VS| at or below this counts as a stable sample (fpm).
    pub max_stable_sink_fpm: VerticalSpeedFpm,
    /// AGL gate heights where stabilization is assessed (ft).
    pub gates_ft: [f64; 2], // [1000, 500]
    /// |bank| at or below this counts as stable (deg).
    pub max_bank_deg: AngleDeg,
    /// |IAS − reference| at or below this counts as stable (kt). Only
    /// assessable when an approach reference speed is supplied.
    pub max_ias_deviation_kt: SpeedKt,
    /// Landing flap configuration: handle index at or above this counts as set.
    pub landing_flaps_index: u8,
}

impl Default for StabilizationCriteria {
    fn default() -> Self {
        // DEVELOPMENT DEFAULTS — not airline policy.
        Self {
            max_stable_sink_fpm: VerticalSpeedFpm::new(1000.0),
            gates_ft: [1000.0, 500.0],
            max_bank_deg: AngleDeg::new(DEV_MAX_BANK_DEG),
            max_ias_deviation_kt: SpeedKt::new(DEV_MAX_IAS_DEVIATION_KT),
            landing_flaps_index: DEV_LANDING_FLAPS_INDEX,
        }
    }
}

/// What was known about each stabilization criterion at one gate crossing.
///
/// Every status is `Option<bool>`: `None` = unknown input (fail-closed),
/// `Some(false)` = known violation, `Some(true)` = known compliant.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GateEvidence {
    /// Gate height crossed (ft AGL).
    pub gate_ft: f64,
    pub sample_seq: u64,
    pub timestamp_ms: u64,
    /// IAS within deviation limits of the reference (None = no reference /
    /// IAS unknown).
    pub ias_status: Option<bool>,
    /// Sink magnitude within limits (None = VS unknown).
    pub vs_status: Option<bool>,
    /// Bank magnitude within limits (None = bank unknown).
    pub bank_status: Option<bool>,
    /// Gear down-and-locked indication (None = gear state unknown).
    pub gear_down: Option<bool>,
    /// Flaps at or beyond the configured landing setting (None = flap
    /// position unknown).
    pub flaps_set: Option<bool>,
}

/// Fail-closed classification of one stabilization gate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GateClassification {
    /// Every criterion known and compliant.
    Stable,
    /// At least one criterion known violated.
    Unstable,
    /// No known violation, but at least one criterion unknown.
    /// NEVER reported as `Stable`.
    Indeterminate,
}

impl GateClassification {
    /// Classify gate evidence:
    /// any known-false status → `Unstable`; else any unknown status →
    /// `Indeterminate`; else `Stable`.
    pub fn classify(e: &GateEvidence) -> Self {
        let statuses = [
            e.ias_status,
            e.vs_status,
            e.bank_status,
            e.gear_down,
            e.flaps_set,
        ];
        if statuses.contains(&Some(false)) {
            Self::Unstable
        } else if statuses.contains(&None) {
            Self::Indeterminate
        } else {
            Self::Stable
        }
    }
}

/// Evidence that an approach ended in a go-around.
///
/// Emitted only when the climb was SUSTAINED
/// ([`DEV_GO_AROUND_SUSTAIN_SAMPLES`] consecutive climbing samples) — a
/// single-sample VS spike never produces evidence.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GoAroundEvidence {
    /// First sample of the sustained climb (go-around initiation).
    pub start_seq: u64,
    /// Lowest AGL reached during the low-altitude descent context (ft).
    pub min_agl_ft: f64,
    /// Sample at which the climb-sustain requirement was confirmed.
    pub climb_confirmed_seq: u64,
}

/// Approach measurements.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ApproachReport {
    /// Gate evidence ordered like `StabilizationCriteria::gates_ft`;
    /// `None` entries = gate never crossed / not assessable.
    pub gates: Vec<Option<GateEvidence>>,
    /// Classification parallel to `gates` (same ordering, same length).
    pub classifications: Vec<Option<GateClassification>>,
    pub max_ias_deviation_kt: Option<f64>,
    pub rms_ias_deviation_kt: Option<f64>,
    /// Maximum sink MAGNITUDE `-vs` while airborne (fpm); descent VS is
    /// negative, this field reports the positive sink rate.
    pub max_sink_rate_fpm: Option<f64>,
    /// Maximum bank MAGNITUDE (deg).
    pub max_bank_deg: Option<f64>,
    pub gear_configured_at_500ft: Option<bool>,
    pub approach_duration_s: Option<f64>,
    /// Go-around evidence; `None` = no sustained-climb go-around detected.
    pub go_around: Option<GoAroundEvidence>,
}

/// Analysis window for one approach: samples between entering the approach
/// phase and touchdown/end.
#[derive(Debug, Default)]
pub struct ApproachAnalyzer {
    criteria: StabilizationCriteria,
    /// Reference speed used for IAS deviation when known (kt); None = unknown.
    reference_ias_kt: Option<f64>,
    samples: Vec<FdrSample>,
    started_at_ms: Option<u64>,
    finished: bool,
}

impl ApproachAnalyzer {
    pub fn new(criteria: StabilizationCriteria, reference_ias_kt: Option<f64>) -> Self {
        Self {
            criteria,
            reference_ias_kt,
            samples: Vec::new(),
            started_at_ms: None,
            finished: false,
        }
    }

    /// Begin collecting at the first approach sample.
    pub fn begin(&mut self, first_sample_ts_ms: u64) {
        if self.started_at_ms.is_none() {
            self.started_at_ms = Some(first_sample_ts_ms);
        }
    }

    pub fn push(&mut self, s: FdrSample) {
        self.begin(s.timestamp.ms);
        self.samples.push(s);
    }

    /// One-shot analysis over an existing sample slice (streaming `push`
    /// remains available for live sessions).
    pub fn with_samples(mut self, samples: &[FdrSample]) -> Self {
        for s in samples {
            self.push(s.clone());
        }
        self
    }

    /// Finish collection and compute all supported metrics.
    pub fn finish(mut self) -> ApproachReport {
        self.finished = true;
        let mut report = ApproachReport::default();

        // Gates: find first crossing of each gate height (descending through).
        for &gate in &self.criteria.gates_ft {
            let evidence = self.gate_evidence(gate);
            let classification = evidence.as_ref().map(GateClassification::classify);
            report.gates.push(evidence);
            report.classifications.push(classification);
        }

        // IAS deviation vs reference (only when a reference is known).
        if let Some(reference) = self.reference_ias_kt {
            let mut max_dev: f64 = 0.0;
            let mut sum_sq = 0.0;
            let mut n = 0usize;
            for s in &self.samples {
                if let Some(ias) = s.indicated_airspeed.filter(|v| v.is_finite()) {
                    let dev = (ias - reference).abs();
                    max_dev = max_dev.max(dev);
                    sum_sq += dev * dev;
                    n += 1;
                }
            }
            if n > 0 {
                report.max_ias_deviation_kt = Some(max_dev);
                report.rms_ias_deviation_kt = Some((sum_sq / n as f64).sqrt());
            }
        }

        // Max sink rate (|VS| max while airborne); reported as positive
        // sink magnitude per the field name.
        let mut max_sink: Option<f64> = None;
        let mut max_bank: Option<f64> = None;
        for s in &self.samples {
            if !matches!(s.on_ground, Some(true)) {
                if let Some(vs) = s.vertical_speed.filter(|v| v.is_finite()) {
                    let sink = -vs;
                    max_sink = Some(max_sink.unwrap_or(0.0).max(sink));
                }
                if let Some(bank) = s.bank.filter(|v| v.is_finite()) {
                    max_bank = Some(max_bank.unwrap_or(0.0).max(bank.abs()));
                }
            }
        }
        report.max_sink_rate_fpm = max_sink;
        report.max_bank_deg = max_bank;

        // Gear configured near the 500 ft gate (first sample at/below it).
        report.gear_configured_at_500ft = self
            .samples
            .iter()
            .find(|s| {
                s.radio_altitude
                    .map(|a| a <= GEAR_CHECK_GATE_FT)
                    .unwrap_or(false)
            })
            .and_then(|s| s.gear_down);

        // Duration.
        if let (Some(t0), Some(t1)) = (
            self.started_at_ms,
            self.samples.last().map(|s| s.timestamp.ms),
        ) {
            report.approach_duration_s = Some((t1.saturating_sub(t0)) as f64 / 1000.0);
        }

        report.go_around = detect_go_around(&self.samples);
        report
    }

    fn gate_evidence(&self, gate_ft: f64) -> Option<GateEvidence> {
        // First descending crossing of the gate.
        let mut prev_agl: Option<f64> = None;
        for s in &self.samples {
            let agl = s.radio_altitude?;
            if let Some(prev) = prev_agl
                && prev > gate_ft
                && agl <= gate_ft
            {
                // Assess stability criteria at this sample. Unknown inputs
                // stay None (fail-closed), never false and never fabricated.
                let vs_status = s
                    .vertical_speed
                    .filter(|v| v.is_finite())
                    .map(|vs| -vs <= self.criteria.max_stable_sink_fpm.value());
                let ias_status = match (self.reference_ias_kt, s.indicated_airspeed) {
                    (Some(reference), Some(ias)) if ias.is_finite() => {
                        Some((ias - reference).abs() <= self.criteria.max_ias_deviation_kt.value())
                    }
                    _ => None,
                };
                let bank_status = s
                    .bank
                    .filter(|v| v.is_finite())
                    .map(|bank| bank.abs() <= self.criteria.max_bank_deg.value());
                let gear_down = s.gear_down;
                let flaps_set = s
                    .flaps_handle_index
                    .map(|idx| idx >= self.criteria.landing_flaps_index);
                return Some(GateEvidence {
                    gate_ft,
                    sample_seq: s.seq,
                    timestamp_ms: s.timestamp.ms,
                    ias_status,
                    vs_status,
                    bank_status,
                    gear_down,
                    flaps_set,
                });
            }
            prev_agl = Some(agl);
        }
        None // gate never crossed inside the window -> not assessable
    }
}

/// Detect a go-around: descent arrested below [`DEV_GO_AROUND_MAX_AGL_FT`]
/// while airborne, then VS ≥ [`DEV_GO_AROUND_MIN_CLIMB_FPM`] sustained for
/// [`DEV_GO_AROUND_SUSTAIN_SAMPLES`] consecutive airborne samples.
///
/// Deterministic fail-closed details:
/// * ground contact before confirmation resets all state (that was a
///   landing/bounce, not a go-around);
/// * an unknown or non-climbing VS breaks the climb run (the sustain must be
///   CONSECUTIVE samples);
/// * `min_agl_ft` tracks the lowest finite AGL observed from the moment the
///   descent context was armed until confirmation.
fn detect_go_around(samples: &[FdrSample]) -> Option<GoAroundEvidence> {
    let mut armed = false;
    let mut min_agl: Option<f64> = None;
    let mut climb_start_seq: Option<u64> = None;
    let mut climb_run = 0usize;

    for s in samples {
        let airborne = !matches!(s.on_ground, Some(true));
        if !airborne {
            // Ground contact before confirmation: reset everything.
            armed = false;
            min_agl = None;
            climb_run = 0;
            climb_start_seq = None;
            continue;
        }
        let agl = s.radio_altitude.filter(|v| v.is_finite());
        let vs = s.vertical_speed.filter(|v| v.is_finite());

        // Arm on descending flight below the dev threshold.
        if !armed {
            if let (Some(agl), Some(vs)) = (agl, vs)
                && agl <= DEV_GO_AROUND_MAX_AGL_FT
                && vs < 0.0
            {
                armed = true;
                min_agl = Some(agl);
            }
            continue;
        }

        // While armed, track the lowest AGL reached.
        if let Some(agl) = agl {
            min_agl = Some(min_agl.map_or(agl, |m: f64| m.min(agl)));
        }

        // Climb run: consecutive airborne samples at/above the dev climb rate.
        match vs {
            Some(vs) if vs >= DEV_GO_AROUND_MIN_CLIMB_FPM => {
                if climb_run == 0 {
                    climb_start_seq = Some(s.seq);
                }
                climb_run += 1;
                if climb_run >= DEV_GO_AROUND_SUSTAIN_SAMPLES {
                    return Some(GoAroundEvidence {
                        start_seq: climb_start_seq?,
                        min_agl_ft: min_agl?,
                        climb_confirmed_seq: s.seq,
                    });
                }
            }
            _ => {
                // Not climbing (or climb rate unknown): sustain broken.
                climb_run = 0;
                climb_start_seq = None;
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fdr::Recorder;
    use fd_core::telemetry::{SimState, SimTimestamp, TelemetrySnapshot};
    use fd_core::units::AltitudeAglFt;

    /// Build one airborne sample via the Recorder with an explicit seq
    /// (each call uses a throwaway recorder, so seq is overridden).
    fn sample_at(i: u64, agl: f64, vs: f64, f: impl FnOnce(&mut TelemetrySnapshot)) -> FdrSample {
        let mut rec = Recorder::new();
        let mut snap = TelemetrySnapshot::empty(SimTimestamp::new(i * 100));
        snap.altitude_agl = Some(AltitudeAglFt::new(agl));
        snap.vertical_speed = Some(VerticalSpeedFpm::new(vs));
        snap.on_ground = Some(false);
        snap.sim_timing.state = SimState::Running;
        f(&mut snap);
        FdrSample {
            seq: i,
            ..rec.record(&snap, "APPROACH")
        }
    }

    #[test]
    fn gate_with_unknown_gear_is_indeterminate_never_stable() {
        // IAS + VS + bank compliant, but gear and flaps unknown.
        let report = ApproachAnalyzer::new(StabilizationCriteria::default(), Some(160.0))
            .with_samples(&[
                sample_at(0, 1200.0, -700.0, |_| {}),
                sample_at(1, 900.0, -700.0, |s| {
                    s.indicated_airspeed = Some(SpeedKt::new(160.0));
                    s.bank = Some(AngleDeg::new(5.0));
                }),
            ])
            .finish();

        let ev = report.gates[0].expect("1000 ft gate crossed");
        assert_eq!(ev.gate_ft, 1000.0);
        assert_eq!(ev.vs_status, Some(true));
        assert_eq!(ev.ias_status, Some(true));
        assert_eq!(ev.gear_down, None);
        assert_eq!(
            report.classifications[0],
            Some(GateClassification::Indeterminate)
        );
        assert_ne!(report.classifications[0], Some(GateClassification::Stable));
    }
    #[test]
    fn fully_known_compliant_gate_is_stable() {
        let report = ApproachAnalyzer::new(StabilizationCriteria::default(), Some(160.0))
            .with_samples(&[
                sample_at(0, 1200.0, -700.0, |s| {
                    s.indicated_airspeed = Some(SpeedKt::new(160.0));
                    s.bank = Some(AngleDeg::new(5.0));
                    s.gear_handle_down = Some(true);
                    s.flaps_handle_index = Some(3);
                }),
                sample_at(1, 900.0, -700.0, |s| {
                    s.indicated_airspeed = Some(SpeedKt::new(160.0));
                    s.bank = Some(AngleDeg::new(-5.0));
                    s.gear_handle_down = Some(true);
                    s.flaps_handle_index = Some(3);
                }),
            ])
            .finish();
        assert_eq!(report.classifications[0], Some(GateClassification::Stable));
    }

    #[test]
    fn known_violation_is_unstable_even_with_other_unknowns() {
        let report = ApproachAnalyzer::new(StabilizationCriteria::default(), None)
            .with_samples(&[
                sample_at(0, 1200.0, -700.0, |s| {
                    s.bank = Some(AngleDeg::new(25.0)); // violates dev 10 deg
                }),
                sample_at(1, 900.0, -700.0, |s| {
                    s.bank = Some(AngleDeg::new(25.0));
                }),
            ])
            .finish();
        let ev = report.gates[0].expect("gate crossed");
        assert_eq!(ev.bank_status, Some(false));
        assert_eq!(ev.ias_status, None); // no reference -> unknown
        assert_eq!(
            report.classifications[0],
            Some(GateClassification::Unstable)
        );
    }

    #[test]
    fn classification_rule_is_fail_closed() {
        let ok = GateEvidence {
            gate_ft: 500.0,
            sample_seq: 1,
            timestamp_ms: 100,
            ias_status: Some(true),
            vs_status: Some(true),
            bank_status: Some(true),
            gear_down: Some(true),
            flaps_set: Some(true),
        };
        assert_eq!(
            GateClassification::classify(&ok),
            GateClassification::Stable
        );
        for none_field in [
            |e: &mut GateEvidence| e.gear_down = None,
            |e: &mut GateEvidence| e.flaps_set = None,
            |e: &mut GateEvidence| e.vs_status = None,
            |e: &mut GateEvidence| e.ias_status = None,
            |e: &mut GateEvidence| e.bank_status = None,
        ] {
            let mut e = ok;
            none_field(&mut e);
            assert_eq!(
                GateClassification::classify(&e),
                GateClassification::Indeterminate,
                "any unknown status must be Indeterminate"
            );
        }
        let mut e = ok;
        e.bank_status = Some(false);
        e.gear_down = None;
        assert_eq!(
            GateClassification::classify(&e),
            GateClassification::Unstable,
            "known violation wins over unrelated unknown"
        );
    }

    #[test]
    fn uncrossed_gate_is_none() {
        let report = ApproachAnalyzer::new(StabilizationCriteria::default(), None)
            .with_samples(&[sample_at(0, 800.0, -300.0, |_| {})])
            .finish();
        assert_eq!(report.gates[0], None); // 1000 ft never crossed
        assert_eq!(report.classifications[0], None);
    }

    #[test]
    fn go_around_requires_sustained_climb() {
        // Descend low, spike up once, sink again, THEN sustain 3 samples.
        let mut samples = Vec::new();
        let mk = |i: u64, agl: f64, vs: f64| sample_at(i, agl, vs, |_| {});
        samples.push(mk(0, 400.0, -600.0)); // descending towards runway
        samples.push(mk(1, 150.0, -500.0)); // below dev threshold, still descending
        samples.push(mk(2, 140.0, -100.0)); // arrest
        samples.push(mk(3, 160.0, 900.0)); // single-sample spike (run=1)
        samples.push(mk(4, 150.0, -50.0)); // sink again -> run broken
        samples.push(mk(5, 180.0, 700.0)); // run=1
        samples.push(mk(6, 260.0, 700.0)); // run=2
        samples.push(mk(7, 340.0, 700.0)); // run=3 -> confirmed here
        let report = ApproachAnalyzer::new(StabilizationCriteria::default(), None)
            .with_samples(&samples)
            .finish();
        let ga = report.go_around.expect("sustained climb confirmed");
        assert_eq!(ga.start_seq, 5);
        assert_eq!(ga.climb_confirmed_seq, 7);
        assert!((ga.min_agl_ft - 140.0).abs() < 1e-9);
    }

    #[test]
    fn short_climb_is_not_a_go_around() {
        // Two climbing samples only (< 3): no evidence.
        let samples = vec![
            sample_at(0, 400.0, -600.0, |_| {}),
            sample_at(1, 150.0, -500.0, |_| {}),
            sample_at(2, 160.0, 800.0, |_| {}),
            sample_at(3, 220.0, 800.0, |_| {}),
            sample_at(4, 280.0, -100.0, |_| {}), // sink again
        ];
        let report = ApproachAnalyzer::new(StabilizationCriteria::default(), None)
            .with_samples(&samples)
            .finish();
        assert_eq!(report.go_around, None);
    }

    #[test]
    fn ground_contact_before_confirmation_is_not_a_go_around() {
        // Bounce: brief climb then touchdown — resets, no go-around.
        let samples = vec![
            sample_at(0, 150.0, -400.0, |_| {}),
            sample_at(1, 130.0, 600.0, |_| {}),
            {
                let mut s = sample_at(2, 20.0, 100.0, |_| {});
                s.on_ground = Some(true);
                s
            },
            sample_at(3, 10.0, 0.0, |_| {}),
        ];
        let report = ApproachAnalyzer::new(StabilizationCriteria::default(), None)
            .with_samples(&samples)
            .finish();
        assert_eq!(report.go_around, None);
    }
}
