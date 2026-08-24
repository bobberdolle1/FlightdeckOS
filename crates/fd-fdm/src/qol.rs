//! Quality of Landing V2: robust touchdown detection with debounce, plus
//! optional runway-relative metrics.
//!
//! Metrics without data are `None` (never zero, never invented).
//!
//! # Detection semantics (contract C10)
//!
//! A touchdown is counted only when ALL of the following hold:
//!
//! * **Airborne evidence**: at least [`DEV_AIRBORNE_EVIDENCE_SAMPLES`]
//!   consecutive known-airborne samples precede the transition;
//! * **Approach/descent context** at the last airborne sample: descending
//!   (VS ≤ [`DEV_CONTEXT_DESCENT_VS_FPM`]) OR low
//!   (AGL ≤ [`DEV_CONTEXT_MAX_AGL_FT`]);
//! * **on_ground transition** (`Some(false)` → `Some(true)`);
//! * **Debounce**: ground contact shorter than
//!   [`DEV_MIN_GROUND_SAMPLES`] samples is treated as flicker (a bounce
//!   artifact) and ignored — flicker-then-ground-again is ONE touchdown.
//!
//! Sign conventions: VS is signed (negative = descent); `TouchdownRecord::
//~ vs_fpm` preserves the signed impact VS. Fields whose names say "rate"
//! or "magnitude" (e.g. QoA `max_sink_rate_fpm`) carry positive magnitudes.
//!
//! Runway-relative metrics are computed ONLY when the touchdown has a known
//! position AND a [`RunwayGeometry`] implementation is supplied. Without
//! either, they stay `None`.

use crate::fdr::{FdrSample, FlightRecording};
use serde::{Deserialize, Serialize};

// DEVELOPMENT DEFAULTS — named so packages can override deliberately.
/// Consecutive airborne samples required before a transition counts.
pub const DEV_AIRBORNE_EVIDENCE_SAMPLES: u32 = 3;
/// VS at or below this (fpm, signed) counts as descent context.
pub const DEV_CONTEXT_DESCENT_VS_FPM: f64 = -100.0;
/// AGL at or below this (ft) counts as low-altitude approach context.
pub const DEV_CONTEXT_MAX_AGL_FT: f64 = 1000.0;
/// Ground contact lasting fewer samples than this is flicker and ignored.
pub const DEV_MIN_GROUND_SAMPLES: u32 = 2;

/// Structural runway geometry consumed by [`analyze_with_runway`].
///
/// Implemented by fd-mission's `RunwayContext` (at app level — fd-fdm must
/// not depend on fd-mission). All methods return `None` when geometry for
/// the given point is unknown: callers NEVER fabricate runway-relative
/// values. Object-safe by design.
pub trait RunwayGeometry {
    /// Signed lateral offset from the centerline in meters (+ right of it).
    fn centerline_offset_m(&self, lat: f64, lon: f64) -> Option<f64>;
    /// Distance from the landing threshold in meters (positive = beyond it).
    fn distance_to_threshold_m(&self, lat: f64, lon: f64) -> Option<f64>;
    /// Remaining runway length ahead of the point in meters.
    fn remaining_runway_m(&self, lat: f64, lon: f64) -> Option<f64>;
}

/// One detected touchdown with the measurements captured at contact.
///
/// `vs_fpm` is the SIGNED impact vertical speed (descent negative) from the
/// touchdown sample, falling back to the last airborne sample only if the
/// touchdown sample has no finite VS. Every measurement without data stays
/// `None`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TouchdownRecord {
    pub seq: u64,
    pub timestamp_ms: u64,
    /// Signed impact vertical speed (fpm; negative = descent).
    pub vs_fpm: Option<f64>,
    pub ias_kt: Option<f64>,
    pub gs_kt: Option<f64>,
    pub pitch_deg: Option<f64>,
    pub bank_deg: Option<f64>,
    pub heading_true_deg: Option<f64>,
    /// Geodetic position (lat_deg, lon_deg); `None` until FDR V2 position
    /// data is available on the sample.
    pub position: Option<(f64, f64)>,
}

/// Landing analysis result.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TouchdownReport {
    /// First qualifying touchdown; `None` = none detected.
    pub touchdown: Option<TouchdownRecord>,
    /// Signed centerline offset at touchdown (m, + right).
    /// `Some` ONLY with position present AND runway context supplied.
    pub centerline_offset_m: Option<f64>,
    /// Distance beyond the threshold at touchdown (m, positive = past it).
    pub distance_beyond_threshold_m: Option<f64>,
    /// Remaining runway ahead of the touchdown point (m).
    pub remaining_runway_m: Option<f64>,
}

/// Extract touchdown metrics from the first qualifying airborne→ground
/// transition (subsequent touchdowns are bounces, not the landing).
///
/// Base analysis without runway context: runway-relative metrics are `None`
/// even when a position is known.
pub fn analyze(samples: &[FdrSample]) -> TouchdownReport {
    detect(samples)
}

/// Like [`analyze`], plus runway-relative metrics evaluated at the touchdown
/// position — but ONLY when the record carries a known position; otherwise
/// all runway metrics stay `None`.
pub fn analyze_with_runway(
    recording: &FlightRecording,
    rw: &dyn RunwayGeometry,
) -> TouchdownReport {
    let mut report = analyze(&recording.samples);
    attach_runway_metrics(&mut report, rw);
    report
}

/// Fill runway-relative metrics from the touchdown record's position.
/// Missing position or unknown geometry keeps the metrics `None`.
fn attach_runway_metrics(report: &mut TouchdownReport, rw: &dyn RunwayGeometry) {
    if let Some(record) = &report.touchdown
        && let Some((lat, lon)) = record.position
    {
        report.centerline_offset_m = rw.centerline_offset_m(lat, lon);
        report.distance_beyond_threshold_m = rw.distance_to_threshold_m(lat, lon);
        report.remaining_runway_m = rw.remaining_runway_m(lat, lon);
    }
}

/// Robust detection with airborne evidence, approach/descent context and
/// ground-flicker debouncing.
fn detect(samples: &[FdrSample]) -> TouchdownReport {
    let mut airborne_run: u32 = 0;
    let mut i = 0usize;
    while i < samples.len() {
        let s = &samples[i];
        match s.on_ground {
            Some(false) => {
                airborne_run += 1;
                i += 1;
            }
            Some(true) => {
                // Transition candidate: need prior airborne evidence and
                // approach/descent context at the last airborne sample.
                let last_airborne = if i > 0 { Some(&samples[i - 1]) } else { None };
                let context_ok = airborne_run >= DEV_AIRBORNE_EVIDENCE_SAMPLES
                    && last_airborne.is_some_and(|p| {
                        let descending = p
                            .vertical_speed
                            .filter(|v| v.is_finite())
                            .is_some_and(|vs| vs <= DEV_CONTEXT_DESCENT_VS_FPM);
                        let low = p
                            .radio_altitude
                            .filter(|v| v.is_finite())
                            .is_some_and(|agl| agl <= DEV_CONTEXT_MAX_AGL_FT);
                        descending || low
                    });
                if !context_ok {
                    airborne_run = 0;
                    i += 1;
                    continue;
                }
                // Debounce: count consecutive ground samples starting here.
                let mut ground_len = 0u32;
                while i + (ground_len as usize) < samples.len()
                    && samples[i + ground_len as usize].on_ground == Some(true)
                {
                    ground_len += 1;
                }
                if ground_len >= DEV_MIN_GROUND_SAMPLES {
                    return TouchdownReport {
                        touchdown: Some(record_at(last_airborne.unwrap(), &samples[i])),
                        ..TouchdownReport::default()
                    };
                }
                // Flicker (< 2 ground samples): skip it; the aircraft is
                // airborne again, so keep accumulating evidence.
                i += ground_len as usize;
            }
            None => {
                // Unknown ground status: fail-closed, evidence resets.
                airborne_run = 0;
                i += 1;
            }
        }
    }
    TouchdownReport::default()
}

fn record_at(prev_airborne: &FdrSample, touchdown: &FdrSample) -> TouchdownRecord {
    // Prefer the touchdown sample's VS (impact rate); fall back to the last
    // airborne sample when unknown there. Signed value preserved.
    let vs = touchdown
        .vertical_speed
        .filter(|v| v.is_finite())
        .or_else(|| prev_airborne.vertical_speed.filter(|v| v.is_finite()));
    let gs = touchdown
        .groundspeed
        .filter(|v| v.is_finite())
        .or_else(|| prev_airborne.groundspeed.filter(|v| v.is_finite()));
    TouchdownRecord {
        seq: touchdown.seq,
        timestamp_ms: touchdown.timestamp.ms,
        vs_fpm: vs,
        ias_kt: touchdown.indicated_airspeed.filter(|v| v.is_finite()),
        gs_kt: gs,
        pitch_deg: touchdown.pitch.filter(|v| v.is_finite()),
        bank_deg: touchdown.bank.filter(|v| v.is_finite()),
        heading_true_deg: touchdown.heading_true.filter(|v| v.is_finite()),
        position: sample_position(touchdown),
    }
}

/// Position of one sample as plain `(lat_deg, lon_deg)`, when known.
///
/// FDR samples do not yet carry geographic position (FDR V2 field, landed
/// at Integration); until then NO recording can produce a position here and
/// the result is always `None`. Coordinates are never fabricated.
fn sample_position(_s: &FdrSample) -> Option<(f64, f64)> {
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fdr::Recorder;
    use fd_core::telemetry::{SimState, SimTimestamp, TelemetrySnapshot};
    use fd_core::units::{AltitudeAglFt, AngleDeg, SpeedKt, VerticalSpeedFpm};

    fn sample(
        rec: &mut Recorder,
        seq: u64,
        agl: f64,
        vs: f64,
        gs: f64,
        on_ground: bool,
    ) -> FdrSample {
        let mut snap = TelemetrySnapshot::empty(SimTimestamp::new(seq * 100));
        snap.altitude_agl = Some(AltitudeAglFt::new(agl));
        snap.vertical_speed = Some(VerticalSpeedFpm::new(vs));
        snap.groundspeed = Some(SpeedKt::new(gs));
        snap.pitch = Some(AngleDeg::new(2.5));
        snap.bank = Some(AngleDeg::new(0.4));
        snap.on_ground = Some(on_ground);
        snap.sim_timing.state = SimState::Running;
        rec.record(&snap, "LANDING")
    }

    /// Descent towards the runway: enough airborne evidence + context.
    fn approach(
        rec: &mut Recorder,
        out: &mut Vec<FdrSample>,
        start: u64,
        n: u32,
        agl: f64,
        vs: f64,
    ) {
        for k in 0..n {
            out.push(sample(rec, start + k as u64, agl, vs, 130.0, false));
        }
    }

    #[test]
    fn touchdown_captured_with_signed_impact_vs() {
        let mut rec = Recorder::new();
        let mut samples = Vec::new();
        approach(&mut rec, &mut samples, 1, 3, 60.0, -150.0);
        let t = sample(&mut rec, 4, 0.0, -140.0, 120.0, true);
        samples.push(t.clone());
        samples.push(sample(&mut rec, 5, 0.0, 0.0, 40.0, true));
        let r = analyze(&samples);
        let td = r.touchdown.expect("touchdown detected");
        assert_eq!(td.seq, t.seq);
        assert_eq!(td.vs_fpm, Some(-140.0), "impact VS stays signed");
        assert_eq!(td.gs_kt, Some(120.0));
        assert_eq!(td.pitch_deg, Some(2.5));
        assert_eq!(td.bank_deg, Some(0.4));
        assert_eq!(td.timestamp_ms, t.timestamp.ms);
        assert_eq!(r.centerline_offset_m, None, "no context -> no metrics");
    }

    #[test]
    fn no_touchdown_yields_no_record() {
        let mut rec = Recorder::new();
        let mut samples = Vec::new();
        approach(&mut rec, &mut samples, 1, 3, 500.0, -200.0);
        let r = analyze(&samples);
        assert_eq!(
            r,
            TouchdownReport::default(),
            "no touchdown, no runway context anywhere"
        );
    }

    #[test]
    fn insufficient_airborne_evidence_rejects_transition() {
        let mut rec = Recorder::new();
        let mut samples = Vec::new();
        // Only 2 airborne samples before the transition (< 3 required).
        approach(&mut rec, &mut samples, 1, 2, 30.0, -150.0);
        samples.push(sample(&mut rec, 3, 0.0, -140.0, 120.0, true));
        samples.push(sample(&mut rec, 4, 0.0, 0.0, 40.0, true));
        assert_eq!(analyze(&samples).touchdown, None);
    }

    #[test]
    fn no_approach_context_rejects_transition() {
        let mut rec = Recorder::new();
        let mut samples = Vec::new();
        // Airborne long enough but level flight HIGH above context limits:
        // neither descending beyond dev limit nor below dev AGL.
        approach(&mut rec, &mut samples, 1, 4, 5000.0, -50.0);
        samples.push(sample(&mut rec, 5, 0.0, -10.0, 80.0, true));
        samples.push(sample(&mut rec, 6, 0.0, 0.0, 20.0, true));
        assert_eq!(analyze(&samples).touchdown, None);
    }

    #[test]
    fn single_sample_ground_flicker_is_ignored() {
        let mut rec = Recorder::new();
        let mut samples = Vec::new();
        approach(&mut rec, &mut samples, 1, 3, 40.0, -140.0);
        samples.push(sample(&mut rec, 4, 0.0, -130.0, 125.0, true)); // flicker
        samples.push(sample(&mut rec, 5, 15.0, -120.0, 124.0, false)); // airborne
        let t = sample(&mut rec, 6, 0.0, -110.0, 118.0, true);
        samples.push(t.clone());
        samples.push(sample(&mut rec, 7, 0.0, 0.0, 60.0, true)); // rollout
        let td = analyze(&samples)
            .touchdown
            .expect("real touchdown after flicker");
        assert_eq!(td.seq, t.seq, "flicker ignored, ONE touchdown reported");
        assert_eq!(td.vs_fpm, Some(-110.0));
    }

    #[test]
    fn two_sample_ground_contact_is_real() {
        let mut rec = Recorder::new();
        let mut samples = Vec::new();
        approach(&mut rec, &mut samples, 1, 3, 40.0, -140.0);
        let t = sample(&mut rec, 4, 0.0, -130.0, 125.0, true);
        samples.push(t.clone());
        samples.push(sample(&mut rec, 5, 0.0, 0.0, 90.0, true));
        let td = analyze(&samples)
            .touchdown
            .expect(">=2 ground samples real");
        assert_eq!(td.seq, t.seq);
    }

    #[test]
    fn unknown_ground_status_resets_evidence() {
        let mut rec = Recorder::new();
        let mut samples = Vec::new();
        approach(&mut rec, &mut samples, 1, 3, 40.0, -140.0);
        // One unknown-status sample breaks the airborne run...
        let mut snap = TelemetrySnapshot::empty(SimTimestamp::new(999));
        snap.altitude_agl = Some(AltitudeAglFt::new(35.0));
        snap.on_ground = None;
        snap.sim_timing.state = SimState::Running;
        samples.push(rec.record(&snap, "LANDING"));
        let _t = sample(&mut rec, 6, 0.0, -130.0, 125.0, true);
        samples.push(sample(&mut rec, 7, 0.0, 0.0, 90.0, true));
        // ...so the next transition has < 3 airborne samples: rejected.
        assert_eq!(analyze(&samples).touchdown, None);
    }

    #[test]
    fn runway_metrics_only_with_position_and_context() {
        struct FixedRunway;
        impl RunwayGeometry for FixedRunway {
            fn centerline_offset_m(&self, _lat: f64, _lon: f64) -> Option<f64> {
                Some(12.5)
            }
            fn distance_to_threshold_m(&self, _lat: f64, _lon: f64) -> Option<f64> {
                Some(870.0)
            }
            fn remaining_runway_m(&self, lat: f64, _lon: f64) -> Option<f64> {
                (lat > 52.000_5).then_some(1130.0)
            }
        }

        // Case 1: context supplied, NO position -> metrics stay None.
        let mut rec = Recorder::new();
        let mut samples = Vec::new();
        approach(&mut rec, &mut samples, 1, 3, 40.0, -140.0);
        samples.push(sample(&mut rec, 4, 0.0, -130.0, 125.0, true));
        samples.push(sample(&mut rec, 5, 0.0, 0.0, 90.0, true));
        let recording = FlightRecording {
            meta: None,
            samples: samples.clone(),
            events: Vec::new(),
        };
        let r = analyze_with_runway(&recording, &FixedRunway);
        assert!(r.touchdown.is_some());
        assert_eq!(r.centerline_offset_m, None);
        assert_eq!(r.distance_beyond_threshold_m, None);
        assert_eq!(r.remaining_runway_m, None);

        // Case 2: position present AND context -> metrics filled. FDR
        // samples cannot carry positions yet, so the record is constructed
        // directly (identical to what `sample_position` will yield once
        // FDR V2 lands).
        let mut report = TouchdownReport {
            touchdown: Some(TouchdownRecord {
                seq: 9,
                timestamp_ms: 900,
                position: Some((52.001, 4.001)),
                ..TouchdownRecord::default()
            }),
            ..TouchdownReport::default()
        };
        attach_runway_metrics(&mut report, &FixedRunway);
        assert_eq!(report.centerline_offset_m, Some(12.5));
        assert_eq!(report.distance_beyond_threshold_m, Some(870.0));
        assert_eq!(report.remaining_runway_m, Some(1130.0));

        // Unknown geometry at the point stays None (never fabricated).
        let mut report = TouchdownReport {
            touchdown: Some(TouchdownRecord {
                position: Some((0.0, 0.0)), // FixedRunway: remaining unknown there
                ..TouchdownRecord::default()
            }),
            ..TouchdownReport::default()
        };
        attach_runway_metrics(&mut report, &FixedRunway);
        assert_eq!(report.centerline_offset_m, Some(12.5));
        assert_eq!(report.remaining_runway_m, None);

        // No position -> no metrics even with context.
        let mut report = TouchdownReport {
            touchdown: Some(TouchdownRecord::default()),
            ..TouchdownReport::default()
        };
        attach_runway_metrics(&mut report, &FixedRunway);
        assert_eq!(report.centerline_offset_m, None);

        // Base analyze() NEVER reports runway metrics.
        let r_base = analyze(&samples);
        assert_eq!(r_base.touchdown.is_some(), true);
        assert_eq!(r_base.centerline_offset_m, None);
        assert_eq!(r_base.distance_beyond_threshold_m, None);
        assert_eq!(r_base.remaining_runway_m, None);
    }
}
