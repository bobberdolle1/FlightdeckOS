//! Quality of Landing: touchdown metrics captured at the airborne → ground
//! transition. Metrics without data are `None` (never zero, never invented).
//! Runway geometry (touchdown distance / centerline offset) is deliberately
//! absent until real runway geometry is available.

use crate::fdr::FdrSample;
use serde::{Deserialize, Serialize};

/// Touchdown measurements.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TouchdownReport {
    pub touchdown_vertical_speed_fpm: Option<f64>,
    pub touchdown_groundspeed_kt: Option<f64>,
    pub touchdown_pitch_deg: Option<f64>,
    pub touchdown_bank_deg: Option<f64>,
    pub timestamp_ms: Option<u64>,
}

/// Extract touchdown metrics from the airborne→ground transition.
///
/// Returns the report for the FIRST touchdown found in the stream
/// (subsequent touchdowns are additional bounces and are not the landing).
pub fn analyze(samples: &[FdrSample]) -> TouchdownReport {
    let mut prev: Option<&FdrSample> = None;
    for s in samples {
        if let Some(p) = prev
            && p.on_ground == Some(false)
            && s.on_ground == Some(true)
        {
            // Prefer the touchdown sample's VS (impact rate); fall back
            // to the last airborne sample when unknown there.
            let vs = s.vertical_speed.or(p.vertical_speed);
            return TouchdownReport {
                touchdown_vertical_speed_fpm: vs.filter(|v| v.is_finite()),
                touchdown_groundspeed_kt: s
                    .groundspeed
                    .filter(|v| v.is_finite())
                    .or(p.groundspeed.filter(|v| v.is_finite())),
                touchdown_pitch_deg: s.pitch.filter(|v| v.is_finite()),
                touchdown_bank_deg: s.bank.filter(|v| v.is_finite()),
                timestamp_ms: Some(s.timestamp.ms),
            };
        }
        prev = Some(s);
    }
    TouchdownReport::default()
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

    #[test]
    fn first_touchdown_captured_with_impact_vs() {
        let mut rec = Recorder::new();
        let a1 = sample(&mut rec, 1, 10.0, -150.0, 130.0, false);
        let t = sample(&mut rec, 2, 0.0, -140.0, 120.0, true);
        let b = sample(&mut rec, 3, 0.0, 0.0, 40.0, true);
        let r = analyze(&[a1, t, b]);
        assert_eq!(r.touchdown_vertical_speed_fpm, Some(-140.0));
        assert_eq!(r.touchdown_groundspeed_kt, Some(120.0));
        assert_eq!(r.touchdown_pitch_deg, Some(2.5));
        assert_eq!(r.timestamp_ms, Some(200));
    }

    #[test]
    fn no_touchdown_yields_all_unknown() {
        let mut rec = Recorder::new();
        let a = sample(&mut rec, 1, 500.0, -200.0, 150.0, false);
        let r = analyze(&[a]);
        assert_eq!(r.touchdown_vertical_speed_fpm, None);
        assert!(matches!(
            r,
            TouchdownReport {
                touchdown_vertical_speed_fpm: None,
                ..
            }
        ));
    }
}
