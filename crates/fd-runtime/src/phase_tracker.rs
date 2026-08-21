//! Phase tracking: wraps the deterministic [`fd_core::phase::FlightPhaseEngine`]
//! with pause-aware gating and change events.

use fd_core::events::EventSeq;
use fd_core::phase::{FlightPhase, FlightPhaseEngine, PhaseAssessment, PhaseTelemetry};
use fd_core::telemetry::{SimState, TelemetrySnapshot};

use crate::trace::TraceEvent;

/// Pause-aware wrapper over the phase engine.
#[derive(Debug, Default)]
pub struct PhaseTracker {
    engine: FlightPhaseEngine,
    last: FlightPhase,
    /// True when the most recent known sim state was paused.
    paused: bool,
}

impl PhaseTracker {
    pub fn new() -> Self {
        Self {
            engine: FlightPhaseEngine::new(),
            last: FlightPhase::Preflight,
            paused: false,
        }
    }

    pub fn current(&self) -> FlightPhase {
        self.engine.current_phase()
    }

    /// Feed one snapshot to the phase engine.
    ///
    /// Returns `None` (no evaluation) while paused; returns the phase-change
    /// event when the phase actually changed (placeholder seq — the runtime
    /// assigns the final monotonic sequence).
    pub fn process(&mut self, snapshot: &TelemetrySnapshot) -> Option<TraceEvent> {
        match snapshot.sim_timing.state {
            SimState::Paused => {
                self.paused = true;
                return None;
            }
            // Unknown after a pause: hold off until Running is confirmed.
            SimState::Unknown if self.paused => return None,
            _ => {}
        }
        self.paused = false;

        let telemetry = PhaseTelemetry::from(snapshot);
        let assessment: PhaseAssessment = self.engine.evaluate(&telemetry);

        if assessment.phase != self.last {
            let from = self.last;
            self.last = assessment.phase;
            Some(TraceEvent::PhaseChange {
                seq: EventSeq::new(u64::MAX),
                ts: snapshot.timestamp,
                from,
                to: assessment.phase,
                confidence: format!("{:?}", assessment.confidence).to_lowercase(),
                evidence: assessment.evidence,
            })
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fd_core::units::{AltitudeAglFt, AltitudeFt, SpeedKt, VerticalSpeedFpm};

    fn snap(
        ts: u64,
        on_ground: bool,
        gs: f64,
        vs: f64,
        alt: f64,
        agl: f64,
        state: SimState,
    ) -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(fd_core::telemetry::SimTimestamp::new(ts));
        s.on_ground = Some(on_ground);
        s.groundspeed = Some(SpeedKt::new(gs));
        s.vertical_speed = Some(VerticalSpeedFpm::new(vs));
        s.altitude_msl = Some(AltitudeFt::new(alt));
        s.altitude_agl = Some(AltitudeAglFt::new(agl));
        s.sim_timing.state = state;
        s
    }

    #[test]
    fn paused_snapshots_produce_no_phase_events() {
        let mut t = PhaseTracker::new();
        // Airborne climb before pause. Preflight -> Climb is an immediate
        // transition (airborne directly from Preflight), so the FIRST
        // climbing sample already emits the phase-change event.
        let evt = t.process(&snap(
            0,
            false,
            250.0,
            2000.0,
            3000.0,
            2900.0,
            SimState::Running,
        ));
        assert!(evt.is_some());

        // Subsequent steady-state climb: no new events.
        assert!(
            t.process(&snap(
                1000,
                false,
                250.0,
                2000.0,
                3100.0,
                3000.0,
                SimState::Running
            ))
            .is_none()
        );
        assert!(
            t.process(&snap(
                2000,
                false,
                250.0,
                2000.0,
                3200.0,
                3100.0,
                SimState::Running
            ))
            .is_none()
        );

        // Pause: no evaluation, no events.
        assert!(
            t.process(&snap(
                3000,
                false,
                250.0,
                0.0,
                3200.0,
                3100.0,
                SimState::Paused
            ))
            .is_none()
        );

        // Unpause with level-flight telemetry: a legitimate evidence-based
        // transition (Climb -> Cruise) is fine; what must NOT happen is a
        // burst of synthetic phases invented while paused.
        if let Some(evt) = t.process(&snap(
            4000,
            false,
            250.0,
            0.0,
            3250.0,
            3150.0,
            SimState::Running,
        )) {
            match evt {
                TraceEvent::PhaseChange { to, evidence, .. } => {
                    assert_eq!(to, FlightPhase::Cruise);
                    assert!(!evidence.is_empty());
                }
                other => panic!("unexpected event while unpausing: {other:?}"),
            }
        }
        assert_eq!(t.current(), FlightPhase::Cruise);
    }
}
