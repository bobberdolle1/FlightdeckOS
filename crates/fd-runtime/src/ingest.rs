//! Snapshot ingestion: turning consecutive canonical snapshots into traced,
//! sequenced state events.

use fd_core::delta::diff;
use fd_core::events::{EventSeq, EventSource};
use fd_core::telemetry::{SimState, TelemetrySnapshot};

use crate::trace::TraceEvent;

/// Ingest one snapshot relative to the previous one.
///
/// Produces, in deterministic order:
/// 1. a `SimStateChanged` event when the simulator pause state changed;
/// 2. a `StateDelta` event when any field changed.
///
/// Events carry placeholder sequence numbers; the runtime assigns final
/// monotonic seqs right before appending.
pub fn ingest(
    prev: &Option<TelemetrySnapshot>,
    next: &TelemetrySnapshot,
    source: EventSource,
) -> Vec<TraceEvent> {
    let placeholder = || EventSeq::new(u64::MAX);
    let mut events = Vec::new();

    if let Some(p) = prev
        && p.sim_timing.state != next.sim_timing.state
    {
        events.push(TraceEvent::SimStateChanged {
            seq: placeholder(),
            ts: next.timestamp,
            from: p.sim_timing.state,
            to: next.sim_timing.state,
        });
    }

    if let Some(p) = prev {
        let changed = diff(p, next);
        if !changed.is_empty() {
            events.push(TraceEvent::StateDelta {
                seq: placeholder(),
                ts: next.timestamp,
                source,
                changed,
            });
        }
    }

    events
}

/// Whether phase evaluation should run for this snapshot.
///
/// Phase evaluation is suppressed while paused or when the sim state is
/// unknown *after* having been known — no false phase sequences.
pub fn should_evaluate_phase(next: &TelemetrySnapshot, prev_paused: bool) -> bool {
    match next.sim_timing.state {
        SimState::Running => true,
        SimState::Paused => false,
        // Unknown: evaluate only before we ever observed a state (startup).
        SimState::Unknown => !prev_paused,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fd_core::telemetry::{SimTimestamp, TelemetrySnapshot};
    use fd_core::units::SpeedKt;

    fn snap(ts: u64, gs: f64, state: SimState) -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(ts));
        s.groundspeed = Some(SpeedKt::new(gs));
        s.sim_timing.state = state;
        s
    }

    #[test]
    fn first_snapshot_produces_no_events() {
        let events = ingest(&None, &snap(0, 0.0, SimState::Running), EventSource::Replay);
        assert!(events.is_empty());
    }

    #[test]
    fn pause_transition_and_delta_are_both_emitted() {
        let prev = Some(snap(0, 0.0, SimState::Running));
        let next = snap(1, 10.0, SimState::Paused);
        let events = ingest(&prev, &next, EventSource::Replay);
        assert_eq!(events.len(), 2);
        assert!(matches!(events[0], TraceEvent::SimStateChanged { .. }));
        assert!(matches!(events[1], TraceEvent::StateDelta { .. }));
    }

    #[test]
    fn phase_evaluation_is_suppressed_while_paused() {
        assert!(!should_evaluate_phase(
            &snap(0, 0.0, SimState::Paused),
            false
        ));
        assert!(should_evaluate_phase(
            &snap(0, 0.0, SimState::Running),
            true
        ));
    }
}
