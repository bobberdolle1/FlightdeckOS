//! Replay-side flight-plan state reconstruction (Task 7 §35-38).
//!
//! Rebuilds the plan understanding of a recorded session from the FDR
//! EVENT STREAM ONLY. This is the replay counterpart of the live
//! `FmsWatcher`: same classification semantics, zero live queries
//! (§38 — no simulator, no network, no current FMS).
//!
//! Causality (§32): events are applied strictly in stream order; an
//! event may only use data it carries.

use crate::fdr::{FdrEvent, FdrEventPayload};
use fd_core::fplan::{FlightPlanChange, ProcedureContext, ProcedurePhase};

/// Reconstructed plan state at a point in the replay.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PlanReplayState {
    /// True once a FlightPlanObserved event was seen.
    pub observed: bool,
    pub device: Option<String>,
    pub primary_entries: Option<usize>,
    pub destination_id: Option<String>,
    pub approach_loaded: Option<bool>,
    /// Distinct revisions (observed + changed events).
    pub revisions: u64,
    /// All classified changes in stream order.
    pub changes: Vec<FlightPlanChange>,
    /// Latest procedure context (None = none deterministically supported).
    pub procedure: Option<ProcedureContext>,
    /// Latest navigation phase derivation.
    pub procedure_phase: ProcedurePhase,
    /// Latest runway context (airport, runway_end pair).
    pub runway: Option<(String, String)>,
}

impl PlanReplayState {
    /// Apply one event. Non-plan events are ignored; events WITHOUT a
    /// typed payload (legacy) cannot advance plan state — replay of a
    /// session that never recorded plan events honestly yields
    /// `observed == false` (§36: replay is incomplete rather than
    /// fabricated).
    pub fn apply(&mut self, event: &FdrEvent) {
        let Some(payload) = &event.payload else {
            return;
        };
        match payload {
            FdrEventPayload::FlightPlanObserved {
                device,
                primary_entries,
                approach_entries,
                destination_id,
                ..
            } => {
                self.observed = true;
                self.device = Some(device.clone());
                self.primary_entries = Some(*primary_entries);
                self.approach_loaded = approach_entries.map(|n| n > 0);
                self.destination_id = destination_id.clone();
                self.revisions += 1;
            }
            FdrEventPayload::FlightPlanChanged {
                changes,
                primary_entries,
                ..
            } => {
                self.revisions += 1;
                self.primary_entries = Some(*primary_entries);
                for c in changes {
                    if matches!(c, FlightPlanChange::ApproachLoaded) {
                        self.approach_loaded = Some(true);
                    }
                    if matches!(c, FlightPlanChange::ApproachCleared) {
                        self.approach_loaded = Some(false);
                    }
                    if matches!(c, FlightPlanChange::PlanCleared) {
                        self.primary_entries = Some(0);
                    }
                    self.changes.push(c.clone());
                }
            }
            FdrEventPayload::ProcedureContextChanged { context } => {
                self.procedure = context.clone();
                self.procedure_phase = match context.as_ref().map(|c| c.kind) {
                    Some(fd_core::fplan::ProcedureKind::Approach) => ProcedurePhase::Approach,
                    Some(fd_core::fplan::ProcedureKind::Sid) => ProcedurePhase::Sid,
                    Some(fd_core::fplan::ProcedureKind::Star) => ProcedurePhase::Star,
                    None => ProcedurePhase::Unknown,
                };
            }
            FdrEventPayload::RunwayContextChanged {
                airport,
                runway_end,
                ..
            } => {
                self.runway = Some((airport.clone(), runway_end.clone()));
            }
        }
    }

    /// Apply a whole event stream in order.
    pub fn replay(events: &[FdrEvent]) -> Self {
        let mut state = Self::default();
        for e in events {
            state.apply(e);
        }
        state
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fdr::FdrEventPayload;
    use fd_core::fplan::{FlightPlanChange, ProcedureKind};
    use fd_core::telemetry::SimTimestamp;

    fn event(seq: u64, payload: FdrEventPayload) -> FdrEvent {
        FdrEvent {
            seq,
            timestamp: SimTimestamp { ms: seq * 250 },
            kind: "flight_plan".into(),
            detail: String::new(),
            payload: Some(payload),
        }
    }

    #[test]
    fn replay_without_plan_events_is_honestly_unobserved() {
        let state = PlanReplayState::replay(&[]);
        assert!(!state.observed);
        assert_eq!(state.revisions, 0);
        // A legacy event (no payload) must not fabricate state either.
        let legacy = FdrEvent {
            seq: 1,
            timestamp: SimTimestamp { ms: 0 },
            kind: "fdm".into(),
            detail: "legacy".into(),
            payload: None,
        };
        let state = PlanReplayState::replay(&[legacy]);
        assert!(!state.observed);
    }

    #[test]
    fn replay_rebuilds_observation_and_changes() {
        let events = [
            event(
                1,
                FdrEventPayload::FlightPlanObserved {
                    device: "StockGps".into(),
                    revision_hash: 0xAA,
                    primary_entries: 3,
                    approach_entries: Some(0),
                    destination_entry: Some(2),
                    destination_id: Some("KSNA".into()),
                },
            ),
            event(
                2,
                FdrEventPayload::FlightPlanChanged {
                    changes: vec![FlightPlanChange::WaypointInserted { index: 1 }],
                    revision_hash: 0xBB,
                    primary_entries: 4,
                    destination_entry: Some(3),
                },
            ),
            event(
                3,
                FdrEventPayload::FlightPlanChanged {
                    changes: vec![FlightPlanChange::ApproachLoaded],
                    revision_hash: 0xCC,
                    primary_entries: 4,
                    destination_entry: Some(3),
                },
            ),
        ];
        let state = PlanReplayState::replay(&events);
        assert!(state.observed);
        assert_eq!(state.device.as_deref(), Some("StockGps"));
        assert_eq!(state.primary_entries, Some(4));
        assert_eq!(state.destination_id.as_deref(), Some("KSNA"));
        assert_eq!(state.revisions, 3);
        assert_eq!(state.changes.len(), 2);
        assert_eq!(state.approach_loaded, Some(true));
    }

    #[test]
    fn replay_tracks_procedure_and_runway_context() {
        let ctx = ProcedureContext {
            kind: ProcedureKind::Approach,
            procedure_ident: "ILS24L".into(),
            airport_ident: "KLAX".into(),
            matched_fixes: 4,
            evidence: "test".into(),
        };
        let events = [
            event(
                1,
                FdrEventPayload::ProcedureContextChanged {
                    context: Some(ctx.clone()),
                },
            ),
            event(
                2,
                FdrEventPayload::RunwayContextChanged {
                    airport: "KLAX".into(),
                    runway_end: "24L/06R".into(),
                    evidence: "dev".into(),
                },
            ),
            event(
                3,
                FdrEventPayload::ProcedureContextChanged { context: None },
            ),
        ];
        let mut state = PlanReplayState::default();
        state.apply(&events[0]);
        assert_eq!(state.procedure_phase, ProcedurePhase::Approach);
        state.apply(&events[1]);
        assert_eq!(state.runway.as_ref().map(|(a, _)| a.as_str()), Some("KLAX"));
        state.apply(&events[2]);
        assert_eq!(state.procedure, None);
        assert_eq!(state.procedure_phase, ProcedurePhase::Unknown);
    }
}
