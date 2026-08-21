//! Action execution pipeline.
//!
//! Lifecycle (Task 1 §10): `Requested → Validated → Dispatched → Verified`,
//! with terminal `Rejected` / `Failed` / `TimedOut` branches. The central
//! rule: **success = observed post-condition**, never a successful API call.
//!
//! One `advance` call pushes every pending action through as many stages as
//! the current snapshot allows (validation → dispatch → verification may all
//! land in one tick). Deadlines are counted per advance after dispatch —
//! deterministic, no wall clock on the decision path.

use std::collections::VecDeque;

use fd_core::actions::{
    ActionCatalog, ActionFailure, ActionId, ActionRejectionReason, ActionRequest, ActionStatus,
};
use fd_core::adapter::SimulatorAdapter;
use fd_core::events::EventSeq;
use fd_core::telemetry::TelemetrySnapshot;

use crate::catalog::postcondition;
use crate::trace::TraceEvent;

/// Number of runtime ticks after dispatch before verification times out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeadlineTicks(pub u64);

impl Default for DeadlineTicks {
    fn default() -> Self {
        Self(150)
    }
}

/// One in-flight action with its pipeline state.
#[derive(Debug, Clone)]
pub struct ActionRecord {
    pub request: ActionRequest,
    pub status: ActionStatus,
    ticks_since_dispatch: u64,
}

impl ActionRecord {
    pub fn new(request: ActionRequest) -> Self {
        Self {
            request,
            status: ActionStatus::Requested,
            ticks_since_dispatch: 0,
        }
    }
}

/// The deterministic action pipeline.
#[derive(Debug, Default)]
pub struct ActionExecutor {
    pending: VecDeque<ActionRecord>,
    next_id: u64,
    deadline: DeadlineTicks,
}

impl ActionExecutor {
    pub fn new(deadline: DeadlineTicks) -> Self {
        Self {
            pending: VecDeque::new(),
            next_id: 0,
            deadline,
        }
    }

    /// Enqueue an action; returns its id and the `ActionRequested` event
    /// (with the caller-provided seq).
    pub fn submit(
        &mut self,
        action: fd_core::actions::CockpitAction,
        actor: fd_core::actions::Actor,
        at: fd_core::telemetry::SimTimestamp,
        seq: EventSeq,
    ) -> (ActionId, TraceEvent) {
        let id = ActionId(self.next_id);
        self.next_id = self.next_id.checked_add(1).expect("action id overflow");
        let request = ActionRequest {
            id,
            action,
            actor,
            at,
        };
        self.pending.push_back(ActionRecord::new(request.clone()));
        let event = TraceEvent::ActionRequested {
            seq,
            ts: at,
            action: request.action,
            actor: request.actor,
            id: request.id,
        };
        (id, event)
    }

    pub fn pending_count(&self) -> usize {
        self.pending.len()
    }

    /// Advance every pending action through as many pipeline stages as the
    /// current snapshot allows, in submission order.
    ///
    /// Returns the trace events generated, in deterministic order, with
    /// placeholder seqs (the runtime retags them before appending).
    pub fn advance(
        &mut self,
        catalog: &ActionCatalog,
        adapter: &mut dyn SimulatorAdapter,
        snapshot: &TelemetrySnapshot,
    ) -> Vec<TraceEvent> {
        let placeholder = EventSeq::new(u64::MAX);
        let mut events = Vec::new();

        // Repeat passes while any action makes progress. A dispatched action
        // with an unconfirmed post-condition makes no progress and waits.
        loop {
            let mut progressed = false;
            let mut keep = VecDeque::with_capacity(self.pending.len());

            while let Some(mut rec) = self.pending.pop_front() {
                let ts = snapshot.timestamp;
                match rec.status {
                    ActionStatus::Requested => {
                        match step_validate(&rec, catalog, adapter, snapshot) {
                            Ok(()) => {
                                rec.status = ActionStatus::Validated;
                                progressed = true;
                                events.push(TraceEvent::ActionValidated {
                                    seq: placeholder,
                                    ts,
                                    id: rec.request.id,
                                });
                                keep.push_back(rec);
                            }
                            Err(reason) => {
                                progressed = true;
                                events.push(TraceEvent::ActionRejected {
                                    seq: placeholder,
                                    ts,
                                    id: rec.request.id,
                                    reason,
                                });
                                // Terminal: not kept in the pending queue.
                            }
                        }
                    }
                    ActionStatus::Validated => match adapter.execute(rec.request.action) {
                        Ok(()) => {
                            rec.status = ActionStatus::Dispatched;
                            rec.ticks_since_dispatch = 0;
                            progressed = true;
                            events.push(TraceEvent::ActionDispatched {
                                seq: placeholder,
                                ts,
                                id: rec.request.id,
                            });
                            keep.push_back(rec);
                        }
                        Err(e) => {
                            progressed = true;
                            events.push(TraceEvent::ActionFailed {
                                seq: placeholder,
                                ts,
                                id: rec.request.id,
                                failure: ActionFailure::WriteFailed(e.to_string()),
                            });
                        }
                    },
                    ActionStatus::Dispatched => {
                        rec.ticks_since_dispatch += 1;
                        match postcondition(rec.request.action, snapshot) {
                            Some(true) => {
                                progressed = true;
                                events.push(TraceEvent::ActionVerified {
                                    seq: placeholder,
                                    ts,
                                    id: rec.request.id,
                                });
                            }
                            Some(false) | None => {
                                if rec.ticks_since_dispatch >= self.deadline.0 {
                                    progressed = true;
                                    events.push(TraceEvent::ActionFailed {
                                        seq: placeholder,
                                        ts,
                                        id: rec.request.id,
                                        failure: ActionFailure::VerificationTimeout,
                                    });
                                } else {
                                    keep.push_back(rec);
                                }
                            }
                        }
                    }
                    // Terminal states must never reappear (they are not kept).
                    ActionStatus::Rejected(_)
                    | ActionStatus::Failed(_)
                    | ActionStatus::TimedOut
                    | ActionStatus::Verified => {
                        unreachable!("terminal action status re-entered the pending queue");
                    }
                }
            }

            self.pending = keep;
            if !progressed || self.pending.is_empty() {
                break;
            }
        }

        events
    }
}

/// Validation step: catalog membership, adapter capability, preconditions.
fn step_validate(
    rec: &ActionRecord,
    catalog: &ActionCatalog,
    adapter: &dyn SimulatorAdapter,
    snapshot: &TelemetrySnapshot,
) -> Result<(), ActionRejectionReason> {
    let kind = rec.request.action.kind();
    let entry = catalog
        .lookup(kind)
        .ok_or(ActionRejectionReason::UnknownAction)?;

    let capability = adapter.capability(rec.request.action);
    if capability.blocks_dispatch() {
        return Err(match capability {
            fd_core::adapter::Capability::Unsupported => {
                ActionRejectionReason::UnsupportedByAdapter
            }
            _ => ActionRejectionReason::AdapterUnavailable,
        });
    }

    for pre in &entry.preconditions {
        (pre.check)(snapshot).map_err(|reason| {
            ActionRejectionReason::PreconditionFailed(format!("{}: {reason}", pre.id))
        })?;
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::catalog::a32nx_default_catalog;
    use crate::replay::ReplayAdapter;
    use fd_core::actions::{Actor, CockpitAction, SwitchPosition};
    use fd_core::telemetry::{NavLogoMode, SimTimestamp};

    fn snapshot_with_beacon(ts: u64, on: Option<bool>) -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(ts));
        s.beacon_light = on;
        s
    }

    fn submit_beacon(ex: &mut ActionExecutor, seq: u64) -> ActionId {
        let (id, _) = ex.submit(
            CockpitAction::SetBeacon(SwitchPosition::On),
            Actor::User,
            SimTimestamp::new(0),
            EventSeq::new(seq),
        );
        id
    }

    #[test]
    fn happy_path_validates_dispatches_then_verifies() {
        let mut ex = ActionExecutor::new(DeadlineTicks(3));
        let cat = a32nx_default_catalog();
        let mut adapter = ReplayAdapter::new(Vec::new());

        let id = submit_beacon(&mut ex, 0);

        // Tick 1: validate + dispatch (beacon currently known-off); the
        // post-condition is contradicted, so verification waits.
        let known_off = snapshot_with_beacon(10, Some(false));
        let evts = ex.advance(&cat, &mut adapter, &known_off);
        assert!(
            evts.iter()
                .any(|e| matches!(e, TraceEvent::ActionValidated { .. }))
        );
        assert!(
            evts.iter()
                .any(|e| matches!(e, TraceEvent::ActionDispatched { .. }))
        );
        assert!(
            !evts
                .iter()
                .any(|e| matches!(e, TraceEvent::ActionVerified { .. }))
        );

        // Tick 2: post-condition observed.
        let on = snapshot_with_beacon(20, Some(true));
        let evts = ex.advance(&cat, &mut adapter, &on);
        assert!(
            evts.iter()
                .any(|e| matches!(e, TraceEvent::ActionVerified { .. }))
        );
        assert_eq!(ex.pending_count(), 0);
        assert_eq!(id, ActionId(0));
    }

    #[test]
    fn already_satisfied_action_verifies_in_single_tick() {
        let mut ex = ActionExecutor::new(DeadlineTicks(3));
        let cat = a32nx_default_catalog();
        let mut adapter = ReplayAdapter::new(Vec::new());
        submit_beacon(&mut ex, 0);

        // Snapshot already shows beacon ON: validate+dispatch+verify at once.
        let on = snapshot_with_beacon(10, Some(true));
        let evts = ex.advance(&cat, &mut adapter, &on);
        for kind in ["action_validated", "action_dispatched", "action_verified"] {
            assert!(
                evts.iter().any(|e| serde_kind(e) == kind),
                "missing {kind} in {evts:?}"
            );
        }
        assert_eq!(ex.pending_count(), 0);
    }

    #[test]
    fn unknown_state_precondition_rejects_before_dispatch() {
        let mut ex = ActionExecutor::new(DeadlineTicks(3));
        let cat = a32nx_default_catalog();
        let mut adapter = ReplayAdapter::new(Vec::new());
        submit_beacon(&mut ex, 0);

        // Beacon never observed -> precondition fails -> Rejected.
        let unknown = snapshot_with_beacon(10, None);
        let evts = ex.advance(&cat, &mut adapter, &unknown);
        assert!(evts.iter().any(|e| matches!(
            e,
            TraceEvent::ActionRejected {
                reason: ActionRejectionReason::PreconditionFailed(_),
                ..
            }
        )));
        assert!(
            !evts
                .iter()
                .any(|e| matches!(e, TraceEvent::ActionDispatched { .. }))
        );
        assert_eq!(ex.pending_count(), 0);
    }

    #[test]
    fn unknown_action_kind_is_rejected() {
        let mut ex = ActionExecutor::new(DeadlineTicks(3));
        let empty = ActionCatalog::default();
        let mut adapter = ReplayAdapter::new(Vec::new());
        ex.submit(
            CockpitAction::SetNavLogo(NavLogoMode::Sys1),
            Actor::User,
            SimTimestamp::new(0),
            EventSeq::new(0),
        );
        let s = snapshot_with_beacon(10, Some(true));
        let evts = ex.advance(&empty, &mut adapter, &s);
        assert!(evts.iter().any(|e| matches!(
            e,
            TraceEvent::ActionRejected {
                reason: ActionRejectionReason::UnknownAction,
                ..
            }
        )));
    }

    #[test]
    fn verification_times_out_after_deadline_ticks() {
        let mut ex = ActionExecutor::new(DeadlineTicks(3)); // deadline = 3 ticks
        let cat = a32nx_default_catalog();
        let mut adapter = ReplayAdapter::new(Vec::new());
        submit_beacon(&mut ex, 0);

        // Validate + dispatch on known-off state; post-condition never
        // observed afterwards.
        let off = snapshot_with_beacon(10, Some(false));
        ex.advance(&cat, &mut adapter, &off);

        let mut failed = false;
        for _ in 0..4 {
            let evts = ex.advance(&cat, &mut adapter, &off);
            if evts.iter().any(|e| {
                matches!(
                    e,
                    TraceEvent::ActionFailed {
                        failure: ActionFailure::VerificationTimeout,
                        ..
                    }
                )
            }) {
                failed = true;
                break;
            }
        }
        assert!(failed, "verification did not time out");
        assert_eq!(ex.pending_count(), 0);
    }

    #[test]
    fn adapter_write_failure_is_action_failed_not_rejected() {
        let mut ex = ActionExecutor::new(DeadlineTicks(3));
        let cat = a32nx_default_catalog();
        let mut adapter = FailingWriteAdapter;
        submit_beacon(&mut ex, 0);
        let s = snapshot_with_beacon(10, Some(false));
        let evts = ex.advance(&cat, &mut adapter, &s);
        assert!(evts.iter().any(|e| {
            matches!(
                e,
                TraceEvent::ActionFailed {
                    failure: ActionFailure::WriteFailed(_),
                    ..
                }
            )
        }));
    }

    /// Test-only classification helper.
    fn serde_kind(e: &TraceEvent) -> &'static str {
        match e {
            TraceEvent::SessionStart { .. } => "session_start",
            TraceEvent::SessionEnd { .. } => "session_end",
            TraceEvent::StateDelta { .. } => "state_delta",
            TraceEvent::PhaseChange { .. } => "phase_change",
            TraceEvent::SimStateChanged { .. } => "sim_state_changed",
            TraceEvent::ActionRequested { .. } => "action_requested",
            TraceEvent::ActionValidated { .. } => "action_validated",
            TraceEvent::ActionDispatched { .. } => "action_dispatched",
            TraceEvent::ActionVerified { .. } => "action_verified",
            TraceEvent::ActionRejected { .. } => "action_rejected",
            TraceEvent::ActionFailed { .. } => "action_failed",
        }
    }

    /// Adapter that validates but fails every write.
    struct FailingWriteAdapter;
    impl SimulatorAdapter for FailingWriteAdapter {
        fn connect(&mut self) -> Result<(), fd_core::adapter::AdapterError> {
            Ok(())
        }
        fn disconnect(&mut self) {}
        fn is_connected(&self) -> bool {
            true
        }
        fn poll(&mut self) -> Result<Vec<TelemetrySnapshot>, fd_core::adapter::AdapterError> {
            Ok(Vec::new())
        }
        fn capability(&self, _: CockpitAction) -> fd_core::adapter::Capability {
            fd_core::adapter::Capability::Supported
        }
        fn execute(&mut self, _: CockpitAction) -> Result<(), fd_core::adapter::AdapterError> {
            Err(fd_core::adapter::AdapterError::WriteFailed(
                "injected failure".into(),
            ))
        }
    }
}
