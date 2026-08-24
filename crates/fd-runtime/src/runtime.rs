//! The deterministic runtime loop.
//!
//! One tick = poll adapter → ingest snapshots → phase evaluation → action
//! pipeline advancement → append traced events with freshly allocated
//! monotonic sequence numbers.
//!
//! Determinism contract: for a fixed (session id, adapter script) the output
//! trace is byte-identical. The decision path never reads wall clock, RNG,
//! or network state.

use fd_core::actions::{ActionCatalog, ActionId, ActionStatus, Actor, CockpitAction};
use fd_core::adapter::{AdapterError, SimulatorAdapter};
use fd_core::events::EventSource;
use fd_core::telemetry::TelemetrySnapshot;
use thiserror::Error;

use fd_sop::engine::{FlowEngine, SopEvent};

use crate::executor::{ActionExecutor, DeadlineTicks};
use crate::ingest;
use crate::phase_tracker::PhaseTracker;
use crate::session::{Session, SessionId};
use crate::trace::{TraceError, TraceEvent, TraceSink};

/// Runtime-level domain errors. Trace/storage failures are FIRST-CLASS:
/// they are never reported as adapter/simulator failures (Task 1.2 F6).
#[derive(Debug, Error)]
pub enum RuntimeError {
    #[error("simulator adapter error: {0}")]
    Adapter(#[from] AdapterError),
    #[error("trace failure: {0}")]
    Trace(#[from] TraceError),
    #[error("runtime poisoned by an earlier trace failure: {0}")]
    Poisoned(String),
}

/// Per-tick statistics (informational only).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TickStats {
    pub snapshots: u64,
    pub events: u64,
}

/// The runtime: owns session, phase tracker, action executor, trace.
///
/// `W` is the trace sink; [`TraceWriter`] is the production implementation,
/// controllable failing sinks are used in tests (F6).
pub struct Runtime<W: TraceSink> {
    adapter: Box<dyn SimulatorAdapter>,
    session: Session,
    phase: PhaseTracker,
    executor: ActionExecutor,
    catalog: ActionCatalog,
    trace: W,
    last: Option<TelemetrySnapshot>,
    /// Optional SOP flow engine (Task 2). Started explicitly via
    /// [`Runtime::start_flow`]; no automatic phase triggers.
    flows: Option<FlowEngine>,
    /// Terminal action outcomes drained for flow processing (applied on the
    /// next tick's flow pass — deterministic single-pass ordering).
    flow_outcomes: Vec<(ActionId, ActionStatus)>,
    /// Set on the first trace failure: the runtime then refuses all further
    /// work (fail-stop). In-memory state that missed its mandatory trace
    /// events is discarded when the poisoned runtime is dropped — it must
    /// never silently continue (Task 1.2 F6).
    poisoned: Option<TraceError>,
}

impl<W: TraceSink> Runtime<W> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        adapter: Box<dyn SimulatorAdapter>,
        trace: W,
        session_id: SessionId,
        catalog: ActionCatalog,
        deadline: DeadlineTicks,
    ) -> Self {
        Self {
            adapter,
            session: Session::with_id(session_id),
            phase: PhaseTracker::new(),
            executor: ActionExecutor::new(deadline),
            catalog,
            trace,
            last: None,
            flows: None,
            flow_outcomes: Vec::new(),
            poisoned: None,
        }
    }

    /// Start an SOP flow instance (explicit start; no automatic phase
    /// triggers in Task 2).
    pub fn start_flow(
        &mut self,
        flow: fd_sop::package::FlowDefinition,
    ) -> Result<(), RuntimeError> {
        self.check_poisoned()?;
        if self.flows.is_some() {
            return Err(RuntimeError::Poisoned("a flow is already active".into()));
        }
        self.flows = Some(FlowEngine::new(flow));
        Ok(())
    }

    /// Current flow status, if a flow was started.
    pub fn flow_status(&self) -> Option<fd_sop::engine::FlowStatus> {
        self.flows.as_ref().map(|f| f.status())
    }

    /// Record a trace failure and poison the runtime.
    fn poison(&mut self, err: TraceError) -> RuntimeError {
        let detail = err.to_string();
        self.poisoned = Some(TraceError::Corrupt(detail.clone()));
        RuntimeError::Trace(err)
    }

    fn check_poisoned(&self) -> Result<(), RuntimeError> {
        match &self.poisoned {
            Some(e) => Err(RuntimeError::Poisoned(e.to_string())),
            None => Ok(()),
        }
    }

    /// Connect the adapter and emit `SessionStart`.
    pub fn start(&mut self) -> Result<(), RuntimeError> {
        self.check_poisoned()?;
        self.adapter.connect().map_err(RuntimeError::Adapter)?;
        let seq = self.session.next_seq();
        if let Err(e) = self.trace.append(&TraceEvent::SessionStart {
            seq,
            session_id: self.session.id,
        }) {
            let err = self.poison(e);
            return Err(err);
        }
        Ok(())
    }

    /// Inject an action request (traced immediately; pipeline runs on tick).
    pub fn submit_action(
        &mut self,
        action: CockpitAction,
        actor: Actor,
        at: fd_core::telemetry::SimTimestamp,
    ) -> Result<fd_core::actions::ActionId, RuntimeError> {
        self.check_poisoned()?;
        let seq = self.session.next_seq();
        let (id, event) = self.executor.submit(action, actor, at, seq);
        if let Err(e) = self.trace.append(&event) {
            return Err(self.poison(e));
        }
        // NOTE: the request enters the pipeline only after its mandatory
        // trace event is durably appended (mutation-after-trace ordering).
        self.executor.commit();
        Ok(id)
    }

    /// Advance the runtime by one tick.
    ///
    /// Trace semantics (Task 1.2 F6): all events of this tick are buffered,
    /// then appended under freshly allocated monotonic sequence numbers. On
    /// the FIRST trace failure the runtime POISONS itself: the error is
    /// returned as [`RuntimeError::Trace`] (never disguised as an adapter
    /// failure), every later call fails with [`RuntimeError::Poisoned`], and
    /// the already-mutated in-memory state is abandoned by dropping the
    /// runtime. The pipeline never continues in a silently untraced mode.
    pub fn tick(&mut self, source: EventSource) -> Result<TickStats, RuntimeError> {
        self.check_poisoned()?;
        if !self.adapter.is_connected() {
            return Err(RuntimeError::Adapter(AdapterError::NotConnected));
        }
        let mut stats = TickStats::default();
        let snapshots = self.adapter.poll().map_err(RuntimeError::Adapter)?;

        let mut events: Vec<TraceEvent> = Vec::new();

        for snapshot in snapshots {
            stats.snapshots += 1;

            // 1. Ingest (sim-state change + state delta).
            events.extend(ingest::ingest(&self.last, &snapshot, source));

            // 2. Phase evaluation (suppressed while paused).
            if ingest::should_evaluate_phase(&snapshot, self.phase_paused())
                && let Some(evt) = self.phase.process(&snapshot)
            {
                events.push(evt);
            }

            // 3. Action pipeline (validation -> dispatch -> verification).
            events.extend(
                self.executor
                    .advance(&self.catalog, self.adapter.as_mut(), &snapshot),
            );

            // 3b. SOP flow pass (Task 2): outcomes from THIS tick's advance
            // are reported first, then newly-ready action steps are staged
            // through the same two-phase submit. Their ActionRequested
            // events join this tick's buffered trace; staged requests are
            // committed together with everything else after the trace
            // append succeeds.
            if let Some(flows) = self.flows.as_mut() {
                let mut pass = fd_sop::engine::FlowTickOutput::default();
                flows.report_outcomes(&self.flow_outcomes, &mut pass);
                let processed = flows.process(&snapshot);
                pass.events.extend(processed.events);
                pass.requests.extend(processed.requests);

                for evt in &pass.events {
                    events.push(sop_event_to_trace(evt.clone(), snapshot.timestamp));
                }
                for req in pass.requests {
                    let seq = self.session.next_seq();
                    let (id, ev) =
                        self.executor
                            .submit(req.action, Actor::Runtime, snapshot.timestamp, seq);
                    self.flows
                        .as_mut()
                        .expect("flows present")
                        .assign_action_id(&req.step_id, id);
                    events.push(ev);
                }
            }

            self.last = Some(snapshot);
        }

        // 4. Append buffered events with final monotonic sequence numbers.
        for evt in &mut events {
            let seq = self.session.next_seq();
            evt.set_seq(seq);
            if let Err(e) = self.trace.append(evt) {
                return Err(self.poison(e));
            }
            stats.events += 1;
        }

        // 5. Trace succeeded: commit staged SOP requests and drain terminal
        // outcomes for the next tick's flow pass. Without a flow engine the
        // outcomes stay in the executor for `take_completed_actions`
        // (diagnostics) — a flow-less runtime has no other consumer.
        self.executor.commit();
        if self.flows.is_some() {
            self.flow_outcomes = self.executor.take_completed();
        }

        Ok(stats)
    }

    fn phase_paused(&self) -> bool {
        self.last
            .as_ref()
            .map(|s| s.sim_timing.state == fd_core::telemetry::SimState::Paused)
            .unwrap_or(false)
    }

    /// Current canonical phase.
    pub fn current_phase(&self) -> fd_core::phase::FlightPhase {
        self.phase.current()
    }

    /// Number of actions currently in the pipeline.
    pub fn pending_action_count(&self) -> usize {
        self.executor.pending_count()
    }

    /// Drain terminal action statuses (Verified/Failed/Rejected/TimedOut)
    /// recorded since the last call. Diagnostics surface for smokes.
    pub fn take_completed_actions(&mut self) -> Vec<(fd_core::actions::ActionId, ActionStatus)> {
        self.executor.take_completed()
    }

    /// Most recent snapshot (for callers/tests).
    pub fn last_snapshot(&self) -> Option<&TelemetrySnapshot> {
        self.last.as_ref()
    }

    /// Emit `SessionEnd` and flush the trace.
    pub fn finish(mut self) -> Result<(), RuntimeError> {
        self.check_poisoned()?;
        let seq = self.session.next_seq();
        if let Err(e) = self.trace.append(&TraceEvent::SessionEnd { seq }) {
            return Err(self.poison(e));
        }
        self.trace.finish().map_err(RuntimeError::Trace)
    }
}

/// Map an SOP event onto the runtime trace (stamping happens via set_seq).
fn sop_event_to_trace(ev: SopEvent, ts: fd_core::telemetry::SimTimestamp) -> TraceEvent {
    match ev {
        SopEvent::FlowStarted { flow } => TraceEvent::FlowStarted {
            seq: fd_core::events::EventSeq::new(u64::MAX),
            ts,
            flow,
        },
        SopEvent::StepReady { flow, step } => TraceEvent::StepReady {
            seq: fd_core::events::EventSeq::new(u64::MAX),
            ts,
            flow,
            step,
        },
        SopEvent::StepWaitingForVerification { flow, step } => {
            TraceEvent::StepWaitingForVerification {
                seq: fd_core::events::EventSeq::new(u64::MAX),
                ts,
                flow,
                step,
            }
        }
        SopEvent::StepActionRequested { flow, step, action } => TraceEvent::StepActionRequested {
            seq: fd_core::events::EventSeq::new(u64::MAX),
            ts,
            flow,
            step,
            action,
        },
        SopEvent::StepVerified { flow, step } => TraceEvent::StepVerified {
            seq: fd_core::events::EventSeq::new(u64::MAX),
            ts,
            flow,
            step,
        },
        SopEvent::StepFailed { flow, step, reason } => TraceEvent::StepFailed {
            seq: fd_core::events::EventSeq::new(u64::MAX),
            ts,
            flow,
            step,
            reason,
        },
        SopEvent::FlowCompleted { flow } => TraceEvent::FlowCompleted {
            seq: fd_core::events::EventSeq::new(u64::MAX),
            ts,
            flow,
        },
        SopEvent::FlowFailed { flow } => TraceEvent::FlowFailed {
            seq: fd_core::events::EventSeq::new(u64::MAX),
            ts,
            flow,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    /// Minimal beacon-only catalog for runtime tests (aircraft catalogs
    /// live in fd-aircraft).
    fn test_catalog() -> fd_core::actions::ActionCatalog {
        use fd_core::actions::{
            ActionCatalog, ActionKind, CatalogEntry, PreconditionDef, SwitchPosition,
        };
        ActionCatalog {
            entries: vec![CatalogEntry {
                kind: ActionKind::SetBeacon,
                preconditions: vec![PreconditionDef {
                    id: "beacon_state_known",
                    check: |s| {
                        if s.beacon_light.is_some() {
                            Ok(())
                        } else {
                            Err("beacon state unknown")
                        }
                    },
                }],
                verify: |a, s| match a {
                    CockpitAction::SetBeacon(pos) => s
                        .beacon_light
                        .map(|on| on == matches!(pos, SwitchPosition::On)),
                    _ => None,
                },
            }],
        }
    }
    use crate::replay::{ReplayAdapter, ReplayStep};
    use crate::trace::TraceWriter;
    use fd_core::actions::{Actor, CockpitAction, SwitchPosition};
    use fd_core::events::EventSource;
    use fd_core::telemetry::{SimTimestamp, TelemetrySnapshot};
    use fd_core::units::{AltitudeAglFt, AltitudeFt, SpeedKt, VerticalSpeedFpm};

    fn trace_path(dir: &tempfile::TempDir, name: &str) -> std::path::PathBuf {
        dir.path().join(name)
    }

    fn beacon_snap(ts: u64, on: bool, paused: bool) -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(ts));
        s.on_ground = Some(true);
        s.groundspeed = Some(SpeedKt::new(0.0));
        s.beacon_light = Some(on);
        s.sim_timing.state = if paused {
            fd_core::telemetry::SimState::Paused
        } else {
            fd_core::telemetry::SimState::Running
        };
        s
    }

    #[test]
    fn flowless_runtime_surfaces_terminal_actions_to_diagnostics() {
        // Regression: tick() unconditionally drained terminal action
        // outcomes into the SOP flow buffer, so a flow-less consumer (the
        // live beacon smoke) never saw Verified. The drain now belongs to
        // flow-equipped runtimes only.
        let dir = tempfile::tempdir().unwrap();
        let adapter = ReplayAdapter::new(vec![
            ReplayStep::Snapshot(beacon_snap(0, false, false)),
            ReplayStep::Snapshot(beacon_snap(1000, true, false)),
            ReplayStep::Snapshot(beacon_snap(2000, true, false)),
        ]);
        let trace = TraceWriter::create(trace_path(&dir, "t.jsonl")).unwrap();
        let mut rt = Runtime::new(
            Box::new(adapter),
            trace,
            SessionId(0),
            test_catalog(),
            DeadlineTicks::default(),
        );
        rt.start().unwrap();
        rt.tick(EventSource::Replay).unwrap();
        rt.submit_action(
            CockpitAction::SetBeacon(SwitchPosition::On),
            Actor::User,
            SimTimestamp::new(500),
        )
        .unwrap();
        rt.tick(EventSource::Replay).unwrap();
        let drained = rt.take_completed_actions();
        assert!(
            drained.is_empty(),
            "no terminal status before the fresh post-dispatch observation: {drained:?}"
        );
        rt.tick(EventSource::Replay).unwrap();
        let drained = rt.take_completed_actions();
        assert_eq!(
            drained.len(),
            1,
            "terminal action must surface: {drained:?}"
        );
        assert_eq!(drained[0].1, ActionStatus::Verified);
        rt.finish().unwrap();
    }

    #[test]
    fn full_action_pipeline_is_traced_in_order() {
        let dir = tempfile::tempdir().unwrap();
        let adapter = ReplayAdapter::new(vec![
            ReplayStep::Snapshot(beacon_snap(0, false, false)),
            ReplayStep::Snapshot(beacon_snap(1000, true, false)),
            // Spec §22: verification consumes a sample strictly newer than
            // the dispatch boundary — a third tick supplies it.
            ReplayStep::Snapshot(beacon_snap(2000, true, false)),
        ]);
        let trace = TraceWriter::create(trace_path(&dir, "t.jsonl")).unwrap();
        let mut rt = Runtime::new(
            Box::new(adapter),
            trace,
            SessionId(0),
            test_catalog(),
            DeadlineTicks::default(),
        );
        rt.start().unwrap();

        // First tick: state becomes known.
        rt.tick(EventSource::Replay).unwrap();
        // Submit action; next tick verifies against the beacon-on snapshot.
        rt.submit_action(
            CockpitAction::SetBeacon(SwitchPosition::On),
            Actor::User,
            SimTimestamp::new(500),
        )
        .unwrap();
        rt.tick(EventSource::Replay).unwrap();
        // Third tick: fresh post-dispatch observation verifies.
        rt.tick(EventSource::Replay).unwrap();
        rt.finish().unwrap();

        let events = crate::trace::read_trace(trace_path(&dir, "t.jsonl")).unwrap();
        assert!(matches!(events[0], TraceEvent::SessionStart { .. }));
        let kinds: Vec<&str> = events
            .iter()
            .map(|e| match e {
                TraceEvent::StateDelta { .. } => "state_delta",
                TraceEvent::PhaseChange { .. } => "phase_change",
                TraceEvent::SimStateChanged { .. } => "sim_state_changed",
                TraceEvent::ActionRequested { .. } => "action_requested",
                TraceEvent::ActionValidated { .. } => "action_validated",
                TraceEvent::ActionDispatched { .. } => "action_dispatched",
                TraceEvent::ActionVerified { .. } => "action_verified",
                TraceEvent::ActionRejected { .. } => "action_rejected",
                TraceEvent::ActionFailed { .. } => "action_failed",
                TraceEvent::SessionEnd { .. } => "session_end",
                TraceEvent::SessionStart { .. } => "session_start",
                TraceEvent::FlowStarted { .. }
                | TraceEvent::StepReady { .. }
                | TraceEvent::StepWaitingForVerification { .. }
                | TraceEvent::StepActionRequested { .. }
                | TraceEvent::StepVerified { .. }
                | TraceEvent::StepFailed { .. }
                | TraceEvent::FlowCompleted { .. }
                | TraceEvent::FlowFailed { .. } => "sop",
                TraceEvent::MissionPhaseChanged { .. }
                | TraceEvent::MissionCompleted { .. }
                | TraceEvent::MissionFailed { .. } => "mission",
            })
            .collect();
        // NOTE: the FIRST snapshot produces no delta (nothing to diff
        // against); it only makes the precondition state known. The second
        // snapshot carries the beacon change and completes the pipeline.
        let expected = [
            "session_start",
            "action_requested",
            "state_delta",
            "action_validated",
            "action_dispatched",
            "action_verified",
            "session_end",
        ];
        assert_eq!(kinds.as_slice(), expected.as_slice());
    }

    /// Controllable failing trace sink for F6 semantics tests.
    struct FailingTraceWriter {
        fail_at_append: usize,
        appends_done: usize,
    }
    impl crate::trace::TraceSink for FailingTraceWriter {
        fn append(&mut self, event: &TraceEvent) -> Result<(), crate::trace::TraceError> {
            self.appends_done += 1;
            if self.appends_done == self.fail_at_append {
                return Err(crate::trace::TraceError::Io("injected disk failure".into()));
            }
            let _ = event;
            Ok(())
        }
        fn finish(self) -> Result<(), crate::trace::TraceError> {
            Ok(())
        }
    }

    #[test]
    fn trace_failure_during_state_event_is_runtime_trace_error_and_poisons() {
        // session_start(1), then the SECOND snapshot's state_delta(2):
        // fail exactly on that delta append.
        let adapter = ReplayAdapter::new(vec![
            ReplayStep::Snapshot(beacon_snap(0, false, false)),
            ReplayStep::Snapshot(beacon_snap(1000, true, false)),
        ]);
        let writer = FailingTraceWriter {
            fail_at_append: 2,
            appends_done: 0,
        };
        let mut rt = Runtime::new(
            Box::new(adapter),
            writer,
            SessionId(0),
            test_catalog(),
            DeadlineTicks::default(),
        );
        rt.start().unwrap();

        // Tick 1: first snapshot -> no delta yet, append budget untouched.
        rt.tick(EventSource::Replay).unwrap();
        let err = rt.tick(EventSource::Replay).unwrap_err();
        assert!(
            matches!(err, RuntimeError::Trace(_)),
            "expected Trace domain error, got {err:?}"
        );

        // Poisoned: every further call fails; nothing can silently continue.
        let err = rt.tick(EventSource::Replay).unwrap_err();
        assert!(matches!(err, RuntimeError::Poisoned(_)));
        drop(rt); // in-memory untraced state is abandoned here
    }

    #[test]
    fn trace_failure_during_action_transition_is_trace_domain_not_adapter() {
        // session_start(1), then action_requested(2): fail on the request.
        let adapter = ReplayAdapter::new(Vec::new());
        let writer = FailingTraceWriter {
            fail_at_append: 2,
            appends_done: 0,
        };
        let mut rt = Runtime::new(
            Box::new(adapter),
            writer,
            SessionId(0),
            test_catalog(),
            DeadlineTicks::default(),
        );
        rt.start().unwrap();

        let err = rt
            .submit_action(
                CockpitAction::SetBeacon(SwitchPosition::On),
                Actor::User,
                SimTimestamp::new(5),
            )
            .unwrap_err();
        assert!(
            matches!(err, RuntimeError::Trace(_)),
            "trace failure must not be reported as adapter/write failure"
        );
        // The staged request was NOT committed into the pipeline.
        assert_eq!(rt.pending_action_count(), 0);
    }

    #[test]
    fn pause_keeps_event_sequence_strictly_monotonic_case_c() {
        let dir = tempfile::tempdir().unwrap();
        let mut s1 = beacon_snap(0, false, false);
        s1.groundspeed = Some(SpeedKt::new(10.0));
        let paused = beacon_snap(1000, false, true);
        let mut s3 = beacon_snap(2000, false, false);
        s3.groundspeed = Some(SpeedKt::new(14.0));

        let adapter = ReplayAdapter::new(vec![
            ReplayStep::Snapshot(s1),
            ReplayStep::Snapshot(paused),
            ReplayStep::Snapshot(s3),
        ]);
        let trace = TraceWriter::create(trace_path(&dir, "t.jsonl")).unwrap();
        let mut rt = Runtime::new(
            Box::new(adapter),
            trace,
            SessionId(0),
            test_catalog(),
            DeadlineTicks::default(),
        );
        rt.start().unwrap();
        for _ in 0..4 {
            rt.tick(EventSource::Replay).unwrap();
        }
        rt.finish().unwrap();

        let events = crate::trace::read_trace(trace_path(&dir, "t.jsonl")).unwrap();
        let seqs: Vec<u64> = events.iter().map(|e| e.seq().value()).collect();
        // Strictly increasing across the pause boundary.
        for w in seqs.windows(2) {
            assert!(w[0] < w[1], "non-monotonic seq: {seqs:?}");
        }
    }

    #[test]
    fn replay_is_byte_deterministic_across_runs() {
        let dir = tempfile::tempdir().unwrap();
        let run = |name: &str| {
            let mut s1 = beacon_snap(0, false, false);
            s1.groundspeed = Some(SpeedKt::new(10.0));
            let mut s2 = beacon_snap(1000, false, false);
            s2.groundspeed = Some(SpeedKt::new(14.0));
            let mut s3 = beacon_snap(2000, false, false);
            s3.groundspeed = Some(SpeedKt::new(18.0));
            let adapter = ReplayAdapter::new(vec![
                ReplayStep::Snapshot(s1),
                ReplayStep::Snapshot(s2),
                ReplayStep::Snapshot(s3),
            ]);
            let trace = TraceWriter::create(trace_path(&dir, name)).unwrap();
            let mut rt = Runtime::new(
                Box::new(adapter),
                trace,
                SessionId(0),
                test_catalog(),
                DeadlineTicks::default(),
            );
            rt.start().unwrap();
            for _ in 0..4 {
                rt.tick(EventSource::Replay).unwrap();
            }
            rt.finish().unwrap();
            std::fs::read(trace_path(&dir, name)).unwrap()
        };

        let a = run("a.jsonl");
        let b = run("b.jsonl");
        assert_eq!(a, b, "replay output must be byte-identical");
    }

    #[test]
    fn paused_phase_produces_no_phase_events() {
        let dir = tempfile::tempdir().unwrap();
        let mut airborne = TelemetrySnapshot::empty(SimTimestamp::new(0));
        airborne.on_ground = Some(false);
        airborne.groundspeed = Some(SpeedKt::new(250.0));
        airborne.vertical_speed = Some(VerticalSpeedFpm::new(2000.0));
        airborne.altitude_msl = Some(AltitudeFt::new(3000.0));
        airborne.altitude_agl = Some(AltitudeAglFt::new(2900.0));
        airborne.sim_timing.state = fd_core::telemetry::SimState::Running;

        let mut paused = airborne.clone();
        paused.timestamp = SimTimestamp::new(1000);
        paused.sim_timing.state = fd_core::telemetry::SimState::Paused;

        let adapter = ReplayAdapter::new(vec![
            ReplayStep::Snapshot(airborne.clone()),
            ReplayStep::Snapshot(paused),
            ReplayStep::Snapshot(airborne),
        ]);
        let trace = TraceWriter::create(trace_path(&dir, "t.jsonl")).unwrap();
        let mut rt = Runtime::new(
            Box::new(adapter),
            trace,
            SessionId(0),
            test_catalog(),
            DeadlineTicks::default(),
        );
        rt.start().unwrap();
        for _ in 0..4 {
            rt.tick(EventSource::Replay).unwrap();
        }
        rt.finish().unwrap();

        let events = crate::trace::read_trace(trace_path(&dir, "t.jsonl")).unwrap();
        let phase_events: Vec<_> = events
            .iter()
            .filter(|e| matches!(e, TraceEvent::PhaseChange { .. }))
            .collect();
        // The only phase event is Preflight -> Climb from the first sample.
        assert_eq!(phase_events.len(), 1);
        assert!(
            events
                .iter()
                .any(|e| matches!(e, TraceEvent::SimStateChanged { .. }))
        );
    }
}
