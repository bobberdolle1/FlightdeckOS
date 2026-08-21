//! The deterministic runtime loop.
//!
//! One tick = poll adapter → ingest snapshots → phase evaluation → action
//! pipeline advancement → append traced events with freshly allocated
//! monotonic sequence numbers.
//!
//! Determinism contract: for a fixed (session id, adapter script) the output
//! trace is byte-identical. The decision path never reads wall clock, RNG,
//! or network state.

use fd_core::actions::{ActionCatalog, Actor, CockpitAction};
use fd_core::adapter::{AdapterError, SimulatorAdapter};
use fd_core::events::EventSource;
use fd_core::telemetry::TelemetrySnapshot;

use crate::executor::{ActionExecutor, DeadlineTicks};
use crate::ingest;
use crate::phase_tracker::PhaseTracker;
use crate::session::{Session, SessionId};
use crate::trace::{TraceError, TraceEvent, TraceWriter};

/// Per-tick statistics (informational only).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TickStats {
    pub snapshots: u64,
    pub events: u64,
}

/// The runtime: owns session, phase tracker, action executor, trace.
pub struct Runtime {
    adapter: Box<dyn SimulatorAdapter>,
    session: Session,
    phase: PhaseTracker,
    executor: ActionExecutor,
    catalog: ActionCatalog,
    trace: TraceWriter,
    last: Option<TelemetrySnapshot>,
}

impl Runtime {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        adapter: Box<dyn SimulatorAdapter>,
        trace: TraceWriter,
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
        }
    }

    /// Connect the adapter and emit `SessionStart`.
    pub fn start(&mut self) -> Result<(), AdapterError> {
        self.adapter.connect()?;
        let seq = self.session.next_seq();
        self.trace
            .append(&TraceEvent::SessionStart {
                seq,
                session_id: self.session.id,
            })
            .map_err(|e| AdapterError::ConnectionFailed(e.to_string()))?;
        Ok(())
    }

    /// Inject an action request (traced immediately; pipeline runs on tick).
    pub fn submit_action(
        &mut self,
        action: CockpitAction,
        actor: Actor,
        at: fd_core::telemetry::SimTimestamp,
    ) -> Result<fd_core::actions::ActionId, TraceError> {
        let seq = self.session.next_seq();
        let (id, event) = self.executor.submit(action, actor, at, seq);
        self.trace.append(&event)?;
        Ok(id)
    }

    /// Advance the runtime by one tick.
    ///
    /// Returns `Err(NotConnected)` when the adapter is not connected; the
    /// caller decides whether that is fatal.
    pub fn tick(&mut self, source: EventSource) -> Result<TickStats, AdapterError> {
        if !self.adapter.is_connected() {
            return Err(AdapterError::NotConnected);
        }
        let mut stats = TickStats::default();
        let snapshots = self.adapter.poll()?;

        for snapshot in snapshots {
            stats.snapshots += 1;
            let mut events: Vec<TraceEvent> = Vec::new();

            // 1. Ingest (sim-state change + state delta).
            events.extend(ingest::ingest(&self.last, &snapshot, source));

            // 2. Phase evaluation (suppressed while paused).
            if ingest::should_evaluate_phase(&snapshot, self.phase_paused())
                && let Some(evt) = self.phase.process(&snapshot)
            {
                events.push(evt);
            }

            // 3. Action pipeline (validation → dispatch → verification).
            events.extend(
                self.executor
                    .advance(&self.catalog, self.adapter.as_mut(), &snapshot),
            );

            // 4. Append with final monotonic sequence numbers.
            for mut evt in events {
                let seq = self.session.next_seq();
                evt.set_seq(seq);
                self.trace
                    .append(&evt)
                    .map_err(|e| AdapterError::WriteFailed(e.to_string()))?;
                stats.events += 1;
            }

            self.last = Some(snapshot);
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

    /// Most recent snapshot (for callers/tests).
    pub fn last_snapshot(&self) -> Option<&TelemetrySnapshot> {
        self.last.as_ref()
    }

    /// Emit `SessionEnd` and flush the trace.
    pub fn finish(mut self) -> Result<(), TraceError> {
        let seq = self.session.next_seq();
        self.trace.append(&TraceEvent::SessionEnd { seq })?;
        self.trace.finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::catalog::a32nx_default_catalog;
    use crate::replay::{ReplayAdapter, ReplayStep};
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
    fn full_action_pipeline_is_traced_in_order() {
        let dir = tempfile::tempdir().unwrap();
        let adapter = ReplayAdapter::new(vec![
            ReplayStep::Snapshot(beacon_snap(0, false, false)),
            ReplayStep::Snapshot(beacon_snap(1000, true, false)),
        ]);
        let trace = TraceWriter::create(trace_path(&dir, "t.jsonl")).unwrap();
        let mut rt = Runtime::new(
            Box::new(adapter),
            trace,
            SessionId(0),
            a32nx_default_catalog(),
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
                a32nx_default_catalog(),
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
            a32nx_default_catalog(),
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
