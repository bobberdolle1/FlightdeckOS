//! Append-only runtime trace (JSONL).
//!
//! Format decision (Task 1 §13): **JSONL over SQLite**.
//!
//! * Append-only requirement: a line-oriented file with append writes is the
//!   simplest structure with that property; crash recovery is trivial (the
//!   last partial line is discarded on read).
//! * Determinism for tests: byte-stable `serde_json` output — no storage
//!   engine ordering/idiosyncrasies to pin.
//! * Human inspectable: one event per line, `jq`-friendly.
//! * Replay: streaming line reader, no index needed at Task 1 scale.
//!
//! SQLite adds query capability we do not need yet; the trace format is
//! versioned so a migration to SQLite (or an SQLite export) remains cheap
//! later. Event ordering comes from the session's monotonic `seq`, never
//! from the file or wall clock.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use fd_core::actions::{ActionFailure, ActionRejectionReason};
use fd_core::delta::DeltaField;
use fd_core::events::{EventSeq, EventSource};
use fd_core::phase::FlightPhase;
use fd_core::telemetry::{SimState, SimTimestamp};
use serde::{Deserialize, Serialize};

use crate::session::SessionId;

/// Trace format version. Readers must reject unknown versions (fail-closed).
/// Task 2: added SOP Flow*/Step* event kinds (deliberate version bump;
/// readers of the old schema must fail closed).
pub const TRACE_VERSION: u8 = 2;

/// A version tag wrapper; readers check this before interpreting a line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TraceVersion;

/// The complete event vocabulary of the Task 1 trace.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum TraceEvent {
    SessionStart {
        seq: EventSeq,
        session_id: SessionId,
    },
    SessionEnd {
        seq: EventSeq,
    },
    StateDelta {
        seq: EventSeq,
        ts: SimTimestamp,
        source: EventSource,
        changed: Vec<DeltaField>,
    },
    PhaseChange {
        seq: EventSeq,
        ts: SimTimestamp,
        from: FlightPhase,
        to: FlightPhase,
        confidence: String,
        evidence: String,
    },
    SimStateChanged {
        seq: EventSeq,
        ts: SimTimestamp,
        from: SimState,
        to: SimState,
    },
    ActionRequested {
        seq: EventSeq,
        ts: SimTimestamp,
        action: fd_core::actions::CockpitAction,
        actor: fd_core::actions::Actor,
        id: fd_core::actions::ActionId,
    },
    ActionValidated {
        seq: EventSeq,
        ts: SimTimestamp,
        id: fd_core::actions::ActionId,
    },
    ActionDispatched {
        seq: EventSeq,
        ts: SimTimestamp,
        id: fd_core::actions::ActionId,
    },
    ActionVerified {
        seq: EventSeq,
        ts: SimTimestamp,
        id: fd_core::actions::ActionId,
    },
    ActionRejected {
        seq: EventSeq,
        ts: SimTimestamp,
        id: fd_core::actions::ActionId,
        reason: ActionRejectionReason,
    },
    ActionFailed {
        seq: EventSeq,
        ts: SimTimestamp,
        id: fd_core::actions::ActionId,
        failure: ActionFailure,
    },
    // -- SOP flow lifecycle (Task 2) --
    FlowStarted {
        seq: EventSeq,
        ts: SimTimestamp,
        flow: String,
    },
    StepReady {
        seq: EventSeq,
        ts: SimTimestamp,
        flow: String,
        step: String,
    },
    StepWaitingForVerification {
        seq: EventSeq,
        ts: SimTimestamp,
        flow: String,
        step: String,
    },
    StepActionRequested {
        seq: EventSeq,
        ts: SimTimestamp,
        flow: String,
        step: String,
        action: fd_core::actions::CockpitAction,
    },
    StepVerified {
        seq: EventSeq,
        ts: SimTimestamp,
        flow: String,
        step: String,
    },
    StepFailed {
        seq: EventSeq,
        ts: SimTimestamp,
        flow: String,
        step: String,
        reason: String,
    },
    FlowCompleted {
        seq: EventSeq,
        ts: SimTimestamp,
        flow: String,
    },
    FlowFailed {
        seq: EventSeq,
        ts: SimTimestamp,
        flow: String,
    },
}

/// A single trace line: `{"v": 1, <event fields...>}`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct TraceLine {
    v: u8,
    #[serde(flatten)]
    event: TraceEvent,
}

/// Storage-agnostic trace sink contract.
///
/// Exists so the runtime can be tested against a controllable failing sink
/// (Task 1.2 F6): trace failures must surface as a distinct domain error and
/// poison the runtime, never as adapter/simulator failures.
pub trait TraceSink {
    fn append(&mut self, event: &TraceEvent) -> Result<(), TraceError>;
    fn finish(self) -> Result<(), TraceError>;
}

/// Append-only JSONL trace writer.
pub struct TraceWriter {
    inner: BufWriter<File>,
}

impl TraceWriter {
    /// Create (or truncate) the trace file at `path`.
    pub fn create(path: impl AsRef<Path>) -> Result<Self, TraceError> {
        let file = File::create(path).map_err(|e| TraceError::Io(e.to_string()))?;
        Ok(Self {
            inner: BufWriter::new(file),
        })
    }

    /// Append one event as a versioned JSON line. Each event is flushed
    /// immediately (crash-safe; volume is tiny at Task 1 scale).
    pub fn append(&mut self, event: &TraceEvent) -> Result<(), TraceError> {
        let line = TraceLine {
            v: TRACE_VERSION,
            event: event.clone(),
        };
        serde_json::to_writer(&mut self.inner, &line)
            .map_err(|e| TraceError::Serde(e.to_string()))?;
        self.inner
            .write_all(b"\n")
            .map_err(|e| TraceError::Io(e.to_string()))?;
        self.inner
            .flush()
            .map_err(|e| TraceError::Io(e.to_string()))
    }

    /// Flush and close the underlying file.
    pub fn finish(self) -> Result<(), TraceError> {
        let mut w = self.inner;
        w.flush().map_err(|e| TraceError::Io(e.to_string()))
    }
}

impl TraceSink for TraceWriter {
    fn append(&mut self, event: &TraceEvent) -> Result<(), TraceError> {
        TraceWriter::append(self, event)
    }

    fn finish(self) -> Result<(), TraceError> {
        TraceWriter::finish(self)
    }
}
impl TraceEvent {
    /// Sequence number carried by this event.
    pub fn seq(&self) -> EventSeq {
        match self {
            Self::SessionStart { seq, .. }
            | Self::SessionEnd { seq }
            | Self::StateDelta { seq, .. }
            | Self::PhaseChange { seq, .. }
            | Self::SimStateChanged { seq, .. }
            | Self::ActionRequested { seq, .. }
            | Self::ActionValidated { seq, .. }
            | Self::ActionDispatched { seq, .. }
            | Self::ActionVerified { seq, .. }
            | Self::ActionRejected { seq, .. }
            | Self::ActionFailed { seq, .. }
            | Self::FlowStarted { seq, .. }
            | Self::StepReady { seq, .. }
            | Self::StepWaitingForVerification { seq, .. }
            | Self::StepActionRequested { seq, .. }
            | Self::StepVerified { seq, .. }
            | Self::StepFailed { seq, .. }
            | Self::FlowCompleted { seq, .. }
            | Self::FlowFailed { seq, .. } => *seq,
        }
    }

    /// Kind tag for diagnostics/tests.
    pub fn kind_str(&self) -> &'static str {
        match self {
            Self::SessionStart { .. } => "session_start",
            Self::SessionEnd { .. } => "session_end",
            Self::StateDelta { .. } => "state_delta",
            Self::PhaseChange { .. } => "phase_change",
            Self::SimStateChanged { .. } => "sim_state_changed",
            Self::ActionRequested { .. } => "action_requested",
            Self::ActionValidated { .. } => "action_validated",
            Self::ActionDispatched { .. } => "action_dispatched",
            Self::ActionVerified { .. } => "action_verified",
            Self::ActionRejected { .. } => "action_rejected",
            Self::ActionFailed { .. } => "action_failed",
            Self::FlowStarted { .. } => "flow_started",
            Self::StepReady { .. } => "step_ready",
            Self::StepWaitingForVerification { .. } => "step_waiting_for_verification",
            Self::StepActionRequested { .. } => "step_action_requested",
            Self::StepVerified { .. } => "step_verified",
            Self::StepFailed { .. } => "step_failed",
            Self::FlowCompleted { .. } => "flow_completed",
            Self::FlowFailed { .. } => "flow_failed",
        }
    }

    /// Overwrite the carried sequence number (used by the runtime when it
    /// assigns final monotonic seqs to events produced during a tick).
    pub fn set_seq(&mut self, seq: EventSeq) {
        match self {
            Self::SessionStart { seq: s, .. } => *s = seq,
            Self::SessionEnd { seq: s } => *s = seq,
            Self::StateDelta { seq: s, .. } => *s = seq,
            Self::PhaseChange { seq: s, .. } => *s = seq,
            Self::SimStateChanged { seq: s, .. } => *s = seq,
            Self::ActionRequested { seq: s, .. } => *s = seq,
            Self::ActionValidated { seq: s, .. } => *s = seq,
            Self::ActionDispatched { seq: s, .. } => *s = seq,
            Self::ActionVerified { seq: s, .. } => *s = seq,
            Self::ActionRejected { seq: s, .. } => *s = seq,
            Self::ActionFailed { seq: s, .. } => *s = seq,
            Self::FlowStarted { seq: s, .. }
            | Self::StepReady { seq: s, .. }
            | Self::StepWaitingForVerification { seq: s, .. }
            | Self::StepActionRequested { seq: s, .. }
            | Self::StepVerified { seq: s, .. }
            | Self::StepFailed { seq: s, .. }
            | Self::FlowCompleted { seq: s, .. }
            | Self::FlowFailed { seq: s, .. } => *s = seq,
        }
    }
}

/// Trace reader: iterate events; fails closed on malformed/foreign versions.
pub fn read_trace(path: impl AsRef<Path>) -> Result<Vec<TraceEvent>, TraceError> {
    let content = std::fs::read_to_string(path).map_err(|e| TraceError::Io(e.to_string()))?;
    let mut events = Vec::new();
    for (idx, line) in content.lines().enumerate() {
        if line.trim().is_empty() {
            continue;
        }
        let parsed: TraceLine = serde_json::from_str(line)
            .map_err(|e| TraceError::Corrupt(format!("line {}: {e}", idx + 1)))?;
        if parsed.v != TRACE_VERSION {
            return Err(TraceError::Corrupt(format!(
                "line {}: unsupported trace version {}",
                idx + 1,
                parsed.v
            )));
        }
        events.push(parsed.event);
    }
    Ok(events)
}

/// Typed trace errors (replay corruption is first-class).
#[derive(Debug, thiserror::Error)]
pub enum TraceError {
    #[error("trace io error: {0}")]
    Io(String),
    #[error("trace serialization error: {0}")]
    Serde(String),
    #[error("trace corrupted: {0}")]
    Corrupt(String),
}

#[cfg(test)]
mod tests {
    use super::*;
    use fd_core::actions::{ActionId, Actor, CockpitAction, SwitchPosition};
    use fd_core::delta::DeltaField;

    fn sample_event(seq: u64) -> TraceEvent {
        TraceEvent::ActionRequested {
            seq: EventSeq::new(seq),
            ts: SimTimestamp::new(1000),
            action: CockpitAction::SetBeacon(SwitchPosition::On),
            actor: Actor::User,
            id: ActionId(0),
        }
    }

    #[test]
    fn write_read_roundtrip_preserves_events_and_order() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trace.jsonl");
        let mut w = TraceWriter::create(&path).unwrap();
        w.append(&TraceEvent::SessionStart {
            seq: EventSeq::new(0),
            session_id: SessionId(1),
        })
        .unwrap();
        w.append(&sample_event(1)).unwrap();
        w.append(&TraceEvent::SessionEnd {
            seq: EventSeq::new(2),
        })
        .unwrap();
        w.finish().unwrap();

        let events = read_trace(&path).unwrap();
        assert_eq!(events.len(), 3);
        assert!(matches!(events[0], TraceEvent::SessionStart { .. }));
        assert_eq!(events[1], sample_event(1));
        assert!(matches!(events[2], TraceEvent::SessionEnd { .. }));
    }

    #[test]
    fn unknown_version_is_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trace.jsonl");
        std::fs::write(&path, r#"{"v":99,"kind":"session_end","seq":1}"#).unwrap();
        let err = read_trace(&path).unwrap_err();
        assert!(err.to_string().contains("unsupported trace version"));
    }

    #[test]
    fn malformed_line_is_rejected_with_line_number() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("trace.jsonl");
        std::fs::write(&path, "not json\n").unwrap();
        let err = read_trace(&path).unwrap_err();
        assert!(err.to_string().contains("line 1"));
    }

    #[test]
    fn state_delta_serializes_as_required_categories() {
        let evt = TraceEvent::StateDelta {
            seq: EventSeq::new(4),
            ts: SimTimestamp::new(9),
            source: EventSource::Replay,
            changed: vec![DeltaField::BeaconLight],
        };
        let s = serde_json::to_string(&TraceLine { v: 1, event: evt }).unwrap();
        assert!(s.contains("\"kind\":\"state_delta\""));
        assert!(s.contains("\"beacon_light\""));
        assert!(s.starts_with("{\"v\":1"));
    }
}
