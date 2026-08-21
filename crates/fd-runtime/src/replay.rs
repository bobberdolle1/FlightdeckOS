//! Deterministic replay: a scripted [`SimulatorAdapter`] fed by a fixture.
//!
//! Replay proves the runtime without a running simulator: identical fixture
//! input must produce an identical output trace (same session id, same
//! ordering). The adapter never mutates scripted data — post-conditions are
//! observed by playing back later snapshots, exactly as with a live sim.

use std::collections::VecDeque;

use fd_core::actions::CockpitAction;
use fd_core::adapter::{AdapterError, Capability, SimulatorAdapter};
use fd_core::telemetry::SimTimestamp;
use fd_core::telemetry::TelemetrySnapshot;
use serde::{Deserialize, Serialize};

/// One fixture step: either a snapshot to feed, or an action to inject.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ReplayStep {
    /// Feed a canonical snapshot (fixture owns timestamps).
    Snapshot(TelemetrySnapshot),
    /// Inject an action request at the given timestamp.
    Action {
        ts: SimTimestamp,
        action: CockpitAction,
    },
}

/// Fixture file format version. Readers reject unknown versions.
pub const REPLAY_VERSION: u8 = 1;

/// A fixture line: `{"v": 1, <step...>}`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReplayLine {
    pub v: u8,
    #[serde(flatten)]
    pub step: ReplayStep,
}

/// Load a fixture file; fails closed on version mismatch / corruption.
pub fn load_fixture(path: impl AsRef<std::path::Path>) -> Result<Vec<ReplayStep>, String> {
    let content = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    parse_fixture(&content)
}

/// Parse fixture text.
pub fn parse_fixture(content: &str) -> Result<Vec<ReplayStep>, String> {
    let mut steps = Vec::new();
    for (idx, line) in content.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let parsed: ReplayLine =
            serde_json::from_str(line).map_err(|e| format!("fixture line {}: {e}", idx + 1))?;
        if parsed.v != REPLAY_VERSION {
            return Err(format!(
                "fixture line {}: unsupported version {}",
                idx + 1,
                parsed.v
            ));
        }
        steps.push(parsed.step);
    }
    Ok(steps)
}

/// Scripted adapter: polls snapshots from its queue in order.
#[derive(Debug, Default)]
pub struct ReplayAdapter {
    queue: VecDeque<TelemetrySnapshot>,
    connected: bool,
    dispatched: u64,
}

impl ReplayAdapter {
    pub fn new(steps: Vec<ReplayStep>) -> Self {
        let mut a = Self::default();
        a.load(steps);
        a
    }

    /// Load steps, retaining existing queue contents (snapshots appended in
    /// order). Action steps are ignored here — the application routes them
    /// to `Runtime::submit_action`.
    pub fn load(&mut self, steps: Vec<ReplayStep>) {
        for s in steps {
            if let ReplayStep::Snapshot(snap) = s {
                self.queue.push_back(snap);
            }
        }
    }

    pub fn push_snapshot(&mut self, snapshot: TelemetrySnapshot) {
        self.queue.push_back(snapshot);
    }

    pub fn dispatched_count(&self) -> u64 {
        self.dispatched
    }
}

impl SimulatorAdapter for ReplayAdapter {
    fn connect(&mut self) -> Result<(), AdapterError> {
        self.connected = true;
        Ok(())
    }

    fn disconnect(&mut self) {
        self.connected = false;
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    /// Deliver the next scripted snapshot, or nothing when the script is
    /// exhausted. One snapshot per poll keeps tick pacing deterministic.
    fn poll(&mut self) -> Result<Vec<TelemetrySnapshot>, AdapterError> {
        if !self.connected {
            return Err(AdapterError::NotConnected);
        }
        Ok(self.queue.pop_front().into_iter().collect())
    }

    fn capability(&self, action: CockpitAction) -> Capability {
        // The replay adapter mirrors the A32NX catalog surface.
        match action {
            CockpitAction::SetBeacon(_) | CockpitAction::SetNavLogo(_) => Capability::Supported,
        }
    }

    fn execute(&mut self, _action: CockpitAction) -> Result<(), AdapterError> {
        // A scripted write is a successful dispatch by definition; the
        // post-condition must still be observed in subsequent snapshots.
        self.dispatched += 1;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fd_core::actions::{Actor, CockpitAction, SwitchPosition};
    use fd_core::events::EventSeq;
    use fd_core::telemetry::NavLogoMode;

    #[test]
    fn fixture_roundtrip_parses_snapshots_and_actions() {
        let mut snap = TelemetrySnapshot::empty(SimTimestamp::new(5));
        snap.on_ground = Some(true);
        let text = format!(
            "{}\n{}\n",
            serde_json::to_string(&ReplayLine {
                v: 1,
                step: ReplayStep::Snapshot(snap)
            })
            .unwrap(),
            serde_json::to_string(&ReplayLine {
                v: 1,
                step: ReplayStep::Action {
                    ts: SimTimestamp::new(6),
                    action: CockpitAction::SetBeacon(SwitchPosition::On),
                }
            })
            .unwrap()
        );
        let steps = parse_fixture(&text).unwrap();
        assert_eq!(steps.len(), 2);
        assert!(matches!(steps[0], ReplayStep::Snapshot(_)));
        assert!(matches!(steps[1], ReplayStep::Action { .. }));
    }

    #[test]
    fn unknown_fixture_version_is_rejected() {
        assert!(parse_fixture("{\"v\":2,\"kind\":\"snapshot\",\"data\":{}}").is_err());
    }

    #[test]
    fn adapter_delivers_one_snapshot_per_poll_and_counts_dispatches() {
        let mut a = ReplayAdapter::new(Vec::new());
        a.push_snapshot(TelemetrySnapshot::empty(SimTimestamp::new(1)));
        a.push_snapshot(TelemetrySnapshot::empty(SimTimestamp::new(2)));
        a.connect().unwrap();
        let first = a.poll().unwrap();
        assert_eq!(first.len(), 1);
        assert_eq!(first[0].timestamp.ms, 1);
        let second = a.poll().unwrap();
        assert_eq!(second.len(), 1);
        assert_eq!(second[0].timestamp.ms, 2);
        assert!(a.poll().unwrap().is_empty()); // script exhausted
        a.execute(CockpitAction::SetNavLogo(NavLogoMode::Sys1))
            .unwrap();
        a.execute(CockpitAction::SetBeacon(SwitchPosition::On))
            .unwrap();
        assert_eq!(a.dispatched_count(), 2);
        let _ = (Actor::User, EventSeq::new(0));
    }
}
