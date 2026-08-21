//! Event sequencing and provenance.
//!
//! Ordering of runtime events never relies on wall clock: every observable
//! event carries a monotonic [`EventSeq`] assigned by the runtime session
//! counter, plus the simulator timestamp when available.

use serde::{Deserialize, Serialize};

/// Monotonic sequence number within one runtime session.
///
/// Assigned by the session counter; the counter is injectable in tests and
/// replay mode for full determinism.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct EventSeq(pub u64);

impl EventSeq {
    pub const fn new(n: u64) -> Self {
        Self(n)
    }

    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Provenance of a state event.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventSource {
    /// Live simulator data.
    Simulator,
    /// Injected by a deterministic replay fixture.
    Replay,
    /// Derived inside the runtime (e.g. phase inference).
    Derived,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seq_is_monotonic_by_construction() {
        let a = EventSeq::new(0);
        let b = EventSeq::new(1);
        assert!(a < b);
    }
}
