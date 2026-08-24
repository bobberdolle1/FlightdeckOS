//! Live write guard (spec §14): live simulator writes are DISABLED by
//! default and must be explicitly armed per process.
//!
//! Invariants:
//! * telemetry polling, FDR recording, and Mission Shadow NEVER require
//!   the guard — only the discrete-action dispatch path consults it;
//! * the guard is process-lifetime state: a restart returns to disabled;
//! * the armed state is never persisted silently.

use std::sync::atomic::{AtomicBool, Ordering};

/// Process-wide live-write inhibit. Cheap, `Sync`, shareable.
#[derive(Debug, Default)]
pub struct LiveWriteGuard {
    armed: AtomicBool,
}

impl LiveWriteGuard {
    /// Disabled guard (the default state of every process).
    pub fn disabled() -> Self {
        Self::default()
    }

    /// Explicitly arm safe writes for this process (developer intent).
    pub fn arm(&self) {
        self.armed.store(true, Ordering::SeqCst);
    }

    /// Disarm: every subsequent write attempt is rejected.
    pub fn disarm(&self) {
        self.armed.store(false, Ordering::SeqCst);
    }

    pub fn is_armed(&self) -> bool {
        self.armed.load(Ordering::SeqCst)
    }

    /// Gate for the dispatch path: fail closed when not armed.
    pub fn ensure_armed(&self) -> Result<(), fd_core::adapter::AdapterError> {
        if self.is_armed() {
            Ok(())
        } else {
            Err(fd_core::adapter::AdapterError::WritesDisabled)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_disabled_and_fails_closed() {
        let g = LiveWriteGuard::disabled();
        assert!(!g.is_armed());
        assert!(g.ensure_armed().is_err());
    }

    #[test]
    fn arm_enables_then_disarm_relocks() {
        let g = LiveWriteGuard::disabled();
        g.arm();
        assert!(g.is_armed());
        assert!(g.ensure_armed().is_ok());
        g.disarm();
        assert!(g.ensure_armed().is_err());
    }

    #[test]
    fn guard_is_process_state_not_persisted() {
        // A fresh guard (what every process start constructs) is locked,
        // even if a previous process had armed its own instance.
        let previous = LiveWriteGuard::disabled();
        previous.arm();
        assert!(previous.is_armed());
        // A fresh guard (what every process start constructs) is locked —
        // the previous instance's armed state does not carry over.
        let fresh = LiveWriteGuard::disabled();
        assert!(!fresh.is_armed(), "armed state must never persist");
    }
}
