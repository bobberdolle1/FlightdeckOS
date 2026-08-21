//! Runtime session: identity and the monotonic event sequence counter.
//!
//! The sequence counter is the ONLY ordering mechanism inside a session.
//! It is injectable (see [`Session::with_id`]) so replay runs are
//! byte-identical across processes and wall-clock times.

use fd_core::events::EventSeq;

/// Session identity. Injected in replay/tests; derived from wall time in live
/// mode (live sessions are not expected to be deterministic).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct SessionId(pub u64);

/// Session state: identity + monotonic sequence counter.
#[derive(Debug)]
pub struct Session {
    pub id: SessionId,
    next_seq: u64,
}

impl Session {
    pub const fn with_id(id: SessionId) -> Self {
        Self { id, next_seq: 0 }
    }

    /// Allocate the next monotonic sequence number.
    ///
    /// Panics on overflow (u64): unreachable in practice, and a panic is
    /// preferrable to silent non-monotonicity.
    pub fn next_seq(&mut self) -> EventSeq {
        let n = self.next_seq;
        self.next_seq = self
            .next_seq
            .checked_add(1)
            .expect("event sequence overflow: monotonicity would be violated");
        EventSeq::new(n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seq_is_strictly_monotonic() {
        let mut s = Session::with_id(SessionId(7));
        let a = s.next_seq();
        let b = s.next_seq();
        let c = s.next_seq();
        assert!(a < b && b < c);
    }

    #[test]
    fn deterministic_seed_gives_deterministic_seq() {
        let mut s1 = Session::with_id(SessionId(0));
        let mut s2 = Session::with_id(SessionId(0));
        for _ in 0..100 {
            assert_eq!(s1.next_seq(), s2.next_seq());
        }
    }
}
