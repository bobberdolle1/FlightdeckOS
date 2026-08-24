//! The verification ladder: how a profile earns the right to act.
//!
//! Maps the spec's trust levels onto one linear enum. The two invariants the
//! spec calls out are encoded here and enforced in tests:
//!
//! * **observed != trusted** — having *seen* state values (`Observed`) says
//!   nothing about whether a package's write bindings are correct;
//! * **correlated != safe to write** — correlating observed values with a
//!   package's claims (`Correlated`) is still evidence gathering, never
//!   authorization.
//!
//! Only [`VerificationState::Trusted`] may execute actions. Everything below
//! it is scaffolding for humans and automated verifiers.

use serde::{Deserialize, Serialize};

/// Linear promotion ladder for an aircraft package/profile.
///
/// `Draft` → `Observed` → `Correlated` → `Documented` → `Tested` → `Trusted`.
#[derive(
    Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
#[serde(rename_all = "snake_case")]
pub enum VerificationState {
    /// Scaffolded from an identity + available field list (Profile Genesis).
    /// Never produced by observation; nothing about it is verified.
    #[default]
    Draft,
    /// A human or tool has confirmed the profile's declared fields exist on
    /// this addon. Existence only — values were not checked against behavior.
    Observed,
    /// Observed values line up with what the profile claims across multiple
    /// samples. Still correlation, not causation: not safe to write.
    Correlated,
    /// The mapping is written down against authoritative documentation
    /// (SDK docs, addon manuals) and every discrepancy resolved or annotated.
    Documented,
    /// Every write binding passed a live simulator validation run
    /// (the same bar as `live_verified = true` in a package manifest).
    Tested,
    /// Fully verified: documented AND live-tested. The only state allowed to
    /// execute cockpit actions.
    Trusted,
}

impl VerificationState {
    /// Gate for the action pipeline: only a fully trusted profile may drive
    /// cockpit actions. Drafts can never execute, no matter who generated
    /// them or how plausible they look.
    pub fn can_execute_actions(self) -> bool {
        self == Self::Trusted
    }

    /// The single next rung of the ladder, or `None` once trusted.
    /// There are no shortcuts: each promotion step must be earned in order.
    pub fn promotion_path(self) -> Option<Self> {
        match self {
            Self::Draft => Some(Self::Observed),
            Self::Observed => Some(Self::Correlated),
            Self::Correlated => Some(Self::Documented),
            Self::Documented => Some(Self::Tested),
            Self::Tested => Some(Self::Trusted),
            Self::Trusted => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const LADDER: [VerificationState; 6] = [
        VerificationState::Draft,
        VerificationState::Observed,
        VerificationState::Correlated,
        VerificationState::Documented,
        VerificationState::Tested,
        VerificationState::Trusted,
    ];

    #[test]
    fn default_is_draft() {
        assert_eq!(VerificationState::default(), VerificationState::Draft);
    }

    #[test]
    fn only_trusted_can_execute_actions() {
        for state in LADDER {
            let expected = state == VerificationState::Trusted;
            assert_eq!(state.can_execute_actions(), expected, "{state:?}");
        }
    }

    #[test]
    fn promotion_path_is_linear_and_terminates_at_trusted() {
        let mut current = VerificationState::Draft;
        for expected in LADDER {
            assert_eq!(current, expected);
            // Trusted is the terminal rung: it has no next step.
            if expected == VerificationState::Trusted {
                break;
            }
            current = current
                .promotion_path()
                .unwrap_or_else(|| panic!("{expected:?} must have a next rung while untrusted"));
        }
        // Walking the whole ladder lands exactly on Trusted, then stops.
        assert_eq!(current, VerificationState::Trusted);
        assert_eq!(current.promotion_path(), None);
        // And there is no way to skip rungs.
        assert_ne!(
            VerificationState::Draft.promotion_path(),
            Some(VerificationState::Trusted)
        );
    }

    #[test]
    fn derived_order_matches_ladder() {
        for pair in LADDER.windows(2) {
            assert!(
                pair[0] < pair[1],
                "{:?} must rank below {:?}",
                pair[0],
                pair[1]
            );
        }
    }

    #[test]
    fn serde_uses_snake_case_names() {
        for (state, name) in [
            (VerificationState::Draft, "draft"),
            (VerificationState::Observed, "observed"),
            (VerificationState::Correlated, "correlated"),
            (VerificationState::Documented, "documented"),
            (VerificationState::Tested, "tested"),
            (VerificationState::Trusted, "trusted"),
        ] {
            assert_eq!(
                serde_json::to_string(&state).unwrap(),
                format!("\"{name}\"")
            );
            let back: VerificationState = serde_json::from_str(&format!("\"{name}\"")).unwrap();
            assert_eq!(back, state);
        }
    }
}
