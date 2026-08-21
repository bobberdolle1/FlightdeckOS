//! Closed action catalog types: the ONLY shape in which a cockpit action may
//! enter the runtime.
//!
//! Upper layers may never see or produce raw simulator variable names. An
//! action is a small typed enum ([`CockpitAction`]); its simulator-specific
//! realization lives below the action executor (in `fd-simconnect`), keyed by
//! [`ActionKind`].

use serde::{Deserialize, Serialize};

use crate::telemetry::{NavLogoMode, SimTimestamp};

/// Monotonic action identifier within a session.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ActionId(pub u64);

/// Who requested an action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Actor {
    /// A human operator.
    User,
    /// The runtime itself (scripted/replay injection).
    Runtime,
}

/// A discrete two-state switch position.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SwitchPosition {
    On,
    Off,
}

/// The closed set of cockpit actions Task 1 understands.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CockpitAction {
    /// Beacon light switch (standard `LIGHT BEACON`).
    SetBeacon(SwitchPosition),
    /// A32NX NAV/LOGO light switch (documented FBW enum).
    SetNavLogo(NavLogoMode),
}

impl CockpitAction {
    pub const fn kind(self) -> ActionKind {
        match self {
            Self::SetBeacon(_) => ActionKind::SetBeacon,
            Self::SetNavLogo(_) => ActionKind::SetNavLogo,
        }
    }
}

/// Catalog key: the *kind* of action, independent of its parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActionKind {
    SetBeacon,
    SetNavLogo,
}

/// Outcome of an action request. "Success" means the *post-condition was
/// observed in simulator state* — never merely that a write call returned OK.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActionStatus {
    Requested,
    Validated,
    Dispatched,
    Verified,
    Rejected(ActionRejectionReason),
    TimedOut,
    Failed(ActionFailure),
}

/// Why an action was rejected *before* dispatch (fail-closed).
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActionRejectionReason {
    /// Not present in the action catalog at all.
    UnknownAction,
    /// Adapter declares it unsupported.
    UnsupportedByAdapter,
    /// Adapter not connected / binding unavailable.
    AdapterUnavailable,
    /// A named precondition failed. Payload: precondition id.
    PreconditionFailed(String),
}

/// Why a dispatched action failed *after* dispatch.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActionFailure {
    /// The simulator write itself failed.
    WriteFailed(String),
    /// Write succeeded but the post-condition was not observed in time.
    VerificationTimeout,
}

/// A named precondition over the current canonical snapshot.
#[derive(Clone, Copy)]
pub struct PreconditionDef {
    pub id: &'static str,
    /// Returns `Err(reason)` when the precondition is not satisfied.
    /// Preconditions run fail-closed: they may only gate, never guess.
    pub check: fn(&crate::telemetry::TelemetrySnapshot) -> Result<(), &'static str>,
}

/// Catalog entry: which actions of a kind are permitted and under which
/// preconditions.
#[derive(Clone)]
pub struct CatalogEntry {
    pub kind: ActionKind,
    pub preconditions: Vec<PreconditionDef>,
}

/// The closed action catalog. Lookup is by [`ActionKind`]; an unknown kind is
/// rejected by the executor before any adapter interaction.
#[derive(Clone, Default)]
pub struct ActionCatalog {
    pub entries: Vec<CatalogEntry>,
}

impl ActionCatalog {
    pub fn lookup(&self, kind: ActionKind) -> Option<&CatalogEntry> {
        self.entries.iter().find(|e| e.kind == kind)
    }
}

/// An action request with its identity.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ActionRequest {
    pub id: ActionId,
    pub action: CockpitAction,
    pub actor: Actor,
    /// Timestamp of the request (simulator time when known, injected in replay).
    pub at: SimTimestamp,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn catalog_lookup_by_kind() {
        let cat = ActionCatalog {
            entries: vec![CatalogEntry {
                kind: ActionKind::SetBeacon,
                preconditions: Vec::new(),
            }],
        };
        assert!(cat.lookup(ActionKind::SetBeacon).is_some());
        assert!(cat.lookup(ActionKind::SetNavLogo).is_none());
    }

    #[test]
    fn action_kind_discriminates_parameters() {
        assert_eq!(
            CockpitAction::SetBeacon(SwitchPosition::On).kind(),
            ActionKind::SetBeacon
        );
        assert_eq!(
            CockpitAction::SetBeacon(SwitchPosition::Off).kind(),
            ActionKind::SetBeacon
        );
    }
}
