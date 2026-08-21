//! Simulator adapter contract.
//!
//! This trait is the ONLY boundary between the runtime and any simulator.
//! It lives in `fd-core` (not in `fd-runtime`) because the implementation
//! crate (`fd-simconnect`) must not depend on the runtime crate — that would
//! create a dependency cycle given `fd-runtime` consumes the trait.
//! Wiring (who constructs which adapter) is owned by `fd-app`.
//!
//! Critical invariant: there is NO raw write API. Adapters receive only
//! closed [`CockpitAction`] values; arbitrary
//! `set_simvar("whatever", value)` is impossible through this contract.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::actions::CockpitAction;
use crate::telemetry::TelemetrySnapshot;

/// Adapter's self-reported ability to perform an action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Capability {
    /// Adapter implements this action for its bound aircraft/simulator.
    Supported,
    /// Not in this adapter's binding table.
    Unsupported,
    /// Implemented but not currently usable (e.g. not connected).
    Unavailable,
    /// Adapter cannot determine capability (reserved for future live
    /// binding inventory).
    Unknown,
}

impl Capability {
    /// Whether this capability blocks action dispatch (fail-closed).
    /// `Unknown` passes gating: preconditions and verification still apply.
    pub const fn blocks_dispatch(self) -> bool {
        matches!(self, Self::Unsupported | Self::Unavailable)
    }
}

/// Typed adapter errors. `anyhow` is permitted only at the application
/// boundary, never in the runtime.
#[derive(Debug, Error)]
pub enum AdapterError {
    #[error("simulator connection failed: {0}")]
    ConnectionFailed(String),
    #[error("adapter is not connected")]
    NotConnected,
    #[error("simulator write failed: {0}")]
    WriteFailed(String),
    #[error("binding unavailable for action: {0:?}")]
    BindingUnavailable(CockpitAction),
    #[error("action not supported by this adapter")]
    UnsupportedAction,
    #[error("adapter poll produced no data")]
    PollTimeout,
}

/// Minimal simulator abstraction boundary for the Task 1 runtime.
///
/// Capability categories (Task 1 §7): connect/disconnect, read state,
/// execute validated discrete actions, report simulator timing state
/// (timing travels inside [`TelemetrySnapshot::sim_timing`]).
pub trait SimulatorAdapter {
    fn connect(&mut self) -> Result<(), AdapterError>;
    fn disconnect(&mut self);
    fn is_connected(&self) -> bool;

    /// Drain the adapter's receive queue into canonical snapshots.
    /// Empty result is normal (no new data since last poll).
    fn poll(&mut self) -> Result<Vec<TelemetrySnapshot>, AdapterError>;

    /// Self-reported capability for an action.
    fn capability(&self, action: CockpitAction) -> Capability;

    /// Execute a validated discrete action. Success here means the write was
    /// *dispatched*; post-condition verification is the runtime's job.
    fn execute(&mut self, action: CockpitAction) -> Result<(), AdapterError>;
}
