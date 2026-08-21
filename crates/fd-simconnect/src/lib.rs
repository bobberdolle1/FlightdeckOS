//! FlightdeckOS SimConnect adapter.
//!
//! Boundary contract: this crate is the ONLY place that knows SimConnect
//! names, L:Var names, MSFS events, and aircraft addon internals. It
//! implements [`fd_core::adapter::SimulatorAdapter`] over a minimal,
//! hand-maintained FFI (`ffi.rs`, dynamic DLL loading — see its decision
//! record for why `simconnect-sys` was rejected).
//!
//! Raw writes exist only in `pub(crate)` functions; the public surface
//! exposes exactly the closed [`CockpitAction`](fd_core::actions::CockpitAction)
//! contract. There is no public `set_simvar(name, value)` anywhere.
//!
//! Runtime requirement: the MSFS SDK client `SimConnect.dll` next to the
//! binary. Without it (or without a running simulator), `connect()` fails
//! with a typed [`AdapterError::ConnectionFailed`] — never a crash.

// The FFI boundary is the one sanctioned unsafe surface of the workspace.
#![allow(unsafe_code)]

pub mod bindings;
// `ffi` is deliberately crate-private: it exposes raw SimConnect function
// pointers, and leaking them would break the closed-action write guarantee
// for downstream crates. The public surface is the typed adapter + bindings.
mod ffi;

#[cfg(windows)]
mod adapter;
#[cfg(windows)]
mod client;
#[cfg(windows)]
mod defs;
#[cfg(windows)]
mod mapping;
#[cfg(windows)]
mod parse;
#[cfg(windows)]
mod write;

#[cfg(windows)]
pub use adapter::SimConnectAdapter;

#[cfg(not(windows))]
use fd_core::actions::CockpitAction;
#[cfg(not(windows))]
use fd_core::adapter::{AdapterError, Capability, SimulatorAdapter};
#[cfg(not(windows))]
use fd_core::telemetry::TelemetrySnapshot;

/// Non-Windows stub: SimConnect is a Windows-only runtime.
#[cfg(not(windows))]
pub struct SimConnectAdapter;

#[cfg(not(windows))]
impl SimulatorAdapter for SimConnectAdapter {
    fn connect(&mut self) -> Result<(), AdapterError> {
        Err(AdapterError::ConnectionFailed(
            "SimConnect is available only on Windows".into(),
        ))
    }
    fn disconnect(&mut self) {}
    fn is_connected(&self) -> bool {
        false
    }
    fn poll(&mut self) -> Result<Vec<TelemetrySnapshot>, AdapterError> {
        Err(AdapterError::NotConnected)
    }
    fn capability(&self, _action: CockpitAction) -> Capability {
        Capability::Unavailable
    }
    fn execute(&mut self, _action: CockpitAction) -> Result<(), AdapterError> {
        Err(AdapterError::NotConnected)
    }
}
