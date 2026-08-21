//! The [`SimulatorAdapter`] implementation over SimConnect.
//!
//! Public surface: exactly the adapter trait. Raw writes stay in
//! `write.rs` (`pub(crate)`) and are reachable only through the closed
//! action catalog lookup in `bindings.rs`.

use fd_core::actions::CockpitAction;
use fd_core::adapter::{AdapterError, Capability, SimulatorAdapter};
use fd_core::telemetry::TelemetrySnapshot;

use crate::bindings::{WritePrimitive, lookup_write};
use crate::client::SimClient;
use crate::mapping::map_telemetry;
use crate::parse::RecvRecord;

/// MSFS SimConnect adapter for the Task 1 runtime.
pub struct SimConnectAdapter {
    client: Option<SimClient>,
    /// Last pause state seen via system events (fallback when the `PAUSED`
    /// simvar is absent from a payload).
    event_paused: Option<bool>,
    /// Last exception code observed (for diagnostics; non-fatal).
    last_exception: Option<u32>,
    pub(crate) connected: bool,
}

impl Default for SimConnectAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl SimConnectAdapter {
    pub fn new() -> Self {
        Self {
            client: None,
            event_paused: None,
            last_exception: None,
            connected: false,
        }
    }

    /// Last exception code reported by the sim, if any.
    pub fn last_exception(&self) -> Option<u32> {
        self.last_exception
    }
}

impl SimulatorAdapter for SimConnectAdapter {
    fn connect(&mut self) -> Result<(), AdapterError> {
        let mut client = SimClient::open("FlightdeckOS")?;
        client.setup()?;
        self.client = Some(client);
        self.connected = true;
        self.event_paused = None;
        Ok(())
    }

    fn disconnect(&mut self) {
        self.client = None;
        self.connected = false;
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    fn poll(&mut self) -> Result<Vec<TelemetrySnapshot>, AdapterError> {
        let Some(client) = self.client.as_mut() else {
            return Err(AdapterError::NotConnected);
        };

        let mut snapshots = Vec::new();
        for record in client.poll() {
            match record {
                RecvRecord::SimObjectData { values } => {
                    let snapshot = map_telemetry(&values, self.event_paused);
                    snapshots.push(snapshot);
                }
                RecvRecord::SystemEventPause(paused) => {
                    self.event_paused = Some(paused);
                }
                RecvRecord::Exception { code } => {
                    self.last_exception = Some(code);
                }
                RecvRecord::Open => {
                    self.connected = true;
                }
                RecvRecord::Quit => {
                    self.connected = false;
                }
                RecvRecord::Ignored => {}
                RecvRecord::Malformed { detail } => {
                    // Malformed records are dropped with a diagnostic; they
                    // never produce state.
                    self.last_exception = Some(u32::MAX - 1);
                    let _ = detail;
                }
            }
        }
        Ok(snapshots)
    }

    fn capability(&self, action: CockpitAction) -> Capability {
        if lookup_write(action).is_none() {
            return Capability::Unsupported;
        }
        if self.connected {
            Capability::Supported
        } else {
            Capability::Unavailable
        }
    }

    fn execute(&mut self, action: CockpitAction) -> Result<(), AdapterError> {
        let Some(client) = self.client.as_ref() else {
            return Err(AdapterError::NotConnected);
        };
        let binding = lookup_write(action).ok_or(AdapterError::UnsupportedAction)?;

        // SAFETY: the client is connected; both primitives are fail-closed
        // (every FFI HRESULT is checked).
        unsafe {
            match binding.primitive {
                WritePrimitive::SimVarWrite { name, unit, value } => {
                    crate::write::write_simvar(client, name, unit, value)
                }
                WritePrimitive::Event { name, param } => {
                    crate::write::fire_event(client, name, param)
                }
            }
        }
    }
}
