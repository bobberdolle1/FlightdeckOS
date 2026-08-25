//! FlightdeckOS X-Plane 12 live simulator adapter.
//!
//! Transport: X-Plane's NATIVE UDP dataref API (port 49000) —
//! * `RREF0` subscribe: simulator streams dataref values to our socket;
//! * `DREF0` write: allowlisted autopilot-target datarefs only;
//! * `CMND0` command: allowlisted autopilot-mode commands only.
//!
//! No plugin is required. There is deliberately NO generic "write dataref"
//! surface outside this crate: every write path is a named, typed function
//! against a fixed allowlist (Task 4 §10). The AI/crew layer has no access
//! to this crate's write path.
//!
//! LIVE means LIVE: telemetry comes only from packets received from a
//! genuinely running X-Plane instance. When packets stop, the adapter
//! reports a typed disconnect — it never substitutes virtual state.

pub mod adapter;
pub mod bridge;
pub mod client;
pub mod guard;
pub mod protocol;
pub mod refs;
mod webapi;

pub use adapter::{BEACON_OFF_COMMAND, BEACON_ON_COMMAND, XPlaneAdapter, XPlaneConfig};
pub use client::XPlaneUdpClient;
pub use guard::LiveWriteGuard;
pub use refs::DataRefId;
