//! FlightdeckOS canonical core.
//!
//! Dependency rule (Task 1): this crate owns the canonical state types and the
//! simulator adapter *contract*. It MUST NOT contain any simulator-specific
//! knowledge (no SimConnect, no L:Var names, no MSFS events, no aircraft
//! addon internals). Aircraft-specific *semantic* state (e.g. A32NX APU N)
//! is allowed here only as typed logical values; the raw binding names live
//! exclusively in `fd-simconnect`.
//!
//! Contains no unsafe code and no FFI.

#![forbid(unsafe_code)]

pub mod actions;
pub mod adapter;
pub mod capability;
pub mod delta;
pub mod events;
pub mod fplan;
pub mod geo;
pub mod identity;
pub mod phase;
pub mod telemetry;
pub mod units;
