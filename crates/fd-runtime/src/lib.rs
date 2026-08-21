//! FlightdeckOS runtime: the deterministic loop that turns raw adapter
//! snapshots into sequenced, traced, phase-aware state — and executes
//! cockpit actions through the closed catalog with post-condition
//! verification.
//!
//! Dependency direction: `fd-runtime → fd-core` only. The runtime consumes
//! [`fd_core::adapter::SimulatorAdapter`]; it never references a concrete
//! simulator implementation.

#![forbid(unsafe_code)]

pub mod executor;
pub mod ingest;
pub mod phase_tracker;
pub mod replay;
pub mod runtime;
pub mod session;
pub mod trace;

pub use executor::{ActionExecutor, ActionRecord, DeadlineTicks};
pub use replay::{ReplayAdapter, ReplayStep};
pub use runtime::{Runtime, RuntimeError, TickStats};
pub use session::{Session, SessionId};
pub use trace::{TRACE_VERSION, TraceEvent, TraceSink, TraceVersion, TraceWriter};
