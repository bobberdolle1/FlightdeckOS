//! FlightdeckOS minimal deterministic SOP/flow engine.
//!
//! Owns procedure SEMANTICS over validated aircraft packages:
//!
//! * flow/step graph resolution (duplicate ids, unknown roles / state
//!   fields / actions, missing or self dependencies, dependency cycles —
//!   all fail-closed);
//! * the [`engine::FlowEngine`]: deterministic per-snapshot evaluation of
//!   observe steps and action steps;
//! * action steps delegate to the EXISTING runtime action pipeline via
//!   two-phase submit; `Dispatched` never completes a step — only the
//!   observed post-condition (`ActionVerified`) does.
//!
//! This crate knows nothing about SimConnect, L:Vars, raw handles, HTTP,
//! UI, or AI. It receives typed snapshots and emits typed requests/events.

pub mod engine;
pub mod package;

pub use engine::{FlowEngine, FlowStatus, SopEvent, StepStatus};
pub use package::{StepKind, ValidatedPackage, load_package};
