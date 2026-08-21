//! FlightdeckOS scenario engine.
//!
//! Deterministic headless execution of flight scenarios against the
//! virtual simulator:
//!
//! * TOML [`spec::ScenarioSpec`] (human-readable, typed);
//! * runner wires package + virtual simulator + runtime + mission
//!   controller + FDR/FDM;
//! * machine-checkable assertions produce a [`report::ScenarioResult`].
//!
//! Every result is labeled HEADLESS VIRTUAL TEST — it never proves real
//! simulator bindings or aircraft performance.

pub mod report;
pub mod runner;
pub mod spec;

pub use report::{ScenarioReport, ScenarioResult};
pub use spec::{ScenarioSpec, ScenarioSpecError};
