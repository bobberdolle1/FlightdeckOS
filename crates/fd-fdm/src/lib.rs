//! FlightdeckOS flight-data layer.
//!
//! Two deliberately separate concepts (Task 3 §14):
//!
//! * **Trace** (fd-runtime): what FlightdeckOS DID — actions, validations,
//!   procedure transitions. Technical audit trail.
//! * **FDR** (this crate): what the AIRCRAFT did — a deterministic stream of
//!   canonical state samples plus attached events (phase changes, actions).
//!   Analysis input.
//!
//! On top of the FDR stream:
//! * [`fdm`] — development-grade FDM/FOQA-style event detection with named,
//!   configurable DEVELOPMENT thresholds (not airline policy);
//! * [`qoa`] — Quality of Approach measurements (unknown input → unknown
//!   metric, never zero);
//! * [`qol`] — Quality of Landing touchdown metrics.

pub mod fdm;
pub mod fdr;
pub mod plan_replay;
pub mod qoa;
pub mod qol;
pub mod session;
pub mod summary;
