//! FlightdeckOS aircraft layer.
//!
//! Owns everything aircraft-specific that the runtime core must not know:
//!
//! * the versioned **aircraft package** format (TOML, fail-closed loading);
//! * the closed [`StateField`] registry packages may reference;
//! * the typed [`Condition`] model and its tri-state evaluator
//!   (`True / False / Unknown` — missing data is never guessed);
//! * crew-role primitives ([`Role`]);
//! * the A32NX action catalog (pre- + post-conditions) consumed by the
//!   runtime's action pipeline.
//!
//! Dependency rules: this crate depends only on `fd-core`. It knows nothing
//! about SimConnect FFI, transport, SOP execution, or AI. Packages can never
//! introduce arbitrary simulator writes: they may reference only the closed
//! [`fd_core::actions::CockpitAction`] set through canonical names, resolved
//! by trusted code in this crate.

pub mod bindings_meta;
pub mod catalog;
pub mod condition;
pub mod error;
pub mod manifest;
pub mod raw_flow;
pub mod roles;
pub mod state_field;

pub use condition::{Condition, RawCondition, TriBool};
pub use error::PackageError;
pub use manifest::{PackageManifest, RUNTIME_API_VERSION, SCHEMA_VERSION, load_manifest};
pub use raw_flow::{RawFlowDef, RawStep, RawStepBody};
pub use roles::Role;
pub use state_field::{StateField, ValueType};
