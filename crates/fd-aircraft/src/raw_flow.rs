//! Raw (serde) shapes for flow/procedure files, shared by the package
//! loader and the SOP semantic resolver.

use serde::{Deserialize, Serialize};

/// Raw condition as written in TOML (`[steps.condition]`).
pub use crate::condition::RawCondition as RawConditionToml;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RawFlowDef {
    pub id: String,
    pub title: String,
    /// Architecture-scope note: this package demonstrates primitives and is
    /// NOT a complete certified airline procedure.
    #[serde(default)]
    pub scope_note: String,
    pub steps: Vec<RawStep>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RawStep {
    pub id: String,
    pub actor: String,
    #[serde(default)]
    pub depends_on: Vec<String>,
    #[serde(flatten)]
    pub body: RawStepBody,
}

/// Two step kinds for Task 2:
/// * `observe` — completes when a typed condition evaluates True;
/// * `action`  — requests a closed CockpitAction; completes only when the
///   runtime action pipeline observes its post-condition.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RawStepBody {
    Observe { condition: RawConditionToml },
    Action { action: String },
}
