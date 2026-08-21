//! Typed, fail-closed package errors.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum PackageError {
    #[error("package io error: {0}")]
    Io(String),
    #[error("package parse error ({file}): {source_text}")]
    Toml {
        file: &'static str,
        source_text: String,
    },
    #[error("unsupported package schema version {found} (this build supports {supported})")]
    SchemaVersion { found: u32, supported: u32 },
    #[error("package targets runtime API version {found} (this build provides {supported})")]
    RuntimeApiVersion { found: u32, supported: u32 },
    #[error("required field `{field}` is empty or missing")]
    EmptyField { field: &'static str },
    #[error("unknown binding name `{0}` (must match the closed trusted set)")]
    UnknownBindingName(String),
    #[error("unknown role `{0}`")]
    UnknownRole(String),
    #[error("unknown state field `{0}`")]
    UnknownStateField(String),
    #[error("unknown action `{0}` (not in the closed CockpitAction catalog)")]
    UnknownAction(String),
    #[error("duplicate flow id `{0}`")]
    DuplicateFlowId(String),
    #[error("duplicate step id `{step}` in flow `{flow}`")]
    DuplicateStepId { flow: String, step: String },
    #[error("step `{step}` in flow `{flow}` depends on missing step `{dep}`")]
    MissingDependency {
        flow: String,
        step: String,
        dep: String,
    },
    #[error("step `{step}` in flow `{flow}` depends on itself")]
    SelfDependency { flow: String, step: String },
    #[error("dependency cycle in flow `{flow}`: {cycle}")]
    DependencyCycle { flow: String, cycle: String },
    #[error("condition type mismatch on field `{field}` with op `{op}`: {detail}")]
    ConditionTypeMismatch {
        field: String,
        op: String,
        detail: String,
    },
    #[error("invalid value: {0}")]
    InvalidValue(String),
}
