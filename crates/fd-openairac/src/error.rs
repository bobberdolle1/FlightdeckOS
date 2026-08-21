//! Typed errors for OpenAIRAC client and gateway integration.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum OpenAiracError {
    #[error("OpenAIRAC gateway HTTP request failed: {0}")]
    HttpError(#[from] reqwest::Error),

    #[error("JSON deserialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    #[error("Schema version mismatch: expected {expected}, received {received}")]
    SchemaVersionMismatch { expected: String, received: String },

    #[error("Gateway returned API error (status {status}): {message}")]
    ApiError { status: u16, message: String },

    #[error("OpenAIRAC gateway is offline or unreachable at {0}")]
    GatewayUnreachable(String),

    #[error("No active flight plan or execution session on OpenAIRAC gateway")]
    NoActiveFlight,

    #[error("Telemetry is currently stale ({age_ms} ms age)")]
    TelemetryStale { age_ms: u64 },

    #[error("Terminal procedures for {0} require official AIP source dataset (SOURCE_REQUIRED)")]
    SourceRequired(String),
}
