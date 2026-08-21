//! Typed errors for AI Crew Runtime and Model Providers.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum AiCrewError {
    #[error("AI model provider error: {0}")]
    ProviderError(String),

    #[error("AI model provider timed out after {0} seconds")]
    Timeout(u64),

    #[error("Unknown tool requested by model: {0}")]
    UnknownTool(String),

    #[error("Tool execution failed ({tool}): {message}")]
    ToolExecutionFailed { tool: String, message: String },

    #[error("Malformed model output: {0}")]
    MalformedOutput(String),

    #[error("AI crew runtime is currently offline or uninitialized")]
    RuntimeUnavailable,
}
