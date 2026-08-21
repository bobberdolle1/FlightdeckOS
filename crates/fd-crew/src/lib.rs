//! FlightdeckOS AI Crew Runtime & Tool Calling Engine.
//!
//! Integrates deterministic OpenAIRAC navigation truth, SOP procedure tracking,
//! and pluggable AI model providers for in-flight crew reasoning and assistance.

pub mod error;
pub mod provider;
pub mod runtime;
pub mod tools;

pub use error::AiCrewError;
pub use provider::{AiCrewPrompt, AiCrewResponse, AiModelProvider, DeterministicAiProvider};
pub use runtime::AiCrewRuntime;
pub use tools::{CrewToolDefinition, CrewToolRegistry, ProposedAction, ToolEvidence};
