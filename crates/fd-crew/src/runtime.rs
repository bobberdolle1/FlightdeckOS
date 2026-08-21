//! AI Crew Runtime: manages conversation history, context updates, and tool dispatch.

use std::sync::Arc;

use fd_openairac::context::CrewFlightContext;
use fd_openairac::types::OpenAiracSnapshotV2;

use crate::error::AiCrewError;
use crate::provider::{AiCrewPrompt, AiCrewResponse, AiModelProvider, DeterministicAiProvider};

/// In-flight AI Crew Runtime.
pub struct AiCrewRuntime {
    provider: Arc<dyn AiModelProvider>,
    context: Option<CrewFlightContext>,
    conversation_history: Vec<(String, String)>,
    system_prompt: String,
}

impl Default for AiCrewRuntime {
    fn default() -> Self {
        Self::new(Arc::new(DeterministicAiProvider))
    }
}

impl AiCrewRuntime {
    pub fn new(provider: Arc<dyn AiModelProvider>) -> Self {
        Self {
            provider,
            context: None,
            conversation_history: Vec::new(),
            system_prompt: "You are the FlightdeckOS AI Co-Pilot / Crew Assistant. Answer all aviation questions using deterministic OpenAIRAC tool facts. Never invent or hallucinate navdata, procedures, or flight state.".to_string(),
        }
    }

    /// Update runtime state from latest OpenAIRAC Snapshot v2.
    pub fn update_from_snapshot(&mut self, snapshot: &OpenAiracSnapshotV2) {
        self.context = Some(CrewFlightContext::from_snapshot_v2(snapshot));
    }

    /// Update runtime state directly from CrewFlightContext.
    pub fn set_context(&mut self, context: CrewFlightContext) {
        self.context = Some(context);
    }

    /// Get current crew flight context if available.
    pub fn context(&self) -> Option<&CrewFlightContext> {
        self.context.as_ref()
    }

    /// Ask a question to the AI crew runtime.
    pub fn ask(&mut self, user_query: &str) -> Result<AiCrewResponse, AiCrewError> {
        let ctx = self
            .context
            .as_ref()
            .ok_or(AiCrewError::RuntimeUnavailable)?;

        let prompt = AiCrewPrompt {
            system_prompt: self.system_prompt.clone(),
            user_query: user_query.to_string(),
            conversation_history: self.conversation_history.clone(),
            flight_context_compact: None,
        };

        let response = self.provider.generate_response(&prompt, ctx)?;

        // Maintain bounded conversation memory
        self.conversation_history
            .push((user_query.to_string(), response.message.clone()));
        if self.conversation_history.len() > 20 {
            self.conversation_history.remove(0);
        }

        Ok(response)
    }

    /// Clear conversation history without affecting flight state.
    pub fn clear_history(&mut self) {
        self.conversation_history.clear();
    }
}
