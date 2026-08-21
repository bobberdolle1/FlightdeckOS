use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioSpec {
    pub name: String,
    pub aircraft_package: String,
}

#[derive(Debug, thiserror::Error)]
pub enum ScenarioSpecError {
    #[error("Parse error: {0}")]
    ParseError(String),
}
