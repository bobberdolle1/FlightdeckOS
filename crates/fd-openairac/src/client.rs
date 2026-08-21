//! HTTP Client communicating with OpenAIRAC 3.2 Gateway.

use reqwest::blocking::Client;
use std::time::Duration;

use crate::error::OpenAiracError;
use crate::types::{
    EXPECTED_COMPACT_SCHEMA, EXPECTED_SNAPSHOT_V2_SCHEMA, OpenAiracArrivalBrief,
    OpenAiracCompactSnapshot, OpenAiracDepartureBrief, OpenAiracEvent, OpenAiracResolvedIdentity,
    OpenAiracSnapshotV2,
};

pub const DEFAULT_GATEWAY_URL: &str = "http://127.0.0.1:8989/api/openairac/v1";

/// Dedicated OpenAIRAC Gateway client.
pub struct OpenAiracClient {
    base_url: String,
    http: Client,
}

impl Default for OpenAiracClient {
    fn default() -> Self {
        Self::new(DEFAULT_GATEWAY_URL)
    }
}

impl OpenAiracClient {
    pub fn new(base_url: impl Into<String>) -> Self {
        let http = Client::builder()
            .timeout(Duration::from_secs(3))
            .build()
            .unwrap_or_else(|_| Client::new());

        Self {
            base_url: base_url.into().trim_end_matches('/').to_string(),
            http,
        }
    }

    /// Check if OpenAIRAC Gateway is responsive.
    pub fn check_health(&self) -> Result<bool, OpenAiracError> {
        let url = format!("{}/status", self.base_url);
        let resp = self.http.get(&url).send()?;
        Ok(resp.status().is_success())
    }

    /// Fetch full authoritative Flightdeck Snapshot v2 with schema negotiation.
    pub fn get_snapshot(&self) -> Result<OpenAiracSnapshotV2, OpenAiracError> {
        let url = format!("{}/flightdeck/snapshot", self.base_url);
        let resp = self.http.get(&url).send()?;
        if !resp.status().is_success() {
            return Err(OpenAiracError::ApiError {
                status: resp.status().as_u16(),
                message: resp.text().unwrap_or_default(),
            });
        }

        let snapshot: OpenAiracSnapshotV2 = resp.json()?;
        if snapshot.schema_version != EXPECTED_SNAPSHOT_V2_SCHEMA {
            return Err(OpenAiracError::SchemaVersionMismatch {
                expected: EXPECTED_SNAPSHOT_V2_SCHEMA.to_string(),
                received: snapshot.schema_version,
            });
        }

        Ok(snapshot)
    }

    /// Fetch context-budget Compact AI Snapshot v1.
    pub fn get_compact_snapshot(&self) -> Result<OpenAiracCompactSnapshot, OpenAiracError> {
        let url = format!("{}/flightdeck/compact", self.base_url);
        let resp = self.http.get(&url).send()?;
        if !resp.status().is_success() {
            return Err(OpenAiracError::ApiError {
                status: resp.status().as_u16(),
                message: resp.text().unwrap_or_default(),
            });
        }

        let compact: OpenAiracCompactSnapshot = resp.json()?;
        if compact.schema_version != EXPECTED_COMPACT_SCHEMA {
            return Err(OpenAiracError::SchemaVersionMismatch {
                expected: EXPECTED_COMPACT_SCHEMA.to_string(),
                received: compact.schema_version,
            });
        }

        Ok(compact)
    }

    /// Poll monotonic flight lifecycle events.
    pub fn get_events(&self, since_id: Option<u64>) -> Result<Vec<OpenAiracEvent>, OpenAiracError> {
        let mut url = format!("{}/flightdeck/events", self.base_url);
        if let Some(id) = since_id {
            url = format!("{}?since_id={}", url, id);
        }

        let resp = self.http.get(&url).send()?;
        if !resp.status().is_success() {
            return Err(OpenAiracError::ApiError {
                status: resp.status().as_u16(),
                message: resp.text().unwrap_or_default(),
            });
        }

        #[derive(serde::Deserialize)]
        struct EventsWrapper {
            events: Vec<OpenAiracEvent>,
        }

        let wrapper: EventsWrapper = resp.json()?;
        Ok(wrapper.events)
    }

    /// Fetch departure briefing.
    pub fn get_departure_brief(&self) -> Result<OpenAiracDepartureBrief, OpenAiracError> {
        let url = format!("{}/flightdeck/departure-brief", self.base_url);
        let resp = self.http.get(&url).send()?;
        if resp.status().as_u16() == 404 {
            return Err(OpenAiracError::NoActiveFlight);
        }
        if !resp.status().is_success() {
            return Err(OpenAiracError::ApiError {
                status: resp.status().as_u16(),
                message: resp.text().unwrap_or_default(),
            });
        }

        Ok(resp.json()?)
    }

    /// Fetch arrival briefing with strict SOURCE_REQUIRED verification.
    pub fn get_arrival_brief(&self) -> Result<OpenAiracArrivalBrief, OpenAiracError> {
        let url = format!("{}/flightdeck/arrival-brief", self.base_url);
        let resp = self.http.get(&url).send()?;
        if resp.status().as_u16() == 404 {
            return Err(OpenAiracError::NoActiveFlight);
        }
        if !resp.status().is_success() {
            return Err(OpenAiracError::ApiError {
                status: resp.status().as_u16(),
                message: resp.text().unwrap_or_default(),
            });
        }

        Ok(resp.json()?)
    }

    /// Resolve airport multi-identity without collapsing provider semantics.
    pub fn resolve_identity(
        &self,
        ident: &str,
    ) -> Result<OpenAiracResolvedIdentity, OpenAiracError> {
        let url = format!("{}/flightdeck/identity/{}", self.base_url, ident.trim());
        let resp = self.http.get(&url).send()?;
        if !resp.status().is_success() {
            return Err(OpenAiracError::ApiError {
                status: resp.status().as_u16(),
                message: resp.text().unwrap_or_default(),
            });
        }

        Ok(resp.json()?)
    }
}
