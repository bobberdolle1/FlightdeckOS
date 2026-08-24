//! X-Plane Local Web API v3 client (REST, session-scoped resource ids).
//!
//! TRUST BOUNDARY (spec §7): this module is adapter-internal. It must
//! never be re-exported as a public "execute command by name" or "set
//! dataref by name" surface — upper layers speak only typed
//! [`fd_core::actions::CockpitAction`]s through the closed catalog.
//!
//! Resource ids are SESSION-SCOPED (spec §8): the numeric id of a
//! dataref/command is stable within one simulator session (even across
//! aircraft reloads) but NOT across simulator restarts. Ids are cached
//! per session and invalidated explicitly; the STABLE identity is the
//! resource NAME, which is re-resolved on every new session.
//!
//! Wire format (developer.x-plane.com/article/x-plane-web-api/):
//! * `GET /api/capabilities` → `{api:{versions:[...]}, x-plane:{version}}`
//! * `GET /api/v3/{datarefs|commands}?filter[name]=<exact>` → `{data:[...]}`
//! * `GET /api/v3/datarefs/{id}/value` → `{data: <value>}`
//! * `PATCH /api/v3/datarefs/{id}/value` body `{"data": v}`
//! * command activation: `POST /api/v3/command/{id}/activate` (v2+
//!   singular path, required `{"duration": s}` body — observed live on
//!   X-Plane 12.4.3; the plural `/commands/.../activate` route does not
//!   exist and returns a bare drogon 404)
//! * errors: HTTP != 2xx with `{error_code, error_message}`

use std::collections::HashMap;

use serde::Deserialize;

/// Loopback-only default (spec §30): never broaden without a separate
/// security design.
pub const DEFAULT_BASE_URL: &str = "http://127.0.0.1:8086";

/// Simulator-reported capabilities (`GET /api/capabilities`).
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct Capabilities {
    pub api: ApiVersions,
    #[serde(rename = "x-plane")]
    pub x_plane: XPlaneVersion,
}

#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct ApiVersions {
    #[serde(default)]
    pub versions: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct XPlaneVersion {
    pub version: String,
}

/// A dataref or command resource as returned by enumeration.
#[allow(dead_code)] // verification-data path for the next action lane; unit-tested
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct DatarefResource {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub value_type: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct CommandResource {
    pub id: u64,
    pub name: String,
    #[serde(default)]
    pub description: Option<String>,
}

#[allow(dead_code)] // verification-data path for the next action lane; unit-tested
#[derive(Debug, Clone, PartialEq)]
pub enum DatarefValue {
    Number(f64),
    Int(i64),
    Text(String),
    Array(Vec<DatarefValue>),
    /// Present-but-null value (e.g. unreadable data refs).
    Null,
}

impl DatarefValue {
    /// Numeric view (float/double/int); `None` for non-numeric kinds.
    #[allow(dead_code)]
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Self::Number(v) => Some(*v),
            Self::Int(v) => Some(*v as f64),
            _ => None,
        }
    }
}

/// Typed transport errors (spec §16) — never a bare String.
#[derive(Debug, thiserror::Error)]
pub enum WebApiError {
    /// The API answered with a structured error payload.
    #[error("web api error [{code}] (http {http}): {message}")]
    Api {
        code: String,
        message: String,
        http: u16,
    },
    /// Could not reach the simulator (connection refused/timeout/DNS).
    #[error("web api transport failure: {0}")]
    Transport(String),
    /// The response was not the JSON shape the API promises.
    #[error("web api malformed response: {0}")]
    Malformed(String),
    /// The requested resource name does not exist in this session.
    #[error("resource not found: {0}")]
    ResourceNotFound(String),
    /// A cached session id was rejected — the simulator session changed.
    #[error("resource session expired (simulator restarted?): {0}")]
    SessionExpired(String),
    /// The simulator does not support the required API version.
    #[error("unsupported api version: required {0}, available {1}")]
    UnsupportedVersion(String, String),
    /// Incoming Traffic is disabled server-side (HTTP 403 per spec).
    #[error("web api disabled by simulator security policy (403)")]
    Disabled,
}

/// Minimal HTTP surface so tests can inject deterministic faults
/// (REST 500, timeout, malformed body) without a network (spec §34).
pub trait WebApiTransport {
    #[allow(dead_code)] // convenience for future GET-only probes
    fn get(&self, path: &str) -> Result<u16, WebApiError>;
    /// Perform a request, returning (status, body). `body` is `None` for
    /// body-less requests.
    fn request(
        &self,
        method: HttpMethod,
        path: &str,
        body: Option<&str>,
    ) -> Result<(u16, String), WebApiError>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HttpMethod {
    Get,
    Post,
    Patch,
}

/// Real transport over `reqwest` (blocking, rustls), loopback by default.
pub struct HttpTransport {
    client: reqwest::blocking::Client,
    base_url: String,
}

impl HttpTransport {
    pub fn new(base_url: &str) -> Result<Self, WebApiError> {
        let client = reqwest::blocking::Client::builder()
            // Bounded connect + total time (Task 6 §45): a wedged server
            // must surface as an error, never an unbounded hang.
            .connect_timeout(std::time::Duration::from_secs(2))
            .timeout(std::time::Duration::from_secs(5))
            .build()
            .map_err(|e| WebApiError::Transport(e.to_string()))?;
        Ok(Self {
            client,
            base_url: base_url.trim_end_matches('/').to_string(),
        })
    }
}

impl WebApiTransport for HttpTransport {
    fn get(&self, path: &str) -> Result<u16, WebApiError> {
        self.request(HttpMethod::Get, path, None).map(|(s, _)| s)
    }

    fn request(
        &self,
        method: HttpMethod,
        path: &str,
        body: Option<&str>,
    ) -> Result<(u16, String), WebApiError> {
        let url = format!("{}{}", self.base_url, path);
        let mut req = match method {
            HttpMethod::Get => self.client.get(&url),
            HttpMethod::Post => self.client.post(&url),
            HttpMethod::Patch => self.client.patch(&url),
        }
        .header("Accept", "application/json");
        if let Some(b) = body {
            req = req
                .header("Content-Type", "application/json")
                .body(b.to_string());
        }
        let resp = req
            .send()
            .map_err(|e| WebApiError::Transport(e.to_string()))?;
        let status = resp.status().as_u16();
        let text = resp
            .text()
            .map_err(|e| WebApiError::Transport(e.to_string()))?;
        Ok((status, text))
    }
}

/// Session cache: stable NAME → current-session numeric id.
#[derive(Debug, Default, Clone)]
#[allow(dead_code)] // session introspection used by tests + future lanes
pub struct ResourceSession {
    datarefs: HashMap<String, u64>,
    commands: HashMap<String, u64>,
}

impl ResourceSession {
    pub fn cached_command(&self, name: &str) -> Option<u64> {
        self.commands.get(name).copied()
    }

    pub fn cached_dataref(&self, name: &str) -> Option<u64> {
        self.datarefs.get(name).copied()
    }

    fn remember_command(&mut self, name: &str, id: u64) {
        self.commands.insert(name.to_string(), id);
    }

    fn remember_dataref(&mut self, name: &str, id: u64) {
        self.datarefs.insert(name.to_string(), id);
    }

    #[allow(dead_code)] // session introspection for tests/diagnostics
    pub fn len(&self) -> usize {
        self.datarefs.len() + self.commands.len()
    }

    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

#[derive(Debug, Deserialize)]
struct Envelope<T> {
    #[serde(default = "Vec::new")]
    data: Vec<T>,
}

#[derive(Debug, Deserialize)]
struct SingleEnvelope {
    data: serde_json::Value,
}

#[derive(Debug, Deserialize)]
struct ErrorPayload {
    error_code: String,
    #[serde(default)]
    error_message: String,
}

/// X-Plane Local Web API client with per-session resource resolution.
pub struct WebApiClient<T: WebApiTransport> {
    transport: T,
    session: ResourceSession,
}

impl<T: WebApiTransport> WebApiClient<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            session: ResourceSession::default(),
        }
    }

    #[allow(dead_code)]
    pub fn session(&self) -> &ResourceSession {
        &self.session
    }

    /// Simulator restart / new session: all cached numeric ids are dead.
    /// Names are stable; they are resolved again on next use.
    pub fn invalidate_session(&mut self) {
        self.session = ResourceSession::default();
    }

    /// Query simulator capabilities and verify v3 support.
    pub fn capabilities(&self) -> Result<Capabilities, WebApiError> {
        let (status, body) = self
            .transport
            .request(HttpMethod::Get, "/api/capabilities", None)?;
        let caps: Capabilities = parse_ok(status, &body)?;
        if !caps.api.versions.iter().any(|v| v == "v3") {
            return Err(WebApiError::UnsupportedVersion(
                "v3".into(),
                format!("{:?}", caps.api.versions),
            ));
        }
        Ok(caps)
    }

    /// Resolve a command NAME to the current-session numeric id
    /// (cached; the name is the stable identity, never the id).
    pub fn resolve_command(&mut self, name: &str) -> Result<u64, WebApiError> {
        if let Some(id) = self.session.cached_command(name) {
            return Ok(id);
        }
        let path = format!("/api/v3/commands?filter[name]={}", urlencode(name));
        let (status, body) = self.transport.request(HttpMethod::Get, &path, None)?;
        let list: Envelope<CommandResource> = parse_ok(status, &body)?;
        let cmd = list
            .data
            .into_iter()
            .find(|c| c.name == name)
            .ok_or_else(|| WebApiError::ResourceNotFound(name.to_string()))?;
        self.session.remember_command(name, cmd.id);
        Ok(cmd.id)
    }

    /// Resolve a dataref NAME to the current-session numeric id.
    #[allow(dead_code)]
    pub fn resolve_dataref(&mut self, name: &str) -> Result<u64, WebApiError> {
        if let Some(id) = self.session.cached_dataref(name) {
            return Ok(id);
        }
        let path = format!("/api/v3/datarefs?filter[name]={}", urlencode(name));
        let (status, body) = self.transport.request(HttpMethod::Get, &path, None)?;
        let list: Envelope<DatarefResource> = parse_ok(status, &body)?;
        let dr = list
            .data
            .into_iter()
            .find(|d| d.name == name)
            .ok_or_else(|| WebApiError::ResourceNotFound(name.to_string()))?;
        self.session.remember_dataref(name, dr.id);
        Ok(dr.id)
    }

    /// Read a dataref value by NAME (resolving through the session cache).
    #[allow(dead_code)]
    pub fn read_dataref(&mut self, name: &str) -> Result<DatarefValue, WebApiError> {
        let id = self.resolve_dataref(name)?;
        let path = format!("/api/v3/datarefs/{id}/value");
        let (status, body) = self.transport.request(HttpMethod::Get, &path, None)?;
        let env: SingleEnvelope = parse_ok(status, &body)?;
        Ok(json_value_to_dataref(env.data))
    }

    /// Write a numeric dataref value by NAME. Adapter-internal only —
    /// NEVER exposed above the typed action layer (spec §7).
    #[allow(dead_code)]
    pub fn write_dataref_f64(&mut self, name: &str, value: f64) -> Result<(), WebApiError> {
        let id = self.resolve_dataref(name)?;
        let path = format!("/api/v3/datarefs/{id}/value");
        let payload = serde_json::json!({ "data": value }).to_string();
        let (status, body) = self
            .transport
            .request(HttpMethod::Patch, &path, Some(&payload))?;
        parse_unit(status, &body).map_err(|e| self.on_possible_session_expiry(name, id, e))
    }

    /// How long an activated command stays active before the simulator
    /// auto-deactivates it (seconds). Long enough for the command handler
    /// to observe `begin`, short enough to stay one-shot (spec §31).
    const COMMAND_PRESS_SECONDS: f64 = 0.1;
    /// Activate a command ONCE by NAME (one-shot press/release; spec §31 —
    /// no held/overlapping activations in this milestone).
    pub fn activate_command(&mut self, name: &str) -> Result<(), WebApiError> {
        let id = self.resolve_command(name)?;
        // One-shot press: `begin` fires immediately (the switch flips),
        // the command auto-deactivates after [`COMMAND_PRESS_SECONDS`].
        let path = format!("/api/v3/command/{id}/activate");
        let payload = format!(r#"{{"duration":{}}}"#, Self::COMMAND_PRESS_SECONDS);
        let (status, body) = self
            .transport
            .request(HttpMethod::Post, &path, Some(&payload))?;
        parse_unit(status, &body).map_err(|e| self.on_possible_session_expiry(name, id, e))
    }

    /// Session-expiry detection (spec §8/§28): a 404 for a previously
    /// resolved id means the simulator session changed and the cached id
    /// is dead. Evict it and report `SessionExpired` so the next attempt
    /// re-resolves by NAME — a stale id can never poison the cache.
    fn on_possible_session_expiry(
        &mut self,
        name: &str,
        used_id: u64,
        err: WebApiError,
    ) -> WebApiError {
        let cached = self
            .session
            .cached_command(name)
            .or_else(|| self.session.cached_dataref(name));
        match (&err, cached) {
            (WebApiError::Api { http: 404, .. }, Some(cached_id)) if cached_id == used_id => {
                self.forget_command(name);
                self.forget_dataref(name);
                WebApiError::SessionExpired(format!("{name} (id {used_id} no longer valid)"))
            }
            _ => err,
        }
    }

    /// Drop a single cached id after the simulator rejected it (e.g. 404
    /// on a previously valid id): the session partially expired.
    #[allow(dead_code)]
    pub fn forget_command(&mut self, name: &str) {
        self.session.commands.remove(name);
    }

    #[allow(dead_code)]
    pub fn forget_dataref(&mut self, name: &str) {
        self.session.datarefs.remove(name);
    }
}

/// Map a non-2xx response to a typed error, distinguishing disabled
/// (403) and session expiry (404 on an id we had resolved).
fn parse_ok<T: serde::de::DeserializeOwned>(status: u16, body: &str) -> Result<T, WebApiError> {
    if status == 403 {
        return Err(WebApiError::Disabled);
    }
    if !(200..300).contains(&status) {
        return Err(api_error(status, body));
    }
    serde_json::from_str(body).map_err(|e| WebApiError::Malformed(e.to_string()))
}

fn parse_unit(status: u16, body: &str) -> Result<(), WebApiError> {
    if status == 403 {
        return Err(WebApiError::Disabled);
    }
    if !(200..300).contains(&status) {
        return Err(api_error(status, body));
    }
    Ok(())
}

fn api_error(status: u16, body: &str) -> WebApiError {
    match serde_json::from_str::<ErrorPayload>(body) {
        Ok(p) => WebApiError::Api {
            code: p.error_code,
            message: p.error_message,
            http: status,
        },
        Err(_) => WebApiError::Malformed(format!("http {status}: {body}")),
    }
}

fn json_value_to_dataref(v: serde_json::Value) -> DatarefValue {
    match v {
        serde_json::Value::Null => DatarefValue::Null,
        serde_json::Value::Number(n) => match n.as_i64() {
            Some(i) => DatarefValue::Int(i),
            None => DatarefValue::Number(n.as_f64().unwrap_or(f64::NAN)),
        },
        serde_json::Value::String(s) => DatarefValue::Text(s),
        serde_json::Value::Array(a) => {
            DatarefValue::Array(a.into_iter().map(json_value_to_dataref).collect())
        }
        other => DatarefValue::Text(other.to_string()),
    }
}

/// Minimal percent-encoding for `filter[name]=` query values.
fn urlencode(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' | b'/' => {
                out.push(b as char)
            }
            _ => out.push_str(&format!("%{b:02X}")),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    /// Deterministic mock transport with scripted responses and fault
    /// injection (spec §34).
    struct MockTransport {
        responses: RefCell<Vec<(u16, String)>>,
        fail_all: RefCell<Option<WebApiError>>,
        seen: RefCell<Vec<(HttpMethod, String)>>,
    }

    impl MockTransport {
        fn new(responses: Vec<(u16, String)>) -> Self {
            Self {
                responses: RefCell::new(responses),
                fail_all: RefCell::new(None),
                seen: RefCell::new(Vec::new()),
            }
        }
    }

    impl WebApiTransport for MockTransport {
        fn get(&self, path: &str) -> Result<u16, WebApiError> {
            self.request(HttpMethod::Get, path, None).map(|(s, _)| s)
        }

        fn request(
            &self,
            method: HttpMethod,
            path: &str,
            _body: Option<&str>,
        ) -> Result<(u16, String), WebApiError> {
            self.seen.borrow_mut().push((method, path.to_string()));
            if let Some(e) = self.fail_all.borrow().as_ref() {
                return Err(match e {
                    WebApiError::Transport(m) => WebApiError::Transport(m.clone()),
                    WebApiError::Malformed(m) => WebApiError::Malformed(m.clone()),
                    WebApiError::ResourceNotFound(n) => WebApiError::ResourceNotFound(n.clone()),
                    WebApiError::SessionExpired(n) => WebApiError::SessionExpired(n.clone()),
                    WebApiError::UnsupportedVersion(a, b) => {
                        WebApiError::UnsupportedVersion(a.clone(), b.clone())
                    }
                    WebApiError::Disabled => WebApiError::Disabled,
                    WebApiError::Api {
                        code,
                        message,
                        http,
                    } => WebApiError::Api {
                        code: code.clone(),
                        message: message.clone(),
                        http: *http,
                    },
                });
            }
            // FIFO: responses are listed in expected call order.
            if self.responses.borrow().is_empty() {
                return Err(WebApiError::Transport("mock queue empty".into()));
            }
            Ok(self.responses.borrow_mut().remove(0))
        }
    }

    const CAPS: &str = r#"{"api":{"versions":["v1","v2","v3"]},"x-plane":{"version":"12.4.3"}}"#;

    #[test]
    fn capabilities_parses_and_verifies_v3() {
        let c = WebApiClient::new(MockTransport::new(vec![(200, CAPS.into())]));
        let caps = c.capabilities().unwrap();
        assert_eq!(caps.x_plane.version, "12.4.3");
        assert!(caps.api.versions.contains(&"v3".into()));
    }

    #[test]
    fn unsupported_version_is_typed_error() {
        let old = r#"{"api":{"versions":["v1"]},"x-plane":{"version":"12.1.1"}}"#;
        let c = WebApiClient::new(MockTransport::new(vec![(200, old.into())]));
        assert!(matches!(
            c.capabilities(),
            Err(WebApiError::UnsupportedVersion(_, _))
        ));
    }

    #[test]
    fn command_resolution_caches_session_id() {
        let c = WebApiClient::new(MockTransport::new(vec![
            (
                200,
                r#"{"data":[{"id":2991,"name":"sim/lights/beacon_lights_toggle","description":"t"}]}"#.into(),
            ),
            (200, CAPS.into()),
        ]));
        let mut c = c;
        let id1 = c
            .resolve_command("sim/lights/beacon_lights_toggle")
            .unwrap();
        let id2 = c
            .resolve_command("sim/lights/beacon_lights_toggle")
            .unwrap();
        assert_eq!(id1, 2991);
        assert_eq!(id2, 2991);
        assert_eq!(c.session().len(), 1, "second resolve must come from cache");
    }

    #[test]
    fn unknown_command_name_is_resource_not_found() {
        let c = WebApiClient::new(MockTransport::new(vec![(200, r#"{"data":[]}"#.into())]));
        let mut c = c;
        assert!(matches!(
            c.resolve_command("sim/does/not/exist"),
            Err(WebApiError::ResourceNotFound(_))
        ));
    }

    #[test]
    fn invalidate_session_clears_cached_ids() {
        let c = WebApiClient::new(MockTransport::new(vec![(
            200,
            r#"{"data":[{"id":7,"name":"x"}]}"#.into(),
        )]));
        let mut c = c;
        c.resolve_command("x").unwrap();
        assert_eq!(c.session().len(), 1);
        c.invalidate_session();
        assert!(c.session().is_empty(), "ids never survive a new session");
    }

    #[test]
    fn api_error_payload_is_typed() {
        let c = WebApiClient::new(MockTransport::new(vec![(
            404,
            r#"{"error_code":"invalid_dataref_name","error_message":"Dataref x doesn't exist"}"#
                .into(),
        )]));
        let mut c = c;
        match c.read_dataref("x") {
            Err(WebApiError::Api { code, http, .. }) => {
                assert_eq!(code, "invalid_dataref_name");
                assert_eq!(http, 404);
            }
            other => panic!("expected Api error, got {other:?}"),
        }
    }

    #[test]
    fn malformed_response_is_typed() {
        let c = WebApiClient::new(MockTransport::new(vec![(
            200,
            "<html>not json</html>".into(),
        )]));
        let mut c = c;
        assert!(matches!(
            c.read_dataref("x"),
            Err(WebApiError::Malformed(_))
        ));
    }

    #[test]
    fn transport_failure_is_typed() {
        let c = WebApiClient::new(MockTransport::new(vec![]));
        let mut c = c;
        assert!(matches!(
            c.resolve_command("x"),
            Err(WebApiError::Transport(_))
        ));
    }

    #[test]
    fn disabled_policy_403_is_typed() {
        let c = WebApiClient::new(MockTransport::new(vec![(403, String::new())]));
        let mut c = c;
        assert!(matches!(c.read_dataref("x"), Err(WebApiError::Disabled)));
    }

    #[test]
    fn read_value_parses_number_int_and_null() {
        let c = WebApiClient::new(MockTransport::new(vec![
            // 1) resolve, 2..4) value reads in call order.
            (
                200,
                r#"{"data":[{"id":5,"name":"sim/cockpit2/switches/beacon_on","value_type":"int"}]}"#.into(),
            ),
            (200, r#"{"data":null}"#.into()),
            (200, r#"{"data":1}"#.into()),
            (200, r#"{"data":0.75}"#.into()),
        ]));
        let mut c = c;
        assert_eq!(
            c.read_dataref("sim/cockpit2/switches/beacon_on").unwrap(),
            DatarefValue::Null
        );
        assert_eq!(
            c.read_dataref("sim/cockpit2/switches/beacon_on").unwrap(),
            DatarefValue::Int(1)
        );
        assert_eq!(
            c.read_dataref("sim/cockpit2/switches/beacon_on")
                .unwrap()
                .as_f64(),
            Some(0.75)
        );
        // resolve happens before the value read (queue order above).
        assert_eq!(
            c.resolve_dataref("sim/cockpit2/switches/beacon_on")
                .unwrap(),
            5
        );
    }

    #[test]
    fn activate_uses_post_and_resolved_id() {
        let c = WebApiClient::new(MockTransport::new(vec![
            // 1) resolve by name, 2) activate (empty 200 body).
            (
                200,
                r#"{"data":[{"id":42,"name":"sim/lights/beacon_lights_on"}]}"#.into(),
            ),
            (200, String::new()),
        ]));
        let mut c = c;
        c.activate_command("sim/lights/beacon_lights_on").unwrap();
        // The mock recorded both requests: resolve (GET filter) then
        // activate (POST with the session id in the path).
        let seen = c.transport.seen_of_last();
        assert!(
            seen.contains("/api/v3/command/42/activate"),
            "activation must POST to the singular v2+ activate route with the resolved session id: {seen}"
        );
    }

    impl MockTransport {
        fn seen_of_last(&self) -> String {
            self.seen
                .borrow()
                .iter()
                .map(|(_, p)| p.clone())
                .collect::<Vec<_>>()
                .join(";")
        }
    }
}
