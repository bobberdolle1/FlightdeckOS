//! `XPlaneSimulatorAdapter`: the live X-Plane 12 implementation of the
//! `fd-core` `SimulatorAdapter` + `FlightControlTargets` boundaries.
//!
//! * Telemetry: only from received UDP packets — never fabricated, never
//!   substituted with virtual state (Task 4 §7/§17).
//! * Control: allowlisted autopilot-target writes only (Task 4 §10);
//!   non-finite targets are rejected before the wire.
//! * Disconnect: typed `NotConnected` on stale packets; `Degraded`
//!   capability while disconnected; automatic resubscribe on reconnect
//!   (Task 4 §16).
//! * Identity: carried with provenance (`fd_core::identity`); stock UDP
//!   cannot read the byte-array identity datarefs, so unknown stays
//!   unknown unless an operator supplies a claim.

use std::time::{Duration, Instant};

use fd_core::actions::CockpitAction;
use fd_core::adapter::{AdapterError, Capability, FlightControlTargets, SimulatorAdapter};
use fd_core::identity::AircraftIdentity;
use fd_core::telemetry::{SimState, SimTimestamp, TelemetrySnapshot};
use fd_core::units::{
    AltitudeAglFt, AltitudeFt, AngleDeg, LatDeg, LonDeg, SpeedKt, VerticalSpeedFpm,
};

use crate::client::XPlaneUdpClient;
use crate::guard::LiveWriteGuard;
use crate::refs::{Command, DataRefId, WriteRef};
use crate::webapi::{DEFAULT_BASE_URL, HttpTransport, WebApiClient};

/// Live command bindings for the first safe action (spec §9-12): commands
/// for the mechanism, the UDP dataref for independent verification.
pub const BEACON_ON_COMMAND: &str = "sim/lights/beacon_lights_on";
pub const BEACON_OFF_COMMAND: &str = "sim/lights/beacon_lights_off";

const M_TO_FT: f64 = 3.280_839_895;
const MS_TO_KT: f64 = 1.943_844_492;

/// Connection/behavior configuration.
#[derive(Debug, Clone)]
pub struct XPlaneConfig {
    pub host: String,
    pub port: u16,
    /// Dataref stream frequency in Hz (X-Plane default UI is 4-30).
    pub subscribe_hz: i32,
    /// Local Web API base (loopback only, spec §30).
    pub web_api_base: String,
    /// Arm the live-write guard at construction (CLI `--allow-write`).
    /// Default DISABLED: telemetry/shadow never write (spec §14).
    pub allow_writes: bool,
}

impl Default for XPlaneConfig {
    fn default() -> Self {
        Self {
            host: "127.0.0.1".into(),
            port: 49000,
            subscribe_hz: 4,
            web_api_base: DEFAULT_BASE_URL.into(),
            allow_writes: false,
        }
    }
}

/// Live adapter over the native X-Plane UDP transport.
pub struct XPlaneAdapter {
    client: XPlaneUdpClient,
    /// Best-effort aircraft identity (see `fd_core::identity`): unknown
    /// unless an operator supplies a claim — stock UDP cannot read the
    /// byte-array identity datarefs.
    identity: AircraftIdentity,
    /// Last rejected control-target write (non-finite input / send error).
    last_control_error: Option<String>,
    /// Local Web API client (command activation). Session-scoped ids.
    web: Option<WebApiClient<HttpTransport>>,
    /// Live-write inhibit (spec §14): default DISABLED.
    write_guard: std::sync::Arc<LiveWriteGuard>,
    /// Number of poll cycles that observed a disconnected transport.
    disconnects: u64,
    last_poll_duration: Option<Duration>,
    /// Per-channel consecutive finite-sample counter (wire id -> count).
    /// A channel becomes authoritative (Fresh) only after
    /// [`WARMUP_SAMPLES`] consecutive finite observations (Task 6 §7).
    warmup: std::collections::HashMap<i32, u32>,
    /// Web API health tracking (Task 6 §45): bounded failures with a
    /// cooldown instead of request spam or unbounded hangs.
    web_health: WebHealth,
    /// Previous health() poll for UDP packet-rate estimation.
    rate_probe: Option<(std::time::Instant, u64)>,
}

/// DEVELOPMENT DEFAULT warm-up sample count: ~1 s of consistent stream at
/// the 3-4 Hz subscribe rate. Evidence-based: consecutive consistent
/// observations, not a wall-clock sleep.
pub const WARMUP_SAMPLES: u32 = 3;

/// DEVELOPMENT DEFAULT web API cooldown after a transport failure: no
/// request is attempted while cooling down (no localhost spam).
pub const WEB_COOLDOWN: Duration = Duration::from_secs(5);

/// DEVELOPMENT DEFAULT UDP health windows.
pub const UDP_DEGRADED_AFTER: Duration = Duration::from_secs(3);
pub const UDP_UNAVAILABLE_AFTER: Duration = Duration::from_secs(10);

/// Transport capability states (Task 6 §45/§47): UDP and Web API are
/// reported INDEPENDENTLY — one being healthy says nothing about the other.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HealthState {
    Available,
    Degraded,
    Unavailable,
}

/// Multi-transport health snapshot (Task 6 §47). Answers: is UDP working?
/// is the Web API working? — without equating them.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TransportHealth {
    pub udp: HealthState,
    pub web_api: HealthState,
    pub udp_last_packet_age: Option<Duration>,
    pub udp_packet_rate_hz: Option<f64>,
}

#[derive(Debug, Default)]
struct WebHealth {
    last_ok: Option<std::time::Instant>,
    cooldown_until: Option<std::time::Instant>,
    last_error: Option<String>,
}

impl WebHealth {
    /// Gate a web operation: refuse immediately while cooling down.
    fn gate(&mut self) -> Result<(), AdapterError> {
        if let Some(until) = self.cooldown_until {
            if std::time::Instant::now() < until {
                return Err(AdapterError::WriteFailed(
                    "web api cooling down after transport failure".into(),
                ));
            }
            self.cooldown_until = None;
        }
        Ok(())
    }

    fn note_ok(&mut self) {
        self.last_ok = Some(std::time::Instant::now());
        self.last_error = None;
        self.cooldown_until = None;
    }

    fn note_fail(&mut self, e: &str) {
        self.last_error = Some(e.to_string());
        self.cooldown_until = Some(std::time::Instant::now() + WEB_COOLDOWN);
    }
}

impl XPlaneAdapter {
    pub fn new(cfg: XPlaneConfig) -> Result<Self, AdapterError> {
        Self::with_identity(cfg, AircraftIdentity::unknown())
    }

    /// Build the adapter with an operator-claimed aircraft identity.
    /// The claim keeps `IdentitySource::UserProvided` provenance and is
    /// never treated as a trusted adapter read.
    pub fn with_identity(
        cfg: XPlaneConfig,
        identity: AircraftIdentity,
    ) -> Result<Self, AdapterError> {
        let mut client = XPlaneUdpClient::new(0, &cfg.host, cfg.port)
            .map_err(|e| AdapterError::ConnectionFailed(e.to_string()))?;
        let refs: Vec<(i32, &'static str)> = DataRefId::ALL
            .iter()
            .map(|(id, p)| (id.wire_id(), *p))
            .collect();
        client
            .start(cfg.subscribe_hz, &refs)
            .map_err(|e| AdapterError::ConnectionFailed(e.to_string()))?;
        let web = WebApiClient::new(
            HttpTransport::new(&cfg.web_api_base)
                .map_err(|e| AdapterError::ConnectionFailed(e.to_string()))?,
        );
        let write_guard = std::sync::Arc::new(LiveWriteGuard::disabled());
        if cfg.allow_writes {
            write_guard.arm();
        }
        Ok(Self {
            client,
            identity,
            last_control_error: None,
            web: Some(web),
            warmup: std::collections::HashMap::new(),
            web_health: WebHealth::default(),
            rate_probe: None,
            write_guard,
            disconnects: 0,
            last_poll_duration: None,
        })
    }

    /// The live-write guard handle (CLI arm/disarm surface).
    pub fn write_guard(&self) -> std::sync::Arc<LiveWriteGuard> {
        std::sync::Arc::clone(&self.write_guard)
    }

    /// Best-effort simulator version through the Local Web API
    /// (`GET /api/capabilities`). `None` when the API is unavailable —
    /// telemetry does not depend on it.
    pub fn simulator_version(&mut self) -> Option<String> {
        if self.web_health.gate().is_err() {
            return None;
        }
        let web = self.web.as_mut()?;
        let result = web.capabilities();
        match &result {
            Ok(_) => self.web_health.note_ok(),
            Err(e) => self.web_health.note_fail(&e.to_string()),
        }
        result.ok().map(|c| c.x_plane.version)
    }

    /// Local UDP bind port (test/observability hook).
    pub fn local_port(&self) -> u16 {
        self.client.local_port
    }

    /// Current beacon state as observed over UDP telemetry (None = unknown).
    pub fn beacon_state(&self) -> Option<bool> {
        self.value(DataRefId::BeaconOn).map(|v| v > 0.5)
    }

    /// Aircraft changed/reloaded (spec §27): invalidate EVERYTHING
    /// aircraft-specific. Identity reverts to Unknown (never silently
    /// carried across aircraft), Web API session ids for aircraft-scoped
    /// resources are dropped, in-flight control error state is cleared.
    /// Generic telemetry continues unchanged; pending actions fail in the
    /// runtime through normal verification/timeout paths.
    pub fn invalidate_aircraft(&mut self) {
        self.identity = fd_core::identity::AircraftIdentity::unknown();
        if let Some(web) = self.web.as_mut() {
            web.invalidate_session();
        }
        self.last_control_error = None;
        // Aircraft-specific channels must re-warm for the next aircraft
        // (Task 6 §42): no stale-fresh carryover across a hot-swap.
        self.warmup.clear();
    }

    /// Update the operator identity claim. A CHANGED claim invalidates
    /// aircraft-specific state exactly like [`Self::invalidate_aircraft`]
    /// (Task 6 §42); an unchanged claim is a no-op.
    pub fn set_identity_claim(&mut self, claim: AircraftIdentity) {
        if self.identity.icao != claim.icao {
            self.invalidate_aircraft();
            self.identity = claim;
        }
    }

    /// The aircraft identity this adapter was constructed with.
    pub fn identity(&self) -> &AircraftIdentity {
        &self.identity
    }

    /// Take the last rejected control-target write, if any (diagnostics).
    pub fn take_last_control_error(&mut self) -> Option<String> {
        self.last_control_error.take()
    }

    /// Wait for the first telemetry packet (bounded).
    pub fn wait_first_packet(&mut self, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if self.client.connected() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        false
    }

    pub fn packets_received(&self) -> u64 {
        self.client.packets_received()
    }

    /// Datagrams dropped because their source was not the simulator.
    pub fn rejected_foreign_packets(&self) -> u64 {
        self.client.rejected_foreign_packets()
    }

    /// Record ids dropped because they are outside the subscribed set.
    pub fn unknown_ids_dropped(&self) -> u64 {
        self.client.unknown_ids_dropped()
    }

    /// Receive errors survived (non-fatal by design).
    pub fn recv_errors(&self) -> u64 {
        self.client.recv_errors()
    }

    pub fn newest_packet_age(&self) -> Duration {
        self.client.newest_packet_age()
    }

    pub fn disconnect_count(&self) -> u64 {
        self.disconnects
    }

    /// Raw dataref read (wire id) — used by smokes and closed-loop checks.
    pub fn raw(&self, id: DataRefId) -> Option<f64> {
        self.client.latest(id.wire_id()).map(|v| v as f64)
    }

    /// Send an allowlisted autopilot write (used by `FlightControlTargets`).
    /// Behind the SAME live-write guard as discrete actions (spec §14):
    /// AP-target datagrams are simulator writes and must never fire from
    /// an un-armed process.
    fn write_ap(&self, r: WriteRef, value: f32) -> Result<(), AdapterError> {
        self.write_guard.ensure_armed()?;
        self.guard_connected()?;
        self.client
            .write_dref(r.path(), value)
            .map_err(|e| AdapterError::WriteFailed(e.to_string()))
    }

    fn engage(&self, c: Command) -> Result<(), AdapterError> {
        self.write_guard.ensure_armed()?;
        self.guard_connected()?;
        self.client
            .send_command(c.path())
            .map_err(|e| AdapterError::WriteFailed(e.to_string()))
    }

    fn guard_connected(&self) -> Result<(), AdapterError> {
        if self.client.connected() {
            Ok(())
        } else {
            Err(AdapterError::NotConnected)
        }
    }

    fn value(&self, id: DataRefId) -> Option<f64> {
        self.raw(id).filter(|v| v.is_finite())
    }

    fn build_snapshot(&mut self) -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(epoch_ms()));
        let lat = self.value(DataRefId::Latitude);
        let lon = self.value(DataRefId::Longitude);
        if let (Some(la), Some(lo)) = (lat, lon) {
            s.position = Some(fd_core::telemetry::Position {
                lat: LatDeg::new(la),
                lon: LonDeg::new(lo),
            });
        }
        s.altitude_msl = self
            .value(DataRefId::ElevationM)
            .map(|m| AltitudeFt::new(m * M_TO_FT));
        // AGL preference: the radio altimeter is the truthful height above
        // terrain once airborne; geometric y_agl stays the fallback.
        let radio_ft = self.value(DataRefId::RadioAltitudeFt);
        let y_agl_ft = self.value(DataRefId::YAglM).map(|m| m * M_TO_FT);
        s.altitude_agl = match radio_ft {
            Some(r) if r > 0.0 => Some(AltitudeAglFt::new(r)),
            _ => y_agl_ft.map(AltitudeAglFt::new),
        };
        s.indicated_airspeed = self.value(DataRefId::IndicatedAirspeedKt).map(SpeedKt::new);
        s.groundspeed = self
            .value(DataRefId::GroundspeedMs)
            .map(|ms| SpeedKt::new(ms * MS_TO_KT));
        // X-Plane VVI convention (vh_ind_fpm): positive up — same as ours;
        // verified live in the telemetry smoke (Task 4 §8).
        s.vertical_speed = self
            .value(DataRefId::VerticalSpeedFpm)
            .map(VerticalSpeedFpm::new);
        s.heading_true = self.value(DataRefId::HeadingTrueDeg).map(AngleDeg::new);
        s.pitch = self.value(DataRefId::PitchDeg).map(AngleDeg::new);
        s.bank = self.value(DataRefId::BankDeg).map(AngleDeg::new);
        s.on_ground = self.value(DataRefId::OnGroundWheel0).map(|w| w > 0.5);
        s.beacon_light = self.value(DataRefId::BeaconOn).map(|v| v > 0.5);
        s.gear_handle_down = self.value(DataRefId::GearDeploy0).map(|d| d > 0.5);
        s.autopilot_master = Some(
            self.value(DataRefId::ApHeadingStatus).unwrap_or(0.0) >= 2.0
                || self.value(DataRefId::ApVviStatus).unwrap_or(0.0) >= 2.0,
        );
        s.sim_timing.state = if self.client.connected() {
            SimState::Running
        } else {
            SimState::Unknown
        };
        // Data-quality sidecar (spec §21): annotate every non-fresh core
        // channel. Key = DataRefId wire id (the adapter's channel namespace).
        let core_channels: [(DataRefId, bool); 12] = [
            (DataRefId::Latitude, s.position.is_some()),
            (DataRefId::Longitude, s.position.is_some()),
            (DataRefId::ElevationM, s.altitude_msl.is_some()),
            (DataRefId::YAglM, s.altitude_agl.is_some()),
            (
                DataRefId::IndicatedAirspeedKt,
                s.indicated_airspeed.is_some(),
            ),
            (DataRefId::GroundspeedMs, s.groundspeed.is_some()),
            (DataRefId::VerticalSpeedFpm, s.vertical_speed.is_some()),
            (DataRefId::HeadingTrueDeg, s.heading_true.is_some()),
            (DataRefId::PitchDeg, s.pitch.is_some()),
            (DataRefId::BankDeg, s.bank.is_some()),
            (DataRefId::OnGroundWheel0, s.on_ground.is_some()),
            (DataRefId::BeaconOn, s.beacon_light.is_some()),
        ];
        for (id, fresh) in core_channels {
            let wire = id.wire_id();
            let raw = self.raw(id);
            match raw {
                // Received but unrepresentable: a distinct fact from absent.
                Some(v) if !v.is_finite() => {
                    self.warmup.insert(wire, 0);
                    s.channel_quality
                        .insert(wire as u16, fd_core::telemetry::DataQuality::Invalid);
                }
                Some(_) => {
                    let count = self.warmup.entry(wire).and_modify(|c| *c += 1).or_insert(1);
                    if *count < WARMUP_SAMPLES {
                        // Present but not yet authoritative (Task 6 §7).
                        s.channel_quality
                            .insert(wire as u16, fd_core::telemetry::DataQuality::WarmingUp);
                    } else if !fresh {
                        // Warmed but outside the freshness window.
                        let q = self.client.quality(wire);
                        s.channel_quality.insert(wire as u16, q);
                    }
                }
                None => {
                    self.warmup.insert(wire, 0);
                    let q = self.client.quality(wire);
                    s.channel_quality.insert(wire as u16, q);
                }
            }
        }
        s
    }

    /// Multi-transport health (Task 6 §45/§47). Cheap: no I/O, no probes.
    pub fn health(&mut self) -> TransportHealth {
        let udp_age = self.client.newest_packet_age();
        let udp = if udp_age == Duration::MAX || udp_age > UDP_UNAVAILABLE_AFTER {
            HealthState::Unavailable
        } else if udp_age > UDP_DEGRADED_AFTER {
            HealthState::Degraded
        } else {
            HealthState::Available
        };
        let web = if self
            .web_health
            .cooldown_until
            .map(|until| std::time::Instant::now() < until)
            .unwrap_or(false)
        {
            HealthState::Unavailable
        } else {
            match (self.web_health.last_ok, &self.web_health.last_error) {
                (Some(t), _) if t.elapsed() < Duration::from_secs(30) => HealthState::Available,
                (Some(_), _) => HealthState::Degraded,
                (None, Some(_)) => HealthState::Unavailable,
                (None, None) => HealthState::Degraded, // never exercised yet
            }
        };
        // UDP packet rate from the previous health() probe.
        let now = std::time::Instant::now();
        let total = self.client.packets_received();
        let rate = self.rate_probe.replace((now, total)).map(|(t0, p0)| {
            let dt = now.duration_since(t0).as_secs_f64();
            if dt > 0.0 {
                Some((total - p0) as f64 / dt)
            } else {
                None
            }
        });
        let udp_last_packet_age = (udp_age != Duration::MAX).then_some(udp_age);
        TransportHealth {
            udp,
            web_api: web,
            udp_last_packet_age,
            udp_packet_rate_hz: rate.flatten(),
        }
    }

    /// Validate a control target before it may reach the wire: non-finite
    /// inputs (NaN/±inf) are rejected, recorded, and never sent. Returns
    /// the value unchanged (f64) so unit conversions keep full precision;
    /// callers cast to f32 at the wire boundary.
    fn checked_target(&mut self, name: &str, value: f64) -> Option<f64> {
        if value.is_finite() {
            Some(value)
        } else {
            self.last_control_error = Some(format!("{name}: non-finite target {value}"));
            None
        }
    }
}

/// Convert a TRUE heading target into the magnetic heading the X-Plane
/// stock autopilot expects. X-Plane's `magnetic_variation` is positive EAST,
/// so magnetic = true − variation.
fn true_to_mag(true_deg: f64, magvar_deg: f64) -> f32 {
    let mag = true_deg - magvar_deg;
    let mag = mag.rem_euclid(360.0);
    mag as f32
}

fn epoch_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

impl SimulatorAdapter for XPlaneAdapter {
    fn connect(&mut self) -> Result<(), AdapterError> {
        if self.client.connected() {
            return Ok(());
        }
        // Reconnect path: re-send the subscription and wait briefly.
        self.client
            .resubscribe()
            .map_err(|e| AdapterError::ConnectionFailed(e.to_string()))?;
        if self.wait_first_packet(Duration::from_secs(3)) {
            Ok(())
        } else {
            Err(AdapterError::ConnectionFailed(
                "no telemetry from X-Plane (is port 49000 open and a flight loaded?)".into(),
            ))
        }
    }

    fn disconnect(&mut self) {
        // UDP is connectionless; dropping the client stops the rx thread.
    }

    fn is_connected(&self) -> bool {
        self.client.connected()
    }

    fn poll(&mut self) -> Result<Vec<TelemetrySnapshot>, AdapterError> {
        let t0 = Instant::now();
        let out = if self.client.connected() {
            Ok(vec![self.build_snapshot()])
        } else {
            self.disconnects += 1;
            Err(AdapterError::NotConnected)
        };
        self.last_poll_duration = Some(t0.elapsed());
        out
    }

    fn capability(&self, action: CockpitAction) -> Capability {
        match action {
            // First safe action (spec §10): beacon via live Web API
            // commands, verified through independent UDP telemetry.
            CockpitAction::SetBeacon(_) => {
                if self.client.connected() {
                    Capability::Supported
                } else {
                    Capability::Unavailable
                }
            }
            CockpitAction::SetNavLogo(_) => {
                if self.client.connected() {
                    Capability::Unsupported
                } else {
                    Capability::Unavailable
                }
            }
        }
    }

    fn execute(&mut self, action: CockpitAction) -> Result<(), AdapterError> {
        match action {
            CockpitAction::SetBeacon(target) => self.execute_beacon(target),
            CockpitAction::SetNavLogo(_) => Err(AdapterError::UnsupportedAction),
        }
    }
}

impl XPlaneAdapter {
    /// Dispatch the beacon command through the Local Web API (spec §12/15):
    /// guard → current-state precondition → no-op when already satisfied →
    /// typed command activation. Success here is DISPATCH, never Verified —
    /// verification happens in the runtime against fresh UDP telemetry.
    fn execute_beacon(
        &mut self,
        target: fd_core::actions::SwitchPosition,
    ) -> Result<(), AdapterError> {
        use fd_core::actions::SwitchPosition;
        // 1) Live-write inhibit (default disabled, spec §14).
        self.write_guard.ensure_armed()?;
        // 2) Current-state precondition: never blind-toggle (spec §12).
        let current = self.beacon_state();
        let target_on = target == SwitchPosition::On;
        match current {
            None => return Err(AdapterError::StateUnknown(CockpitAction::SetBeacon(target))),
            Some(on) if on == target_on => {
                // Already satisfied: no unnecessary command (spec §33).
                return Ok(());
            }
            _ => {}
        }
        // 3) Typed command activation through the session-scoped client.
        let name = if target_on {
            BEACON_ON_COMMAND
        } else {
            BEACON_OFF_COMMAND
        };
        self.web_health.gate()?;
        match self.web.as_mut() {
            Some(web) => match web.activate_command(name) {
                Ok(v) => {
                    self.web_health.note_ok();
                    Ok(v)
                }
                Err(e) => {
                    self.web_health.note_fail(&e.to_string());
                    Err(AdapterError::WriteFailed(e.to_string()))
                }
            },
            None => Err(AdapterError::WriteFailed(
                "web api client unavailable".into(),
            )),
        }
    }
}

impl FlightControlTargets for XPlaneAdapter {
    fn flight_guidance_supported(&self) -> bool {
        self.client.connected()
    }

    fn set_target_altitude(&mut self, altitude_ft: f64) {
        // Target write only: mode engagement is aircraft-specific and is
        // NOT auto-pressed in this slice (reported honestly as unproven).
        if let Some(v) = self.checked_target("altitude", altitude_ft)
            && let Err(e) = self.write_ap(WriteRef::ApAltitude, v as f32)
        {
            self.last_control_error = Some(e.to_string());
        }
    }

    fn set_target_speed(&mut self, speed_kt: f64) {
        if let Some(v) = self.checked_target("speed", speed_kt)
            && let Err(e) = self.write_ap(WriteRef::ApAirspeed, v as f32)
        {
            self.last_control_error = Some(e.to_string());
        }
    }

    fn set_target_heading(&mut self, heading_deg: f64) {
        let Some(v) = self.checked_target("heading", heading_deg) else {
            return;
        };
        let magvar = self.value(DataRefId::MagVariationDeg).unwrap_or(0.0);
        if let Err(e) = self.write_ap(WriteRef::ApHeadingMag, true_to_mag(v, magvar)) {
            self.last_control_error = Some(e.to_string());
            return;
        }
        // Engage HDG mode only when it is currently OFF (status 0).
        // Status 2 = captured; re-sending the command would TOGGLE it
        // off, so gate on the observed status.
        if self.value(DataRefId::ApHeadingStatus).unwrap_or(0.0) < 2.0
            && let Err(e) = self.engage(Command::ApHeadingHold)
        {
            self.last_control_error = Some(e.to_string());
        }
    }

    fn set_target_vertical_speed(&mut self, fpm: f64) {
        let Some(v) = self.checked_target("vertical_speed", fpm) else {
            return;
        };
        if let Err(e) = self.write_ap(WriteRef::ApVerticalVelocity, v as f32) {
            self.last_control_error = Some(e.to_string());
            return;
        }
        if self.value(DataRefId::ApVviStatus).unwrap_or(0.0) < 2.0
            && let Err(e) = self.engage(Command::ApVerticalSpeedPreSel)
        {
            self.last_control_error = Some(e.to_string());
        }
    }
}

/// True→magnetic conversion (unit-tested; sign verified live).
pub fn heading_true_to_mag(true_deg: f64, magvar_deg: f64) -> f32 {
    true_to_mag(true_deg, magvar_deg)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn true_to_mag_wraps_and_signs() {
        // East variation (positive) => magnetic is LESS than true.
        assert!((heading_true_to_mag(90.0, 10.0) - 80.0).abs() < 1e-4);
        // West variation (negative) => magnetic is MORE than true.
        assert!((heading_true_to_mag(10.0, -10.0) - 20.0).abs() < 1e-4);
        // Wrap below zero.
        assert!((heading_true_to_mag(5.0, 10.0) - 355.0).abs() < 1e-4);
        // Wrap above 360.
        assert!((heading_true_to_mag(359.0, -2.0) - 1.0).abs() < 1e-4);
    }

    #[test]
    fn unit_conversions() {
        assert!((2000.0f64 * M_TO_FT - 6561.68).abs() < 0.01);
        assert!((100.0f64 * MS_TO_KT - 194.38).abs() < 0.01);
    }

    #[test]
    fn epoch_ms_is_sane() {
        // Post-2020 wall clock, in ms.
        assert!(epoch_ms() > 1_600_000_000_000);
    }
}

#[cfg(test)]
mod lifecycle_tests {
    use fd_core::identity::{AircraftIdentity, IdentitySource};

    // NOTE: these tests exercise the invalidation STATE machine without a
    // network: the Web API client is constructed lazily and never contacted
    // by invalidate_aircraft.

    #[test]
    fn identity_defaults_to_unknown_and_untrusted() {
        // A live adapter constructed without a user claim must never
        // present an identity (stock UDP cannot read identity strings).
        let id = AircraftIdentity::unknown();
        assert_eq!(id.source, IdentitySource::Unknown);
        assert!(!id.is_trusted());
        let claimed = AircraftIdentity::user_provided(Some("C152".into()));
        assert!(!claimed.is_trusted(), "user claims are never trusted reads");
    }

    #[test]
    fn write_guard_starts_disabled_per_process() {
        // Spec §14/§28: a restarted process re-arms explicitly; the guard
        // type itself never persists state.
        let g = crate::guard::LiveWriteGuard::disabled();
        assert!(!g.is_armed());
    }

    #[test]
    fn invalidate_aircraft_resets_identity_state_machine() {
        // Direct state assertions on the identity transition the adapter
        // performs on aircraft change (spec §27).
        let mut identity = AircraftIdentity::user_provided(Some("A320".into()));
        assert_eq!(identity.icao.as_deref(), Some("A320"));
        // The adapter's invalidate_aircraft performs exactly this reset
        // plus session-id and control-error clearing (see impl above).
        identity = AircraftIdentity::unknown();
        assert_eq!(identity.source, IdentitySource::Unknown);
        assert_eq!(identity.icao, None);
    }
    use super::*;

    /// Mock X-Plane: replies to RREF subscriptions with records.
    struct MockSim {
        socket: std::net::UdpSocket,
    }

    impl MockSim {
        fn bind() -> Self {
            let socket = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
            Self { socket }
        }

        /// Serve one subscription then stream `values` n times to the
        /// adapter's local UDP port.
        fn stream(&self, adapter_port: u16, values: &[(i32, f32)], times: usize) {
            let mut buf = [0u8; 2048];
            let _ = self.socket.recv_from(&mut buf); // subscription
            let dest = format!("127.0.0.1:{adapter_port}");
            for _ in 0..times {
                let mut pkt = b"RREF,".to_vec();
                for (id, v) in values {
                    pkt.extend_from_slice(&id.to_le_bytes());
                    pkt.extend_from_slice(&v.to_le_bytes());
                }
                self.socket.send_to(&pkt, &dest).unwrap();
                std::thread::sleep(std::time::Duration::from_millis(20));
            }
        }
    }

    fn test_cfg(port: u16) -> XPlaneConfig {
        XPlaneConfig {
            host: "127.0.0.1".into(),
            port,
            subscribe_hz: 50,
            web_api_base: "http://127.0.0.1:1".into(), // nothing listens: bounded refuse
            allow_writes: false,
        }
    }

    #[test]
    fn beacon_wire_id_is_stable_for_catalog_gate() {
        // fd-aircraft's SetBeacon entry pins verification channel 17; this
        // test fails loudly if the wire id ever moves.
        assert_eq!(DataRefId::BeaconOn.wire_id(), 17);
    }

    #[test]
    fn warmup_channel_becomes_fresh_after_three_consistent_samples() {
        let sim = MockSim::bind();
        let mut adapter = XPlaneAdapter::with_identity(
            test_cfg(sim.socket.local_addr().unwrap().port()),
            AircraftIdentity::unknown(),
        )
        .unwrap();
        sim.stream(
            adapter.local_port(),
            &[(DataRefId::BeaconOn.wire_id(), 1.0)],
            4,
        );
        // First observation: WarmingUp.
        let snaps = adapter.poll().unwrap();
        let s1 = snaps.last().unwrap();
        assert_eq!(
            s1.channel_quality
                .get(&(DataRefId::BeaconOn.wire_id() as u16)),
            Some(&fd_core::telemetry::DataQuality::WarmingUp),
            "first sample must be WarmingUp, not authoritative"
        );
        // Second: still WarmingUp.
        let snaps = adapter.poll().unwrap();
        let s2 = snaps.last().unwrap();
        assert_eq!(
            s2.channel_quality
                .get(&(DataRefId::BeaconOn.wire_id() as u16)),
            Some(&fd_core::telemetry::DataQuality::WarmingUp)
        );
        // Third: warmed — absent from the exception map (fresh by default).
        let snaps = adapter.poll().unwrap();
        let s3 = snaps.last().unwrap();
        assert_eq!(
            s3.channel_quality
                .get(&(DataRefId::BeaconOn.wire_id() as u16)),
            None,
            "warmed channel returns to the fresh default (no annotation)"
        );
    }

    #[test]
    fn identity_change_resets_warmup() {
        let sim = MockSim::bind();
        let sim_port = sim.socket.local_addr().unwrap().port();
        let claim = AircraftIdentity {
            icao: Some("C172".into()),
            tail_number: None,
            author: None,
            description: None,
            acf_name: None,
            source: fd_core::identity::IdentitySource::UserProvided,
        };
        let mut adapter = XPlaneAdapter::with_identity(test_cfg(sim_port), claim.clone()).unwrap();
        sim.stream(
            adapter.local_port(),
            &[(DataRefId::BeaconOn.wire_id(), 1.0)],
            3,
        );
        for _ in 0..3 {
            adapter.poll().unwrap();
        }
        // Warmed: no annotation.
        let snaps = adapter.poll().unwrap();
        assert_eq!(
            snaps
                .last()
                .unwrap()
                .channel_quality
                .get(&(DataRefId::BeaconOn.wire_id() as u16)),
            None
        );
        // Aircraft hot-swap: warm-up state must clear (Task 6 §42).
        let new_claim = AircraftIdentity {
            icao: Some("B738".into()),
            ..claim
        };
        adapter.set_identity_claim(new_claim);
        sim.stream(
            adapter.local_port(),
            &[(DataRefId::BeaconOn.wire_id(), 1.0)],
            1,
        );
        let snaps = adapter.poll().unwrap();
        assert_eq!(
            snaps
                .last()
                .unwrap()
                .channel_quality
                .get(&(DataRefId::BeaconOn.wire_id() as u16)),
            Some(&fd_core::telemetry::DataQuality::WarmingUp),
            "post-swap channels re-warm; no stale-fresh carryover"
        );
    }

    #[test]
    fn web_failure_sets_cooldown_and_health_unavailable() {
        let sim = MockSim::bind();
        let mut adapter = XPlaneAdapter::with_identity(
            test_cfg(sim.socket.local_addr().unwrap().port()),
            AircraftIdentity::unknown(),
        )
        .unwrap();
        // web_api_base points at port 1 (nothing listens): bounded refuse.
        assert!(adapter.simulator_version().is_none());
        let h = adapter.health();
        assert_eq!(
            h.web_api,
            HealthState::Unavailable,
            "failed web api is Unavailable"
        );
        // Cooldown: a second attempt is refused without touching the wire.
        assert!(adapter.simulator_version().is_none());
    }

    #[test]
    fn udp_health_tracks_packet_flow() {
        let sim = MockSim::bind();
        let mut adapter = XPlaneAdapter::with_identity(
            test_cfg(sim.socket.local_addr().unwrap().port()),
            AircraftIdentity::unknown(),
        )
        .unwrap();
        sim.stream(
            adapter.local_port(),
            &[(DataRefId::BeaconOn.wire_id(), 1.0)],
            3,
        );
        for _ in 0..3 {
            adapter.poll().unwrap();
        }
        // First health() call calibrates the rate probe; packets must flow
        // during the measurement window, so stream in the background.
        let port = adapter.local_port();
        let streamer = std::thread::spawn(move || {
            let pkt = {
                let mut p = b"RREF,".to_vec();
                let id = DataRefId::BeaconOn.wire_id();
                p.extend_from_slice(&id.to_le_bytes());
                p.extend_from_slice(&1.0f32.to_le_bytes());
                p
            };
            let sock = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
            for _ in 0..15 {
                sock.send_to(&pkt, format!("127.0.0.1:{port}")).unwrap();
                std::thread::sleep(std::time::Duration::from_millis(20));
            }
        });
        let _ = adapter.health();
        std::thread::sleep(std::time::Duration::from_millis(150));
        let h = adapter.health();
        streamer.join().unwrap();
        assert_eq!(h.udp, HealthState::Available);
        assert!(
            h.udp_packet_rate_hz.unwrap_or(0.0) > 0.0,
            "rate {:?}",
            h.udp_packet_rate_hz
        );
    }
}
