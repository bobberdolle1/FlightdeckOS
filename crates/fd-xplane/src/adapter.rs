//! `XPlaneSimulatorAdapter`: the live X-Plane 12 implementation of the
//! `fd-core` `SimulatorAdapter` + `FlightControlTargets` boundaries.
//!
//! * Telemetry: only from received UDP packets — never fabricated, never
//!   substituted with virtual state (Task 4 §7/§17).
//! * Control: allowlisted autopilot-target writes only (Task 4 §10).
//! * Disconnect: typed `NotConnected` on stale packets; `Degraded`
//!   capability while disconnected; automatic resubscribe on reconnect
//!   (Task 4 §16).

use std::time::{Duration, Instant};

use fd_core::actions::CockpitAction;
use fd_core::adapter::{AdapterError, Capability, FlightControlTargets, SimulatorAdapter};
use fd_core::telemetry::{SimState, SimTimestamp, TelemetrySnapshot};
use fd_core::units::{
    AltitudeAglFt, AltitudeFt, AngleDeg, LatDeg, LonDeg, SpeedKt, VerticalSpeedFpm,
};

use crate::client::XPlaneUdpClient;
use crate::refs::{Command, DataRefId, WriteRef};

const M_TO_FT: f64 = 3.280_839_895;
const MS_TO_KT: f64 = 1.943_844_492;

/// Connection/behavior configuration.
#[derive(Debug, Clone)]
pub struct XPlaneConfig {
    pub host: String,
    pub port: u16,
    /// Dataref stream frequency in Hz (X-Plane default UI is 4-30).
    pub subscribe_hz: i32,
}

impl Default for XPlaneConfig {
    fn default() -> Self {
        Self {
            host: "127.0.0.1".into(),
            port: 49000,
            subscribe_hz: 4,
        }
    }
}

/// Live adapter over the native X-Plane UDP transport.
pub struct XPlaneAdapter {
    client: XPlaneUdpClient,
    /// Snapshot observed at the moment connection was lost (diagnostics).
    disconnects: u64,
    last_poll_duration: Option<Duration>,
}

impl XPlaneAdapter {
    pub fn new(cfg: XPlaneConfig) -> Result<Self, AdapterError> {
        let mut client = XPlaneUdpClient::new(0, &cfg.host, cfg.port)
            .map_err(|e| AdapterError::ConnectionFailed(e.to_string()))?;
        let refs: Vec<(i32, &'static str)> = DataRefId::ALL
            .iter()
            .map(|(id, p)| (id.wire_id(), *p))
            .collect();
        client
            .start(cfg.subscribe_hz, &refs)
            .map_err(|e| AdapterError::ConnectionFailed(e.to_string()))?;
        Ok(Self {
            client,
            disconnects: 0,
            last_poll_duration: None,
        })
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
    fn write_ap(&self, r: WriteRef, value: f32) -> Result<(), AdapterError> {
        self.guard_connected()?;
        self.client
            .write_dref(r.path(), value)
            .map_err(|e| AdapterError::WriteFailed(e.to_string()))
    }

    fn engage(&self, c: Command) -> Result<(), AdapterError> {
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

    fn build_snapshot(&self) -> TelemetrySnapshot {
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
        let agl = self
            .value(DataRefId::YAglM)
            .map(|m| AltitudeAglFt::new(m * M_TO_FT));
        s.altitude_agl = agl;
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
        s
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

    fn capability(&self, _action: CockpitAction) -> Capability {
        // No discrete cockpit actions are allowlisted in this slice; the
        // live control path is FlightControlTargets (autopilot targets).
        if self.client.connected() {
            Capability::Unsupported
        } else {
            Capability::Unavailable
        }
    }

    fn execute(&mut self, _action: CockpitAction) -> Result<(), AdapterError> {
        Err(AdapterError::UnsupportedAction)
    }
}

impl FlightControlTargets for XPlaneAdapter {
    fn flight_guidance_supported(&self) -> bool {
        self.client.connected()
    }

    fn set_target_altitude(&mut self, altitude_ft: f64) {
        // Target write only: mode engagement is aircraft-specific and is
        // NOT auto-pressed in this slice (reported honestly as unproven).
        let _ = self.write_ap(WriteRef::ApAltitude, altitude_ft as f32);
    }

    fn set_target_speed(&mut self, speed_kt: f64) {
        let _ = self.write_ap(WriteRef::ApAirspeed, speed_kt as f32);
    }

    fn set_target_heading(&mut self, heading_deg: f64) {
        let magvar = self.value(DataRefId::MagVariationDeg).unwrap_or(0.0);
        if self
            .write_ap(WriteRef::ApHeadingMag, true_to_mag(heading_deg, magvar))
            .is_ok()
        {
            // Engage HDG mode only when it is currently OFF (status 0).
            // Status 2 = captured; re-sending the command would TOGGLE it
            // off, so gate on the observed status.
            if self.value(DataRefId::ApHeadingStatus).unwrap_or(0.0) < 2.0 {
                let _ = self.engage(Command::ApHeadingHold);
            }
        }
    }

    fn set_target_vertical_speed(&mut self, fpm: f64) {
        if self
            .write_ap(WriteRef::ApVerticalVelocity, fpm as f32)
            .is_ok()
        {
            if self.value(DataRefId::ApVviStatus).unwrap_or(0.0) < 2.0 {
                let _ = self.engage(Command::ApVerticalSpeedPreSel);
            }
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
}
