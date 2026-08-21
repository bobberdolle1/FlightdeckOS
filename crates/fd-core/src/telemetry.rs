//! Canonical telemetry state: a minimal generic core plus a small,
//! aircraft-specific extension for the A32NX.
//!
//! Task 1 deliberately does NOT build a giant universal aircraft schema.
//! The generic core contains only what is genuinely cross-aircraft and what
//! the flight phase engine needs. Aircraft-specific values live in
//! [`A32NxState`], which is a placeholder for aircraft-package-provided state
//! (Phase 2 will move it into the aircraft package). Missing data is
//! represented as `None` — FlightdeckOS never fabricates values.
//!
//! All fields are optional because a simulator read may legitimately be
//! unavailable for an aircraft/binding; "unknown" is a first-class state.

use serde::{Deserialize, Serialize};

use crate::units::{
    AltitudeAglFt, AltitudeFt, AngleDeg, LatDeg, LonDeg, Percent, SpeedKt, VerticalSpeedFpm,
};

/// Logical simulation timestamp in milliseconds.
///
/// Sourced from the simulator clock (`ABSOLUTE TIME`) when available; in
/// replay mode it is injected by the fixture. Ordering MUST rely on this
/// value plus the monotonic event sequence — never on wall clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct SimTimestamp {
    pub ms: u64,
}

impl SimTimestamp {
    pub const fn new(ms: u64) -> Self {
        Self { ms }
    }
}

/// Geographic position.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Position {
    pub lat: LatDeg,
    pub lon: LonDeg,
}

/// Coarse simulator execution state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimState {
    Running,
    Paused,
    #[default]
    Unknown,
}

/// Simulator timing awareness: pause / sim-rate / slew.
///
/// First-class runtime concerns (Task 1 §12): during pause no derived events
/// may be generated; slew/teleport must not produce false phase sequences.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SimTiming {
    pub state: SimState,
    /// Simulation rate (1.0 = real time). `None` = not reported by simulator.
    pub sim_rate: Option<f64>,
    /// Slew mode active flag. `None` = not reported.
    pub slew_active: Option<bool>,
}

impl Default for SimTiming {
    fn default() -> Self {
        Self {
            state: SimState::Unknown,
            sim_rate: None,
            slew_active: None,
        }
    }
}

/// NAV/LOGO light switch position on the A32NX (documented enum, 0/1/2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NavLogoMode {
    Off,
    Sys1,
    Sys2,
}

impl NavLogoMode {
    /// Raw A32NX `A32NX_LIGHTS_NAV_LOGO` numeric value.
    pub const fn raw(self) -> f64 {
        match self {
            Self::Off => 0.0,
            Self::Sys1 => 1.0,
            Self::Sys2 => 2.0,
        }
    }

    pub const fn from_raw(v: f64) -> Option<Self> {
        match v as u8 {
            0 => Some(Self::Off),
            1 => Some(Self::Sys1),
            2 => Some(Self::Sys2),
            _ => None,
        }
    }
}

/// A32NX-specific canonical state (Task 1 subset — 5 proven read bindings).
///
/// These are *logical* values. The raw `L:`-var names are mapped in
/// `fd-simconnect::bindings` and never appear here.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct A32NxState {
    /// APU RPM, percent of max. Source: `A32NX_APU_N`.
    pub apu_n_percent: Option<Percent>,
    /// APU bleed air valve open. Source: `A32NX_APU_BLEED_AIR_VALVE_OPEN`.
    pub apu_bleed_valve_open: Option<bool>,
    /// Physical flaps handle position 0..=4. Source: `A32NX_FLAPS_HANDLE_INDEX`.
    pub flaps_handle_index: Option<u8>,
    /// NAV/LOGO switch. Source: `A32NX_LIGHTS_NAV_LOGO`.
    pub nav_logo: Option<NavLogoMode>,
    /// PACK 1 pushbutton pressed (on). Source: `A32NX_OVHD_COND_PACK_1_PB_IS_ON`.
    pub pack_1_pb_on: Option<bool>,
}

/// A single canonical snapshot of aircraft + simulator state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TelemetrySnapshot {
    pub timestamp: SimTimestamp,

    // -- generic core -------------------------------------------------------
    pub position: Option<Position>,
    pub altitude_msl: Option<AltitudeFt>,
    pub altitude_agl: Option<AltitudeAglFt>,
    pub groundspeed: Option<SpeedKt>,
    pub indicated_airspeed: Option<SpeedKt>,
    pub vertical_speed: Option<VerticalSpeedFpm>,
    pub heading_true: Option<AngleDeg>,
    pub pitch: Option<AngleDeg>,
    pub bank: Option<AngleDeg>,
    pub on_ground: Option<bool>,
    /// Gear lever selected down (true = down).
    pub gear_handle_down: Option<bool>,
    /// Generic flaps handle index (0 = up).
    pub flaps_handle_index: Option<u8>,
    /// Engine combustion per engine slot (1..=4); `None` = unknown engine.
    pub engine_combustion: Option<[Option<bool>; 4]>,
    pub autopilot_master: Option<bool>,
    pub autothrottle_arm: Option<bool>,
    /// Beacon light state (standard `LIGHT BEACON` simvar).
    pub beacon_light: Option<bool>,

    // -- simulator timing ---------------------------------------------------
    pub sim_timing: SimTiming,

    // -- aircraft-specific extension -----------------------------------------
    pub a32nx: A32NxState,
}

impl TelemetrySnapshot {
    /// Empty (all-unknown) snapshot at a timestamp.
    pub const fn empty(timestamp: SimTimestamp) -> Self {
        Self {
            timestamp,
            position: None,
            altitude_msl: None,
            altitude_agl: None,
            groundspeed: None,
            indicated_airspeed: None,
            vertical_speed: None,
            heading_true: None,
            pitch: None,
            bank: None,
            on_ground: None,
            gear_handle_down: None,
            flaps_handle_index: None,
            engine_combustion: None,
            autopilot_master: None,
            autothrottle_arm: None,
            beacon_light: None,
            sim_timing: SimTiming {
                state: SimState::Unknown,
                sim_rate: None,
                slew_active: None,
            },
            a32nx: A32NxState {
                apu_n_percent: None,
                apu_bleed_valve_open: None,
                flaps_handle_index: None,
                nav_logo: None,
                pack_1_pb_on: None,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_snapshot_is_all_unknown() {
        let s = TelemetrySnapshot::empty(SimTimestamp::new(0));
        assert!(s.altitude_msl.is_none());
        assert!(s.on_ground.is_none());
        assert_eq!(s.sim_timing.state, SimState::Unknown);
        assert!(s.a32nx.apu_n_percent.is_none());
    }

    #[test]
    fn nav_logo_mode_raw_mapping_is_total_and_injective() {
        for m in [NavLogoMode::Off, NavLogoMode::Sys1, NavLogoMode::Sys2] {
            assert_eq!(NavLogoMode::from_raw(m.raw()), Some(m));
        }
        assert_eq!(NavLogoMode::from_raw(7.0), None);
    }

    #[test]
    fn timestamp_ordering_is_by_ms() {
        assert!(SimTimestamp::new(1) < SimTimestamp::new(2));
        assert_eq!(SimTimestamp::new(5), SimTimestamp::new(5));
    }
}
