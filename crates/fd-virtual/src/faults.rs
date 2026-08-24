//! Deterministic fault injection (spec §41).
//!
//! Every knob is a pure function of the simulator's tick counter: no RNG,
//! no wall clock. A given [`FaultConfig`] plus an identical action sequence
//! always produces byte-identical snapshots — fault runs are replayable
//! exactly like nominal ones.
//!
//! Semantics:
//! * [`FaultConfig::ignore_actions_for_ticks`] — the adapter ACCEPTS every
//!   cockpit action (`execute` returns `Ok(())`) but never applies its
//!   postcondition while the tick counter is inside the window. This models
//!   a fire-and-forget write that lies about success; the runtime's
//!   post-condition verifier must catch it.
//! * [`FaultConfig::telemetry_freeze_until_tick`] — the world (kinematics,
//!   systems AND its clock) stops advancing entirely until this tick;
//!   polls return the same frozen snapshot, timestamp included.
//! * [`FaultConfig::unknown_sensor_fields`] — named canonical telemetry
//!   fields read back as `None` (unknown) instead of their true value.
//!   Names must be members of [`MASKABLE_FIELDS`].
//! * [`FaultConfig::disconnect_until_tick`] — `is_connected()` reports
//!   false and poll/execute fail with [`fd_core::adapter::AdapterError::NotConnected`]
//!   until this tick.

use serde::{Deserialize, Serialize};

/// Closed set of canonical snapshot fields that can be masked to `None`
/// by [`FaultConfig::unknown_sensor_fields`]. Deliberately small: exactly
/// the fields [`crate::VirtualSimulator`] populates.
pub const MASKABLE_FIELDS: [&str; 10] = [
    "position",
    "altitude_msl",
    "altitude_agl",
    "groundspeed",
    "ias",
    "vertical_speed",
    "heading_true",
    "pitch",
    "bank",
    "on_ground",
];

/// Deterministic fault injection configuration. `Default` = no faults
/// (the nominal simulator path is bit-for-bit unchanged).
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct FaultConfig {
    /// Adapter accepts actions but never applies their postcondition for
    /// the first N ticks (`tick < N`).
    pub ignore_actions_for_ticks: u64,
    /// World is fully frozen (snapshot identical every poll) while
    /// `tick < N`.
    pub telemetry_freeze_until_tick: u64,
    /// Canonical telemetry fields forced to unknown (`None`). Must be a
    /// subset of [`MASKABLE_FIELDS`]; see [`FaultConfig::validate`].
    pub unknown_sensor_fields: Vec<String>,
    /// Adapter reports disconnected (poll/execute fail) while `tick < N`.
    pub disconnect_until_tick: u64,
}

impl FaultConfig {
    /// Resolve accepted alias names to canonical [`MASKABLE_FIELDS`] names.
    /// Unknown names return `None`.
    pub fn canonical_field(name: &str) -> Option<&'static str> {
        match name {
            "vs" => Some("vertical_speed"),
            other => MASKABLE_FIELDS.iter().find(|f| **f == other).copied(),
        }
    }

    /// Fail-closed validation: reject mask names outside the closed set
    /// (after alias resolution) so typos cannot silently disable a fault.
    pub fn validate(&self) -> Result<(), String> {
        for name in &self.unknown_sensor_fields {
            if Self::canonical_field(name).is_none() {
                return Err(format!(
                    "unknown sensor field `{name}` (maskable fields: {})",
                    MASKABLE_FIELDS.join(", ")
                ));
            }
        }
        Ok(())
    }

    /// Copy with alias names resolved to canonical names, so downstream
    /// matching only ever sees [`MASKABLE_FIELDS`] members.
    pub fn normalized(&self) -> Self {
        let mut unknown_sensor_fields: Vec<String> = self
            .unknown_sensor_fields
            .iter()
            .map(|n| Self::canonical_field(n).unwrap_or(n).to_string())
            .collect();
        unknown_sensor_fields.sort();
        unknown_sensor_fields.dedup();
        Self {
            ignore_actions_for_ticks: self.ignore_actions_for_ticks,
            telemetry_freeze_until_tick: self.telemetry_freeze_until_tick,
            unknown_sensor_fields,
            disconnect_until_tick: self.disconnect_until_tick,
        }
    }

    /// True when no fault is configured at all.
    pub const fn is_noop(&self) -> bool {
        self.ignore_actions_for_ticks == 0
            && self.telemetry_freeze_until_tick == 0
            && self.disconnect_until_tick == 0
            && self.unknown_sensor_fields.is_empty()
    }
}
