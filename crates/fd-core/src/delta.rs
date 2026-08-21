//! State deltas: named changed-field sets between consecutive snapshots.
//!
//! The runtime emits a [`StateDelta`] per snapshot whose relevant fields
//! differ from the previous one. Field comparison is plain equality on the
//! typed values — no tolerance logic yet (tolerances are a future
//! aircraft-package concern, not a core one).

use serde::{Deserialize, Serialize};

use crate::events::{EventSeq, EventSource};
use crate::telemetry::{SimTimestamp, TelemetrySnapshot};

/// A changed field of a snapshot, used for delta reporting and triggering.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeltaField {
    Position,
    AltitudeMsl,
    AltitudeAgl,
    Groundspeed,
    IndicatedAirspeed,
    VerticalSpeed,
    HeadingTrue,
    Pitch,
    Bank,
    OnGround,
    GearHandleDown,
    FlapsHandleIndex,
    EngineCombustion,
    AutopilotMaster,
    AutothrottleArm,
    BeaconLight,
    SimState,
    SimRate,
    SlewActive,
    /// An aircraft-specific extension value changed. The payload is the
    /// opaque numeric field id (meaning owned by the aircraft layer).
    AircraftValue(u16),
}

/// An observable state event with a monotonic sequence number.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StateDelta {
    /// Monotonic sequence within the runtime session.
    pub seq: EventSeq,
    pub timestamp: SimTimestamp,
    /// Where this delta came from: live simulator, replay fixture, or derived.
    pub source: EventSource,
    /// Changed fields. Empty only if nothing observable changed (the runtime
    /// does not emit empty deltas).
    pub changed: Vec<DeltaField>,
}

/// Compare two snapshots and return the list of changed fields.
///
/// Deterministic: pure function of its inputs.
pub fn diff(prev: &TelemetrySnapshot, next: &TelemetrySnapshot) -> Vec<DeltaField> {
    let mut changed = Vec::new();
    let mut chk = |cond: bool, field: DeltaField| {
        if cond {
            changed.push(field);
        }
    };

    chk(prev.position != next.position, DeltaField::Position);
    chk(
        prev.altitude_msl != next.altitude_msl,
        DeltaField::AltitudeMsl,
    );
    chk(
        prev.altitude_agl != next.altitude_agl,
        DeltaField::AltitudeAgl,
    );
    chk(
        prev.groundspeed != next.groundspeed,
        DeltaField::Groundspeed,
    );
    chk(
        prev.indicated_airspeed != next.indicated_airspeed,
        DeltaField::IndicatedAirspeed,
    );
    chk(
        prev.vertical_speed != next.vertical_speed,
        DeltaField::VerticalSpeed,
    );
    chk(
        prev.heading_true != next.heading_true,
        DeltaField::HeadingTrue,
    );
    chk(prev.pitch != next.pitch, DeltaField::Pitch);
    chk(prev.bank != next.bank, DeltaField::Bank);
    chk(prev.on_ground != next.on_ground, DeltaField::OnGround);
    chk(
        prev.gear_handle_down != next.gear_handle_down,
        DeltaField::GearHandleDown,
    );
    chk(
        prev.flaps_handle_index != next.flaps_handle_index,
        DeltaField::FlapsHandleIndex,
    );
    chk(
        prev.engine_combustion != next.engine_combustion,
        DeltaField::EngineCombustion,
    );
    chk(
        prev.autopilot_master != next.autopilot_master,
        DeltaField::AutopilotMaster,
    );
    chk(
        prev.autothrottle_arm != next.autothrottle_arm,
        DeltaField::AutothrottleArm,
    );
    chk(
        prev.beacon_light != next.beacon_light,
        DeltaField::BeaconLight,
    );
    chk(
        prev.sim_timing.state != next.sim_timing.state,
        DeltaField::SimState,
    );
    chk(
        prev.sim_timing.sim_rate != next.sim_timing.sim_rate,
        DeltaField::SimRate,
    );
    chk(
        prev.sim_timing.slew_active != next.sim_timing.slew_active,
        DeltaField::SlewActive,
    );
    // Aircraft-specific extension values: compare the union of keys.
    let mut ext_keys: std::collections::BTreeSet<u16> =
        prev.aircraft_values.keys().copied().collect();
    ext_keys.extend(next.aircraft_values.keys().copied());
    for key in ext_keys {
        if prev.aircraft_values.get(&key) != next.aircraft_values.get(&key) {
            changed.push(DeltaField::AircraftValue(key));
        }
    }

    changed
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::{SpeedKt, VerticalSpeedFpm};

    fn snap(gs: f64) -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(0));
        s.groundspeed = Some(SpeedKt::new(gs));
        s
    }

    #[test]
    fn identical_snapshots_produce_empty_diff() {
        let a = snap(10.0);
        let b = snap(10.0);
        assert!(diff(&a, &b).is_empty());
    }

    #[test]
    fn single_field_change_is_reported() {
        let a = snap(10.0);
        let mut b = snap(20.0);
        b.vertical_speed = Some(VerticalSpeedFpm::new(0.0));
        let d = diff(&a, &b);
        assert_eq!(d, vec![DeltaField::Groundspeed, DeltaField::VerticalSpeed]);
    }

    #[test]
    fn timing_state_change_is_reported() {
        let mut a = snap(0.0);
        a.sim_timing.state = crate::telemetry::SimState::Running;
        let mut b = a.clone();
        b.sim_timing.state = crate::telemetry::SimState::Paused;
        assert_eq!(diff(&a, &b), vec![DeltaField::SimState]);
    }

    #[test]
    fn aircraft_extension_changes_are_reported() {
        let a = snap(0.0);
        let mut b = a.clone();
        b.aircraft_values.insert(7, 99.0);
        assert_eq!(diff(&a, &b), vec![DeltaField::AircraftValue(7)]);
    }
}
