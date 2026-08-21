//! Allowlisted dataref/command table for the X-Plane adapter.
//!
//! Task 4 §10: every write is a named, typed entry against THIS table.
//! There is no generic string-write surface. Units and sign conventions are
//! verified against the X-Plane 12.4.3 `DataRefs.txt` shipped with the
//! target installation.

/// Subscribed telemetry datarefs (read path).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataRefId {
    Latitude,
    Longitude,
    ElevationM,
    YAglM,
    IndicatedAirspeedKt,
    GroundspeedMs,
    VerticalSpeedFpm,
    HeadingTrueDeg,
    PitchDeg,
    BankDeg,
    OnGroundWheel0,
    GearDeploy0,
    MagVariationDeg,
    ApHeadingStatus,
    ApVviStatus,
    /// The altitude dialed into the AP (read-back of the write target).
    ApAltitudeTarget,
}

impl DataRefId {
    /// Stable wire id used in RREF0 subscriptions.
    pub const fn wire_id(self) -> i32 {
        self as i32
    }

    pub const ALL: [(DataRefId, &'static str); 16] = [
        (DataRefId::Latitude, "sim/flightmodel/position/latitude"),
        (DataRefId::Longitude, "sim/flightmodel/position/longitude"),
        (DataRefId::ElevationM, "sim/flightmodel/position/elevation"),
        (DataRefId::YAglM, "sim/flightmodel/position/y_agl"),
        (
            DataRefId::IndicatedAirspeedKt,
            "sim/flightmodel/position/indicated_airspeed",
        ),
        (
            DataRefId::GroundspeedMs,
            "sim/flightmodel/position/groundspeed",
        ),
        (
            DataRefId::VerticalSpeedFpm,
            "sim/flightmodel/position/vh_ind_fpm",
        ),
        (DataRefId::HeadingTrueDeg, "sim/flightmodel/position/psi"),
        (DataRefId::PitchDeg, "sim/flightmodel/position/theta"),
        (DataRefId::BankDeg, "sim/flightmodel/position/phi"),
        (
            DataRefId::OnGroundWheel0,
            "sim/flightmodel2/gear/on_ground[0]",
        ),
        (
            DataRefId::GearDeploy0,
            "sim/aircraft/parts/acf_gear_deploy[0]",
        ),
        (
            DataRefId::MagVariationDeg,
            "sim/flightmodel/position/magnetic_variation",
        ),
        (
            DataRefId::ApHeadingStatus,
            "sim/cockpit2/autopilot/heading_status",
        ),
        (DataRefId::ApVviStatus, "sim/cockpit2/autopilot/vvi_status"),
        (
            DataRefId::ApAltitudeTarget,
            "sim/cockpit/autopilot/altitude",
        ),
    ];

    pub fn path(self) -> &'static str {
        Self::ALL
            .iter()
            .find(|(id, _)| *id == self)
            .map(|(_, p)| *p)
            .expect("every DataRefId must appear in ALL")
    }
}

/// Extra read-only AP target used by the closed-loop smoke (the dialed
/// altitude). Declared separately so `ALL` stays purely telemetry + status.
#[derive(Debug, Clone, Copy)]
/// Allowlisted WRITE datarefs (autopilot targets only).
pub enum WriteRef {
    /// Magnetic heading target, degrees magnetic.
    ApHeadingMag,
    /// Altitude target, feet MSL.
    ApAltitude,
    /// Vertical speed target, fpm.
    ApVerticalVelocity,
    /// Airspeed target, knots (below the knots/mach transition).
    ApAirspeed,
}

impl WriteRef {
    pub fn path(self) -> &'static str {
        match self {
            WriteRef::ApHeadingMag => "sim/cockpit/autopilot/heading_mag",
            WriteRef::ApAltitude => "sim/cockpit/autopilot/altitude",
            WriteRef::ApVerticalVelocity => "sim/cockpit/autopilot/vertical_velocity",
            WriteRef::ApAirspeed => "sim/cockpit/autopilot/airspeed",
        }
    }
}

/// Allowlisted commands (autopilot mode engagement only).
pub enum Command {
    ApHeadingHold,
    ApVerticalSpeedPreSel,
}

impl Command {
    pub fn path(self) -> &'static str {
        match self {
            Command::ApHeadingHold => "sim/autopilot/heading_hold",
            Command::ApVerticalSpeedPreSel => "sim/autopilot/vertical_speed_pre_sel",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wire_ids_unique() {
        let mut ids: Vec<i32> = DataRefId::ALL.iter().map(|(id, _)| id.wire_id()).collect();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), DataRefId::ALL.len());
    }

    #[test]
    fn known_paths() {
        assert_eq!(
            DataRefId::VerticalSpeedFpm.path(),
            "sim/flightmodel/position/vh_ind_fpm"
        );
        assert_eq!(
            WriteRef::ApHeadingMag.path(),
            "sim/cockpit/autopilot/heading_mag"
        );
        assert_eq!(Command::ApHeadingHold.path(), "sim/autopilot/heading_hold");
    }
}
