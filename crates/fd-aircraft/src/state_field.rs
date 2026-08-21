//! Closed state-field registry: the ONLY fields an aircraft package may
//! reference in conditions.
//!
//! The set is deliberately small (Task 2 slice). Unknown field names fail
//! package validation — there is no string-path reflection into runtime
//! structs.
//!
//! Numeric ids for adapter-populated fields are the single trusted mapping
//! between canonical opaque extension values (`TelemetrySnapshot::
//! aircraft_values`, filled by fd-simconnect) and their typed meaning here.

use serde::{Deserialize, Serialize};

use crate::error::PackageError;

/// Value domain of a state field. Used to reject condition type mismatches
/// at package-validation time (before any evaluation can run).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValueType {
    Boolean,
    Numeric,
}

/// Closed set of package-addressable state fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StateField {
    /// Generic core: aircraft is on ground.
    OnGround,
    /// Generic core: beacon light observed ON.
    BeaconLight,
    /// A32NX: APU RPM in percent of max.
    ApuNPercent,
    /// A32NX: APU bleed air valve open.
    ApuBleedValveOpen,
    /// A32NX: physical flaps handle position 0..=4.
    FlapsHandleIndex,
    /// A32NX: NAV/LOGO light switch enum (0=off, 1=sys1, 2=sys2).
    NavLogoSwitch,
    /// A32NX: PACK 1 pushbutton pressed on.
    Pack1PbOn,
}

/// Stable numeric id of adapter-populated extension values.
///
/// `None` for generic core fields (they live directly on the snapshot).
/// These ids MUST match the constants in `fd-simconnect::bindings` (the
/// adapter writes them); a cross-check test exists there.
impl StateField {
    pub const fn ext_id(self) -> Option<u16> {
        match self {
            Self::OnGround | Self::BeaconLight => None,
            Self::ApuNPercent => Some(1),
            Self::ApuBleedValveOpen => Some(2),
            Self::FlapsHandleIndex => Some(3),
            Self::NavLogoSwitch => Some(4),
            Self::Pack1PbOn => Some(5),
        }
    }

    pub const fn value_type(self) -> ValueType {
        match self {
            Self::OnGround | Self::BeaconLight | Self::ApuBleedValveOpen | Self::Pack1PbOn => {
                ValueType::Boolean
            }
            Self::ApuNPercent | Self::FlapsHandleIndex | Self::NavLogoSwitch => ValueType::Numeric,
        }
    }

    /// Strict name resolution for package data; unknown names are rejected.
    pub fn from_name(name: &str) -> Result<Self, PackageError> {
        // Human-edited TOML: case- and underscore-insensitive resolution.
        let canon = name.to_ascii_lowercase().replace('_', "");
        match canon.as_str() {
            "onground" => Ok(Self::OnGround),
            "beaconlight" => Ok(Self::BeaconLight),
            "apunpercent" => Ok(Self::ApuNPercent),
            "apubleedvalveopen" => Ok(Self::ApuBleedValveOpen),
            "flapshandleindex" => Ok(Self::FlapsHandleIndex),
            "navlogoswitch" => Ok(Self::NavLogoSwitch),
            "pack1pbon" => Ok(Self::Pack1PbOn),
            other => Err(PackageError::UnknownStateField(other.to_string())),
        }
    }
}
