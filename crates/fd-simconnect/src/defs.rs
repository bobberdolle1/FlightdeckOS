//! SimConnect constants: data definition, request ids, client event ids,
//! and the telemetry datum table.
//!
//! The datum table order IS the wire order of the `FLOAT64` payload in
//! `SIMCONNECT_RECV_SIMOBJECT_DATA`. Indexes are used by `mapping.rs`;
//! changing order changes the mapping — keep both in lockstep.

use crate::ffi;

/// Data definition id for the telemetry snapshot.
pub const DEFINE_TELEMETRY: ffi::DWORD = 1;
/// Data request id for the periodic telemetry request.
pub const REQUEST_TELEMETRY: ffi::DWORD = 1;
/// Data definition id reused for single-variable writes.
pub const DEFINE_WRITE: ffi::DWORD = 2;

/// Client event ids (subscriptions and one-shot action events).
pub const EVT_PAUSE: ffi::DWORD = 1;
pub const EVT_UNPAUSE: ffi::DWORD = 2;
pub const EVT_ACTION: ffi::DWORD = 3;

/// System event names (documented in the MSFS SDK). Both the classic and
/// MSFS-era spellings are subscribed; the `PAUSED` simvar in the datum table
/// remains the authoritative source.
pub const SYSTEM_EVENT_PAUSE_NAMES: &[&str] = &["Pause", "Paused"];
pub const SYSTEM_EVENT_UNPAUSE_NAMES: &[&str] = &["Unpause", "Unpaused"];

/// `(datum name, units)` pairs for the telemetry definition. Order = wire
/// order. All values are requested as `FLOAT64`.
pub const DATUMS: &[(&str, &str)] = &[
    ("PLANE LATITUDE", "degrees"),
    ("PLANE LONGITUDE", "degrees"),
    ("PLANE ALTITUDE", "feet"),
    ("PLANE ALT ABOVE GROUND", "feet"),
    ("GROUND VELOCITY", "knots"),
    ("AIRSPEED INDICATED", "knots"),
    ("VERTICAL SPEED", "feet per minute"),
    ("PLANE HEADING DEGREES TRUE", "degrees"),
    ("PLANE PITCH DEGREES", "radians"),
    ("PLANE BANK DEGREES", "radians"),
    ("SIM ON GROUND", "bool"),
    ("GEAR HANDLE POSITION", "bool"),
    ("FLAPS HANDLE INDEX", "number"),
    ("GENERAL ENG COMBUSTION:1", "bool"),
    ("GENERAL ENG COMBUSTION:2", "bool"),
    ("AUTOPILOT MASTER", "bool"),
    ("AUTOPILOT THROTTLE ARM", "bool"),
    ("LIGHT BEACON", "bool"),
    ("PAUSED", "bool"),
    ("SIMULATION RATE", "number"),
    ("IS SLEW ACTIVE", "bool"),
    ("L:A32NX_APU_N", "percent"),
    ("L:A32NX_APU_BLEED_AIR_VALVE_OPEN", "bool"),
    ("L:A32NX_FLAPS_HANDLE_INDEX", "number"),
    ("L:A32NX_LIGHTS_NAV_LOGO", "number"),
    ("L:A32NX_OVHD_COND_PACK_1_PB_IS_ON", "bool"),
    ("ABSOLUTE TIME", "seconds"),
];

pub const DATUM_COUNT: usize = DATUMS.len();

// Datum indexes (must match DATUMS order).
pub const IDX_LAT: usize = 0;
pub const IDX_LON: usize = 1;
pub const IDX_ALT_MSL: usize = 2;
pub const IDX_ALT_AGL: usize = 3;
pub const IDX_GS: usize = 4;
pub const IDX_IAS: usize = 5;
pub const IDX_VS: usize = 6;
pub const IDX_HEADING: usize = 7;
pub const IDX_PITCH: usize = 8;
pub const IDX_BANK: usize = 9;
pub const IDX_ON_GROUND: usize = 10;
pub const IDX_GEAR: usize = 11;
pub const IDX_FLAPS: usize = 12;
pub const IDX_ENG1: usize = 13;
pub const IDX_ENG2: usize = 14;
pub const IDX_AP: usize = 15;
pub const IDX_ATHR: usize = 16;
pub const IDX_BEACON: usize = 17;
pub const IDX_PAUSED: usize = 18;
pub const IDX_SIM_RATE: usize = 19;
pub const IDX_SLEW: usize = 20;
pub const IDX_A32NX_APU_N: usize = 21;
pub const IDX_A32NX_APU_BLEED: usize = 22;
pub const IDX_A32NX_FLAPS: usize = 23;
pub const IDX_A32NX_NAV_LOGO: usize = 24;
pub const IDX_A32NX_PACK1: usize = 25;
pub const IDX_ABS_TIME: usize = 26;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn index_constants_match_datum_order() {
        assert_eq!(DATUMS[IDX_LAT].0, "PLANE LATITUDE");
        assert_eq!(DATUMS[IDX_A32NX_APU_N].0, "L:A32NX_APU_N");
        assert_eq!(DATUMS[IDX_ABS_TIME].0, "ABSOLUTE TIME");
        assert_eq!(DATUM_COUNT, 27);
    }
}
