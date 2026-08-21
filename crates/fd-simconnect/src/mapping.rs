//! Pure mapping of raw `FLOAT64` datum arrays to canonical snapshots.
//!
//! Fail-closed semantics: non-finite or out-of-range values map to `None`
//! (unknown) — FlightdeckOS never fabricates state.

use fd_core::telemetry::{
    A32NxState, NavLogoMode, Position, SimState, SimTimestamp, SimTiming, TelemetrySnapshot,
};
use fd_core::units::{
    AltitudeAglFt, AltitudeFt, AngleDeg, LatDeg, LonDeg, Percent, SpeedKt, VerticalSpeedFpm,
};

use crate::defs::*;

fn as_bool(v: f64) -> Option<bool> {
    if !v.is_finite() { None } else { Some(v != 0.0) }
}

fn as_number(v: f64) -> Option<f64> {
    if v.is_finite() { Some(v) } else { None }
}

/// Map a raw datum array to a canonical snapshot (pure).
///
/// `event_paused`: last-known pause state from system events; used only when
/// the authoritative `PAUSED` simvar is absent from the payload.
pub fn map_telemetry(values: &[f64], event_paused: Option<bool>) -> TelemetrySnapshot {
    let v = |i: usize| values.get(i).copied().unwrap_or(f64::NAN);

    let abs_time_secs = as_number(v(IDX_ABS_TIME));
    let timestamp = SimTimestamp::new(
        abs_time_secs
            .map(|s| (s.max(0.0) * 1000.0) as u64)
            .unwrap_or(0),
    );

    let paused = as_bool(v(IDX_PAUSED)).or(event_paused);

    let mut snapshot = TelemetrySnapshot::empty(timestamp);

    let lat = as_number(v(IDX_LAT));
    let lon = as_number(v(IDX_LON));
    if let (Some(lat), Some(lon)) = (lat, lon)
        && lat.abs() <= 90.0
        && lon.abs() <= 180.0
    {
        snapshot.position = Some(Position {
            lat: LatDeg::new(lat),
            lon: LonDeg::new(lon),
        });
    }

    snapshot.altitude_msl = as_number(v(IDX_ALT_MSL)).map(AltitudeFt::new);
    snapshot.altitude_agl = as_number(v(IDX_ALT_AGL)).map(AltitudeAglFt::new);
    snapshot.groundspeed = as_number(v(IDX_GS)).map(SpeedKt::new);
    snapshot.indicated_airspeed = as_number(v(IDX_IAS)).map(SpeedKt::new);
    snapshot.vertical_speed = as_number(v(IDX_VS)).map(VerticalSpeedFpm::new);
    snapshot.heading_true = as_number(v(IDX_HEADING)).map(|d| AngleDeg::new(d.rem_euclid(360.0)));
    snapshot.pitch = as_number(v(IDX_PITCH)).map(|r| AngleDeg::new(r.to_degrees()));
    snapshot.bank = as_number(v(IDX_BANK)).map(|r| AngleDeg::new(r.to_degrees()));
    snapshot.on_ground = as_bool(v(IDX_ON_GROUND));
    snapshot.gear_handle_down = as_bool(v(IDX_GEAR));

    let flaps = v(IDX_FLAPS);
    if flaps.is_finite() && (0.0..=4.0).contains(&flaps) && flaps.fract() == 0.0 {
        snapshot.flaps_handle_index = Some(flaps as u8);
    }

    let eng1 = as_bool(v(IDX_ENG1));
    let eng2 = as_bool(v(IDX_ENG2));
    if eng1.is_some() || eng2.is_some() {
        snapshot.engine_combustion = Some([eng1, eng2, None, None]);
    }

    snapshot.autopilot_master = as_bool(v(IDX_AP));
    snapshot.autothrottle_arm = as_bool(v(IDX_ATHR));
    snapshot.beacon_light = as_bool(v(IDX_BEACON));

    snapshot.sim_timing = SimTiming {
        state: match paused {
            None => SimState::Unknown,
            Some(true) => SimState::Paused,
            Some(false) => SimState::Running,
        },
        sim_rate: as_number(v(IDX_SIM_RATE)).filter(|r| *r >= 0.0),
        slew_active: as_bool(v(IDX_SLEW)),
    };

    snapshot.a32nx = A32NxState {
        apu_n_percent: as_number(v(IDX_A32NX_APU_N)).map(Percent::new),
        apu_bleed_valve_open: as_bool(v(IDX_A32NX_APU_BLEED)),
        flaps_handle_index: {
            let f = v(IDX_A32NX_FLAPS);
            if f.is_finite() && (0.0..=4.0).contains(&f) && f.fract() == 0.0 {
                Some(f as u8)
            } else {
                None
            }
        },
        nav_logo: NavLogoMode::from_raw(v(IDX_A32NX_NAV_LOGO)),
        pack_1_pb_on: as_bool(v(IDX_A32NX_PACK1)),
    };

    snapshot
}

#[cfg(test)]
mod tests {
    use super::*;

    fn zero_values() -> Vec<f64> {
        vec![0.0; DATUM_COUNT]
    }

    #[test]
    fn zero_payload_maps_to_defined_defaults() {
        let s = map_telemetry(&zero_values(), None);
        // Zero is a real observation for these bools, not "unknown".
        assert_eq!(s.on_ground, Some(false));
        assert_eq!(s.beacon_light, Some(false));
        assert_eq!(s.a32nx.apu_n_percent, Some(Percent::new(0.0)));
        assert_eq!(s.sim_timing.state, SimState::Running); // PAUSED=0
    }

    #[test]
    fn nan_values_map_to_none() {
        let mut values = zero_values();
        values[IDX_ALT_MSL] = f64::NAN;
        values[IDX_BEACON] = f64::NAN;
        let s = map_telemetry(&values, None);
        assert!(s.altitude_msl.is_none());
        assert!(s.beacon_light.is_none());
    }

    #[test]
    fn pause_flag_maps_to_sim_state() {
        let mut values = zero_values();
        values[IDX_PAUSED] = 1.0;
        let s = map_telemetry(&values, None);
        assert_eq!(s.sim_timing.state, SimState::Paused);
    }

    #[test]
    fn system_event_fallback_used_when_simvar_absent() {
        let mut values = zero_values();
        values[IDX_PAUSED] = f64::NAN;
        let s = map_telemetry(&values, Some(true));
        assert_eq!(s.sim_timing.state, SimState::Paused);
    }

    #[test]
    fn radians_are_converted_to_degrees() {
        let mut values = zero_values();
        values[IDX_PITCH] = std::f64::consts::FRAC_PI_2; // 90°
        let s = map_telemetry(&values, None);
        let pitch = s.pitch.unwrap().value();
        assert!((pitch - 90.0).abs() < 1e-9, "pitch = {pitch}");
    }

    #[test]
    fn absolute_time_maps_to_ms() {
        let mut values = zero_values();
        values[IDX_ABS_TIME] = 42.5;
        let s = map_telemetry(&values, None);
        assert_eq!(s.timestamp.ms, 42_500);
    }

    #[test]
    fn a32nx_nav_logo_enum_decodes() {
        let mut values = zero_values();
        values[IDX_A32NX_NAV_LOGO] = 2.0;
        let s = map_telemetry(&values, None);
        assert_eq!(s.a32nx.nav_logo, Some(NavLogoMode::Sys2));
        values[IDX_A32NX_NAV_LOGO] = 9.0;
        let s = map_telemetry(&values, None);
        assert_eq!(s.a32nx.nav_logo, None); // out of enum range → unknown
    }

    #[test]
    fn engine_combustion_only_from_observed_engines() {
        let mut values = zero_values();
        values[IDX_ENG1] = 1.0;
        values[IDX_ENG2] = f64::NAN;
        let s = map_telemetry(&values, None);
        assert_eq!(s.engine_combustion, Some([Some(true), None, None, None]));
    }

    #[test]
    fn out_of_range_position_is_rejected() {
        let mut values = zero_values();
        values[IDX_LAT] = 95.0; // invalid latitude
        values[IDX_LON] = 10.0;
        let s = map_telemetry(&values, None);
        assert!(s.position.is_none());
    }
}
