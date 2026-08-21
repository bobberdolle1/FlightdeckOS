//! Bounded kinematic flight model.
//!
//! **TEST MODEL — NOT AIRCRAFT PERFORMANCE DATA.**
//!
//! Targets are COMMANDS: altitude/speed/heading change at bounded rates and
//! integrate over simulated time. The model never teleports to a target.
//! Position integrates from track/GS over the fixed timestep.

use serde::{Deserialize, Serialize};

/// Development kinematic limits. TEST MODEL values, not performance data.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct KinematicLimits {
    /// Max climb rate (fpm), positive.
    pub max_climb_fpm: f64,
    /// Max descent rate (fpm), positive magnitude.
    pub max_descent_fpm: f64,
    /// Max turn rate (deg/s).
    pub max_turn_rate_deg_s: f64,
    /// Max acceleration (kt/s).
    pub max_accel_kt_s: f64,
    /// Max deceleration (kt/s).
    pub max_decel_kt_s: f64,
}

impl Default for KinematicLimits {
    fn default() -> Self {
        // DEVELOPMENT DEFAULTS for a jet-transport-shaped test model.
        Self {
            max_climb_fpm: 2500.0,
            max_descent_fpm: 2200.0,
            max_turn_rate_deg_s: 3.0,
            max_accel_kt_s: 1.5,
            max_decel_kt_s: 1.8,
        }
    }
}

fn wrap180(deg: f64) -> f64 {
    let mut d = (deg + 180.0).rem_euclid(360.0) - 180.0;
    if d == -180.0 {
        d = 180.0;
    }
    d
}

/// Kinematic aircraft state + current targets.
#[derive(Debug, Clone, PartialEq)]
pub struct KinematicState {
    // -- state ---------------------------------------------------------------
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_ft: f64,
    pub ground_elevation_ft: f64,
    pub heading_deg: f64,
    pub ias_kt: f64,
    pub groundspeed_kt: f64,
    pub vertical_speed_fpm: f64,
    pub pitch_deg: f64,
    pub bank_deg: f64,
    pub on_ground: bool,

    // -- targets -------------------------------------------------------------
    target_altitude_ft: f64,
    target_speed_kt: f64,
    target_heading_deg: f64,
    target_vertical_speed_fpm: Option<f64>,

    // -- limits --------------------------------------------------------------
    limits: KinematicLimits,
}

impl KinematicState {
    pub fn new(lat: f64, lon: f64, ground_elevation_ft: f64, limits: KinematicLimits) -> Self {
        Self {
            latitude_deg: lat,
            longitude_deg: lon,
            altitude_ft: ground_elevation_ft,
            ground_elevation_ft,
            heading_deg: 0.0,
            ias_kt: 0.0,
            groundspeed_kt: 0.0,
            vertical_speed_fpm: 0.0,
            pitch_deg: 0.0,
            bank_deg: 0.0,
            on_ground: true,
            target_altitude_ft: ground_elevation_ft,
            target_speed_kt: 0.0,
            target_heading_deg: 0.0,
            target_vertical_speed_fpm: None,
            limits,
        }
    }

    pub const fn target_altitude_ft(&self) -> f64 {
        self.target_altitude_ft
    }
    pub const fn target_speed_kt(&self) -> f64 {
        self.target_speed_kt
    }
    pub const fn target_heading_deg(&self) -> f64 {
        self.target_heading_deg
    }

    pub fn set_target_altitude(&mut self, ft: f64) {
        self.target_altitude_ft = ft.clamp(self.ground_elevation_ft, 45_000.0);
    }

    pub fn set_target_speed(&mut self, kt: f64) {
        self.target_speed_kt = kt.clamp(0.0, 500.0);
    }

    pub fn set_target_heading(&mut self, deg: f64) {
        self.target_heading_deg = deg.rem_euclid(360.0);
    }

    pub fn set_target_vertical_speed(&mut self, fpm: f64) {
        let clamped = fpm.clamp(-self.limits.max_descent_fpm, self.limits.max_climb_fpm);
        self.target_vertical_speed_fpm = Some(clamped);
    }

    /// Radio altitude (AGL).
    pub const fn agl_ft(&self) -> f64 {
        self.altitude_ft - self.ground_elevation_ft
    }

    /// Advance the model by `dt_s` seconds of simulated time.
    ///
    /// All rates are bounded; values integrate toward targets and never
    /// teleport. Touchdown happens when a descending state reaches ground
    /// elevation — never by an instant command.
    pub fn advance(&mut self, dt_s: f64) {
        assert!(dt_s > 0.0, "dt must be positive");

        // --- speed: bounded acceleration/deceleration ----------------------
        let speed_err = self.target_speed_kt - self.ias_kt;
        let max_d_speed = if speed_err >= 0.0 {
            self.limits.max_accel_kt_s * dt_s
        } else {
            self.limits.max_decel_kt_s * dt_s
        };
        let d_speed = speed_err.clamp(-max_d_speed, max_d_speed);
        self.ias_kt = (self.ias_kt + d_speed).clamp(0.0, 520.0);

        // Ground roll / liftoff: below rotation speed the aircraft stays on
        // the ground and GS equals IAS.
        const ROTATION_IAS_KT: f64 = 140.0;
        if self.on_ground && self.ias_kt < ROTATION_IAS_KT {
            self.groundspeed_kt = self.ias_kt;
            self.vertical_speed_fpm = 0.0;
            self.pitch_deg = 0.0;
            self.altitude_ft = self.ground_elevation_ft;
        } else {
            // Liftoff transition (once): pitch up, leave the ground.
            if self.on_ground {
                self.on_ground = false;
                self.pitch_deg = 5.0;
            }

            // --- vertical ---------------------------------------------------
            let target_vs = match self.target_vertical_speed_fpm {
                Some(cmd) => cmd.clamp(-self.limits.max_descent_fpm, self.limits.max_climb_fpm),
                None => {
                    let err = self.target_altitude_ft - self.altitude_ft;
                    if err.abs() <= 1.0 {
                        0.0
                    } else {
                        // Proportional closure bounded by climb/descent limits.
                        (err / dt_s * 60.0)
                            .clamp(-self.limits.max_descent_fpm, self.limits.max_climb_fpm)
                    }
                }
            };

            // VS changes toward its target with a bounded rate (fpm per s).
            let vs_rate = 1500.0 * dt_s;
            self.vertical_speed_fpm +=
                (target_vs - self.vertical_speed_fpm).clamp(-vs_rate, vs_rate);

            // Altitude integrates from vertical speed.
            self.altitude_ft += self.vertical_speed_fpm / 60.0 * dt_s;

            // Level-off snap: no oscillation around the target altitude.
            if self.target_vertical_speed_fpm.is_none() {
                let err = self.target_altitude_ft - self.altitude_ft;
                if err.abs() <= 1.0
                    || (err > 0.0) == (self.vertical_speed_fpm >= 0.0)
                        && err.abs() <= self.vertical_speed_fpm.abs() / 60.0 * dt_s + 1.0
                {
                    self.altitude_ft = self.target_altitude_ft;
                    self.vertical_speed_fpm = 0.0;
                }
            }
            // Never descend below ground without declaring touchdown:
            if self.altitude_ft < self.ground_elevation_ft {
                self.altitude_ft = self.ground_elevation_ft;
                self.on_ground = true;
                self.pitch_deg = 0.0;
            }
        }

        // --- heading: bounded turn rate (airborne) ---------------------------
        if !self.on_ground {
            let err = wrap180(self.target_heading_deg - self.heading_deg);
            let max_turn = self.limits.max_turn_rate_deg_s * dt_s;
            let turn = err.clamp(-max_turn, max_turn);
            self.heading_deg = (self.heading_deg + turn).rem_euclid(360.0);
        }

        // --- position integration --------------------------------------------
        if !self.on_ground {
            let dist_nm = self.groundspeed_kt * dt_s / 3600.0;
            let (dlat, dlon) = dead_reckon(
                self.latitude_deg,
                self.longitude_deg,
                self.heading_deg,
                dist_nm,
            );
            self.latitude_deg = dlat;
            self.longitude_deg = dlon;
        }
    }
}

/// Minimal spherical dead reckoning (good enough for a test route).
fn dead_reckon(lat_deg: f64, lon_deg: f64, bearing_deg: f64, dist_nm: f64) -> (f64, f64) {
    const R_NM: f64 = 3440.065;
    let lat1 = lat_deg.to_radians();
    let lon1 = lon_deg.to_radians();
    let brg = bearing_deg.to_radians();
    let dr = dist_nm / R_NM;
    let lat2 = (lat1.cos() * dr.cos() - lat1.sin() * dr.sin() * brg.cos()).acos();
    let lon2 = lon1
        + brg.sin()
            * dr.sin()
            * lat2
                .cos()
                .atan2(lat1.cos() * dr.cos() - lat1.sin() * dr.sin() * brg.cos());
    let lat2 = lat2.asin();
    (
        lat2.to_degrees().clamp(-90.0, 90.0),
        lon2.to_degrees().rem_euclid(360.0),
    )
}

/// Great-circle distance between two points (nm). Development helper.
pub fn distance_nm(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const R_NM: f64 = 3440.065;
    let (lat1, lon1, lat2, lon2) = (
        lat1.to_radians(),
        lon1.to_radians(),
        lat2.to_radians(),
        lon2.to_radians(),
    );
    let dlat = lat2 - lat1;
    let dlon = lon2 - lon1;
    let a = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    R_NM * 2.0 * a.sqrt().atan2((1.0 - a).sqrt())
}

/// Initial bearing from point 1 to point 2 (degrees true).
pub fn initial_bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (lat1, lon1, lat2, lon2) = (
        lat1.to_radians(),
        lon1.to_radians(),
        lat2.to_radians(),
        lon2.to_radians(),
    );
    let dlon = lon2 - lon1;
    let y = dlon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
    y.atan2(x).to_degrees().rem_euclid(360.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> KinematicState {
        KinematicState::new(55.972642, 37.414589, 622.0, KinematicLimits::default())
    }

    #[test]
    fn bounded_turn_rate_never_teleports_heading() {
        let mut m = model();
        m.set_target_heading(180.0);
        // One small step cannot flip 180 degrees instantly.
        m.advance(0.1); // 0.1 s
        let turned = wrap180(m.heading_deg - 0.0);
        assert!(turned.abs() < 3.0, "turned too fast: {turned}");
    }

    #[test]
    fn bounded_speed_change_is_gradual() {
        let mut m = model();
        m.set_target_speed(250.0);
        let mut prev = m.ias_kt;
        for _ in 0..50 {
            m.advance(1.0);
            assert!(m.ias_kt >= prev - 1e-9);
            assert!(m.ias_kt - prev <= 1.5 * 1.0 + 1e-9);
            prev = m.ias_kt;
        }
        assert!(m.ias_kt < 250.0 || (prev - 250.0).abs() < 1.0);
    }

    #[test]
    fn altitude_integrates_toward_target_without_overshoot() {
        let mut m = model();
        m.set_target_speed(250.0);
        m.set_target_altitude(10_000.0);
        // Liftoff first (speed up past rotation).
        for _ in 0..200 {
            m.advance(1.0);
        }
        m.on_ground = false;
        let mut reached = false;
        for _ in 0..20_000 {
            m.advance(1.0);
            if m.altitude_ft == 10_000.0 {
                reached = true;
                break;
            }
            assert!(m.altitude_ft <= 10_000.0 + 1e-6, "overshot target altitude");
        }
        assert!(reached, "never reached target altitude");
        assert_eq!(m.vertical_speed_fpm, 0.0);
    }

    #[test]
    fn invalid_vertical_command_is_clamped_to_limits() {
        let mut m = model();
        m.set_target_vertical_speed(999_999.0);
        // Clamped into [-2200, 2500]; no panic, no teleport.
        m.advance(1.0);
        assert!(m.vertical_speed_fpm <= 2500.0);
    }

    #[test]
    fn descent_touches_down_through_ground_elevation() {
        let mut m = model();
        m.on_ground = false;
        m.altitude_ft = 1500.0;
        m.ground_elevation_ft = 79.0;
        m.set_target_altitude(79.0);
        m.set_target_vertical_speed(-800.0);
        let mut touched_at_vs = None;
        for _ in 0..1000 {
            m.advance(1.0);
            if m.on_ground {
                touched_at_vs = Some(m.vertical_speed_fpm);
                break;
            }
        }
        assert!(m.on_ground);
        assert!((m.altitude_ft - 79.0).abs() < 1e-9);
        assert!(touched_at_vs.is_some());
    }

    #[test]
    fn distance_helper_matches_known_reference() {
        // UUEE -> ULLI great-circle ~ 334 nm (open-airac coordinates).
        let d = distance_nm(55.972642, 37.414589, 59.800278, 30.2625);
        assert!(d > 320.0 && d < 350.0, "distance {d}");
    }
}
