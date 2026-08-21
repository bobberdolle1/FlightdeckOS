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
    /// Latched at rotation; cleared at touchdown. Prevents the model from
    /// re-grounding itself immediately after liftoff while level at field
    /// elevation.
    lifted_off: bool,

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
            lifted_off: false,
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

    /// Start airborne at an explicit MSL altitude (fault/scenario setups).
    pub fn start_airborne_at(&mut self, altitude_ft: f64) {
        self.altitude_ft = altitude_ft;
        self.on_ground = altitude_ft <= self.ground_elevation_ft;
        if !self.on_ground {
            self.lifted_off = true;
            self.ias_kt = 250.0;
            self.groundspeed_kt = 250.0;
        }
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

        // Ground roll / liftoff / airborne determination.
        //
        // Once rotated past Vr the model is AIRBORNE and stays airborne
        // until it descends through field elevation with negative VS
        // (a real touchdown). Level flight at field elevation right after
        // liftoff must NOT slam the model back onto the ground.
        const ROTATION_IAS_KT: f64 = 140.0;
        if !self.lifted_off {
            if self.ias_kt >= ROTATION_IAS_KT {
                // Rotation -> liftoff.
                self.lifted_off = true;
                self.on_ground = false;
                self.pitch_deg = 5.0;
            } else {
                // Ground roll.
                self.groundspeed_kt = self.ias_kt;
                self.vertical_speed_fpm = 0.0;
                self.pitch_deg = 0.0;
                self.altitude_ft = self.ground_elevation_ft;
            }
        }

        if self.on_ground {
            // Still in ground roll (rotation not reached this tick).
            self.groundspeed_kt = self.ias_kt;
            self.vertical_speed_fpm = 0.0;
            self.altitude_ft = self.ground_elevation_ft;
        } else {
            // --- vertical ---------------------------------------------------
            // Target VS: explicit command wins; otherwise proportional
            // closure toward the target altitude with bounded rates.
            let target_vs = match self.target_vertical_speed_fpm {
                Some(cmd) => cmd.clamp(-self.limits.max_descent_fpm, self.limits.max_climb_fpm),
                None => {
                    let err = self.target_altitude_ft - self.altitude_ft;
                    if err.abs() <= 1.0 {
                        0.0
                    } else {
                        (err / dt_s * 60.0)
                            .clamp(-self.limits.max_descent_fpm, self.limits.max_climb_fpm)
                    }
                }
            };

            // VS moves toward its target with a bounded VS-change rate.
            let vs_rate = 1500.0 * dt_s; // fpm per second (development value)
            self.vertical_speed_fpm +=
                (target_vs - self.vertical_speed_fpm).clamp(-vs_rate, vs_rate);

            // Altitude integrates from vertical speed.
            let prev_alt = self.altitude_ft;
            self.altitude_ft += self.vertical_speed_fpm / 60.0 * dt_s;

            // Level-off snap: crossing the target altitude this step lands
            // exactly on it (no oscillation).
            if self.target_vertical_speed_fpm.is_none() {
                let crossed = (prev_alt < self.target_altitude_ft)
                    != (self.altitude_ft < self.target_altitude_ft);
                if crossed || self.altitude_ft == self.target_altitude_ft {
                    self.altitude_ft = self.target_altitude_ft;
                    self.vertical_speed_fpm = 0.0;
                }
            }

            // Touchdown: DESCENDING through field elevation. Level flight
            // at field elevation right after liftoff is still airborne.
            if self.altitude_ft <= self.ground_elevation_ft && self.vertical_speed_fpm < 0.0 {
                self.altitude_ft = self.ground_elevation_ft;
                // Keep the impact vertical speed for THIS tick so FDR/QoL
                // observe the touchdown rate; the ground-roll branch zeroes
                // it from the next tick on.
                self.on_ground = true;
                self.lifted_off = false;
                self.pitch_deg = 0.0;
            }
        }

        // Test model: no wind — ground speed follows IAS while airborne.
        self.groundspeed_kt = self.ias_kt;

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

/// Minimal spherical dead reckoning (standard direct-geodesic formulas,
/// spherical earth — good enough for a test route).
fn dead_reckon(lat_deg: f64, lon_deg: f64, bearing_deg: f64, dist_nm: f64) -> (f64, f64) {
    const R_NM: f64 = 3440.065;
    let phi1 = lat_deg.to_radians();
    let lam1 = lon_deg.to_radians();
    let theta = bearing_deg.to_radians();
    let delta = dist_nm / R_NM;

    let sin_phi2 = phi1.sin() * delta.cos() + phi1.cos() * delta.sin() * theta.cos();
    let phi2 = sin_phi2.asin();
    let lam2 =
        lam1 + (theta.sin() * delta.sin() * phi1.cos()).atan2(delta.cos() - phi1.sin() * sin_phi2);

    (
        phi2.to_degrees().clamp(-90.0, 90.0),
        lam2.to_degrees().rem_euclid(360.0),
    )
}
