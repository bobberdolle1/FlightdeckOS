//! Deterministic mission controller.
//!
//! Coordinates flight phases for the headless scenario. It commands through
//! the [`FlightControlTargets`] trait only; phase transitions are decided
//! from canonical state (never invented).

use serde::{Deserialize, Serialize};

use fd_core::adapter::FlightControlTargets;
use fd_core::telemetry::TelemetrySnapshot;

use crate::route::RouteFollower;

/// Mission phases (extensible; later phases delegate to SOP/control/ATC).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MissionPhase {
    Preflight,
    Takeoff,
    Climb,
    Cruise,
    Descent,
    Approach,
    Landing,
    Parked,
    Completed,
    Failed,
}

/// Inputs the controller reads each tick.
pub struct MissionContext<'a> {
    pub snapshot: &'a TelemetrySnapshot,
    /// Distance to destination in nm (route follower provides this).
    pub distance_to_destination_nm: f64,
    /// Bearing to the active route waypoint (deg true).
    pub bearing_to_waypoint_deg: f64,
}

/// Commands emitted by the controller for one tick.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct MissionCommands {
    pub set_target_altitude_ft: Option<f64>,
    pub set_target_speed_kt: Option<f64>,
    pub set_target_heading_deg: Option<f64>,
    pub set_target_vertical_speed_fpm: Option<f64>,
}

/// Development mission parameters. TEST VALUES — not airline policy.
#[derive(Debug, Clone)]
pub struct MissionParameters {
    pub cruise_altitude_ft: f64,
    pub cruise_speed_kt: f64,
    pub climb_speed_kt: f64,
    pub takeoff_target_speed_kt: f64,
    pub approach_speed_kt: f64,
    /// Begin descent when this distance to destination remains (nm).
    pub descent_distance_nm: f64,
    /// Approach gate: distance at which approach phase starts (nm).
    pub approach_gate_nm: f64,
    /// Landing gear-down speed gate.
    pub landing_speed_kt: f64,
}

impl Default for MissionParameters {
    fn default() -> Self {
        // DEVELOPMENT DEFAULTS — TEST MODEL, not airline policy.
        Self {
            cruise_altitude_ft: 34_000.0,
            cruise_speed_kt: 450.0,
            climb_speed_kt: 300.0,
            takeoff_target_speed_kt: 160.0,
            approach_speed_kt: 160.0,
            descent_distance_nm: 120.0,
            approach_gate_nm: 30.0,
            landing_speed_kt: 140.0,
        }
    }
}

/// Development model: fixed climb-out vertical speed (fpm).
const fn climb_out_vs_fpm() -> f64 {
    2200.0
}

/// Development model: descend toward a low approach altitude; the actual
/// field elevation comes from the virtual world model.
const fn approach_elevation_estimate_ft() -> f64 {
    2_000.0
}

/// One pure controller tick: what the mission WOULD command this tick, and
/// the phase it WOULD advance to (`None` = remain in `phase`).
///
/// This is the single source of truth for both
/// [`MissionController::step`] and Shadow Mode ([`crate::shadow`]), which
/// replays this exact decision without mutating any mission state.
fn intended_tick(
    phase: &MissionPhase,
    ctx: &MissionContext,
    params: &MissionParameters,
) -> (MissionCommands, Option<MissionPhase>) {
    let mut cmds = MissionCommands::default();
    let mut next_phase = None;
    let snap = ctx.snapshot;
    let ias = snap.indicated_airspeed.map(|v| v.value()).unwrap_or(0.0);
    let agl = snap.altitude_agl.map(|v| v.value());
    let on_ground = snap.on_ground.unwrap_or(true);

    match *phase {
        MissionPhase::Preflight => {
            // Ground state prepared externally; command takeoff roll.
            cmds.set_target_heading_deg = Some(ctx.bearing_to_waypoint_deg);
            cmds.set_target_speed_kt = Some(params.takeoff_target_speed_kt);
            if !on_ground
                || ias >= params.takeoff_target_speed_kt * 0.9 && agl.unwrap_or(0.0) > 50.0
            {
                next_phase = Some(MissionPhase::Takeoff);
            }
        }
        MissionPhase::Takeoff => {
            // Rotation/climb-out: accelerate to climb speed and climb.
            cmds.set_target_speed_kt = Some(params.climb_speed_kt);
            cmds.set_target_heading_deg = Some(ctx.bearing_to_waypoint_deg);
            cmds.set_target_vertical_speed_fpm = Some(climb_out_vs_fpm());
            if let Some(alt) = agl
                && alt >= 1500.0
            {
                next_phase = Some(MissionPhase::Climb);
                cmds.set_target_altitude_ft = Some(params.cruise_altitude_ft);
                cmds.set_target_vertical_speed_fpm = None; // proportional climb
            }
        }
        MissionPhase::Climb => {
            cmds.set_target_speed_kt = Some(params.climb_speed_kt);
            cmds.set_target_altitude_ft = Some(params.cruise_altitude_ft);
            cmds.set_target_heading_deg = Some(ctx.bearing_to_waypoint_deg);

            if ctx.distance_to_destination_nm <= params.descent_distance_nm {
                next_phase = Some(MissionPhase::Descent);
                cmds.set_target_altitude_ft = Some(approach_elevation_estimate_ft());
                cmds.set_target_vertical_speed_fpm = Some(-1800.0);
            } else if let Some(alt) = snap.altitude_msl.map(|v| v.value())
                && (alt - params.cruise_altitude_ft).abs() <= 200.0
            {
                next_phase = Some(MissionPhase::Cruise);
                cmds.set_target_speed_kt = Some(params.cruise_speed_kt);
            }
        }
        MissionPhase::Cruise => {
            cmds.set_target_speed_kt = Some(params.cruise_speed_kt);
            cmds.set_target_altitude_ft = Some(params.cruise_altitude_ft);
            cmds.set_target_heading_deg = Some(ctx.bearing_to_waypoint_deg);

            if ctx.distance_to_destination_nm <= params.descent_distance_nm {
                next_phase = Some(MissionPhase::Descent);
                cmds.set_target_altitude_ft = Some(approach_elevation_estimate_ft());
                cmds.set_target_vertical_speed_fpm = Some(-1800.0);
            }
        }
        MissionPhase::Descent => {
            cmds.set_target_speed_kt = Some(params.approach_speed_kt.max(200.0));
            cmds.set_target_vertical_speed_fpm = Some(-1800.0);
            cmds.set_target_heading_deg = Some(ctx.bearing_to_waypoint_deg);
            if let Some(alt) = snap.altitude_msl.map(|v| v.value()) {
                if alt > 5_000.0 {
                    // Keep descending toward a low approach altitude.
                    cmds.set_target_altitude_ft = Some(3_000.0);
                } else {
                    cmds.set_target_altitude_ft = Some(2_000.0);
                }
            }
            if ctx.distance_to_destination_nm <= params.approach_gate_nm {
                next_phase = Some(MissionPhase::Approach);
            }
        }
        MissionPhase::Approach => {
            cmds.set_target_speed_kt = Some(params.landing_speed_kt);
            // Development value: gentle final descent so a NOMINAL
            // mission does not trip the hard-touchdown FDM threshold.
            cmds.set_target_vertical_speed_fpm = Some(-450.0);
            cmds.set_target_heading_deg = Some(ctx.bearing_to_waypoint_deg);
            // Landing is the model's touchdown; when on ground we move on.
            if on_ground {
                next_phase = Some(MissionPhase::Landing);
            }
        }
        MissionPhase::Landing => {
            // Decelerate on the ground toward the terminal; parked when slow.
            cmds.set_target_speed_kt = Some(5.0);
            if ias <= 6.0 && on_ground {
                next_phase = Some(MissionPhase::Parked);
            }
        }
        MissionPhase::Parked => {
            cmds.set_target_speed_kt = Some(0.0);
            next_phase = Some(MissionPhase::Completed);
        }
        MissionPhase::Completed | MissionPhase::Failed => {}
    }

    (cmds, next_phase)
}

/// Commands the mission WOULD emit for `phase` on `ctx` this tick.
///
/// Pure: reads no controller state and mutates nothing. Shadow Mode
/// compares these intended commands against the observed autopilot
/// selections to detect divergence between autonomy and reality.
pub fn intended_commands(
    phase: &MissionPhase,
    ctx: &MissionContext,
    params: &MissionParameters,
) -> MissionCommands {
    intended_tick(phase, ctx, params).0
}

/// The phase the mission WOULD advance to this tick (`None` = remain in
/// `phase`). Pure companion to [`intended_commands`].
pub fn intended_next_phase(
    phase: &MissionPhase,
    ctx: &MissionContext,
    params: &MissionParameters,
) -> Option<MissionPhase> {
    intended_tick(phase, ctx, params).1
}

/// The deterministic mission controller.
#[derive(Debug)]
pub struct MissionController {
    pub params: MissionParameters,
    phase: MissionPhase,
}

impl MissionController {
    pub fn new(params: MissionParameters) -> Self {
        Self {
            params,
            phase: MissionPhase::Preflight,
        }
    }

    pub const fn phase(&self) -> MissionPhase {
        self.phase
    }

    /// One controller pass. Emits guidance targets and may transition the
    /// mission phase. Delegates the decision to the pure [`intended_tick`]
    /// core, so [`crate::shadow`] can replay the identical decision
    /// read-only; external behavior is unchanged.
    pub fn step(
        &mut self,
        ctx: &MissionContext,
        controls: &mut dyn FlightControlTargets,
        _route: &mut RouteFollower,
    ) -> MissionCommands {
        let (cmds, next_phase) = intended_tick(&self.phase, ctx, &self.params);
        if let Some(next) = next_phase {
            self.phase = next;
        }

        // Apply emitted commands to the control-target boundary.
        if let Some(v) = cmds.set_target_altitude_ft {
            controls.set_target_altitude(v);
        }
        if let Some(v) = cmds.set_target_speed_kt {
            controls.set_target_speed(v);
        }
        if let Some(v) = cmds.set_target_heading_deg {
            controls.set_target_heading(v);
        }
        if let Some(v) = cmds.set_target_vertical_speed_fpm {
            controls.set_target_vertical_speed(v);
        }
        cmds
    }

    /// Force-fail the mission (used by assertion/reporting layers).
    pub const fn fail(&mut self) {
        self.phase = MissionPhase::Failed;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::route::{RouteFollower, Waypoint};
    use fd_core::telemetry::{SimState, SimTimestamp, TelemetrySnapshot};
    use fd_core::units::{AltitudeAglFt, AltitudeFt, SpeedKt};

    /// A minimal FlightControlTargets double that records commands and
    /// applies NOTHING (controller-level tests only).
    #[derive(Default)]
    struct RecordingControls {
        alt: Option<f64>,
        speed: Option<f64>,
        heading: Option<f64>,
        vs: Option<f64>,
    }
    impl FlightControlTargets for RecordingControls {
        fn flight_guidance_supported(&self) -> bool {
            true
        }
        fn set_target_altitude(&mut self, v: f64) {
            self.alt = Some(v);
        }
        fn set_target_speed(&mut self, v: f64) {
            self.speed = Some(v);
        }
        fn set_target_heading(&mut self, v: f64) {
            self.heading = Some(v);
        }
        fn set_target_vertical_speed(&mut self, v: f64) {
            self.vs = Some(v);
        }
    }

    fn snap(alt_ft: f64, agl_ft: f64, ias: f64, on_ground: bool) -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(0));
        s.altitude_msl = Some(AltitudeFt::new(alt_ft));
        s.altitude_agl = Some(AltitudeAglFt::new(agl_ft));
        s.indicated_airspeed = Some(SpeedKt::new(ias));
        s.on_ground = Some(on_ground);
        s.sim_timing.state = SimState::Running;
        s
    }

    fn route() -> RouteFollower {
        let wpts = vec![
            Waypoint {
                id: "UUEE".into(),
                lat_deg: 55.972642,
                lon_deg: 37.414589,
            },
            Waypoint {
                id: "ULLI".into(),
                lat_deg: 59.800278,
                lon_deg: 30.2625,
            },
        ];
        RouteFollower::new(wpts, 5.0)
    }
    #[allow(dead_code)]
    fn ctx<'a>(snap: &'a TelemetrySnapshot, route: &RouteFollower) -> MissionContext<'a> {
        let lat = snap
            .position
            .as_ref()
            .map(|p| p.lat.value())
            .unwrap_or(55.97);
        let lon = snap
            .position
            .as_ref()
            .map(|p| p.lon.value())
            .unwrap_or(37.41);
        let (bearing, dist) = route.guidance(lat, lon);
        MissionContext {
            snapshot: snap,
            distance_to_destination_nm: dist,
            bearing_to_waypoint_deg: bearing,
        }
    }

    #[test]
    fn ground_state_commands_takeoff_roll() {
        let mut c = MissionController::new(MissionParameters::default());
        let r = route();
        let s = snap(622.0, 0.0, 0.0, true);
        let (bearing, dist) = {
            let (b, d) = r.guidance(55.972642, 37.414589);
            (b, d)
        };
        let ctx = MissionContext {
            snapshot: &s,
            distance_to_destination_nm: dist,
            bearing_to_waypoint_deg: bearing,
        };
        let mut controls = RecordingControls::default();
        let mut r2 = route();
        let cmds = c.step(&ctx, &mut controls, &mut r2);
        assert_eq!(c.phase(), MissionPhase::Preflight);
        assert_eq!(cmds.set_target_speed_kt, Some(160.0));
        // Impossible transition guard: Preflight never jumps to Cruise.
        assert_ne!(c.phase(), MissionPhase::Cruise);
    }

    #[test]
    fn airborne_low_altitude_transitions_to_climb() {
        let mut c = MissionController::new(MissionParameters::default());
        let r = route();
        let s = snap(622.0 + 1600.0, 1600.0, 300.0, false);
        let (bearing, dist) = {
            let (b, d) = r.guidance(55.98, 37.3);
            (b, d)
        };
        let ctx = MissionContext {
            snapshot: &s,
            distance_to_destination_nm: dist,
            bearing_to_waypoint_deg: bearing,
        };
        let mut controls = RecordingControls::default();
        let _ = c.step(&ctx, &mut controls, &mut route());
        // Airborne entry lands in Takeoff first (climb-out).
        assert_eq!(c.phase(), MissionPhase::Takeoff);
        let s2 = snap(622.0 + 1700.0, 1700.0, 300.0, false);
        let (bearing, dist) = {
            let (b, d) = r.guidance(55.99, 37.2);
            (b, d)
        };
        let ctx2 = MissionContext {
            snapshot: &s2,
            distance_to_destination_nm: dist,
            bearing_to_waypoint_deg: bearing,
        };
        let mut controls2 = RecordingControls::default();
        let _ = c.step(&ctx2, &mut controls2, &mut route());
        assert_eq!(c.phase(), MissionPhase::Climb);
        assert_eq!(controls2.alt, Some(c.params.cruise_altitude_ft));
    }

    #[test]
    fn failure_phase_is_explicit() {
        let mut c = MissionController::new(MissionParameters::default());
        c.fail();
        assert_eq!(c.phase(), MissionPhase::Failed);
    }

    /// The extracted pure decision core must be (1) side-effect free and
    /// (2) identical to what `step` emits, tick after tick — this is the
    /// contract Shadow Mode relies on.
    #[test]
    fn intended_commands_is_pure_and_matches_step_output() {
        let mut driven = MissionController::new(MissionParameters::default());
        let mut replay = MissionController::new(driven.params.clone());

        let snaps = [
            snap(622.0 + 1600.0, 1600.0, 300.0, false), // Preflight -> Takeoff
            snap(622.0 + 1700.0, 1700.0, 300.0, false), // Takeoff -> Climb
            snap(622.0 + 1800.0, 1800.0, 300.0, false), // steady climb
        ];
        for s in &snaps {
            let ctx = MissionContext {
                snapshot: s,
                distance_to_destination_nm: 400.0,
                bearing_to_waypoint_deg: 45.0,
            };
            let phase_before = driven.phase();

            // Purity: repeated replay is stable and advances nothing.
            let intended = intended_commands(&phase_before, &ctx, &replay.params);
            let again = intended_commands(&phase_before, &ctx, &replay.params);
            assert_eq!(intended, again);
            assert_eq!(replay.phase, phase_before);

            // Parity: the mutating step emits exactly the pure intent.
            let mut controls = RecordingControls::default();
            let stepped = driven.step(&ctx, &mut controls, &mut route());
            assert_eq!(stepped, intended);

            // And the read-only twin progresses identically when its own
            // derived transition is applied explicitly.
            if let Some(next) = intended_next_phase(&phase_before, &ctx, &replay.params) {
                replay.phase = next;
            }
            assert_eq!(driven.phase(), replay.phase);
        }
        assert_eq!(driven.phase(), MissionPhase::Climb);
    }
}
