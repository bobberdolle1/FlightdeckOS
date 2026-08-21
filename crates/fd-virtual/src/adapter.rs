//! The headless [`VirtualSimulator`]: implements the SAME boundaries as a
//! real simulator adapter —
//!
//! * [`SimulatorAdapter`] (telemetry + closed cockpit actions), and
//! * [`FlightControlTargets`] (continuous guidance targets).
//!
//! It is driven by the deterministic virtual clock: every `advance_tick`
//! advances the world by one fixed simulated step, integrates the
//! kinematic/semantic models, and produces the next canonical snapshot.
//! No wall clock, no sleeps.
//!
//! Proof-domain note (Task 3 §27): a virtual success proves ORCHESTRATION,
//! never real simulator bindings or aircraft performance.

use fd_core::actions::CockpitAction;
use fd_core::adapter::{AdapterError, Capability, FlightControlTargets, SimulatorAdapter};
use fd_core::telemetry::{SimState, SimTimestamp, TelemetrySnapshot};
use fd_core::units::{AltitudeAglFt, AltitudeFt, AngleDeg, SpeedKt, VerticalSpeedFpm};

use crate::VirtualClock;
use crate::kinematics::{KinematicLimits, KinematicState};
use crate::systems::SystemsState;

/// The headless virtual simulator.
pub struct VirtualSimulator {
    clock: VirtualClock,
    kinematics: KinematicState,
    systems: SystemsState,
}

impl VirtualSimulator {
    /// Build with explicit initial conditions and a fixed timestep.
    pub fn new(
        lat_deg: f64,
        lon_deg: f64,
        ground_elevation_ft: f64,
        initial_heading_deg: f64,
        dt_ms: u64,
    ) -> Self {
        let mut kinematics = KinematicState::new(
            lat_deg,
            lon_deg,
            ground_elevation_ft,
            KinematicLimits::default(),
        );
        kinematics.set_target_heading(initial_heading_deg);
        kinematics.heading_deg = initial_heading_deg.rem_euclid(360.0);
        Self {
            clock: VirtualClock::new(dt_ms),
            kinematics,
            systems: SystemsState::cold_and_dark(),
        }
    }

    /// Simulated timestamp of the CURRENT state.
    fn now(&self) -> SimTimestamp {
        SimTimestamp::new(self.clock.sim_ms())
    }

    /// Number of simulated ticks executed so far.
    pub const fn ticks(&self) -> u64 {
        self.clock.ticks()
    }

    /// Access to the semantic systems model for scenario setup
    /// (e.g. pre-positioning engines for a mid-air start).
    pub fn systems_mut(&mut self) -> &mut SystemsState {
        &mut self.systems
    }

    /// Advance the world one fixed step; returns the post-step snapshot.
    pub fn advance_tick(&mut self) -> TelemetrySnapshot {
        self.systems.advance(self.clock.dt_ms);
        self.kinematics.advance(self.clock.dt_ms as f64 / 1000.0);
        self.clock.advance();
        self.snapshot()
    }

    /// Current canonical snapshot without advancing.
    pub fn snapshot(&self) -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(self.now());
        s.position = Some(fd_core::telemetry::Position {
            lat: fd_core::units::LatDeg::new(self.kinematics.latitude_deg),
            lon: fd_core::units::LonDeg::new(self.kinematics.longitude_deg),
        });
        s.altitude_msl = Some(AltitudeFt::new(self.kinematics.altitude_ft));
        s.altitude_agl = Some(AltitudeAglFt::new(self.kinematics.agl_ft()));
        s.groundspeed = Some(SpeedKt::new(self.kinematics.groundspeed_kt));
        s.indicated_airspeed = Some(SpeedKt::new(self.kinematics.ias_kt));
        s.vertical_speed = Some(VerticalSpeedFpm::new(self.kinematics.vertical_speed_fpm));
        s.heading_true = Some(AngleDeg::new(self.kinematics.heading_deg));
        s.pitch = Some(AngleDeg::new(self.kinematics.pitch_deg));
        s.bank = Some(AngleDeg::new(self.kinematics.bank_deg));
        s.on_ground = Some(self.kinematics.on_ground);
        s.gear_handle_down = Some(true); // fixed gear-down test model
        s.flaps_handle_index = Some(0);
        s.engine_combustion = Some([
            Some(self.systems.engines_running),
            Some(self.systems.engines_running),
            None,
            None,
        ]);
        s.beacon_light = Some(self.systems.beacon_on);
        // The virtual world never pauses on its own: the runner simply stops
        // advancing it. PAUSED=false keeps runtime semantics clean.
        s.sim_timing.state = SimState::Running;
        s.sim_timing.sim_rate = Some(1.0);
        s.sim_timing.slew_active = Some(false);

        // A32NX extension ids — MUST match fd-simconnect EXT_ID_* mapping:
        s.aircraft_values.insert(1, self.systems.apu_n_percent);
        s.aircraft_values.insert(
            2,
            if self.systems.apu_bleed_open {
                1.0
            } else {
                0.0
            },
        );
        s.aircraft_values.insert(3, 0.0);
        s.aircraft_values.insert(4, 0.0);
        s.aircraft_values
            .insert(5, if self.systems.pack_1_pb_on { 1.0 } else { 0.0 });
        s
    }
}

impl FlightControlTargets for VirtualSimulator {
    fn flight_guidance_supported(&self) -> bool {
        true // bounded-rate guidance supported by construction
    }

    fn set_target_altitude(&mut self, altitude_ft: f64) {
        self.kinematics.set_target_altitude(altitude_ft);
    }

    fn set_target_speed(&mut self, speed_kt: f64) {
        self.kinematics.set_target_speed(speed_kt);
    }

    fn set_target_heading(&mut self, heading_deg: f64) {
        self.kinematics.set_target_heading(heading_deg);
    }

    fn set_target_vertical_speed(&mut self, fpm: f64) {
        self.kinematics.set_target_vertical_speed(fpm);
    }
}

impl SimulatorAdapter for VirtualSimulator {
    fn connect(&mut self) -> Result<(), AdapterError> {
        Ok(())
    }

    fn disconnect(&mut self) {}

    fn is_connected(&self) -> bool {
        true
    }

    /// Deliver the CURRENT canonical snapshot (one per poll). The runner
    /// advances the world explicitly between polls.
    fn poll(&mut self) -> Result<Vec<TelemetrySnapshot>, AdapterError> {
        Ok(vec![self.snapshot()])
    }

    fn capability(&self, action: CockpitAction) -> Capability {
        match action {
            // Modeled by the semantic systems layer:
            CockpitAction::SetBeacon(_) => Capability::Supported,
            // Not modeled in the Task 3 virtual aircraft:
            CockpitAction::SetNavLogo(_) => Capability::Unsupported,
        }
    }

    fn execute(&mut self, action: CockpitAction) -> Result<(), AdapterError> {
        match action {
            CockpitAction::SetBeacon(pos) => {
                self.systems
                    .set_beacon(matches!(pos, fd_core::actions::SwitchPosition::On));
                Ok(())
            }
            CockpitAction::SetNavLogo(_) => Err(AdapterError::UnsupportedAction),
        }
    }
}
