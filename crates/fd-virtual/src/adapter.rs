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
use crate::faults::FaultConfig;
use crate::kinematics::{KinematicLimits, KinematicState};
use crate::systems::SystemsState;

/// The headless virtual simulator.
pub struct VirtualSimulator {
    clock: VirtualClock,
    kinematics: KinematicState,
    systems: SystemsState,
    /// Deterministic fault injection (spec §41); default = no faults.
    faults: FaultConfig,
    /// Advance attempts (ticks requested), including frozen ones. The
    /// freeze window is measured in ATTEMPTS: a frozen world cannot advance
    /// its own clock, so the window can never elapse otherwise.
    fault_ticks: u64,
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
            faults: FaultConfig::default(),
            fault_ticks: 0,
        }
    }

    /// Attach deterministic fault injection (builder style).
    ///
    /// Panics when `faults.unknown_sensor_fields` names a field outside the
    /// closed [`crate::faults::MASKABLE_FIELDS`] set: that is a programmer
    /// error to fail at construction time, not silently ignore
    /// mid-scenario. Scenario files validate first via
    /// [`FaultConfig::validate`] so the error surfaces as data instead.
    #[must_use]
    pub fn with_faults(mut self, faults: FaultConfig) -> Self {
        if let Err(e) = faults.validate() {
            panic!("invalid FaultConfig: {e}");
        }
        self.faults = faults.normalized();
        self
    }

    /// Attach fault injection in place (for callers holding the simulator
    /// behind a shared handle). Validates like [`Self::with_faults`].
    pub fn set_faults(&mut self, faults: FaultConfig) {
        if let Err(e) = faults.validate() {
            panic!("invalid FaultConfig: {e}");
        }
        self.faults = faults.normalized();
    }

    /// Read-only access to the active fault configuration.
    pub const fn faults(&self) -> &FaultConfig {
        &self.faults
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

    /// Access to the kinematic model for scenario setup
    /// (e.g. starting airborne at an altitude).
    pub fn kinematics_mut(&mut self) -> &mut KinematicState {
        &mut self.kinematics
    }

    /// Advance the world one fixed step; returns the post-step snapshot.
    pub fn advance_tick(&mut self) -> TelemetrySnapshot {
        // Fault: telemetry freeze — while inside the freeze window the world
        // (systems, kinematics AND its clock) does not advance at all, so
        // every poll returns the same frozen snapshot, timestamp included.
        // Pure function of the tick counter; resuming is exact because no
        // state moved meanwhile.
        self.fault_ticks += 1;
        if self.fault_ticks <= self.faults.telemetry_freeze_until_tick {
            return self.snapshot();
        }
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

        // Fault: unknown sensor fields — named canonical fields read back as
        // unknown (`None`). Names outside MASKABLE_FIELDS are rejected at
        // construction time, so every arm here covers a real field.
        for name in &self.faults.unknown_sensor_fields {
            match name.as_str() {
                "position" => s.position = None,
                "altitude_msl" => s.altitude_msl = None,
                "altitude_agl" => s.altitude_agl = None,
                "groundspeed" => s.groundspeed = None,
                "ias" => s.indicated_airspeed = None,
                "vertical_speed" | "vs" => s.vertical_speed = None,
                "heading_true" => s.heading_true = None,
                "pitch" => s.pitch = None,
                "bank" => s.bank = None,
                "on_ground" => s.on_ground = None,
                other => unreachable!("unvalidated mask field `{other}`"),
            }
        }
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

    /// Fault: disconnect — reports disconnected until the configured tick,
    /// then connected again. Pure function of the tick counter.
    fn is_connected(&self) -> bool {
        self.clock.ticks() >= self.faults.disconnect_until_tick
    }

    /// Deliver the CURRENT canonical snapshot (one per poll). The runner
    /// advances the world explicitly between polls.
    fn poll(&mut self) -> Result<Vec<TelemetrySnapshot>, AdapterError> {
        if !self.is_connected() {
            return Err(AdapterError::NotConnected);
        }
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
        // Fault: disconnect — writes fail like on a real disconnected link.
        if !self.is_connected() {
            return Err(AdapterError::NotConnected);
        }
        // Fault: ignored actions — ACCEPT the write but never apply its
        // postcondition. The runtime's post-condition verifier must catch
        // this lie via verification timeout; nothing here reports failure.
        if self.clock.ticks() < self.faults.ignore_actions_for_ticks {
            return Ok(());
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DEFAULT_DT_MS;
    use fd_core::actions::SwitchPosition;
    use fd_core::units::LatDeg;

    fn sim() -> VirtualSimulator {
        VirtualSimulator::new(55.97, 37.41, 622.0, 330.0, DEFAULT_DT_MS)
    }

    #[test]
    fn default_config_is_fault_free() {
        let mut s = sim();
        assert!(s.faults().is_noop());
        assert!(s.is_connected());
        s.execute(CockpitAction::SetBeacon(SwitchPosition::On))
            .unwrap();
        assert_eq!(s.snapshot().beacon_light, Some(true));
        let before = s.advance_tick();
        assert_eq!(before.timestamp.ms, DEFAULT_DT_MS);
        assert_eq!(
            before.indicated_airspeed.map(|v| v.value()),
            Some(0.0),
            "engines cold: aircraft stays parked"
        );
    }

    #[test]
    fn ignore_actions_accepts_but_never_applies_inside_window() {
        let mut s = sim().with_faults(FaultConfig {
            ignore_actions_for_ticks: 3,
            ..FaultConfig::default()
        });
        // Inside the window (ticks 0..3): Accepted, but postcondition absent.
        for _ in 0..3 {
            s.execute(CockpitAction::SetBeacon(SwitchPosition::On))
                .unwrap();
            assert_eq!(s.snapshot().beacon_light, Some(false));
            s.advance_tick();
        }
        // Window closed: the same action now applies.
        s.execute(CockpitAction::SetBeacon(SwitchPosition::On))
            .unwrap();
        assert_eq!(s.snapshot().beacon_light, Some(true));
    }

    #[test]
    fn telemetry_freeze_returns_identical_snapshot_then_resumes() {
        let mut s = sim().with_faults(FaultConfig {
            telemetry_freeze_until_tick: 3,
            ..FaultConfig::default()
        });
        s.kinematics_mut().start_airborne_at(10_000.0);
        let frozen = s.snapshot();
        // Two frozen steps: byte-identical snapshots INCLUDING timestamps.
        let a = s.advance_tick();
        let b = s.advance_tick();
        assert_eq!(a.timestamp.ms, frozen.timestamp.ms);
        assert_eq!(b.timestamp.ms, frozen.timestamp.ms);
        assert_eq!(
            a.altitude_msl.map(|v| v.value()),
            b.altitude_msl.map(|v| v.value())
        );
        assert_eq!(s.ticks(), 0, "world clock is frozen too");
        // Third call leaves the window (ticks() == 3 == freeze bound? no:
        // ticks() is still 0 after two frozen calls, so this one freezes as
        // well — the bound is compared against the WORLD tick counter).
        let _c = s.advance_tick();
        assert_eq!(s.ticks(), 0);
        // Now past the window: state advances again.
        let d = s.advance_tick();
        assert_eq!(s.ticks(), 1);
        assert!(d.timestamp.ms > frozen.timestamp.ms);
    }

    #[test]
    fn disconnect_until_tick_blocks_poll_and_execute_deterministically() {
        let mut s = sim().with_faults(FaultConfig {
            disconnect_until_tick: 2,
            ..FaultConfig::default()
        });
        assert!(!s.is_connected());
        assert!(matches!(s.poll(), Err(AdapterError::NotConnected)));
        assert!(matches!(
            s.execute(CockpitAction::SetBeacon(SwitchPosition::On)),
            Err(AdapterError::NotConnected)
        ));
        s.advance_tick();
        assert!(!s.is_connected());
        s.advance_tick();
        assert!(s.is_connected());
        s.poll().unwrap();
        s.execute(CockpitAction::SetBeacon(SwitchPosition::On))
            .unwrap();
        assert_eq!(s.snapshot().beacon_light, Some(true));
    }

    #[test]
    fn unknown_sensor_fields_read_back_as_none() {
        let mut s = sim().with_faults(FaultConfig {
            unknown_sensor_fields: vec!["ias".into(), "vs".into(), "altitude_msl".into()],
            ..FaultConfig::default()
        });
        s.kinematics_mut().start_airborne_at(9_000.0);
        let snap = s.snapshot();
        assert_eq!(snap.indicated_airspeed, None, "`ias` masked");
        assert_eq!(snap.vertical_speed, None, "`vs` masked");
        assert_eq!(snap.altitude_msl, None, "`altitude_msl` masked");
        // Untouched fields stay known.
        assert!(snap.position.is_some());
        assert!(snap.altitude_agl.is_some());
        assert_eq!(
            snap.position.as_ref().map(|p| p.lat.value()),
            Some(LatDeg::new(55.97).value())
        );
    }

    #[test]
    fn alias_vs_and_canonical_names_are_equivalent_masks() {
        let mut a = sim().with_faults(FaultConfig {
            unknown_sensor_fields: vec!["vertical_speed".into()],
            ..FaultConfig::default()
        });
        let mut b = sim().with_faults(FaultConfig {
            unknown_sensor_fields: vec!["vs".into()],
            ..FaultConfig::default()
        });
        a.kinematics_mut().start_airborne_at(5_000.0);
        b.kinematics_mut().start_airborne_at(5_000.0);
        assert_eq!(a.snapshot().vertical_speed, None);
        assert_eq!(b.snapshot().vertical_speed, None);
    }

    #[test]
    #[should_panic(expected = "invalid FaultConfig")]
    fn unvalidated_mask_name_fails_at_construction() {
        let _ = sim().with_faults(FaultConfig {
            unknown_sensor_fields: vec!["airspeed_bogus".into()],
            ..FaultConfig::default()
        });
    }

    #[test]
    fn faults_are_a_pure_function_of_the_tick_counter() {
        // Identical configs + identical action sequences => identical
        // observable traces (determinism contract of spec §41).
        let run = || {
            let mut s = sim().with_faults(FaultConfig {
                ignore_actions_for_ticks: 2,
                telemetry_freeze_until_tick: 1,
                disconnect_until_tick: 3,
                unknown_sensor_fields: vec!["ias".into()],
            });
            let mut trace = Vec::new();
            for _ in 0..5 {
                let beacon = s.execute(CockpitAction::SetBeacon(SwitchPosition::On));
                let poll = s.poll().map(|mut v| v.pop().map(|p| p.timestamp.ms));
                s.advance_tick();
                trace.push((beacon.is_ok(), poll.is_ok(), s.ticks(), s.is_connected()));
            }
            trace
        };
        assert_eq!(run(), run());
    }
}
