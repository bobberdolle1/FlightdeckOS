//! Deterministic flight phase engine.
//!
//! Extracted from the OpenAIRAC project (`crates/openairac-charts/src/efb.rs`,
//! MIT license, same author; commit `c7a2bfd` of `bobberdolle1/open-airac`).
//! Extraction instead of a dependency was chosen deliberately:
//!
//! * `openairac-charts` is an umbrella crate pulling in the SQLite store,
//!   procedure model, rusqlite and reqwest — disproportionate weight for
//!   ~200 lines of pure logic;
//! * FlightdeckOS needs to evolve this engine independently (typed units,
//!   pause/time-scale input); open-airac's copy must stay untouched.
//!
//! Behavioral fidelity is preserved: same thresholds, same hysteresis
//! (2 consecutive ticks), same immediate liftoff/touchdown transitions, same
//! slew/teleport detection (altitude jump > 10 000 ft within < 5 s resets to
//! Preflight/Cruise with `Medium` confidence). The only change is the
//! timestamp representation: `chrono::DateTime<Utc>` → [`SimTimestamp`] (u64
//! ms), which is deterministic and serde-friendly.
//!
//! If open-airac later extracts `efb` into a standalone crate, this module
//! should be replaced by that dependency and deleted.

use serde::{Deserialize, Serialize};

use crate::telemetry::{SimTimestamp, TelemetrySnapshot};

/// Deterministic flight phase classifications.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum FlightPhase {
    #[default]
    Preflight,
    TaxiOut,
    Takeoff,
    InitialClimb,
    Departure,
    Climb,
    Cruise,
    Descent,
    Arrival,
    Approach,
    Final,
    Landing,
    TaxiIn,
    Parked,
    Unknown,
}

impl FlightPhase {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Preflight => "PREFLIGHT",
            Self::TaxiOut => "TAXI_OUT",
            Self::Takeoff => "TAKEOFF",
            Self::InitialClimb => "INITIAL_CLIMB",
            Self::Departure => "DEPARTURE",
            Self::Climb => "CLIMB",
            Self::Cruise => "CRUISE",
            Self::Descent => "DESCENT",
            Self::Arrival => "ARRIVAL",
            Self::Approach => "APPROACH",
            Self::Final => "FINAL",
            Self::Landing => "LANDING",
            Self::TaxiIn => "TAXI_IN",
            Self::Parked => "PARKED",
            Self::Unknown => "UNKNOWN",
        }
    }

    pub const fn is_airborne(self) -> bool {
        matches!(
            self,
            Self::Takeoff
                | Self::InitialClimb
                | Self::Departure
                | Self::Climb
                | Self::Cruise
                | Self::Descent
                | Self::Arrival
                | Self::Approach
                | Self::Final
        )
    }

    pub const fn is_terminal_arrival(self) -> bool {
        matches!(
            self,
            Self::Arrival | Self::Approach | Self::Final | Self::Landing
        )
    }
}

/// Confidence level of flight phase inference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PhaseConfidence {
    High,
    Medium,
    Low,
    Unknown,
}

/// Flight phase assessment result with evidence trail.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhaseAssessment {
    pub phase: FlightPhase,
    pub confidence: PhaseConfidence,
    pub evidence: String,
    pub timestamp: SimTimestamp,
}

/// Legacy fallback used when no route/runway distance is known: with this
/// value no arrival/approach threshold can match (behavioral fidelity with
/// the original open-airac engine).
pub const NO_DISTANCES_FALLBACK_NM: f64 = 999.0;

// DEVELOPMENT DEFAULTS for distance-driven phase transitions. These gate
// ONLY callers that actually supply distances (`PhaseTelemetry::
//~ with_distances`); callers leaving them `None` keep the exact legacy
//~ fallback behavior above.
/// Destination/runway distance entering the approach phase (NM).
pub const APPROACH_ENTRY_DEST_NM: f64 = 20.0;
/// Destination/runway distance entering the final approach segment (NM).
pub const FINAL_APPROACH_DEST_NM: f64 = 5.0;
/// Destination distance entering terminal arrival (NM).
pub const ARRIVAL_DEST_NM: f64 = 60.0;
/// Distance from departure within which climb counts as terminal departure (NM).
pub const DEPARTURE_FROM_DEP_NM: f64 = 30.0;
/// Destination distance inside which descent counts as enroute descent phase (NM).
pub const ENROUTE_DESCENT_DEST_NM: f64 = 150.0;

/// Input aircraft telemetry for flight phase assessment (raw values).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhaseTelemetry {
    pub on_ground: bool,
    pub altitude_msl_ft: f64,
    pub altitude_agl_ft: Option<f64>,
    pub groundspeed_kt: f64,
    pub vertical_speed_fpm: f64,
    pub distance_to_dest_nm: Option<f64>,
    /// Distance to the landing runway/threshold (NM), when known. Consumed
    /// for approach/final gating when destination distance is absent.
    #[serde(default)]
    pub distance_to_runway_nm: Option<f64>,
    pub distance_from_dep_nm: Option<f64>,
    pub active_procedure_kind: Option<char>, // 'D'=SID, 'E'=STAR, 'F'=Approach
    pub timestamp: SimTimestamp,
}

impl PhaseTelemetry {
    /// Attach known distances (NM) so the arrival/approach chain becomes
    /// reachable. Callers without route data leave both `None` and keep the
    /// exact legacy fallback behavior ([`NO_DISTANCES_FALLBACK_NM`]).
    pub fn with_distances(mut self, dest_nm: Option<f64>, runway_nm: Option<f64>) -> Self {
        self.distance_to_dest_nm = dest_nm;
        self.distance_to_runway_nm = runway_nm;
        self
    }
}

impl From<&TelemetrySnapshot> for PhaseTelemetry {
    fn from(s: &TelemetrySnapshot) -> Self {
        Self {
            on_ground: s.on_ground.unwrap_or(true),
            altitude_msl_ft: s.altitude_msl.map(|v| v.value()).unwrap_or(0.0),
            altitude_agl_ft: s.altitude_agl.map(|v| v.value()),
            groundspeed_kt: s.groundspeed.map(|v| v.value()).unwrap_or(0.0),
            vertical_speed_fpm: s.vertical_speed.map(|v| v.value()).unwrap_or(0.0),
            // No route context wired yet (Integration supplies distances
            // via `with_distances`): the engine falls back to its documented
            // defaults (999 NM / no procedure) — unchanged legacy behavior.
            distance_to_dest_nm: None,
            distance_to_runway_nm: None,
            distance_from_dep_nm: None,
            active_procedure_kind: None,
            timestamp: s.timestamp,
        }
    }
}

/// Deterministic flight phase engine with hysteresis and slew protection.
#[derive(Debug, Clone, Default)]
pub struct FlightPhaseEngine {
    current_phase: FlightPhase,
    consecutive_ticks: u32,
    last_telemetry: Option<PhaseTelemetry>,
    has_been_airborne: bool,
}

impl FlightPhaseEngine {
    pub fn new() -> Self {
        Self {
            current_phase: FlightPhase::Preflight,
            consecutive_ticks: 0,
            last_telemetry: None,
            has_been_airborne: false,
        }
    }

    pub const fn current_phase(&self) -> FlightPhase {
        self.current_phase
    }

    pub fn evaluate(&mut self, telem: &PhaseTelemetry) -> PhaseAssessment {
        // 1. Detect Teleportation / Slew artifacts
        if let Some(prev) = &self.last_telemetry {
            let dt_secs = telem.timestamp.ms.saturating_sub(prev.timestamp.ms) as f64 / 1000.0;
            if dt_secs > 0.0 && dt_secs < 5.0 {
                let alt_jump = (telem.altitude_msl_ft - prev.altitude_msl_ft).abs();
                if alt_jump > 10_000.0 {
                    // Sudden impossible altitude change -> slew/teleport detected!
                    self.current_phase = if telem.on_ground {
                        FlightPhase::Preflight
                    } else {
                        FlightPhase::Cruise
                    };
                    self.consecutive_ticks = 0;
                    self.last_telemetry = Some(telem.clone());
                    return PhaseAssessment {
                        phase: self.current_phase,
                        confidence: PhaseConfidence::Medium,
                        evidence: format!(
                            "Teleport/Slew detected (Altitude jump {alt_jump:.0} ft in {dt_secs:.1}s); state reset"
                        ),
                        timestamp: telem.timestamp,
                    };
                }
            }
        }
        let (candidate, evidence) = self.infer_raw_phase(telem);

        if !telem.on_ground && telem.groundspeed_kt > 50.0 {
            self.has_been_airborne = true;
        }

        // Hysteresis: require 2 consecutive ticks for major phase shift
        // unless sudden liftoff/touchdown.
        if candidate == self.current_phase {
            self.consecutive_ticks += 1;
        } else {
            let immediate_transition = (telem.on_ground
                && self.current_phase == FlightPhase::Final)
                || (!telem.on_ground
                    && matches!(
                        self.current_phase,
                        FlightPhase::Takeoff | FlightPhase::Preflight
                    ));

            if immediate_transition || self.consecutive_ticks >= 2 {
                self.current_phase = candidate;
                self.consecutive_ticks = 1;
            } else {
                self.consecutive_ticks += 1;
            }
        }
        self.last_telemetry = Some(telem.clone());

        PhaseAssessment {
            phase: self.current_phase,
            confidence: PhaseConfidence::High,
            evidence,
            timestamp: telem.timestamp,
        }
    }

    fn infer_raw_phase(&self, telem: &PhaseTelemetry) -> (FlightPhase, String) {
        if telem.on_ground {
            if !self.has_been_airborne {
                if telem.groundspeed_kt < 3.0 {
                    (
                        FlightPhase::Preflight,
                        "On ground, stationary (GS < 3 kt)".to_string(),
                    )
                } else if telem.groundspeed_kt < 45.0 {
                    (
                        FlightPhase::TaxiOut,
                        format!("On ground, taxiing out (GS {:.0} kt)", telem.groundspeed_kt),
                    )
                } else {
                    (
                        FlightPhase::Takeoff,
                        format!(
                            "On ground, takeoff roll (GS {:.0} kt)",
                            telem.groundspeed_kt
                        ),
                    )
                }
            } else if telem.groundspeed_kt > 45.0 {
                (
                    FlightPhase::Landing,
                    format!("Touchdown rollout (GS {:.0} kt)", telem.groundspeed_kt),
                )
            } else if telem.groundspeed_kt > 3.0 {
                (
                    FlightPhase::TaxiIn,
                    format!(
                        "On ground, taxiing to gate (GS {:.0} kt)",
                        telem.groundspeed_kt
                    ),
                )
            } else {
                (
                    FlightPhase::Parked,
                    "On ground, parked at destination (GS < 3 kt)".to_string(),
                )
            }
        } else {
            let dist_dest = telem
                .distance_to_dest_nm
                .or(telem.distance_to_runway_nm)
                .unwrap_or(NO_DISTANCES_FALLBACK_NM);
            let agl = telem.altitude_agl_ft.unwrap_or(telem.altitude_msl_ft);
            let proc = telem.active_procedure_kind.unwrap_or(' ');

            if agl < 1500.0
                && telem.vertical_speed_fpm > 300.0
                && (self.current_phase == FlightPhase::Takeoff
                    || self.current_phase == FlightPhase::InitialClimb
                    || !self.has_been_airborne)
            {
                (
                    FlightPhase::InitialClimb,
                    format!(
                        "Airborne, climbing rapidly (VS {:.0} fpm, AGL {:.0} ft)",
                        telem.vertical_speed_fpm, agl
                    ),
                )
            } else if proc == 'D'
                || (telem
                    .distance_from_dep_nm
                    .unwrap_or(NO_DISTANCES_FALLBACK_NM)
                    < DEPARTURE_FROM_DEP_NM
                    && telem.vertical_speed_fpm > 200.0)
            {
                (
                    FlightPhase::Departure,
                    "Flying SID / Terminal Departure phase".to_string(),
                )
            } else if proc == 'F'
                || (dist_dest < APPROACH_ENTRY_DEST_NM
                    && agl < 4000.0
                    && telem.vertical_speed_fpm < -100.0)
            {
                if dist_dest < FINAL_APPROACH_DEST_NM && agl < 1500.0 {
                    (
                        FlightPhase::Final,
                        format!(
                            "On final approach segment (Dist {:.1} NM, AGL {:.0} ft)",
                            dist_dest, agl
                        ),
                    )
                } else {
                    (
                        FlightPhase::Approach,
                        format!(
                            "On instrument approach procedure (Dist {:.1} NM)",
                            dist_dest
                        ),
                    )
                }
            } else if proc == 'E'
                || (dist_dest < ARRIVAL_DEST_NM && telem.vertical_speed_fpm < -200.0)
            {
                (
                    FlightPhase::Arrival,
                    format!(
                        "Terminal Arrival (STAR) / Descent towards destination (Dist {:.1} NM)",
                        dist_dest
                    ),
                )
            } else if telem.vertical_speed_fpm < -300.0 && dist_dest < ENROUTE_DESCENT_DEST_NM {
                (
                    FlightPhase::Descent,
                    format!(
                        "Enroute descent (VS {:.0} fpm, Dist {:.0} NM)",
                        telem.vertical_speed_fpm, dist_dest
                    ),
                )
            } else if telem.vertical_speed_fpm > 300.0 {
                (
                    FlightPhase::Climb,
                    format!(
                        "Enroute climb (VS {:.0} fpm, Alt {:.0} ft)",
                        telem.vertical_speed_fpm, telem.altitude_msl_ft
                    ),
                )
            } else {
                (
                    FlightPhase::Cruise,
                    format!(
                        "Enroute cruise (Alt {:.0} ft, GS {:.0} kt)",
                        telem.altitude_msl_ft, telem.groundspeed_kt
                    ),
                )
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn telem(
        ts: u64,
        on_ground: bool,
        gs: f64,
        vs: f64,
        alt_msl: f64,
        agl: Option<f64>,
    ) -> PhaseTelemetry {
        PhaseTelemetry {
            on_ground,
            altitude_msl_ft: alt_msl,
            altitude_agl_ft: agl,
            groundspeed_kt: gs,
            vertical_speed_fpm: vs,
            distance_to_dest_nm: None,
            distance_to_runway_nm: None,
            distance_from_dep_nm: None,
            active_procedure_kind: None,
            timestamp: SimTimestamp::new(ts),
        }
    }

    #[test]
    fn hysteresis_requires_two_ticks_for_taxi() {
        let mut e = FlightPhaseEngine::new();
        assert_eq!(e.current_phase(), FlightPhase::Preflight);
        // Tick 1: candidate TaxiOut, not enough ticks yet.
        e.evaluate(&telem(1000, true, 10.0, 0.0, 100.0, Some(0.0)));
        assert_eq!(e.current_phase(), FlightPhase::Preflight);
        // Tick 2: still not enough.
        e.evaluate(&telem(2000, true, 12.0, 0.0, 100.0, Some(0.0)));
        assert_eq!(e.current_phase(), FlightPhase::Preflight);
        // Tick 3: hysteresis satisfied.
        e.evaluate(&telem(3000, true, 14.0, 0.0, 100.0, Some(0.0)));
        assert_eq!(e.current_phase(), FlightPhase::TaxiOut);
    }

    #[test]
    fn liftoff_transition_is_immediate() {
        let mut e = FlightPhaseEngine::new();
        // Reach Takeoff on the ground.
        for (i, gs) in [(1000, 60.0), (2000, 65.0), (3000, 70.0)] {
            e.evaluate(&telem(i, true, gs, 0.0, 100.0, Some(0.0)));
        }
        assert_eq!(e.current_phase(), FlightPhase::Takeoff);
        // Airborne: immediate transition to InitialClimb.
        let a = e.evaluate(&telem(4000, false, 160.0, 2500.0, 1200.0, Some(150.0)));
        assert_eq!(a.phase, FlightPhase::InitialClimb);
    }

    #[test]
    fn slew_jump_resets_to_cruise_with_medium_confidence() {
        let mut e = FlightPhaseEngine::new();
        // Establish Climb (airborne).
        e.evaluate(&telem(0, false, 250.0, 2000.0, 3000.0, Some(2900.0)));
        e.evaluate(&telem(1000, false, 250.0, 2000.0, 3100.0, Some(3000.0)));
        e.evaluate(&telem(2000, false, 250.0, 2000.0, 3200.0, Some(3100.0)));
        assert_eq!(e.current_phase(), FlightPhase::Climb);
        // +22 000 ft in 2 s: slew.
        let a = e.evaluate(&telem(4000, false, 450.0, 0.0, 25_200.0, Some(25_100.0)));
        assert_eq!(a.phase, FlightPhase::Cruise);
        assert_eq!(a.confidence, PhaseConfidence::Medium);
        assert!(
            a.evidence.contains("Teleport/Slew"),
            "evidence: {}",
            a.evidence
        );
    }

    #[test]
    fn snapshot_conversion_uses_defaults_when_data_missing() {
        let s = TelemetrySnapshot::empty(SimTimestamp::new(42));
        let p = PhaseTelemetry::from(&s);
        assert!(p.on_ground); // default: treat unknown as on-ground (conservative)
        assert_eq!(p.altitude_msl_ft, 0.0);
        assert_eq!(p.timestamp.ms, 42);
    }

    #[test]
    fn with_distances_arrival_and_approach_become_reachable() {
        let mut e = FlightPhaseEngine::new();
        // Establish Cruise airborne.
        e.evaluate(&telem(0, false, 250.0, 0.0, 10_000.0, Some(9_500.0)));
        // Descending, 40 NM out: ARRIVAL threshold (dev 60 NM) reached.
        let t = |ts: u64, dist: f64| {
            telem(ts, false, 230.0, -800.0, 5_000.0, Some(4_600.0)).with_distances(Some(dist), None)
        };
        e.evaluate(&t(1_000, 40.0));
        let a = e.evaluate(&t(2_000, 38.0));
        assert_eq!(a.phase, FlightPhase::Arrival, "evidence: {}", a.evidence);

        // 15 NM out, low and descending: approach entry (dev 20 NM).
        let t2 = |ts: u64, dist: f64| {
            telem(ts, false, 190.0, -600.0, 2_800.0, Some(2_400.0)).with_distances(Some(dist), None)
        };
        e.evaluate(&t2(3_000, 15.0));
        let a = e.evaluate(&t2(4_000, 14.0));
        assert_eq!(a.phase, FlightPhase::Approach, "evidence: {}", a.evidence);

        // 3 NM out, very low: final segment (dev 5 NM).
        let t3 = |ts: u64, dist: f64| {
            telem(ts, false, 140.0, -700.0, 900.0, Some(600.0)).with_distances(Some(dist), None)
        };
        e.evaluate(&t3(5_000, 3.0));
        let a = e.evaluate(&t3(6_000, 3.0));
        assert_eq!(a.phase, FlightPhase::Final, "evidence: {}", a.evidence);
    }

    #[test]
    fn runway_distance_alone_drives_approach_chain() {
        let mut e = FlightPhaseEngine::new();
        e.evaluate(&telem(0, false, 250.0, 0.0, 8_000.0, Some(7_600.0)));
        // Only runway distance known; destination distance absent.
        let t = |ts: u64, dist: f64| {
            telem(ts, false, 200.0, -900.0, 3_000.0, Some(2_600.0)).with_distances(None, Some(dist))
        };
        e.evaluate(&t(1_000, 12.0));
        let a = e.evaluate(&t(2_000, 11.0));
        assert_eq!(a.phase, FlightPhase::Approach, "evidence: {}", a.evidence);
    }

    #[test]
    fn without_distances_terminal_arrival_stays_unreachable() {
        let mut e = FlightPhaseEngine::new();
        e.evaluate(&telem(0, false, 250.0, 0.0, 8_000.0, Some(7_600.0)));
        // Identical trajectory to the reachable test but distances None:
        // legacy 999 NM fallback keeps arrival/approach/final unreachable.
        let t = |ts: u64| telem(ts, false, 180.0, -900.0, 2_000.0, Some(1_600.0));
        for ts in [1_000u64, 2_000, 3_000, 4_000, 5_000, 6_000] {
            let a = e.evaluate(&t(ts));
            assert!(
                !a.phase.is_terminal_arrival(),
                "no distance data must never yield {:?}",
                a.phase
            );
        }
    }

    #[test]
    fn builder_sets_distance_fields() {
        let base = telem(0, false, 200.0, -500.0, 3_000.0, Some(2_600.0));
        assert_eq!(base.distance_to_dest_nm, None);
        assert_eq!(base.distance_to_runway_nm, None);
        let with = base.with_distances(Some(42.0), Some(7.5));
        assert_eq!(with.distance_to_dest_nm, Some(42.0));
        assert_eq!(with.distance_to_runway_nm, Some(7.5));
    }
}
