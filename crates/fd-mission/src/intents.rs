//! High-level flight intent (contract C8): a data-only description of what
//! mission autonomy intends this tick.
//!
//! # Single decision source (invariant)
//!
//! [`intent_from_tick`] classifies the exact [`MissionCommands`] produced by
//! [`intended_commands`] (the command half of the controller's pure
//! `intended_tick`) for the given `(phase, ctx, params)`. It runs no phase
//! state machine of its own, so an intent and the guidance commands the
//! controller acts on can never disagree — they are two views of one tick
//! output. Callers MUST pass commands that were produced for the very same
//! `(phase, ctx, params)` passed here; [`intent_for_tick`] enforces this
//! identity by construction.
//!
//! # Never dispatched (zero-write by construction)
//!
//! Nothing in this module imports an adapter or action type; a
//! [`HighLevelIntent`] is observation vocabulary for shadow/debrief
//! consumers and never a [`CockpitAction`](fd_core::catalog) equivalent.
//! There is no dispatch path to reach — see the mirrored compile-level
//! argument in [`crate::shadow`].
//!
//! # Deferred enrichment
//!
//! `FollowRouteLeg::leg_index` is always `None` today because
//! [`MissionContext`] carries no route reference. The route-leg shadow
//! channel is likewise deferred until the route monitor observation type
//! lands (Lane R owns `monitor.rs`) and Integration wires it.

use serde::{Deserialize, Serialize};

use crate::controller::{
    DESCENT_VS_FPM, MissionCommands, MissionContext, MissionParameters, MissionPhase,
    intended_commands,
};

/// Deterministic machine-readable justification for an intent: fixed
/// `"because x<y"`-style tokens, no natural-language prose.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Reason(pub String);

impl Reason {
    /// The token string.
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// What mission autonomy intends this tick. Pure data — never dispatched,
/// never converted into an action by this crate.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "intent", rename_all = "snake_case")]
pub enum HighLevelIntent {
    ClimbTo {
        target_ft: f64,
        reason: Reason,
    },
    DescendTo {
        target_ft: f64,
        reason: Reason,
    },
    MaintainAltitude {
        target_ft: f64,
        reason: Reason,
    },
    FollowRouteLeg {
        leg_index: Option<usize>,
        target_heading_deg: Option<f64>,
        reason: Reason,
    },
    PrepareDescent {
        reason: Reason,
    },
    ConfigureForApproach {
        reason: Reason,
    },
}

impl HighLevelIntent {
    /// Stable snake_case variant tag; the aggregation key used in shadow
    /// summaries and observability counters.
    pub const fn variant_name(&self) -> &'static str {
        match self {
            Self::ClimbTo { .. } => "climb_to",
            Self::DescendTo { .. } => "descend_to",
            Self::MaintainAltitude { .. } => "maintain_altitude",
            Self::FollowRouteLeg { .. } => "follow_route_leg",
            Self::PrepareDescent { .. } => "prepare_descent",
            Self::ConfigureForApproach { .. } => "configure_for_approach",
        }
    }

    /// Why autonomy holds this intent on this tick.
    pub const fn reason(&self) -> &Reason {
        match self {
            Self::ClimbTo { reason, .. }
            | Self::DescendTo { reason, .. }
            | Self::MaintainAltitude { reason, .. }
            | Self::FollowRouteLeg { reason, .. }
            | Self::PrepareDescent { reason }
            | Self::ConfigureForApproach { reason } => reason,
        }
    }
}

/// Derive the tick's high-level intent from the intended commands.
///
/// `cmds` MUST be the output of
/// [`intended_commands(phase, ctx, params)`](intended_commands) for the
/// same arguments passed here (single decision source — see module docs).
///
/// Returns `None` for ground/terminal phases (`Landing`, `Parked`,
/// `Completed`, `Failed`): the mission there only decelerates/parks and
/// issues no flight-guidance intent, and the intent vocabulary has no
/// ground-taxi variant — absence is reported as `None`, never fabricated.
pub fn intent_from_tick(
    phase: &MissionPhase,
    cmds: &MissionCommands,
    ctx: &MissionContext<'_>,
    params: &MissionParameters,
) -> Option<HighLevelIntent> {
    let altitude_target = cmds.set_target_altitude_ft;
    let heading = cmds.set_target_heading_deg;

    match *phase {
        MissionPhase::Preflight => Some(HighLevelIntent::FollowRouteLeg {
            leg_index: None,
            target_heading_deg: heading,
            reason: Reason("because takeoff_roll_heading".into()),
        }),
        MissionPhase::Takeoff => Some(HighLevelIntent::FollowRouteLeg {
            leg_index: None,
            target_heading_deg: heading,
            reason: Reason("because climbout_heading".into()),
        }),
        MissionPhase::Climb | MissionPhase::Cruise => {
            // Top-of-descent trigger: recognizable purely from the command
            // shape — the descent VS is commanded only on trigger ticks in
            // these phases (`DESCENT_VS_FPM` is shared with the controller).
            if cmds.set_target_vertical_speed_fpm == Some(DESCENT_VS_FPM)
                && altitude_target.is_some()
            {
                Some(HighLevelIntent::PrepareDescent {
                    reason: Reason(descent_trigger_reason(ctx, params)),
                })
            } else if let Some(target_ft) = altitude_target {
                if *phase == MissionPhase::Climb {
                    Some(HighLevelIntent::ClimbTo {
                        target_ft,
                        reason: Reason("because altitude<target".into()),
                    })
                } else {
                    Some(HighLevelIntent::MaintainAltitude {
                        target_ft,
                        reason: Reason("because altitude_at_target".into()),
                    })
                }
            } else {
                Some(HighLevelIntent::FollowRouteLeg {
                    leg_index: None,
                    target_heading_deg: heading,
                    reason: Reason("because active_leg_guidance".into()),
                })
            }
        }
        MissionPhase::Descent => Some(match altitude_target {
            Some(target_ft) => HighLevelIntent::DescendTo {
                target_ft,
                reason: Reason("because descending_to_approach_floor".into()),
            },
            None => HighLevelIntent::FollowRouteLeg {
                leg_index: None,
                target_heading_deg: heading,
                reason: Reason("because active_leg_guidance".into()),
            },
        }),
        MissionPhase::Approach => Some(HighLevelIntent::ConfigureForApproach {
            reason: Reason("because distance_to_destination<approach_gate".into()),
        }),
        MissionPhase::Landing
        | MissionPhase::Parked
        | MissionPhase::Completed
        | MissionPhase::Failed => None,
    }
}

/// Replay the pure controller tick and map its commands to an intent in one
/// step: identical to
/// `intent_from_tick(phase, &intended_commands(phase, ctx, params), ctx, params)`,
/// which guarantees the single-decision-source invariant by construction.
pub fn intent_for_tick(
    phase: &MissionPhase,
    ctx: &MissionContext<'_>,
    params: &MissionParameters,
) -> Option<HighLevelIntent> {
    intent_from_tick(phase, &intended_commands(phase, ctx, params), ctx, params)
}

/// Deterministic justification for the top-of-descent trigger. The commands
/// stay authoritative (they ARE the decision); this annotates whether the
/// known threshold comparison agrees with them.
fn descent_trigger_reason(ctx: &MissionContext<'_>, params: &MissionParameters) -> String {
    if ctx.distance_to_destination_nm <= params.descent_distance_nm {
        "because distance_to_destination_nm<=descent_distance_nm".into()
    } else {
        "because descent_command_active".into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fd_core::telemetry::{SimState, SimTimestamp, TelemetrySnapshot};
    use fd_core::units::{AltitudeFt, SpeedKt};

    fn snap(alt_msl: Option<f64>, ias: f64, on_ground: bool) -> TelemetrySnapshot {
        let mut s = TelemetrySnapshot::empty(SimTimestamp::new(0));
        s.altitude_msl = alt_msl.map(AltitudeFt::new);
        s.indicated_airspeed = Some(SpeedKt::new(ias));
        s.on_ground = Some(on_ground);
        s.sim_timing.state = SimState::Running;
        s
    }

    fn ctx(snap: &TelemetrySnapshot, dist_nm: f64) -> MissionContext<'_> {
        MissionContext {
            snapshot: snap,
            distance_to_destination_nm: dist_nm,
            bearing_to_waypoint_deg: 45.0,
        }
    }

    /// Climb far from top of descent intends climbing to the cruise level.
    #[test]
    fn climb_far_from_tod_intends_climb_to_cruise() {
        let params = MissionParameters::default();
        let s = snap(Some(20_000.0), 290.0, false);
        let c = ctx(&s, 400.0);
        let intent = intent_for_tick(&MissionPhase::Climb, &c, &params).unwrap();
        assert_eq!(
            intent,
            HighLevelIntent::ClimbTo {
                target_ft: 34_000.0,
                reason: Reason("because altitude<target".into()),
            }
        );
        assert_eq!(intent.variant_name(), "climb_to");
        assert_eq!(intent.reason().as_str(), "because altitude<target");
    }

    /// Cruise away from any trigger holds the cruise altitude.
    #[test]
    fn cruise_holds_altitude_with_reason() {
        let params = MissionParameters::default();
        let s = snap(Some(34_000.0), 450.0, false);
        let c = ctx(&s, 400.0);
        let intent = intent_for_tick(&MissionPhase::Cruise, &c, &params).unwrap();
        assert_eq!(
            intent,
            HighLevelIntent::MaintainAltitude {
                target_ft: 34_000.0,
                reason: Reason("because altitude_at_target".into()),
            }
        );
        assert_eq!(intent.variant_name(), "maintain_altitude");
    }

    /// Crossing the descent threshold flips both climb and cruise into
    /// PrepareDescent with the threshold-comparison reason token.
    #[test]
    fn descent_trigger_yields_prepare_descent_in_climb_and_cruise() {
        let params = MissionParameters::default();
        let expected_reason = "because distance_to_destination_nm<=descent_distance_nm";
        for phase in [MissionPhase::Climb, MissionPhase::Cruise] {
            let s = snap(Some(30_000.0), 300.0, false);
            let c = ctx(&s, params.descent_distance_nm - 1.0);
            let intent = intent_for_tick(&phase, &c, &params).unwrap();
            assert_eq!(intent.variant_name(), "prepare_descent", "{phase:?}");
            assert_eq!(intent.reason().as_str(), expected_reason, "{phase:?}");
        }
    }

    /// Descence intends the floor altitude the tick actually commands:
    /// high → 3000 ft estimate, low → 2000 ft estimate.
    #[test]
    fn descent_intends_descend_to_commanded_floor() {
        let params = MissionParameters::default();
        let hi = snap(Some(8_000.0), 250.0, false);
        let intent = intent_for_tick(&MissionPhase::Descent, &ctx(&hi, 50.0), &params).unwrap();
        assert_eq!(
            intent,
            HighLevelIntent::DescendTo {
                target_ft: 3_000.0,
                reason: Reason("because descending_to_approach_floor".into()),
            }
        );
        let lo = snap(Some(4_000.0), 220.0, false);
        let intent = intent_for_tick(&MissionPhase::Descent, &ctx(&lo, 20.0), &params).unwrap();
        assert_eq!(
            intent,
            HighLevelIntent::DescendTo {
                target_ft: 2_000.0,
                reason: Reason("because descending_to_approach_floor".into()),
            }
        );
        assert_eq!(intent.variant_name(), "descend_to");
    }

    /// Approach configures for approach; ground roll/climb-out follow the
    /// active leg heading; terminal phases have no flight-guidance intent.
    #[test]
    fn approach_configures_ground_roll_follows_and_terminal_is_none() {
        let params = MissionParameters::default();

        let air = snap(Some(2_500.0), 180.0, false);
        let intent = intent_for_tick(&MissionPhase::Approach, &ctx(&air, 10.0), &params).unwrap();
        assert_eq!(intent.variant_name(), "configure_for_approach");
        assert_eq!(
            intent.reason().as_str(),
            "because distance_to_destination<approach_gate"
        );

        let ground = snap(Some(622.0), 0.0, true);
        let intent =
            intent_for_tick(&MissionPhase::Preflight, &ctx(&ground, 400.0), &params).unwrap();
        assert_eq!(
            intent,
            HighLevelIntent::FollowRouteLeg {
                leg_index: None,
                target_heading_deg: Some(45.0),
                reason: Reason("because takeoff_roll_heading".into()),
            }
        );

        let rising = snap(Some(700.0), 120.0, false);
        let intent =
            intent_for_tick(&MissionPhase::Takeoff, &ctx(&rising, 400.0), &params).unwrap();
        assert_eq!(
            intent,
            HighLevelIntent::FollowRouteLeg {
                leg_index: None,
                target_heading_deg: Some(45.0),
                reason: Reason("because climbout_heading".into()),
            }
        );

        // Terminal phases: no flight-guidance intent, never fabricated.
        let stopped = snap(Some(622.0), 4.0, true);
        for phase in [
            MissionPhase::Landing,
            MissionPhase::Parked,
            MissionPhase::Completed,
            MissionPhase::Failed,
        ] {
            assert_eq!(
                intent_for_tick(&phase, &ctx(&stopped, 0.0), &params),
                None,
                "{phase:?}"
            );
        }
    }

    /// Mapping raw tick commands and replaying via `intent_for_tick`
    /// agree exactly — the single-decision-source invariant.
    #[test]
    fn intent_from_tick_matches_replay_helper() {
        let params = MissionParameters::default();
        let cases = [
            (MissionPhase::Preflight, Some(622.0), 0.0, true, 400.0),
            (MissionPhase::Takeoff, Some(1_500.0), 150.0, false, 400.0),
            (MissionPhase::Climb, Some(20_000.0), 290.0, false, 400.0),
            (MissionPhase::Climb, Some(30_000.0), 300.0, false, 100.0),
            (MissionPhase::Cruise, Some(34_000.0), 450.0, false, 400.0),
            (MissionPhase::Cruise, Some(34_000.0), 450.0, false, 90.0),
            (MissionPhase::Descent, Some(8_000.0), 250.0, false, 50.0),
            (MissionPhase::Approach, Some(2_500.0), 180.0, false, 10.0),
            (MissionPhase::Landing, Some(622.0), 100.0, true, 0.0),
        ];
        for (phase, alt, ias, on_ground, dist) in cases {
            let s = snap(alt, ias, on_ground);
            let c = ctx(&s, dist);
            let cmds = intended_commands(&phase, &c, &params);
            assert_eq!(
                intent_from_tick(&phase, &cmds, &c, &params),
                intent_for_tick(&phase, &c, &params),
                "{phase:?}"
            );
        }
    }
}
