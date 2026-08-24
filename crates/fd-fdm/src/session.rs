//! Flight session lifecycle (Task 6 §10): a state machine driven purely by
//! telemetry evidence — never by UI scripting.
//!
//! [`SessionTracker::advance`] is a deterministic function of
//! (current state, evidence). Unknown data never advances the lifecycle
//! past what the evidence supports (fail-closed): e.g. airborne requires a
//! KNOWN AGL/on-ground pair, parked requires KNOWN ground speed.
//!
//! DEVELOPMENT DEFAULTS: transition thresholds carry named constants and
//! are not airline policy.

use serde::{Deserialize, Serialize};

/// DEVELOPMENT DEFAULT: AGL at or above this (ft), with wheels-up report,
/// counts as airborne.
pub const AIRBORNE_AGL_FT: f64 = 50.0;
/// DEVELOPMENT DEFAULT: ground speed at or below this (kt), on ground,
/// after having been airborne, counts as parked.
pub const PARKED_SPEED_KT: f64 = 1.0;
/// DEVELOPMENT DEFAULT: consecutive samples a flight-state condition must
/// hold before the transition fires (debounces noisy sensors). Lifecycle
/// facts (connected/identity/first-sample) are single-evidence.
pub const TRANSITION_SUSTAIN_SAMPLES: u32 = 2;
/// DEVELOPMENT DEFAULT: AGL at or below this (ft) while descending marks
/// the landing phase of the session.
pub const LANDING_AGL_FT: f64 = 3000.0;

/// Session lifecycle states (Task 6 §10; names adapted to existing
/// vocabulary).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FlightSessionState {
    /// No simulator contact yet.
    AwaitingSimulator,
    /// Transport alive, telemetry flowing.
    Connected,
    /// Aircraft identity resolved.
    AircraftDetected,
    /// First sample recorded.
    Recording,
    /// Sustained airborne evidence.
    Airborne,
    /// Descending low, approach context.
    Landing,
    /// Back on the ground and slow after a flight.
    Parked,
    /// Explicitly closed (operator, reload, restart). Terminal.
    SessionClosed,
}

/// One observation's evidence for the lifecycle transition. Every field is
/// independently unknown (`None`) — the tracker never guesses.
#[derive(Debug, Clone, Copy, Default)]
pub struct SessionEvidence {
    pub connected: bool,
    pub identity_known: bool,
    pub sample_recorded: bool,
    pub altitude_agl_ft: Option<f64>,
    pub on_ground: Option<bool>,
    pub groundspeed_kt: Option<f64>,
    pub descending: Option<bool>,
}

/// Pending flight-state transition being sustained.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Pending {
    Airborne,
    Landing,
    Parked,
}

/// Deterministic session lifecycle tracker.
#[derive(Debug, Clone)]
pub struct SessionTracker {
    state: FlightSessionState,
    /// Consecutive samples the pending condition has held.
    sustain: u32,
    /// Pending flight-state transition (lifecycle transitions are
    /// single-evidence and applied immediately).
    pending: Option<Pending>,
    /// Whether the session ever went airborne (gates the Parked state).
    ever_airborne: bool,
}

impl SessionTracker {
    pub fn new() -> Self {
        Self {
            state: FlightSessionState::AwaitingSimulator,
            sustain: 0,
            pending: None,
            ever_airborne: false,
        }
    }

    pub fn state(&self) -> FlightSessionState {
        self.state
    }

    pub fn ever_airborne(&self) -> bool {
        self.ever_airborne
    }

    /// Feed one observation; returns the (possibly unchanged) state.
    ///
    /// A lost transport is recorded by transport health, not by silently
    /// finishing the flight (Task 6 §44/§57): only an explicit
    /// [`close`](Self::close) reaches `SessionClosed`.
    pub fn advance(&mut self, ev: SessionEvidence) -> FlightSessionState {
        if self.state == FlightSessionState::SessionClosed {
            return self.state;
        }

        // Single-evidence lifecycle facts.
        self.state = match self.state {
            FlightSessionState::AwaitingSimulator if ev.connected => FlightSessionState::Connected,
            FlightSessionState::Connected if ev.identity_known => {
                FlightSessionState::AircraftDetected
            }
            FlightSessionState::AircraftDetected if ev.sample_recorded => {
                FlightSessionState::Recording
            }
            _ => self.state,
        };

        // Sustained flight-state transitions (Recording/Airborne/Landing).
        let desired: Option<Pending> = match self.state {
            FlightSessionState::Recording
            | FlightSessionState::Airborne
            | FlightSessionState::Landing => Self::flight_desired(&ev, self.ever_airborne),
            _ => None,
        };
        match desired {
            None => {
                self.sustain = 0;
                self.pending = None;
            }
            Some(target) => {
                if self.pending == Some(target) {
                    self.sustain += 1;
                } else {
                    self.pending = Some(target);
                    self.sustain = 1;
                }
                if self.sustain >= TRANSITION_SUSTAIN_SAMPLES {
                    self.apply(target);
                }
            }
        }
        self.state
    }

    /// Desired flight-state transition from evidence.
    fn flight_desired(ev: &SessionEvidence, ever_airborne: bool) -> Option<Pending> {
        match (ev.altitude_agl_ft, ev.on_ground) {
            (Some(agl), Some(false)) => {
                if agl < AIRBORNE_AGL_FT {
                    // Below the airborne floor: no flight-state evidence
                    // (the noisy moments right at touchdown).
                    None
                } else if ev.descending == Some(true) && agl <= LANDING_AGL_FT {
                    Some(Pending::Landing)
                } else {
                    Some(Pending::Airborne)
                }
            }
            (Some(_), Some(true)) => {
                if ever_airborne {
                    match ev.groundspeed_kt {
                        Some(gs) if gs <= PARKED_SPEED_KT => Some(Pending::Parked),
                        _ => None,
                    }
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    fn apply(&mut self, target: Pending) {
        self.state = match target {
            Pending::Airborne => {
                self.ever_airborne = true;
                FlightSessionState::Airborne
            }
            Pending::Landing => FlightSessionState::Landing,
            Pending::Parked => FlightSessionState::Parked,
        };
        self.sustain = 0;
        self.pending = None;
    }

    /// Explicit session close (operator stop, reload, restart). Idempotent.
    pub fn close(&mut self) -> FlightSessionState {
        self.state = FlightSessionState::SessionClosed;
        self.state
    }
}

impl Default for SessionTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drive the tracker from AwaitingSimulator to Recording.
    fn bootstrap(t: &mut SessionTracker) {
        t.advance(SessionEvidence {
            connected: true,
            ..Default::default()
        });
        t.advance(SessionEvidence {
            connected: true,
            identity_known: true,
            ..Default::default()
        });
        t.advance(ground_ev(true));
    }

    fn ground_ev(connected: bool) -> SessionEvidence {
        SessionEvidence {
            connected,
            identity_known: true,
            sample_recorded: true,
            altitude_agl_ft: Some(3.0),
            on_ground: Some(true),
            groundspeed_kt: Some(0.0),
            descending: Some(false),
        }
    }

    fn air_ev(agl: f64) -> SessionEvidence {
        SessionEvidence {
            connected: true,
            identity_known: true,
            sample_recorded: true,
            altitude_agl_ft: Some(agl),
            on_ground: Some(false),
            groundspeed_kt: Some(80.0),
            descending: Some(false),
        }
    }

    #[test]
    fn lifecycle_walks_connected_to_airborne() {
        let mut t = SessionTracker::new();
        assert_eq!(t.state(), FlightSessionState::AwaitingSimulator);
        assert_eq!(
            t.advance(SessionEvidence::default()),
            FlightSessionState::AwaitingSimulator
        );
        assert_eq!(
            t.advance(SessionEvidence {
                connected: true,
                ..Default::default()
            }),
            FlightSessionState::Connected
        );
        assert_eq!(
            t.advance(SessionEvidence {
                connected: true,
                identity_known: true,
                ..Default::default()
            }),
            FlightSessionState::AircraftDetected
        );
        assert_eq!(t.advance(ground_ev(true)), FlightSessionState::Recording);
        assert_eq!(
            t.advance(air_ev(2000.0)),
            FlightSessionState::Recording,
            "one sample does not make airborne"
        );
        assert_eq!(
            t.advance(air_ev(2000.0)),
            FlightSessionState::Airborne,
            "sustained airborne"
        );
        assert!(t.ever_airborne());
    }

    #[test]
    fn unknown_evidence_never_advances_flight_state() {
        let mut t = SessionTracker::new();
        bootstrap(&mut t);
        assert_eq!(t.state(), FlightSessionState::Recording);
        for _ in 0..5 {
            t.advance(SessionEvidence {
                connected: true,
                identity_known: true,
                sample_recorded: true,
                altitude_agl_ft: None,
                on_ground: Some(false),
                groundspeed_kt: None,
                descending: None,
            });
        }
        assert_eq!(
            t.state(),
            FlightSessionState::Recording,
            "unknown AGL never produces airborne"
        );
    }

    #[test]
    fn airborne_to_landing_to_parked() {
        let mut t = SessionTracker::new();
        bootstrap(&mut t);
        t.advance(air_ev(2000.0));
        t.advance(air_ev(2000.0));
        assert_eq!(t.state(), FlightSessionState::Airborne);
        let mut ev = air_ev(800.0);
        ev.descending = Some(true);
        t.advance(ev);
        t.advance(ev);
        assert_eq!(t.state(), FlightSessionState::Landing);
        let mut parked = ground_ev(true);
        parked.groundspeed_kt = Some(0.5);
        t.advance(parked);
        t.advance(parked);
        assert_eq!(t.state(), FlightSessionState::Parked);
    }

    #[test]
    fn parked_requires_prior_airborne() {
        let mut t = SessionTracker::new();
        bootstrap(&mut t);
        assert_eq!(t.state(), FlightSessionState::Recording);
        let mut slow = ground_ev(true);
        slow.groundspeed_kt = Some(0.2);
        t.advance(slow);
        t.advance(slow);
        assert_eq!(
            t.state(),
            FlightSessionState::Recording,
            "the session never claims a completed flight it did not observe"
        );
    }

    #[test]
    fn goaround_returns_to_airborne() {
        let mut t = SessionTracker::new();
        bootstrap(&mut t);
        let high = air_ev(4000.0);
        t.advance(high);
        t.advance(high);
        assert_eq!(t.state(), FlightSessionState::Airborne);
        let mut low = air_ev(500.0);
        low.descending = Some(true);
        t.advance(low);
        t.advance(low);
        assert_eq!(t.state(), FlightSessionState::Landing);
        t.advance(high);
        t.advance(high);
        assert_eq!(
            t.state(),
            FlightSessionState::Airborne,
            "go-around returns to Airborne"
        );
    }

    #[test]
    fn close_is_terminal() {
        let mut t = SessionTracker::new();
        t.advance(SessionEvidence {
            connected: true,
            ..Default::default()
        });
        t.close();
        assert_eq!(t.state(), FlightSessionState::SessionClosed);
        assert_eq!(
            t.advance(ground_ev(true)),
            FlightSessionState::SessionClosed
        );
    }
}
