//! Simulator-independent route monitoring (Task 6 §17, §19, §20).
//!
//! A [`RouteMonitor`] consumes normalized aircraft position and derives the
//! active leg, cross-track error, distance remaining and destination
//! proximity. It is PURE observation: nothing here dispatches, writes, or
//! commands — route monitoring never controls the aircraft (Task 6 §52).
//!
//! Unknown route (empty waypoints / `RouteSource::Unknown`) produces an
//! honest empty observation and never an off-route event (§52: missing route
//! never produces fake route compliance).

use crate::route::Waypoint;
use fd_core::geo::{distance_nm, initial_bearing_deg};
use serde::{Deserialize, Serialize};

/// Where a route came from. Evidence travels with the route (Task 6 §18):
/// no single canonical source is assumed forever.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RouteSource {
    /// Planned/resolved from OpenAIRAC data. Provenance string identifies
    /// the dataset revision (e.g. `"world.openairac.sqlite@2026-08-20T19:15:00Z"`).
    OpenAirac { provenance: String },
    /// Operator-supplied (CLI/config).
    Operator,
    /// Deterministic scenario definition (headless tests).
    Scenario,
    /// No usable route.
    Unknown,
}

impl RouteSource {
    /// Stable human-readable provenance token for reports/debriefs.
    pub fn as_str(&self) -> &'static str {
        match self {
            RouteSource::OpenAirac { .. } => "openairac",
            RouteSource::Operator => "operator",
            RouteSource::Scenario => "scenario",
            RouteSource::Unknown => "unknown",
        }
    }
}

/// A simulator-independent route: ordered waypoints plus source evidence.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteState {
    pub source: RouteSource,
    pub waypoints: Vec<Waypoint>,
}

impl RouteState {
    /// A usable route (source known, at least 2 waypoints).
    pub fn is_usable(&self) -> bool {
        !matches!(self.source, RouteSource::Unknown) && self.waypoints.len() >= 2
    }
}

/// One observation of aircraft position against the route.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct RouteObservation {
    /// Index of the active leg (from-waypoint index). `None` = unknown route
    /// or route complete.
    pub active_leg: Option<usize>,
    /// Distance to the active leg's end waypoint (nm).
    pub distance_to_waypoint_nm: Option<f64>,
    /// Signed cross-track error vs the active leg (nm). Positive = aircraft
    /// right of the leg (viewed from `from` toward `to`).
    pub cross_track_error_nm: Option<f64>,
    /// Distance remaining along the route from the current position to the
    /// final waypoint (nm): remaining legs + distance to active waypoint.
    pub distance_remaining_nm: Option<f64>,
    /// Final waypoint captured.
    pub route_complete: bool,
    /// Direct great-circle distance to the destination (nm).
    pub destination_distance_nm: Option<f64>,
}

/// DEVELOPMENT DEFAULT capture radius for waypoint advance.
/// NOT an FMS sequence rule; a coarse dev default for leg progression.
pub const WAYPOINT_CAPTURE_RADIUS_NM: f64 = 2.5;

/// A deterministic route monitor built from a [`RouteState`].
#[derive(Debug, Clone)]
pub struct RouteMonitor {
    waypoints: Vec<Waypoint>,
    /// Cumulative pre-computed leg bearings (from -> to), degrees true.
    leg_bearings_deg: Vec<f64>,
    /// Cumulative pre-computed leg lengths (nm).
    leg_lengths_nm: Vec<f64>,
    /// Index of the currently active leg (waypoint we are flying FROM).
    active_leg: usize,
    capture_radius_nm: f64,
    route_complete: bool,
}

impl RouteMonitor {
    /// Build from a route state. Unknown/short routes yield a monitor that
    /// always reports empty observations (never fabricates).
    pub fn new(state: &RouteState) -> Self {
        let wps = state.waypoints.clone();
        let mut bearings = Vec::new();
        let mut lengths = Vec::new();
        for pair in wps.windows(2) {
            bearings.push(initial_bearing_deg(
                pair[0].lat_deg,
                pair[0].lon_deg,
                pair[1].lat_deg,
                pair[1].lon_deg,
            ));
            lengths.push(distance_nm(
                pair[0].lat_deg,
                pair[0].lon_deg,
                pair[1].lat_deg,
                pair[1].lon_deg,
            ));
        }
        Self {
            waypoints: wps,
            leg_bearings_deg: bearings,
            leg_lengths_nm: lengths,
            active_leg: 0,
            capture_radius_nm: WAYPOINT_CAPTURE_RADIUS_NM,
            route_complete: false,
        }
    }

    /// Observe one position. Deterministic pure function of (state, position).
    pub fn update(&mut self, lat_deg: f64, lon_deg: f64) -> RouteObservation {
        if self.leg_bearings_deg.is_empty() || self.waypoints.len() < 2 {
            return RouteObservation::default();
        }
        // Advance the active leg while the next waypoint is captured:
        // within the capture radius, or passed abeam (nearer to the next
        // waypoint than to the current one — handles jump-past positions
        // such as teleports and coarse scenario steps).
        if !self.route_complete {
            while self.active_leg + 1 < self.waypoints.len() {
                let next = &self.waypoints[self.active_leg + 1];
                let current = &self.waypoints[self.active_leg];
                let d_next = distance_nm(lat_deg, lon_deg, next.lat_deg, next.lon_deg);
                let d_current = distance_nm(lat_deg, lon_deg, current.lat_deg, current.lon_deg);
                if d_next <= self.capture_radius_nm || d_next < d_current {
                    self.active_leg += 1;
                } else {
                    break;
                }
            }
            if self.active_leg + 1 == self.waypoints.len() {
                self.route_complete = true;
            }
        }

        let dest = self.waypoints.last().unwrap();
        let destination_distance_nm =
            Some(distance_nm(lat_deg, lon_deg, dest.lat_deg, dest.lon_deg));

        if self.route_complete {
            return RouteObservation {
                active_leg: None,
                distance_to_waypoint_nm: None,
                cross_track_error_nm: None,
                distance_remaining_nm: Some(0.0),
                route_complete: true,
                destination_distance_nm,
            };
        }

        let leg = self.active_leg;
        let from = &self.waypoints[leg];
        let to = &self.waypoints[leg + 1];
        let distance_to_waypoint_nm = Some(distance_nm(lat_deg, lon_deg, to.lat_deg, to.lon_deg));

        // Cross-track error via standard great-circle cross-track formula:
        // xtk = asin(sin(δ13/R) · sin(θ13 − θ12)) · R, where 1=from, 2=to,
        // 3=aircraft. Sign: positive when the aircraft is RIGHT of the track
        // (θ13 > θ12, i.e. bearing to aircraft rotated clockwise from leg
        // bearing, for the northern-hemisphere-neutral spherical convention
        // used by the formula).
        let dist_from_nm = distance_nm(lat_deg, lon_deg, from.lat_deg, from.lon_deg);
        let bearing_to_ac = initial_bearing_deg(from.lat_deg, from.lon_deg, lat_deg, lon_deg);
        let leg_bearing = self.leg_bearings_deg[leg];
        let angular = (dist_from_nm / 3440.065).sin() // 3440.065 nm = 1 rad
            * (bearing_to_ac - leg_bearing).to_radians().sin();
        let cross_track_error_nm = Some(angular.asin() * 3440.065);

        // Remaining: distance to active waypoint + all legs after it.
        let remaining_legs: f64 = self.leg_lengths_nm[leg + 1..].iter().sum();
        let distance_remaining_nm = Some(distance_to_waypoint_nm.unwrap() + remaining_legs);

        RouteObservation {
            active_leg: Some(leg),
            distance_to_waypoint_nm,
            cross_track_error_nm,
            distance_remaining_nm,
            route_complete: false,
            destination_distance_nm,
        }
    }
}

/// DEVELOPMENT DEFAULT off-route configuration (Task 6 §20).
///
/// NOT an airline off-route deviation standard. Clearly named development
/// configuration: |cross-track| beyond `max_xtk_nm` for `sustain_samples`
/// consecutive updates raises a single development event.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OffRouteConfig {
    pub max_xtk_nm: f64,
    pub sustain_samples: u32,
}

impl Default for OffRouteConfig {
    fn default() -> Self {
        Self {
            max_xtk_nm: 5.0,
            sustain_samples: 3,
        }
    }
}

/// A development-level route deviation event (§20).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OffRouteEvent {
    /// First sample seq where the sustained deviation began.
    pub since_seq: u64,
    /// Peak |xtk| observed during the sustained deviation (nm).
    pub peak_xtk_nm: f64,
}

/// Deterministic off-route detector over [`RouteMonitor`] observations.
///
/// Unknown route (None cross-track) never produces events (§52).
#[derive(Debug, Clone)]
pub struct OffRouteDetector {
    config: OffRouteConfig,
    sustained_count: u32,
    since_seq: Option<u64>,
    peak_xtk_nm: f64,
    active: bool,
}

impl OffRouteDetector {
    pub fn new(config: OffRouteConfig) -> Self {
        Self {
            config,
            sustained_count: 0,
            since_seq: None,
            peak_xtk_nm: 0.0,
            active: false,
        }
    }

    /// Feed one observation. `seq` is the caller's sample sequence. Returns
    /// `Some(event)` exactly once per sustained deviation episode onset.
    pub fn update(&mut self, seq: u64, obs: &RouteObservation) -> Option<OffRouteEvent> {
        match obs.cross_track_error_nm {
            None => {
                // Unknown route or complete: reset evidence, no event.
                self.sustained_count = 0;
                self.since_seq = None;
                self.peak_xtk_nm = 0.0;
                self.active = false;
                None
            }
            Some(xtk) => {
                let deviating = xtk.abs() > self.config.max_xtk_nm;
                if deviating {
                    if self.sustained_count == 0 {
                        self.since_seq = Some(seq);
                        self.peak_xtk_nm = xtk.abs();
                    } else {
                        self.peak_xtk_nm = self.peak_xtk_nm.max(xtk.abs());
                    }
                    self.sustained_count += 1;
                    if self.sustained_count >= self.config.sustain_samples && !self.active {
                        self.active = true;
                        return Some(OffRouteEvent {
                            since_seq: self.since_seq.unwrap_or(seq),
                            peak_xtk_nm: self.peak_xtk_nm,
                        });
                    }
                } else {
                    self.sustained_count = 0;
                    self.since_seq = None;
                    self.peak_xtk_nm = 0.0;
                    self.active = false;
                }
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn route() -> RouteState {
        RouteState {
            source: RouteSource::Scenario,
            waypoints: vec![
                Waypoint {
                    id: "A".into(),
                    lat_deg: 0.0,
                    lon_deg: 0.0,
                },
                Waypoint {
                    id: "B".into(),
                    lat_deg: 0.0,
                    lon_deg: 1.0,
                },
                Waypoint {
                    id: "C".into(),
                    lat_deg: 0.0,
                    lon_deg: 2.0,
                },
            ],
        }
    }

    #[test]
    fn unknown_route_yields_empty_observation() {
        let mut m = RouteMonitor::new(&RouteState {
            source: RouteSource::Unknown,
            waypoints: vec![],
        });
        let obs = m.update(10.0, 20.0);
        assert_eq!(obs, RouteObservation::default());
        assert!(!obs.route_complete);
    }

    #[test]
    fn on_leg_has_near_zero_cross_track() {
        let mut m = RouteMonitor::new(&route());
        // Halfway between A(0,0) and B(0,1): xtk must be ~0.
        let obs = m.update(0.0, 0.5);
        let xtk = obs.cross_track_error_nm.unwrap();
        assert!(xtk.abs() < 1e-6, "xtk {xtk}");
        assert_eq!(obs.active_leg, Some(0));
        let remaining = obs.distance_remaining_nm.unwrap();
        assert!((remaining - 90.0).abs() < 1.0, "remaining {remaining}"); // 30nm to B + 60nm B->C
    }

    #[test]
    fn cross_track_sign_right_positive() {
        let mut m = RouteMonitor::new(&route());
        // Leg runs EAST along the equator. Facing east, RIGHT side = south.
        // 0.1 deg north of the leg: LEFT side -> negative.
        let obs = m.update(0.1, 0.5);
        let xtk = obs.cross_track_error_nm.unwrap();
        assert!(
            xtk < 0.0,
            "north of eastbound leg must be negative, got {xtk}"
        );
        // 0.1 deg south: RIGHT side -> positive.
        let obs = m.update(-0.1, 0.5);
        let xtk = obs.cross_track_error_nm.unwrap();
        assert!(
            xtk > 0.0,
            "south of eastbound leg must be positive, got {xtk}"
        );
    }

    #[test]
    fn waypoint_capture_advances_leg() {
        let mut m = RouteMonitor::new(&route());
        // At B: capture radius 2.5nm, B is within radius -> leg 1 active.
        let obs = m.update(0.0, 1.0);
        assert_eq!(obs.active_leg, Some(1));
        // Far past C: route complete.
        let obs = m.update(0.0, 5.0);
        assert!(obs.route_complete);
        assert_eq!(obs.distance_remaining_nm, Some(0.0));
    }

    #[test]
    fn destination_distance_always_present_when_route_known() {
        let mut m = RouteMonitor::new(&route());
        let obs = m.update(0.0, 0.5);
        assert!(obs.destination_distance_nm.unwrap() > 0.0);
    }

    #[test]
    fn off_route_requires_sustained_deviation() {
        let mut m = RouteMonitor::new(&route());
        let mut det = OffRouteDetector::new(OffRouteConfig::default());
        let mut event_seq = None;
        // 6nm right of leg for 3 consecutive samples -> event on 3rd.
        for seq in 0..3u64 {
            m = RouteMonitor::new(&route()); // fresh monitor: no capture side effects
            let obs = m.update(0.15, 0.5); // ~9nm north of leg
            if let Some(ev) = det.update(seq, &obs) {
                event_seq = Some((seq, ev));
                break;
            }
        }
        let (seq, ev) = event_seq.expect("sustained deviation must raise event");
        assert_eq!(seq, 2);
        assert_eq!(ev.since_seq, 0);
        assert!(ev.peak_xtk_nm > 5.0);
    }

    #[test]
    fn brief_deviation_does_not_raise_event() {
        let mut m = RouteMonitor::new(&route());
        let mut det = OffRouteDetector::new(OffRouteConfig::default());
        let obs_far = m.update(0.15, 0.5);
        assert!(det.update(0, &obs_far).is_none());
        assert!(det.update(1, &obs_far).is_none());
        // Back on leg before sustain threshold.
        let obs_on = m.update(0.0, 0.5);
        assert!(det.update(2, &obs_on).is_none());
        // Deviation again: counter restarted, no event yet.
        assert!(det.update(3, &obs_far).is_none());
        assert!(det.update(4, &obs_far).is_none());
        assert!(det.update(5, &obs_far).is_some());
    }

    #[test]
    fn unknown_cross_track_never_events() {
        let mut det = OffRouteDetector::new(OffRouteConfig::default());
        let empty = RouteObservation::default(); // xtk None
        for seq in 0..10u64 {
            assert!(det.update(seq, &empty).is_none());
        }
    }
}
