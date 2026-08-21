//! Waypoints, legs and the minimal route follower.

use serde::{Deserialize, Serialize};

/// A named geographic waypoint.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Waypoint {
    pub id: String,
    pub lat_deg: f64,
    pub lon_deg: f64,
}

/// One route leg between consecutive waypoints.
#[derive(Debug, Clone)]
pub struct RouteLeg {
    pub from: Waypoint,
    pub to: Waypoint,
    /// Great-circle distance (nm).
    pub distance_nm: f64,
    /// Initial true bearing (deg).
    pub bearing_deg: f64,
}

/// Minimal deterministic route follower.
///
/// Tracks the current leg; commands a target heading toward the active
/// waypoint and advances when within the capture radius. Development
/// navigation for the test route — not an FMS.
#[derive(Debug)]
pub struct RouteFollower {
    waypoints: Vec<Waypoint>,
    current_leg: usize,
    pub capture_radius_nm: f64,
}

impl RouteFollower {
    /// Build from waypoints; computes leg distances/bearings up front
    /// (deterministic).
    pub fn new(waypoints: Vec<Waypoint>, capture_radius_nm: f64) -> Self {
        assert!(waypoints.len() >= 2, "route needs at least 2 waypoints");
        Self {
            waypoints,
            current_leg: 0,
            capture_radius_nm,
        }
    }

    pub fn origin(&self) -> &Waypoint {
        &self.waypoints[0]
    }

    pub fn destination(&self) -> &Waypoint {
        &self.waypoints[self.waypoints.len() - 1]
    }

    pub fn current_waypoint(&self) -> &Waypoint {
        &self.waypoints[(self.current_leg + 1).min(self.waypoints.len() - 1)]
    }

    pub fn remaining_legs(&self) -> usize {
        self.waypoints.len() - 1 - self.current_leg
    }

    /// Active-leg bearing/distance from the CURRENT position toward the
    /// active waypoint.
    pub fn guidance(&self, lat: f64, lon: f64) -> (f64, f64) {
        let target = self.current_waypoint();
        (
            fd_core::geo::initial_bearing_deg(lat, lon, target.lat_deg, target.lon_deg),
            fd_core::geo::distance_nm(lat, lon, target.lat_deg, target.lon_deg),
        )
    }

    /// Advance the active leg when the waypoint is captured. Returns true if
    /// the leg advanced.
    pub fn maybe_advance(&mut self, lat: f64, lon: f64) -> bool {
        let target = self.current_waypoint();
        let dist = fd_core::geo::distance_nm(lat, lon, target.lat_deg, target.lon_deg);
        if dist <= self.capture_radius_nm && self.current_leg + 2 < self.waypoints.len() {
            self.current_leg += 1;
            return true;
        }
        false
    }

    /// Distance to destination from the current position (nm).
    pub fn distance_to_destination(&self, lat: f64, lon: f64) -> f64 {
        let dest = self.destination();
        fd_core::geo::distance_nm(lat, lon, dest.lat_deg, dest.lon_deg)
    }
}
