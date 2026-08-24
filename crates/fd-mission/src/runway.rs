//! Runway-relative awareness (Task 6 §16).
//!
//! Derived values (centerline offset, threshold distance, remaining runway)
//! exist ONLY when concrete runway geometry is supplied. An unknown runway
//! yields `None` — never fabricated precision (Task 6 §52).
//!
//! Geometry source: OpenAIRAC world store runway records (threshold
//! coordinates per runway end + true heading + length). This module holds
//! only the plain structural types so fd-fdm can consume them through a
//! trait without a dependency on this crate (see fd-fdm `qol::RunwayGeometry`).

use serde::{Deserialize, Serialize};

/// One runway end (threshold) with surveyed geometry.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RunwayEnd {
    /// End designator, e.g. `"06C"` or `"24C"`.
    pub ident: String,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub elevation_ft: f64,
    /// TRUE heading of the runway direction FROM this end (degrees).
    pub true_heading_deg: f64,
}

/// A physical runway with both surveyed ends. `ends[0]` is the
/// low-end/`le_` end, `ends[1]` the high-end/`he_` end (OpenAIRAC order).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Runway {
    pub airport_icao: String,
    /// Low-end designator (e.g. `"06C"`).
    pub le_ident: String,
    /// High-end designator (e.g. `"24C"`).
    pub he_ident: String,
    pub length_ft: f64,
    /// `[le_end, he_end]`.
    pub ends: [RunwayEnd; 2],
}

/// The runway selected for landing, with the selection evidence.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RunwayContext {
    pub runway: Runway,
    /// Index into `runway.ends` of the landing end.
    pub landing_end: usize,
    /// Deterministic reason the end was selected (e.g.
    /// `"operator_declared"`, `"headwind_max:06C@8kt"`).
    pub evidence: String,
}

impl RunwayContext {
    /// The landing-end threshold geometry.
    pub fn landing_end(&self) -> &RunwayEnd {
        &self.runway.ends[self.landing_end]
    }

    /// Signed perpendicular offset from the landing runway centerline at the
    /// given position, in meters. Positive = RIGHT of centerline (viewed
    /// from the landing threshold along the landing heading).
    ///
    /// Planar approximation in a local tangent frame at the threshold —
    /// valid for the sub-10nm distances landing analysis operates on.
    pub fn centerline_offset_m(&self, lat_deg: f64, lon_deg: f64) -> Option<f64> {
        let end = self.landing_end();
        Some(local_frame_offset_m(
            end.lat_deg,
            end.lon_deg,
            end.true_heading_deg,
            lat_deg,
            lon_deg,
        ))
    }

    /// Distance from the given position to the landing threshold, meters
    /// (great-circle).
    pub fn distance_to_threshold_m(&self, lat_deg: f64, lon_deg: f64) -> Option<f64> {
        let end = self.landing_end();
        let nm = fd_core::geo::distance_nm(lat_deg, lon_deg, end.lat_deg, end.lon_deg);
        Some(nm * 1852.0)
    }

    /// Remaining runway from the given position: projection of the position
    /// onto the runway axis measured from the threshold toward the far end,
    /// clamped to [0, length]. Meters.
    pub fn remaining_runway_m(&self, lat_deg: f64, lon_deg: f64) -> Option<f64> {
        let end = self.landing_end();
        let (along_m, _) = local_frame_components_m(
            end.lat_deg,
            end.lon_deg,
            end.true_heading_deg,
            lat_deg,
            lon_deg,
        );
        let length_m = self.runway.length_ft * 0.3048;
        Some((length_m - along_m).clamp(0.0, length_m))
    }

    /// Signed heading difference between the aircraft heading and the
    /// landing runway heading, in `[-180, 180)` degrees. Positive = aircraft
    /// heading right of runway heading.
    pub fn heading_diff_deg(&self, heading_true_deg: f64) -> f64 {
        let end = self.landing_end();
        let diff = (heading_true_deg - end.true_heading_deg).rem_euclid(360.0);
        if diff >= 180.0 { diff - 360.0 } else { diff }
    }
}

/// Local tangent frame at an origin: returns (along, cross) components in
/// meters of the offset to a position. `along` = projection onto the
/// heading direction, `cross` = positive right of heading.
fn local_frame_components_m(
    origin_lat: f64,
    origin_lon: f64,
    heading_deg: f64,
    lat_deg: f64,
    lon_deg: f64,
) -> (f64, f64) {
    const M_PER_DEG_LAT: f64 = 111_320.0;
    let m_per_deg_lon = M_PER_DEG_LAT * origin_lat.to_radians().cos();
    let north_m = (lat_deg - origin_lat) * M_PER_DEG_LAT;
    let east_m = (lon_deg - origin_lon) * m_per_deg_lon;
    let (sin, cos) = heading_deg.to_radians().sin_cos();
    // along = north*cos + east*sin; cross(right) = -north*sin + east*cos
    (north_m * cos + east_m * sin, -north_m * sin + east_m * cos)
}

fn local_frame_offset_m(
    origin_lat: f64,
    origin_lon: f64,
    heading_deg: f64,
    lat_deg: f64,
    lon_deg: f64,
) -> f64 {
    let (_, cross) =
        local_frame_components_m(origin_lat, origin_lon, heading_deg, lat_deg, lon_deg);
    cross
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Straight runway pointing EAST (090 true): threshold at origin.
    /// 1 deg latitude = 111,320 m; 1 deg longitude at lat 0 = 111,320 m.
    fn east_runway() -> RunwayContext {
        RunwayContext {
            runway: Runway {
                airport_icao: "TEST".into(),
                le_ident: "09".into(),
                he_ident: "27".into(),
                length_ft: 6561.68, // ~2000 m
                ends: [
                    RunwayEnd {
                        ident: "09".into(),
                        lat_deg: 0.0,
                        lon_deg: 0.0,
                        elevation_ft: 100.0,
                        true_heading_deg: 90.0,
                    },
                    RunwayEnd {
                        ident: "27".into(),
                        lat_deg: 0.0,
                        lon_deg: 2000.0 / 111_320.0,
                        elevation_ft: 100.0,
                        true_heading_deg: 270.0,
                    },
                ],
            },
            landing_end: 0,
            evidence: "test".into(),
        }
    }

    #[test]
    fn centerline_offset_sign_convention() {
        let ctx = east_runway();
        // Facing east (090), RIGHT side = south.
        // 500 m SOUTH of threshold: right -> positive.
        let north = 500.0 / 111_320.0;
        let xtk = ctx.centerline_offset_m(-north, 0.0).unwrap();
        assert!((xtk - 500.0).abs() < 1.0, "south offset {xtk}");
        // 500 m NORTH: left -> negative.
        let xtk = ctx.centerline_offset_m(north, 0.0).unwrap();
        assert!((xtk + 500.0).abs() < 1.0, "north offset {xtk}");
        // On centerline: ~0.
        let xtk = ctx.centerline_offset_m(0.0, 0.005).unwrap();
        assert!(xtk.abs() < 0.5, "centerline {xtk}");
    }

    #[test]
    fn threshold_distance_great_circle() {
        let ctx = east_runway();
        let d = ctx.distance_to_threshold_m(0.0, 0.0).unwrap();
        assert!(d.abs() < 1.0);
        let d = ctx.distance_to_threshold_m(0.0, 0.01).unwrap();
        assert!((d - 1113.2).abs() < 5.0, "d {d}");
    }

    #[test]
    fn remaining_runway_projection_and_clamp() {
        let ctx = east_runway();
        // 500 m along the runway: remaining ~1500 m.
        let along = 500.0 / 111_320.0;
        let rem = ctx.remaining_runway_m(0.0, along).unwrap();
        assert!((rem - 1500.0).abs() < 5.0, "rem {rem}");
        // Beyond the far end: clamped to 0.
        let rem = ctx.remaining_runway_m(0.0, 0.05).unwrap();
        assert_eq!(rem, 0.0);
        // Before the threshold: clamped to full length.
        let rem = ctx.remaining_runway_m(0.0, -0.01).unwrap();
        assert!((rem - 2000.0).abs() < 5.0, "rem {rem}");
    }

    #[test]
    fn heading_diff_wraps_signed() {
        let ctx = east_runway();
        assert!((ctx.heading_diff_deg(95.0) - 5.0).abs() < 1e-9);
        assert!((ctx.heading_diff_deg(85.0) + 5.0).abs() < 1e-9);
        // 350 deg vs 090: -100 (left), not +260.
        assert!((ctx.heading_diff_deg(350.0) + 100.0).abs() < 1e-9);
    }

    #[test]
    fn reciprocal_end_selection_changes_frame() {
        let mut ctx = east_runway();
        ctx.landing_end = 1; // land on 27 (westbound)
        // Facing west, RIGHT side = north -> north offset positive.
        let north = 500.0 / 111_320.0;
        let xtk = ctx.centerline_offset_m(north, 0.0).unwrap();
        assert!(xtk > 0.0, "north of westbound must be positive, got {xtk}");
    }
}
