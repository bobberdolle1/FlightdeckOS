//! Minimal great-circle geometry helpers (spherical earth).
//!
//! Development-grade navigation math shared by mission/route code. Good to
//! a fraction of a nm for test scenarios; not survey-grade.

/// Earth radius in nautical miles (mean spherical).
pub const EARTH_RADIUS_NM: f64 = 3440.065;

/// Great-circle distance between two points (nm).
pub fn distance_nm(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (lat1, lon1, lat2, lon2) = (
        lat1.to_radians(),
        lon1.to_radians(),
        lat2.to_radians(),
        lon2.to_radians(),
    );
    let dlat = lat2 - lat1;
    let dlon = lon2 - lon1;
    let a = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    EARTH_RADIUS_NM * 2.0 * a.sqrt().atan2((1.0 - a).sqrt())
}

/// Initial true bearing from point 1 to point 2 (degrees, 0..=360).
pub fn initial_bearing_deg(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (lat1, lon1, lat2, lon2) = (
        lat1.to_radians(),
        lon1.to_radians(),
        lat2.to_radians(),
        lon2.to_radians(),
    );
    let dlon = lon2 - lon1;
    let y = dlon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
    y.atan2(x).to_degrees().rem_euclid(360.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_distance_for_same_point() {
        assert!(distance_nm(55.0, 37.0, 55.0, 37.0).abs() < 1e-9);
    }

    #[test]
    fn bearing_north_is_zero() {
        let b = initial_bearing_deg(55.0, 30.0, 56.0, 30.0);
        assert!(b.abs() < 0.1, "bearing {b}");
    }
}
