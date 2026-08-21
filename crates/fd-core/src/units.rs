//! Pragmatically typed physical units.
//!
//! Units are newtypes over `f64`. They exist to prevent naked `f64` from
//! flowing through state where the unit is safety-relevant (altitudes,
//! speeds, angles). This is deliberately NOT a full units framework: no
//! arithmetic trait soup, no dimension algebra. The inner value is public
//! for pragmatic access; the type system only guards against accidental
//! cross-unit assignment.

use serde::{Deserialize, Serialize};

macro_rules! unit_newtype {
    ($(#[$doc:meta])* $name:ident) => {
        $(#[$doc])*
        #[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(pub f64);

        impl $name {
            pub const fn new(v: f64) -> Self {
                Self(v)
            }

            pub const fn value(self) -> f64 {
                self.0
            }
        }
    };
}

unit_newtype!(
    /// Altitude above mean sea level, feet.
    AltitudeFt
);
unit_newtype!(
    /// Altitude above ground level, feet.
    AltitudeAglFt
);
unit_newtype!(
    /// Speed, knots.
    SpeedKt
);
unit_newtype!(
    /// Vertical speed, feet per minute.
    VerticalSpeedFpm
);
unit_newtype!(
    /// Angle, degrees.
    AngleDeg
);
unit_newtype!(
    /// Latitude, degrees (-90..=90).
    LatDeg
);
unit_newtype!(
    /// Longitude, degrees (-180..=180).
    LonDeg
);
unit_newtype!(
    /// Percentage, 0..=100.
    Percent
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serde_roundtrip_is_transparent_number() {
        let v: AltitudeFt = serde_json::from_str("1234.5").unwrap();
        assert_eq!(v, AltitudeFt::new(1234.5));
        let s = serde_json::to_string(&v).unwrap();
        assert_eq!(s, "1234.5");
    }

    #[test]
    fn typed_units_are_distinct() {
        let alt = AltitudeFt::new(1000.0);
        let spd = SpeedKt::new(1000.0);
        // Compile-time proof of distinctness: cannot assign one to the other.
        #[allow(clippy::no_effect)]
        fn takes_altitude(_v: AltitudeFt) {}
        takes_altitude(alt);
        assert_eq!(spd.value(), 1000.0);
    }
}
