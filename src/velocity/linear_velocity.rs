use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DVec3 as Vec3;
use overload::overload;
use std::{ops, time::Duration};

extern crate approx;

use crate::transform::Translation;

#[derive(Debug, Clone, Copy, PartialEq, Approx)]
pub struct LinVel(pub Vec3);

impl LinVel {
    pub const ZERO: Self = Self(Vec3::ZERO);

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vec3::new(x, y, z))
    }

    pub const fn splat(v: f64) -> Self {
        Self(Vec3::splat(v))
    }

    pub const fn with_x(x: f64) -> Self {
        Self(Vec3::new(x, 0., 0.))
    }

    pub const fn with_y(y: f64) -> Self {
        Self(Vec3::new(0., y, 0.))
    }

    pub const fn with_z(z: f64) -> Self {
        Self(Vec3::new(0., 0., z))
    }
}

overload!((a: ?LinVel) + (b: ?LinVel) -> LinVel{ LinVel( a.0 + b.0 ) });
overload!((a: ?LinVel) - (b: ?LinVel) -> LinVel{ LinVel( a.0 - b.0 ) });
overload!((a: &mut LinVel) += (b: ?LinVel) { a.0 += b.0 });
overload!((a: &mut LinVel) -= (b: ?LinVel) { a.0 -= b.0 });

overload!((a: ?LinVel) * (b: f64) -> LinVel{ LinVel( a.0 * b ) });
overload!((a: &mut LinVel) *= (b: f64) { a.0 *= b });

overload!((a: ?LinVel) * (b: ?Duration) -> Translation{ Translation(a.0 * b.as_secs_f64()) });

overload!(-(a: ?LinVel) -> LinVel{ LinVel( -a.0 ) });

#[cfg(test)]
mod same_type_arithmetic {

    use approx::assert_ulps_eq;

    use super::LinVel;

    #[test]
    fn addition() {
        let v1 = LinVel::splat(2.0);
        let v2 = LinVel::with_x(-0.5);

        assert_ulps_eq!(v1 + v2, LinVel::new(1.5, 2., 2.));
        assert_ulps_eq!(v2 + v1, LinVel::new(1.5, 2., 2.));
    }

    #[test]
    fn subtraction() {
        let v1 = LinVel::splat(10.2);
        let v2 = LinVel::new(4.3, -12., 0.1);

        assert_ulps_eq!(v1 - v2, LinVel::new(5.9, 22.2, 10.1));
        assert_ulps_eq!(v2 - v1, LinVel::new(-5.9, -22.2, -10.1));
    }

    #[test]
    fn neg() {
        let v1 = LinVel::with_z(176.45);
        assert_ulps_eq!(-v1, LinVel::new(0., 0., -176.45));
    }
}

#[cfg(test)]
mod scalar_arithmetic {
    use super::LinVel;
    use approx::assert_ulps_eq;

    #[test]
    fn scale() {
        let v1 = LinVel::splat(42.);
        assert_ulps_eq!(v1 * 2., LinVel::splat(84.));

        let v2 = LinVel::with_y(0.1);
        assert_ulps_eq!(v2 * 10., LinVel::with_y(1.));
    }
}

#[cfg(test)]
mod unit_conversion {
    use super::*;
    use approx::assert_ulps_eq;

    #[test]
    fn to_translation() {
        let v = LinVel::new(83.7, 47.1, 139.2);
        let t = Duration::from_secs_f64(5.);
        assert_ulps_eq!(v * t, Translation::new(418.5, 235.5, 696.));
    }
}
