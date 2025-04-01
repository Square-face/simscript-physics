#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
#[cfg(feature = "approx")]
use derives::Approx;

use glam::DVec3 as Vec3;
use overload::overload;
use std::{iter::Sum, ops, time::Duration};

use crate::{linear_trait::Vec3Wrap, transform::Translation};

use super::{AngVel, Velocity};

/// Linear velocity in 3D space.
///
/// This struct wraps a [Vec3] to provide a strongly typed representation of linear velocity,
/// making operations and transformations explicit.
#[cfg_attr(feature = "approx", derive(Approx))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct LinVel(pub Vec3);

impl Vec3Wrap for LinVel {
    const ZERO: Self = Self(Vec3::ZERO);
    const ONE: Self = Self(Vec3::ONE);

    const X: Self = Self(Vec3::X);
    const Y: Self = Self(Vec3::Y);
    const Z: Self = Self(Vec3::Z);

    const NEG_X: Self = Self(Vec3::NEG_X);
    const NEG_Y: Self = Self(Vec3::NEG_Y);
    const NEG_Z: Self = Self(Vec3::NEG_Z);

    #[inline]
    #[must_use]
    fn from_vec3(v: Vec3) -> Self {
        Self(v)
    }
}

impl LinVel {
    /// Scales the velocity by a time duration in seconds, returning a [Translation].
    #[inline]
    #[must_use]
    pub fn mul_secs(&self, rhs: f64) -> Translation {
        Translation::from_vec3(self.0 * rhs)
    }

    /// Scales the velocity by a [Duration] returning a [Translation].
    ///
    /// Note: this function uses [LinVel::mul_secs] internally, if performance is of the essence,
    /// it might be a good idea to use it directly to avoid unnecessary [Duration::as_secs_f64]
    /// calls
    #[inline]
    #[must_use]
    pub fn mul_dur(&self, rhs: &Duration) -> Translation {
        self.mul_secs(rhs.as_secs_f64())
    }

    /// Converts [LinVel] into a [Velocity] with zero angular velocity.
    #[inline]
    #[must_use]
    pub const fn to_vel(self) -> Velocity {
        Velocity::new(self, AngVel::ZERO)
    }

    /// Creates a [Velocity] from [LinVel] with a specified angular velocity.
    #[inline]
    #[must_use]
    pub const fn with_angular(self, ang: AngVel) -> Velocity {
        Velocity::new(self, ang)
    }
}

impl From<Vec3> for LinVel {
    #[inline]
    #[must_use]
    fn from(value: Vec3) -> Self {
        Self::from_vec3(value)
    }
}

impl From<LinVel> for Vec3 {
    #[inline]
    #[must_use]
    fn from(value: LinVel) -> Self {
        value.0
    }
}

impl From<Velocity> for LinVel {
    #[inline]
    #[must_use]
    fn from(value: Velocity) -> Self {
        value.linear
    }
}

impl Sum for LinVel {
    #[inline]
    #[must_use]
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::ZERO, |a, b| a + b)
    }
}

overload!((a: ?LinVel) + (b: ?LinVel) -> LinVel{ LinVel( a.0 + b.0 ) });
overload!((a: ?LinVel) - (b: ?LinVel) -> LinVel{ LinVel( a.0 - b.0 ) });
overload!((a: &mut LinVel) += (b: ?LinVel) { a.0 += b.0 });
overload!((a: &mut LinVel) -= (b: ?LinVel) { a.0 -= b.0 });

overload!((a: ?LinVel) * (b: f64) -> LinVel{ LinVel( a.0 * b ) });
overload!((a: ?LinVel) / (b: f64) -> LinVel{ LinVel( a.0 / b ) });
overload!((a: &mut LinVel) *= (b: f64) { a.0 *= b });
overload!((a: &mut LinVel) /= (b: f64) { a.0 /= b });

overload!((a: ?LinVel) * (b: Duration) -> Translation{ a.mul_dur(&b) });
overload!((a: ?LinVel) * (b: &Duration) -> Translation{ a.mul_dur(b) });

overload!(-(a: ?LinVel) -> LinVel{ LinVel( -a.0 ) });

#[cfg(test)]
mod arithmetic {
    use super::*;
    use approx::assert_ulps_eq;

    #[cfg(test)]
    mod same_type {
        use super::*;

        #[test]
        fn add() {
            let v1 = LinVel::splat(2.0);
            let v2 = LinVel::with_x(-0.5);

            assert_ulps_eq!(v1 + v2, LinVel::new(1.5, 2., 2.));
            assert_ulps_eq!(v2 + v1, LinVel::new(1.5, 2., 2.));
        }

        #[test]
        fn sub() {
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
    mod scalar {
        use super::*;

        #[test]
        fn mul() {
            let v1 = LinVel::splat(42.);
            let v2 = LinVel::with_y(0.1);

            assert_ulps_eq!(v1 * 2., LinVel::splat(84.));
            assert_ulps_eq!(v2 * 10., LinVel::with_y(1.));
        }

        #[test]
        fn div() {
            let v1 = LinVel::splat(63.);
            let v2 = LinVel::with_y(1.0);

            assert_ulps_eq!(v1 / 3., LinVel::splat(21.));
            assert_ulps_eq!(v2 / 10., LinVel::with_y(0.1));
        }
    }

    #[cfg(test)]
    mod time {
        use super::*;

        #[test]
        fn mul() {
            let v = LinVel::new(83.7, 47.1, 139.2);
            let t = Duration::from_secs_f64(5.);
            assert_ulps_eq!(v * t, Translation::new(418.5, 235.5, 696.));
        }
    }
}

#[cfg(test)]
mod traits {
    use super::*;

    #[cfg(test)]
    mod from {
        use approx::assert_ulps_eq;

        use super::*;

        #[test]
        fn vec3_into_linvel() {
            let v1 = Vec3::new(33.59, 89.08, 46.54);
            let v2 = Vec3::new(44.45, 30.98, 65.68);
            let v3 = Vec3::new(80.58, 35.28, 83.22);

            let l1: LinVel = v1.into();
            let l2: LinVel = v2.into();
            let l3: LinVel = v3.into();

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }

        #[test]
        fn linvel_into_vec3() {
            let l1 = LinVel::new(15.98, 43.88, 47.36);
            let l2 = LinVel::new(75.68, 91.02, 12.66);
            let l3 = LinVel::new(10.82, 54.16, 52.37);

            let v1: Vec3 = l1.into();
            let v2: Vec3 = l2.into();
            let v3: Vec3 = l3.into();

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }

        #[test]
        fn vec3_from_linvel() {
            let v1 = Vec3::new(78.40, 50.84, 98.58);
            let v2 = Vec3::new(18.03, 10.49, 45.86);
            let v3 = Vec3::new(25.11, 12.18, 27.54);

            let l1 = LinVel::from(v1);
            let l2 = LinVel::from(v2);
            let l3 = LinVel::from(v3);

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }

        #[test]
        fn linvel_from_vec3() {
            let l1 = LinVel::new(66.96, 23.12, 49.18);
            let l2 = LinVel::new(16.67, 40.61, 19.60);
            let l3 = LinVel::new(61.88, 12.68, 26.26);

            let v1 = Vec3::from(l1);
            let v2 = Vec3::from(l2);
            let v3 = Vec3::from(l3);

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }
    }
}
