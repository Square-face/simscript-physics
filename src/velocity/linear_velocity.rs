#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
#[cfg(feature = "approx")]
use approx_derive::Approx;

use glam::DVec3 as Vec3;
use overload::overload;
use std::{iter::Sum, ops, time::Duration};

use crate::transform::Translation;

use super::{AngVel, Velocity};

/// Linear velocity in 3D space.
///
/// This struct wraps a [`Vec3`] to provide a strongly typed representation of linear velocity,
/// making operations and transformations explicit.
#[cfg_attr(feature = "approx", derive(Approx))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct LinVel(pub Vec3);

impl LinVel {
    /// A zero linear velocity vector.
    pub const ZERO: Self = Self::splat(0.);

    /// Linear velocity of magnitude one in all directions.
    pub const ONE: Self = Self::splat(1.);

    /// Unit linear velocity in the positive X direction.
    pub const X: Self = Self::with_x(1.);
    /// Unit linear velocity in the positive Y direction.
    pub const Y: Self = Self::with_y(1.);
    /// Unit linear velocity in the positive Z direction.
    pub const Z: Self = Self::with_z(1.);

    /// Unit linear velocity in the negative X direction.
    pub const NEG_X: Self = Self::with_x(-1.);
    /// Unit linear velocity in the negative Y direction.
    pub const NEG_Y: Self = Self::with_y(-1.);
    /// Unit linear velocity in the negative Z direction.
    pub const NEG_Z: Self = Self::with_z(-1.);

    /// Creates a new [`LinVel`] with the specified `x`, `y`, and `z` components.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vec3::new(x, y, z))
    }

    /// Creates a [`LinVel`] from an existing [`Vec3`].
    pub const fn from_vec3(v: Vec3) -> Self {
        Self(v)
    }

    /// Creates a [`LinVel`] where all components are set to `v`.
    pub const fn splat(v: f64) -> Self {
        Self::new(v, v, v)
    }

    /// Creates a [`LinVel`] with only the X component set.
    pub const fn with_x(x: f64) -> Self {
        Self::new(x, 0., 0.)
    }

    /// Creates a [`LinVel`] with only the Y component set.
    pub const fn with_y(y: f64) -> Self {
        Self::new(0., y, 0.)
    }

    /// Creates a [`LinVel`] with only the Z component set.
    pub const fn with_z(z: f64) -> Self {
        Self::new(0., 0., z)
    }
}

impl LinVel {
    /// Scales the velocity by a time duration in seconds, returning a [`Translation`].
    pub fn mul_secs(&self, rhs: f64) -> Translation {
        Translation::from_vec3(self.0 * rhs)
    }

    /// Scales the velocity by a [`Duration`] returning a [`Translation`].
    ///
    /// Note: this function uses [`LinVel::mul_secs`] internally, if performance is of the essence,
    /// it might be a good idea to use it directly to avoid unnecessary [Duration::as_secs_f64]
    /// calls
    pub fn mul_dur(&self, rhs: &Duration) -> Translation {
        self.mul_secs(rhs.as_secs_f64())
    }

    /// Converts [`LinVel`] into a [`Velocity`] with zero angular velocity.
    pub const fn to_vel(self) -> Velocity {
        Velocity::new(self, AngVel::ZERO)
    }

    /// Creates a [`Velocity`] from [`LinVel`] with a specified angular velocity.
    pub const fn with_angular(self, ang: AngVel) -> Velocity {
        Velocity::new(self, ang)
    }
}

impl From<Vec3> for LinVel {
    fn from(value: Vec3) -> Self {
        Self::from_vec3(value)
    }
}

impl From<LinVel> for Vec3 {
    fn from(value: LinVel) -> Self {
        value.0
    }
}

impl From<Velocity> for LinVel {
    fn from(value: Velocity) -> Self {
        value.linear
    }
}

impl Sum for LinVel {
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
mod constructors {
    use super::*;
    use approx::assert_ulps_eq;

    #[test]
    fn new() {
        assert_ulps_eq!(LinVel::new(1., 2., 3.).0, Vec3::new(1., 2., 3.));
    }

    #[test]
    fn splat() {
        assert_ulps_eq!(LinVel::splat(2.).0, Vec3::new(2., 2., 2.));
    }

    #[test]
    fn with() {
        assert_ulps_eq!(LinVel::with_x(2.).0, Vec3::new(2., 0., 0.));
        assert_ulps_eq!(LinVel::with_y(22.).0, Vec3::new(0., 22., 0.));
        assert_ulps_eq!(LinVel::with_z(-5.).0, Vec3::new(0., 0., -5.));
    }

    #[test]
    fn consts() {
        assert_ulps_eq!(LinVel::ZERO.0, Vec3::new(0., 0., 0.));

        assert_ulps_eq!(LinVel::X.0, Vec3::new(1., 0., 0.));
        assert_ulps_eq!(LinVel::Y.0, Vec3::new(0., 1., 0.));
        assert_ulps_eq!(LinVel::Z.0, Vec3::new(0., 0., 1.));

        assert_ulps_eq!(LinVel::NEG_X.0, Vec3::new(-1., 0., 0.));
        assert_ulps_eq!(LinVel::NEG_Y.0, Vec3::new(0., -1., 0.));
        assert_ulps_eq!(LinVel::NEG_Z.0, Vec3::new(0., 0., -1.));
    }
}

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
