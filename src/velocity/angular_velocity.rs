#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "approx")]
use {
    approx::{AbsDiffEq, RelativeEq, UlpsEq},
    approx_derive::Approx,
};

use super::{LinVel, Velocity};
use crate::transform::Rotation;
use glam::{DQuat as Quat, DVec3 as Vec3};
use overload::overload;
use std::{iter::Sum, ops, time::Duration};

/// Angular velocity in 3D space.
///
/// This struct wraps a [Vec3] to provide a strongly typed representation of angular velocity,
/// making operations and transformations explicit.
#[cfg_attr(feature = "approx", derive(Approx))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct AngVel(pub Vec3);

impl AngVel {
    /// A zero angular velocity vector.
    pub const ZERO: Self = Self::splat(0.);

    /// Angular velocity of magnitude one in all directions.
    pub const ONE: Self = Self::splat(1.);

    /// Unit angular velocity in the positive X direction.
    pub const X: Self = Self::with_x(1.);
    /// Unit angular velocity in the positive Y direction.
    pub const Y: Self = Self::with_y(1.);
    /// Unit angular velocity in the positive Z direction.
    pub const Z: Self = Self::with_z(1.);

    /// Unit angular velocity in the negative X direction.
    pub const NEG_X: Self = Self::with_x(-1.);
    /// Unit angular velocity in the negative Y direction.
    pub const NEG_Y: Self = Self::with_y(-1.);
    /// Unit angular velocity in the negative Z direction.
    pub const NEG_Z: Self = Self::with_z(-1.);

    /// Creates a new [AngVel] with the specified `x`, `y`, and `z` components.
    #[inline]
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vec3::new(x, y, z))
    }

    /// Creates an [AngVel] from an existing [Vec3].
    #[inline]
    #[must_use]
    pub const fn from_vec3(v: Vec3) -> Self {
        Self(v)
    }

    /// Creates an [AngVel] where all components are set to `v`.
    #[inline]
    #[must_use]
    pub const fn splat(v: f64) -> Self {
        Self::new(v, v, v)
    }

    /// Creates an [AngVel] with only the X component set.
    #[inline]
    #[must_use]
    pub const fn with_x(x: f64) -> Self {
        Self::new(x, 0., 0.)
    }

    /// Creates an [AngVel] with only the Y component set.
    #[inline]
    #[must_use]
    pub const fn with_y(y: f64) -> Self {
        Self::new(0., y, 0.)
    }

    /// Creates an [AngVel] with only the Z component set.
    #[inline]
    #[must_use]
    pub const fn with_z(z: f64) -> Self {
        Self::new(0., 0., z)
    }
}

impl AngVel {
    /// Scales the velocity by a time duration in seconds, returning a [Rotation].
    #[inline]
    #[must_use]
    pub fn mul_secs(&self, rhs: f64) -> Rotation {
        let delta = self.0 * rhs;
        Rotation::new(Quat::from_scaled_axis(delta))
    }

    /// Scales the velocity by a [Duration] returning a [Rotation].
    ///
    /// Note: this function uses [AngVel::mul_secs] internally, if performance is of the essence,
    /// it might be a good idea to use it directly to avoid unnecessary [Duration::as_secs_f64]
    /// calls
    #[inline]
    #[must_use]
    pub fn mul_dur(&self, rhs: &Duration) -> Rotation {
        self.mul_secs(rhs.as_secs_f64())
    }

    /// Converts [AngVel] into a [Velocity] with zero angular velocity.
    #[inline]
    #[must_use]
    pub const fn to_vel(self) -> Velocity {
        Velocity::new(LinVel::ZERO, self)
    }

    /// Creates a [Velocity] from [AngVel] with a specified angular velocity.
    #[inline]
    #[must_use]
    pub const fn with_linear(self, lin: LinVel) -> Velocity {
        Velocity::new(lin, self)
    }
}

impl From<AngVel> for Vec3 {
    #[inline]
    #[must_use]
    fn from(value: AngVel) -> Self {
        value.0
    }
}
impl From<Vec3> for AngVel {
    #[inline]
    #[must_use]
    fn from(value: Vec3) -> Self {
        Self::from_vec3(value)
    }
}

impl From<Velocity> for AngVel {
    #[inline]
    #[must_use]
    fn from(value: Velocity) -> Self {
        value.angular
    }
}

impl Sum for AngVel {
    #[inline]
    #[must_use]
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::ZERO, |a, b| a + b)
    }
}

overload!((a: ?AngVel) + (b: ?AngVel) -> AngVel{ AngVel( a.0 + b.0 ) });
overload!((a: ?AngVel) - (b: ?AngVel) -> AngVel{ AngVel( a.0 - b.0 ) });
overload!((a: &mut AngVel) += (b: ?AngVel) { a.0 += b.0 });
overload!((a: &mut AngVel) -= (b: ?AngVel) { a.0 -= b.0 });

overload!((a: ?AngVel) * (b: Duration) -> Rotation{ a.mul_dur(&b) });
overload!((a: ?AngVel) * (b: &Duration) -> Rotation{ a.mul_dur(b) });

overload!((a: ?AngVel) * (b: f64) -> AngVel{ AngVel( a.0 * b ) });
overload!((a: ?AngVel) / (b: f64) -> AngVel{ AngVel( a.0 / b ) });
overload!((a: &mut AngVel) *= (b: f64) { a.0 *= b });
overload!((a: &mut AngVel) /= (b: f64) { a.0 /= b });

overload!(-(a: ?AngVel) -> AngVel{ AngVel( -a.0 ) });

#[cfg(test)]
mod constructors {
    use approx::assert_ulps_eq;

    use super::*;

    #[test]
    fn new_splat_fromvec3() {
        assert_ulps_eq!(
            AngVel::new(64.54, 77.86, 2.77).0,
            Vec3::new(64.54, 77.86, 2.77)
        );
        assert_ulps_eq!(AngVel::splat(61.07).0, Vec3::new(61.07, 61.07, 61.07));

        let v = Vec3::new(12.57, 22.66, 58.90);
        assert_ulps_eq!(AngVel::from_vec3(v).0, v);
    }

    #[test]
    fn with() {
        assert_ulps_eq!(AngVel::with_x(66.60).0, Vec3::new(66.60, 0., 0.));
        assert_ulps_eq!(AngVel::with_y(76.75).0, Vec3::new(0., 76.75, 0.));
        assert_ulps_eq!(AngVel::with_z(21.58).0, Vec3::new(0., 0., 21.58));
    }

    #[test]
    fn consts() {
        assert_ulps_eq!(AngVel::ZERO, AngVel::splat(0.));

        assert_ulps_eq!(AngVel::X, AngVel::with_x(1.));
        assert_ulps_eq!(AngVel::Y, AngVel::with_y(1.));
        assert_ulps_eq!(AngVel::Z, AngVel::with_z(1.));

        assert_ulps_eq!(AngVel::NEG_X, AngVel::with_x(-1.));
        assert_ulps_eq!(AngVel::NEG_Y, AngVel::with_y(-1.));
        assert_ulps_eq!(AngVel::NEG_Z, AngVel::with_z(-1.));
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
            let av1 = AngVel::new(1.0, 2.0, 3.0);
            let av2 = AngVel::new(4.0, 5.0, 6.0);
            let result = av1 + av2;
            assert_ulps_eq!(result, AngVel::new(5.0, 7.0, 9.0));
        }

        #[test]
        fn sub() {
            let av1 = AngVel::new(5.0, 7.0, 9.0);
            let av2 = AngVel::new(1.0, 2.0, 3.0);
            let result = av1 - av2;
            assert_ulps_eq!(result, AngVel::new(4.0, 5.0, 6.0));
        }

        #[test]
        fn neg() {
            let av = AngVel::new(33.50, -88.84, 75.72);
            let result = -av;
            assert_ulps_eq!(result, AngVel::new(-33.50, 88.84, -75.72));
        }
    }

    #[cfg(test)]
    mod scalar {
        use super::*;

        #[test]
        fn mul() {
            let av = AngVel::new(1.0, -2.0, 3.0);
            assert_ulps_eq!(av * 2.0, AngVel::new(2.0, -4.0, 6.0));
        }

        #[test]
        fn div() {
            let av = AngVel::new(9.0, -3.0, 3.0);
            assert_ulps_eq!(av / 3.0, AngVel::new(3.0, -1.0, 1.0));
        }
    }

    #[cfg(test)]
    mod time {
        use super::*;

        #[test]
        fn mul() {
            let av = AngVel::new(84.48, 48.38, 25.55);
            let dur = Duration::from_secs_f64(1.6);
            assert_ulps_eq!(av * dur, av.mul_secs(1.6));
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

            let l1: AngVel = v1.into();
            let l2: AngVel = v2.into();
            let l3: AngVel = v3.into();

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }

        #[test]
        fn linvel_into_vec3() {
            let l1 = AngVel::new(15.98, 43.88, 47.36);
            let l2 = AngVel::new(75.68, 91.02, 12.66);
            let l3 = AngVel::new(10.82, 54.16, 52.37);

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

            let l1 = AngVel::from(v1);
            let l2 = AngVel::from(v2);
            let l3 = AngVel::from(v3);

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }

        #[test]
        fn linvel_from_vec3() {
            let l1 = AngVel::new(66.96, 23.12, 49.18);
            let l2 = AngVel::new(16.67, 40.61, 19.60);
            let l3 = AngVel::new(61.88, 12.68, 26.26);

            let v1 = Vec3::from(l1);
            let v2 = Vec3::from(l2);
            let v3 = Vec3::from(l3);

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }
    }
}

#[cfg(test)]
mod to_rotation {
    use std::f64::consts::PI;

    use super::*;
    use approx::assert_ulps_eq;

    #[test]
    fn no_time() {
        let av = AngVel::X;
        let dur = Duration::ZERO;
        let res = av * dur;
        let expected = Quat::IDENTITY;
        assert_ulps_eq!(res.0, expected);
    }

    #[test]
    fn single_axis() {
        let ax = AngVel::X;
        let ay = AngVel::Y;
        let az = AngVel::Z;

        let ex = Quat::from_rotation_x(1.);
        let ey = Quat::from_rotation_y(1.);
        let ez = Quat::from_rotation_z(1.);

        assert_ulps_eq!(ax.mul_secs(1.).0, ex);
        assert_ulps_eq!(ay.mul_secs(1.).0, ey);
        assert_ulps_eq!(az.mul_secs(1.).0, ez);

        assert_ulps_eq!(ax.mul_secs(2.).0, ex * ex);
        assert_ulps_eq!(ay.mul_secs(2.).0, ey * ey);
        assert_ulps_eq!(az.mul_secs(2.).0, ez * ez);
    }

    #[test]
    fn compound_axis() {
        let axy: AngVel = Vec3::new(1., 1., 0.).normalize().into();
        let ayz: AngVel = Vec3::new(0., 1., 1.).normalize().into();
        let azx: AngVel = Vec3::new(1., 0., 1.).normalize().into();

        let ex = Quat::from_rotation_x(PI / 2.);
        let ey = Quat::from_rotation_y(PI / 2.);
        let ez = Quat::from_rotation_z(PI / 2.);

        assert_ulps_eq!(axy.mul_secs(PI).0, ex * ey * ex);
        assert_ulps_eq!(ayz.mul_secs(PI).0, ey * ez * ey);
        assert_ulps_eq!(azx.mul_secs(PI).0, ez * ex * ez);
    }
}
