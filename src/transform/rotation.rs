#![allow(clippy::suspicious_op_assign_impl)]
#![allow(clippy::suspicious_arithmetic_impl)]

#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
#[cfg(feature = "approx")]
use approx_derive::Approx;

use glam::DQuat as Quat;
use overload::overload;
use std::ops;

use super::Transform;

/// Represents an object's orientation in 3D space, using a quaternion.
/// This struct wraps a `glam::DQuat` and provides common rotation operations.
#[cfg_attr(feature = "approx", derive(Approx))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Rotation(pub Quat);

impl Rotation {
    /// A constant representing no rotation (identity quaternion).
    pub const ZERO: Self = Self::new(Quat::IDENTITY);

    /// Creates a new `Rotation` from the given quaternion.
    #[inline]
    #[must_use]
    pub const fn new(ang: Quat) -> Self {
        Self(ang)
    }

    /// Creates a `Rotation` representing a rotation around the x-axis.
    ///
    /// # Arguments
    /// * `ang` - The angle in radians.
    #[inline]
    #[must_use]
    pub fn from_x(ang: f64) -> Self {
        Self(Quat::from_rotation_x(ang))
    }

    /// Creates a `Rotation` representing a rotation around the y-axis.
    ///
    /// # Arguments
    /// * `ang` - The angle in radians.
    #[inline]
    #[must_use]
    pub fn from_y(ang: f64) -> Self {
        Self(Quat::from_rotation_y(ang))
    }

    /// Creates a `Rotation` representing a rotation around the z-axis.
    ///
    /// # Arguments
    /// * `ang` - The angle in radians.
    #[inline]
    #[must_use]
    pub fn from_z(ang: f64) -> Self {
        Self(Quat::from_rotation_z(ang))
    }

    /// Returns a new `Rotation` with a normalized quaternion.
    #[inline]
    #[must_use]
    pub fn normalize(&self) -> Self {
        Self::new(self.0.normalize())
    }
}

/// Implements conversion from `Quat` to `Rotation`.
impl From<Quat> for Rotation {
    #[inline]
    #[must_use]
    fn from(value: Quat) -> Self {
        Self::new(value)
    }
}

/// Implements conversion from `Rotation` to `Quat`.
impl From<Rotation> for Quat {
    #[inline]
    #[must_use]
    fn from(value: Rotation) -> Self {
        value.0
    }
}

/// Implements conversion from `Transform` to `Rotation`.
impl From<Transform> for Rotation {
    #[inline]
    #[must_use]
    fn from(value: Transform) -> Self {
        value.rotation
    }
}

overload!((a: ?Rotation) + (b: ?Rotation) -> Rotation{ Rotation( b.0 * a.0 ) });
overload!((a: ?Rotation) - (b: ?Rotation) -> Rotation{ Rotation( b.0.inverse() * a.0) });
overload!((a: &mut Rotation) += (b: ?Rotation) { a.0 = b.0 * a.0 });
overload!((a: &mut Rotation) -= (b: ?Rotation) { a.0 = b.0.inverse() * a.0});

overload!(-(a: ?Rotation) -> Rotation{ Rotation(a.0.inverse()) });

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_ulps_eq;
    use color_eyre::Result;
    use glam::EulerRot;
    use rstest::rstest;
    use rstest_reuse::{apply, template};
    use std::{f64::consts::FRAC_PI_3, sync::Once};

    static INIT: Once = Once::new();

    #[ctor::ctor]
    fn setup() {
        INIT.call_once(|| {
            color_eyre::install();
        })
    }

    #[template]
    #[rstest]
    #[case(Quat::IDENTITY, Quat::IDENTITY)]
    #[case(Quat::from_rotation_x(-5.25), Quat::from_rotation_y(6.24))]
    #[case(Quat::from_rotation_z(-23.83), Quat::from_rotation_x(-53.67))]
    fn double_quat(#[case] q1: Quat, #[case] q2: Quat) {}

    #[template]
    #[rstest]
    #[case(Quat::IDENTITY)]
    #[case(Quat::from_rotation_x(-5.25))]
    #[case(Quat::from_rotation_z(-23.83))]
    #[case(Quat::from_rotation_z(FRAC_PI_3))]
    fn single_quat(#[case] q: Quat) {}

    #[template]
    #[rstest]
    #[case((0., 0., 0.))]
    #[case((0., 75.91, -73.07))]
    #[case((34.72, 0., -86.77))]
    #[case((-10.78, 85.32 ,0.))]
    #[case((-39.56, 70.77, -68.74))]
    #[case((93.69, 51.02, 47.78))]
    #[case((-19.39, 61.29, -88.92))]
    #[case((96.86, -28.67, 69.41))]
    fn xyz_cases(#[case] (x, y, z): (f64, f64, f64)) {}

    #[template]
    #[rstest]
    #[case((43.42,80.02,39.05,-75.06))]
    #[case((22.49,95.44,41.56,-86.73))]
    #[case((-97.71, -74.05, -81.30, 0.10))]
    #[case((60.18, 20.08, -9.72, 71.92))]
    fn xyzw_cases(#[case] (x, y, z): (f64, f64, f64)) {}

    #[rstest]
    #[case(Rotation::from_x(30.60), Quat::from_rotation_x(30.60))]
    #[case(Rotation::from_x(81.62), Quat::from_rotation_x(81.62))]
    #[case(Rotation::from_y(12.28), Quat::from_rotation_y(12.28))]
    #[case(Rotation::from_y(26.29), Quat::from_rotation_y(26.29))]
    #[case(Rotation::from_z(37.10), Quat::from_rotation_z(37.10))]
    #[case(Rotation::from_z(6.31), Quat::from_rotation_z(6.31))]
    fn contstructors(#[case] actual: Rotation, #[case] expected: Quat) {
        assert_ulps_eq!(actual.0, expected)
    }

    #[rstest]
    #[case(Rotation::ZERO, Quat::IDENTITY)]
    fn constants(#[case] actual: Rotation, #[case] expected: Quat) {
        assert_ulps_eq!(actual.0, expected)
    }

    #[apply(single_quat)]
    fn new(q: Quat) {
        let r = Rotation::new(q);
        assert_ulps_eq!(r.0, q);
    }

    #[cfg(test)]
    #[allow(clippy::eq_op)]
    mod arithmetic {
        use super::*;

        #[apply(single_quat)]
        fn neg(#[case] q: Quat) {
            let r = Rotation::new(q);
            assert_ulps_eq!((-r).0, q.inverse());
        }

        #[apply(double_quat)]
        fn add(q1: Quat, q2: Quat) {
            let r1 = Rotation::new(q1);
            let r2 = Rotation::new(q2);

            assert_ulps_eq!((r1 + r1).0, q1 * q1);
            assert_ulps_eq!((r2 + r2).0, q2 * q2);

            assert_ulps_eq!((r1 + r2).0, q2 * q1);
            assert_ulps_eq!((r2 + r1).0, q1 * q2);
        }

        #[apply(double_quat)]
        fn sub(q1: Quat, q2: Quat) {
            let r1 = Rotation::new(q1);
            let r2 = Rotation::new(q2);

            assert_ulps_eq!((r1 - r1).0, Quat::IDENTITY);
            assert_ulps_eq!((r2 - r2).0, Quat::IDENTITY);

            assert_ulps_eq!((r1 - r2).0, q2.inverse() * q1);

            assert_ulps_eq!((r2 - r1).0, q1.inverse() * q2);
        }

        #[apply(double_quat)]
        fn addassign(q1: Quat, q2: Quat) -> Result<()> {
            let mut a = Rotation::new(q1);
            let b = Rotation::new(q2);

            a += b;

            assert_ulps_eq!(a.0, q2 * q1);
            Ok(())
        }

        #[apply(double_quat)]
        fn subassign(q1: Quat, q2: Quat) -> Result<()> {
            let mut r1 = Rotation::new(q1);
            let r2 = Rotation::new(q2);

            r1 -= r2;

            assert_ulps_eq!(r1.0, q2.inverse() * q1);
            Ok(())
        }
    }

    #[cfg(test)]
    mod methods {
        use super::*;

        #[apply(xyzw_cases)]
        fn normalize(#[case] (x, y, z, w): (f64, f64, f64, f64)) -> Result<()> {
            let q = Quat::from_xyzw(x, y, z, w);
            let r = Rotation::new(q);

            assert_ulps_eq!(r.normalize().0, q.normalize());

            Ok(())
        }
    }

    #[cfg(test)]
    mod traits {
        use super::*;

        #[apply(xyz_cases)]
        fn quat_to_rotation(#[case] (x, y, z): (f64, f64, f64)) -> Result<()> {
            let q = Quat::from_euler(EulerRot::YXZ, x, y, z);
            let t = Rotation::new(q);

            let t_conv: Rotation = q.into();

            assert_ulps_eq!(Rotation::from(q), t);
            assert_ulps_eq!(t_conv.0, q);

            Ok(())
        }

        #[apply(xyz_cases)]
        fn rotation_to_quat(#[case] (x, y, z): (f64, f64, f64)) -> Result<()> {
            let q = Quat::from_euler(EulerRot::YXZ, x, y, z);
            let r = Rotation::new(q);

            let q_conv: Quat = r.into();

            assert_ulps_eq!(Quat::from(r), q);
            assert_ulps_eq!(q_conv, q);

            Ok(())
        }
    }
}
