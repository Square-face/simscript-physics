#![allow(clippy::suspicious_op_assign_impl)]
#![allow(clippy::suspicious_arithmetic_impl)]

use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DQuat as Quat;
use overload::overload;
use std::ops;

/// Represents an objects orientation in 3d space, represented as a quaternion
#[derive(Debug, Default, Clone, Copy, PartialEq, Approx)]
pub struct Rotation(pub Quat);

impl Rotation {
    pub const ZERO: Self = Self::new(Quat::IDENTITY);

    /// Create a new [Rotation] with the given starting rotation
    #[inline]
    #[must_use]
    pub const fn new(ang: Quat) -> Self {
        Self(ang)
    }

    /// A [Rotation] around the x axis
    #[inline]
    #[must_use]
    pub fn from_x(ang: f64) -> Self {
        Self(Quat::from_rotation_x(ang))
    }

    /// A [Rotation] around the y axis
    #[inline]
    #[must_use]
    pub fn from_y(ang: f64) -> Self {
        Self(Quat::from_rotation_y(ang))
    }

    /// A [Rotation] around the z axis
    #[inline]
    #[must_use]
    pub fn from_z(ang: f64) -> Self {
        Self(Quat::from_rotation_z(ang))
    }

    /// Returns a copy where the internal quaternion is normalized
    #[inline]
    #[must_use]
    pub fn normalize(&self) -> Self {
        Self::new(self.0.normalize())
    }

    /// Re normalizes the internal quaternion in place
    #[inline]
    pub fn renormalize(&mut self) {
        self.0 = self.0.normalize();
    }
}

impl From<Quat> for Rotation {
    #[inline]
    #[must_use]
    fn from(value: Quat) -> Self {
        Self::new(value)
    }
}

impl From<Rotation> for Quat {
    #[inline]
    #[must_use]
    fn from(value: Rotation) -> Quat {
        value.0
    }
}

overload!((a: ?Rotation) + (b: ?Rotation) -> Rotation{ Rotation( b.0 * a.0 ) });
overload!((a: ?Rotation) - (b: ?Rotation) -> Rotation{ Rotation( b.0.inverse() * a.0) });
overload!((a: &mut Rotation) += (b: ?Rotation) { a.0 = b.0 * a.0 });
overload!((a: &mut Rotation) -= (b: ?Rotation) { a.0 = b.0.inverse() * a.0});

overload!(-(a: ?Rotation) -> Rotation{ Rotation(a.0.inverse()) });

#[cfg(test)]
mod constructors {
    use super::*;
    use approx::assert_ulps_eq;
    use rstest::rstest;
    use rstest_reuse::{apply, template};
    use std::{
        f64::consts::{FRAC_PI_3, PI},
        sync::Once,
    };

    static INIT: Once = Once::new();

    #[ctor::ctor]
    fn setup() {
        INIT.call_once(|| {
            color_eyre::install();
        })
    }

    #[template]
    #[rstest]
    #[case(Quat::IDENTITY)]
    #[case(Quat::from_rotation_x(-5.25))]
    #[case(Quat::from_rotation_z(-23.83))]
    #[case(Quat::from_rotation_z(PI / 5.))]
    fn rotation_cases(#[case] q: Quat) {}

    #[template]
    #[rstest]
    #[case(0.)]
    #[case(PI)]
    #[case(FRAC_PI_3)]
    fn angle_cases(#[case] angle: f64) {}

    #[apply(rotation_cases)]
    fn new(#[case] q: Quat) {
        let r = Rotation::new(q);
        assert_ulps_eq!(r.0, q);
    }

    #[apply(angle_cases)]
    fn from_x(#[case] angle: f64) {
        let q = Quat::from_rotation_x(angle);
        let r = Rotation::from_x(angle);
        assert_ulps_eq!(r.0, q);
    }

    #[apply(angle_cases)]
    fn from_y(#[case] angle: f64) {
        let q = Quat::from_rotation_y(angle);
        let r = Rotation::from_y(angle);
        assert_ulps_eq!(r.0, q);
    }

    #[apply(angle_cases)]
    fn from_z(#[case] angle: f64) {
        let q = Quat::from_rotation_z(angle);
        let r = Rotation::from_z(angle);
        assert_ulps_eq!(r.0, q);
    }

    mod constants {
        use super::*;

        #[rstest]
        fn zero() {
            assert_ulps_eq!(Rotation::ZERO.0, Quat::IDENTITY);
        }
    }
}

#[cfg(test)]
#[allow(clippy::eq_op)]
mod arithmetic {
    use std::sync::Once;

    use super::*;
    use approx::assert_ulps_eq;
    use color_eyre::Result;
    use rstest::*;
    use rstest_reuse::{self, *};

    static INIT: Once = Once::new();

    #[ctor::ctor]
    fn setup() {
        INIT.call_once(|| {
            color_eyre::install();
        })
    }

    #[rstest]
    #[case(Quat::from_rotation_x(1.0), Quat::from_rotation_x(-1.0))]
    fn neg(#[case] q: Quat, #[case] expected: Quat) {
        let r = Rotation::new(q);
        assert_ulps_eq!((-r).0, expected);
    }

    #[template]
    #[rstest]
    #[case(Quat::IDENTITY, Quat::IDENTITY)]
    #[case(Quat::from_rotation_x(-5.25), Quat::from_rotation_y(6.24))]
    #[case(Quat::from_rotation_z(-23.83), Quat::from_rotation_x(-53.67))]
    fn rotation_cases(#[case] q1: Quat, #[case] q2: Quat) {}

    #[apply(rotation_cases)]
    fn add(q1: Quat, q2: Quat) {
        let r1 = Rotation::new(q1);
        let r2 = Rotation::new(q2);

        assert_ulps_eq!((r1 + r1).0, q1 * q1);
        assert_ulps_eq!((r2 + r2).0, q2 * q2);

        assert_ulps_eq!((r1 + r2).0, q2 * q1);
        assert_ulps_eq!((r2 + r1).0, q1 * q2);
    }

    #[apply(rotation_cases)]
    fn sub(q1: Quat, q2: Quat) {
        let r1 = Rotation::new(q1);
        let r2 = Rotation::new(q2);

        assert_ulps_eq!((r1 - r1).0, Quat::IDENTITY);
        assert_ulps_eq!((r2 - r2).0, Quat::IDENTITY);

        assert_ulps_eq!((r1 - r2).0, q2.inverse() * q1);

        assert_ulps_eq!((r2 - r1).0, q1.inverse() * q2);
    }

    #[apply(rotation_cases)]
    fn addassign(#[case] q1: Quat, #[case] q2: Quat) -> Result<()> {
        let mut a = Rotation::new(q1);
        let b = Rotation::new(q2);

        a += b;

        assert_ulps_eq!(a.0, q2 * q1);
        Ok(())
    }

    #[apply(rotation_cases)]
    fn subassign(#[case] q1: Quat, #[case] q2: Quat) -> Result<()> {
        let mut r1 = Rotation::new(q1);
        let r2 = Rotation::new(q2);

        r1 -= r2;

        assert_ulps_eq!(r1.0, q2.inverse() * q1);
        Ok(())
    }
}

#[cfg(test)]
mod methods {
    use std::sync::Once;

    use super::*;
    use approx::assert_ulps_eq;
    use color_eyre::Result;
    use rstest::*;
    use rstest_reuse::{self, *};

    static INIT: Once = Once::new();

    #[ctor::ctor]
    fn setup() {
        INIT.call_once(|| {
            color_eyre::install();
        })
    }

    #[template]
    #[rstest]
    #[case((43.42,80.02,39.05,-75.06))]
    #[case((22.49,95.44,41.56,-86.73))]
    #[case((-97.71, -74.05, -81.30, 0.10))]
    #[case((60.18, 20.08, -9.72, 71.92))]
    fn xyzw_cases(#[case] (x, y, z): (f64, f64, f64)) {}

    #[apply(xyzw_cases)]
    fn normalize(#[case] (x, y, z, w): (f64, f64, f64, f64)) -> Result<()> {
        let q = Quat::from_xyzw(x, y, z, w);
        let r = Rotation::new(q);

        assert_ulps_eq!(r.normalize().0, q.normalize());

        Ok(())
    }

    #[apply(xyzw_cases)]
    fn renormalize(#[case] (x, y, z, w): (f64, f64, f64, f64)) -> Result<()> {
        let q = Quat::from_xyzw(x, y, z, w);
        let mut r = Rotation::new(q);

        r.renormalize();

        assert_ulps_eq!(r.0, q.normalize());

        Ok(())
    }

    mod traits {
        use super::*;
        use glam::EulerRot;

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
        fn rotation_cases(#[case] (x, y, z): (f64, f64, f64)) {}

        #[apply(rotation_cases)]
        fn vec3_to_translation(#[case] (x, y, z): (f64, f64, f64)) -> Result<()> {
            let q = Quat::from_euler(EulerRot::YXZ, x, y, z);
            let t = Rotation::new(q);

            let t_conv: Rotation = q.into();

            assert_ulps_eq!(Rotation::from(q), t);
            assert_ulps_eq!(t_conv.0, q);

            Ok(())
        }

        #[apply(rotation_cases)]
        fn translation_to_vec3(#[case] (x, y, z): (f64, f64, f64)) -> Result<()> {
            let q = Quat::from_euler(EulerRot::YXZ, x, y, z);
            let r = Rotation::new(q);

            let q_conv: Quat = r.into();

            assert_ulps_eq!(Quat::from(r), q);
            assert_ulps_eq!(q_conv, q);

            Ok(())
        }
    }
}
