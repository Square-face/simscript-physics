#![allow(clippy::suspicious_op_assign_impl)]
#![allow(clippy::suspicious_arithmetic_impl)]

use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DQuat as Quat;
use overload::overload;
use std::ops;

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

    /// Re normalizes the internal quaternion
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
    use std::f64::consts::{FRAC_PI_3, PI};

    #[test]
    fn new() {
        let q1 = Quat::IDENTITY;
        let q2 = Quat::from_rotation_z(PI / 5.);

        let r1 = Rotation::new(q1);
        let r2 = Rotation::new(q2);

        assert_ulps_eq!(r1.0, q1);
        assert_ulps_eq!(r2.0, q2);
    }

    #[test]
    fn from_x() {
        let q1 = Quat::from_rotation_x(0.);
        let q2 = Quat::from_rotation_x(FRAC_PI_3);

        let r1 = Rotation::from_x(0.);
        let r2 = Rotation::from_x(FRAC_PI_3);

        assert_ulps_eq!(r1.0, q1);
        assert_ulps_eq!(r2.0, q2);
    }

    #[test]
    fn from_y() {
        let q1 = Quat::from_rotation_y(0.);
        let q2 = Quat::from_rotation_y(FRAC_PI_3);

        let r1 = Rotation::from_y(0.);
        let r2 = Rotation::from_y(FRAC_PI_3);

        assert_ulps_eq!(r1.0, q1);
        assert_ulps_eq!(r2.0, q2);
    }

    #[test]
    fn from_z() {
        let q1 = Quat::from_rotation_z(0.);
        let q2 = Quat::from_rotation_z(FRAC_PI_3);

        let r1 = Rotation::from_z(0.);
        let r2 = Rotation::from_z(FRAC_PI_3);

        assert_ulps_eq!(r1.0, q1);
        assert_ulps_eq!(r2.0, q2);
    }

    mod constants {
        use super::*;

        #[test]
        fn zero() {
            assert_ulps_eq!(Rotation::ZERO.0, Quat::IDENTITY);
        }
    }
}

#[cfg(test)]
mod aritmetic {
    use super::*;
    use approx::assert_ulps_eq;

    #[test]
    fn neg() {
        let a = Quat::from_rotation_x(1.0);
        let b = Quat::from_rotation_x(-1.0);
        let r = Rotation::new(a);

        assert_ulps_eq!((-r).0, b);
    }

    #[test]
    fn add() {
        let a = Quat::from_rotation_x(-5.25);
        let b = Quat::from_rotation_y(6.24);
        let c = Quat::from_rotation_z(-9.18);

        let r1 = Rotation::new(a);
        let r2 = Rotation::new(b);
        let r3 = Rotation::new(c);

        assert_ulps_eq!((r1 + r1).0, Quat::from_rotation_x(-10.5));
        assert_ulps_eq!((r2 + r2).0, Quat::from_rotation_y(12.48));
        assert_ulps_eq!((r3 + r3).0, Quat::from_rotation_z(-18.36));

        assert_ulps_eq!((r1 + r2).0, b * a);
        assert_ulps_eq!((r1 + r3).0, c * a);

        assert_ulps_eq!((r2 + r1).0, a * b);
        assert_ulps_eq!((r2 + r3).0, c * b);

        assert_ulps_eq!((r3 + r1).0, a * c);
        assert_ulps_eq!((r3 + r2).0, b * c);
    }

    #[test]
    fn sub() {
        let a = Quat::from_rotation_x(-5.53);
        let b = Quat::from_rotation_y(0.31);
        let c = Quat::from_rotation_z(1.06);

        let r1 = Rotation::new(a);
        let r2 = Rotation::new(b);
        let r3 = Rotation::new(c);

        assert_ulps_eq!((r1 - r1).0, Quat::IDENTITY);
        assert_ulps_eq!((r2 - r2).0, Quat::IDENTITY);
        assert_ulps_eq!((r3 - r3).0, Quat::IDENTITY);

        assert_ulps_eq!((r1 - r2).0, b.inverse() * a);
        assert_ulps_eq!((r1 - r3).0, c.inverse() * a);

        assert_ulps_eq!((r2 - r1).0, a.inverse() * b);
        assert_ulps_eq!((r2 - r3).0, c.inverse() * b);

        assert_ulps_eq!((r3 - r1).0, a.inverse() * c);
        assert_ulps_eq!((r3 - r2).0, b.inverse() * c);
    }
}

#[cfg(test)]
mod assign_arithmetic {
    use super::*;
    use approx::assert_ulps_eq;
    use glam::DQuat as Quat;

    #[test]
    fn test_addassign() {
        let q1 = Quat::from_rotation_x(44.41);
        let q2 = Quat::from_rotation_y(94.24);
        let mut a = Rotation::new(q1);
        let b = Rotation::new(q2);

        a += b;

        assert_ulps_eq!(a.0, q2 * q1);
    }

    #[test]
    fn test_subassign() {
        let q1 = Quat::from_rotation_x(0.5);
        let q2 = Quat::from_rotation_y(0.5);
        let mut a = Rotation::new(q1);
        let b = Rotation::new(q2);

        a -= b;

        assert_ulps_eq!(a.0, b.0.inverse() * q1);
    }
}

#[cfg(test)]
mod methods {
    use super::*;
    use approx::assert_ulps_eq;

    #[test]
    fn normalize() {
        let q1 = Quat::from_xyzw(50.55, 61.21, -77.89, -49.89);
        let q2 = Quat::from_xyzw(-10.71, -9.36, 57.51, -61.57);
        let r1 = Rotation::new(q1);
        let r2 = Rotation::new(q2);

        assert_ulps_eq!(r1.normalize().0, q1.normalize());
        assert_ulps_eq!(r2.normalize().0, q2.normalize());
    }

    #[test]
    fn renormalize() {
        let q1 = Quat::from_xyzw(-67.55, 93.46, -52.36, -58.15);
        let q2 = Quat::from_xyzw(-37.16, 33.91, 28.74, 74.56);
        let mut r1 = Rotation::new(q1);
        let mut r2 = Rotation::new(q2);

        r1.renormalize();
        r2.renormalize();

        assert_ulps_eq!(r1.0, q1.normalize());
        assert_ulps_eq!(r2.0, q2.normalize());
    }

    mod traits {
        use super::*;

        use glam::EulerRot;

        #[test]
        fn vec3_to_trans() {
            let v1 = Quat::from_euler(EulerRot::YXZ, -39.56, 70.77, -68.74);
            let v2 = Quat::from_euler(EulerRot::YXZ, 93.69, 51.02, 47.78);

            let t2: Rotation = v2.into();

            assert_ulps_eq!(Rotation::from(v1), Rotation::new(v1));
            assert_ulps_eq!(t2, Rotation::new(v2));
        }

        #[test]
        fn trans_to_vec3() {
            let v1 = Quat::from_euler(EulerRot::YXZ, -19.39, 61.29, -88.92);
            let v2 = Quat::from_euler(EulerRot::YXZ, 96.86, -28.67, 69.41);

            let t1 = Rotation::new(v1);
            let t2 = Rotation::new(v2);

            let v3: Quat = t2.into();

            assert_ulps_eq!(Quat::from(t1), v1);
            assert_ulps_eq!(v3, v2);
        }
    }
}
