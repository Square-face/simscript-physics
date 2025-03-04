#![allow(clippy::suspicious_op_assign_impl)]
#![allow(clippy::suspicious_arithmetic_impl)]

use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DQuat as Quat;
use overload::overload;
use std::ops;

#[derive(Debug, Clone, Copy, PartialEq, Approx)]
pub struct Rotation(pub Quat);

impl Rotation {
    pub const ZERO: Self = Self::new(Quat::IDENTITY);

    /// Create a new [Rotation] with the given starting rotation
    pub const fn new(ang: Quat) -> Self {
        Self(ang)
    }
}

overload!((a: ?Rotation) + (b: ?Rotation) -> Rotation{ Rotation( b.0 * a.0 ) });
overload!((a: ?Rotation) - (b: ?Rotation) -> Rotation{ Rotation( b.0.inverse() * a.0) });
overload!((a: &mut Rotation) += (b: ?Rotation) { a.0 = b.0 * a.0 });
overload!((a: &mut Rotation) -= (b: ?Rotation) { a.0 = b.0.inverse() * a.0});

overload!(-(a: ?Rotation) -> Rotation{ Rotation(a.0.inverse()) });

#[cfg(test)]
mod constructors {
    use std::f64::consts::PI;

    use super::*;

    #[test]
    fn test_new() {
        let q = Quat::from_rotation_z(PI / 5.);
        let rotation = Rotation::new(q);
        assert_eq!(rotation.0, q);
    }

    #[test]
    fn test_zero() {
        assert_eq!(Rotation::ZERO.0, Quat::IDENTITY);
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
mod compound_arithmetic {
    use super::*;
    use approx::assert_ulps_eq;
    use glam::DQuat as Quat;

    #[test]
    #[should_panic]
    fn test_1() {
        let q1 = Quat::from_rotation_x(0.3);
        let q2 = Quat::from_rotation_y(0.5);

        let a = Rotation::new(q1);
        let b = Rotation::new(q2);

        // With rotation, the order of operation matters, this is invalid
        assert_ulps_eq!(a + b, b + a);
    }
}

#[cfg(test)]
mod rotating_vectors {
    use std::f64::consts::PI;

    use approx::assert_ulps_eq;
    use glam::DVec3;

    use super::*;

    #[test]
    fn unit() {
        let v = DVec3::new(1.0, 0.0, 0.0);
        let r = Rotation::new(Quat::from_rotation_z(PI / 2.));
        assert_ulps_eq!(r.0 * v, DVec3::new(0.0, 1.0, 0.0));
    }
}
