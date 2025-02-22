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

overload!((a: ?Rotation) + (b: ?Rotation) -> Rotation{ Rotation( a.0 * b.0 ) });
overload!((a: ?Rotation) - (b: ?Rotation) -> Rotation{ Rotation( a.0 * (-b).0) });
overload!((a: &mut Rotation) += (b: ?Rotation) { a.0 *= b.0 });
overload!((a: &mut Rotation) -= (b: ?Rotation) { a.0 *= (-b).0 });

overload!(-(a: ?Rotation) -> Rotation{ Rotation(a.0.inverse()) });

#[cfg(test)]
mod constructors {
    use super::*;
    use glam::DQuat as Quat;

    #[test]
    fn test_new() {
        let q = Quat::IDENTITY;
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
    use glam::DQuat as Quat;

    #[test]
    fn test_negation() {
        let q = Quat::from_rotation_x(1.0);
        let rotation = Rotation::new(q);

        // The inverse of a quaternion should undo its effect
        assert_ulps_eq!(-rotation.0, -q);
    }

    #[test]
    fn test_addition() {
        let a = Rotation::new(Quat::from_rotation_x(0.5));

        // Multiplication order matters for quaternions
        assert_ulps_eq!((a + a).0, Quat::from_rotation_x(1.0));
    }

    #[test]
    fn test_subtraction() {
        let a = Rotation::new(Quat::from_rotation_y(0.2));

        // Multiplication order matters for quaternions
        assert_ulps_eq!((a - a).0, Quat::IDENTITY);
    }
}

#[cfg(test)]
mod assign_arithmetic {
    use super::*;
    use approx::assert_ulps_eq;
    use glam::DQuat as Quat;

    #[test]
    fn test_addassign() {
        let q1 = Quat::from_rotation_x(0.5);
        let q2 = Quat::from_rotation_y(0.5);
        let mut a = Rotation::new(q1);
        let b = Rotation::new(q2);

        a += b;

        assert_ulps_eq!(a.0, q1 * q2);
    }

    #[test]
    fn test_subassign() {
        let q1 = Quat::from_rotation_x(0.5);
        let q2 = Quat::from_rotation_y(0.5);
        let mut a = Rotation::new(q1);
        let b = Rotation::new(q2);

        a -= b;

        assert_ulps_eq!(a.0, q1 * (-b).0);
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
        let r = Rotation::new(Quat::from_rotation_z(PI/2.));
        assert_ulps_eq!(r.0*v, DVec3::new(0.0, 1.0, 0.0));
    }
}
