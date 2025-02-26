use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

#[derive(Debug, Clone, Copy, PartialEq, Approx)]
pub struct Translation(pub Vec3);

impl Translation {
    pub const ZERO: Self = Self::splat(0.);

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

// Adding a subtracting with self is a valid op
overload!((a: ?Translation) + (b: ?Translation) -> Translation{ Translation( a.0 + b.0 ) });
overload!((a: ?Translation) - (b: ?Translation) -> Translation{ Translation( a.0 - b.0 ) });
overload!((a: &mut Translation) += (b: ?Translation) { a.0 += b.0 });
overload!((a: &mut Translation) -= (b: ?Translation) { a.0 -= b.0 });

// Multiplying or dividing by scalars shouldn't change the type
overload!((a: ?Translation) * (b: f64) -> Translation{ Translation( a.0 * b ) });
overload!((a: ?Translation) / (b: f64) -> Translation{ Translation( a.0 / b ) });
overload!((a: &mut Translation) *= (b: f64) { a.0 *= b });
overload!((a: &mut Translation) /= (b: f64) { a.0 /= b });

overload!(-(a: ?Translation) -> Translation{ Translation( -a.0 ) });

#[cfg(test)]
mod constructors {
    use super::*;
    use approx::assert_ulps_eq;

    #[test]
    fn test_new() {
        let v = Translation::new(1.0, 2.0, 3.0);
        assert_ulps_eq!(v.0, Vec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn test_splat() {
        let v = Translation::splat(2.0);
        assert_ulps_eq!(v.0, Vec3::new(2.0, 2.0, 2.0));
    }

    #[test]
    fn test_with_x() {
        let v = Translation::with_x(3.0);
        assert_ulps_eq!(v.0, Vec3::new(3.0, 0.0, 0.0));
    }

    #[test]
    fn test_with_y() {
        let v = Translation::with_y(4.0);
        assert_ulps_eq!(v.0, Vec3::new(0.0, 4.0, 0.0));
    }

    #[test]
    fn test_with_z() {
        let v = Translation::with_z(5.0);
        assert_ulps_eq!(v.0, Vec3::new(0.0, 0.0, 5.0));
    }

    #[test]
    fn test_zero() {
        assert_ulps_eq!(Translation::ZERO, Translation::splat(0.0));
    }
}

#[cfg(test)]
mod arithmetic {
    use super::*;
    use approx::assert_ulps_eq;

    #[test]
    fn test_add() {
        let a = Translation::new(1.0, 2.0, 3.0);
        let b = Translation::new(4.0, 5.0, 6.0);
        let result = a + b;
        assert_ulps_eq!(result.0, Vec3::new(5.0, 7.0, 9.0));
    }

    #[test]
    fn test_sub() {
        let a = Translation::new(4.0, 5.0, 6.0);
        let b = Translation::new(1.0, 2.0, 3.0);
        let result = a - b;
        assert_ulps_eq!(result.0, Vec3::new(3.0, 3.0, 3.0));
    }

    #[test]
    fn test_neg() {
        let a = Translation::new(1.0, -2.0, 3.0);
        let result = -a;
        assert_ulps_eq!(result.0, Vec3::new(-1.0, 2.0, -3.0));
    }

    #[test]
    fn test_mul_scalar() {
        let a = Translation::new(1.0, -2.0, 3.0);
        let result = a * 2.0;
        assert_ulps_eq!(result.0, Vec3::new(2.0, -4.0, 6.0));
    }
}
#[cfg(test)]
mod assignment_arithmetic {
    use super::*;
    use approx::assert_ulps_eq;

    #[test]
    fn test_add_assign() {
        let mut a = Translation::new(1.0, 2.0, 3.0);
        let b = Translation::new(4.0, 5.0, 6.0);
        a += b;
        assert_ulps_eq!(a.0, Vec3::new(5.0, 7.0, 9.0));
    }

    #[test]
    fn test_sub_assign() {
        let mut a = Translation::new(4.0, 5.0, 6.0);
        let b = Translation::new(1.0, 2.0, 3.0);
        a -= b;
        assert_ulps_eq!(a.0, Vec3::new(3.0, 3.0, 3.0));
    }

    #[test]
    fn test_mul_assign_scalar() {
        let mut a = Translation::new(1.0, -2.0, 3.0);
        a *= 2.0;
        assert_ulps_eq!(a.0, Vec3::new(2.0, -4.0, 6.0));
    }
}
#[cfg(test)]
mod equality {
    use super::*;
    use approx::{assert_abs_diff_eq, assert_relative_eq, assert_ulps_eq};

    #[test]
    fn test_approx_eq() {
        let a = Translation::with_x(0.1) + Translation::with_x(0.2);
        let b = Translation::with_x(0.3);

        assert_ne!(a, b); // Normal compare should fail this

        // But using approx should work
        assert_abs_diff_eq!(a, b, epsilon = 1e-5);
        assert_relative_eq!(a, b, epsilon = 1e-5);
        assert_ulps_eq!(a, b);
    }
}
