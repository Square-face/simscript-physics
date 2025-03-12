use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

/// Represents a linear transformation in 3d space
///
/// The translation is being represented in meters
#[derive(Debug, Clone, Copy, PartialEq, Approx)]
pub struct Translation(pub Vec3);

impl Translation {
    pub const ZERO: Self = Self::splat(0.);

    pub const X: Self = Self::from_x(1.);
    pub const Y: Self = Self::from_y(1.);
    pub const Z: Self = Self::from_z(1.);

    pub const NEG_X: Self = Self::from_x(-1.);
    pub const NEG_Y: Self = Self::from_y(-1.);
    pub const NEG_Z: Self = Self::from_z(-1.);

    /// Create a new translation
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vec3::new(x, y, z))
    }

    /// Create a new translation from a [Vec3]
    pub const fn from_vec3(v: Vec3) -> Self {
        Self(v)
    }

    /// Create a new translation with all values as the same
    pub const fn splat(v: f64) -> Self {
        Self::new(v, v, v)
    }

    /// Create a new translation with all values as zero except x
    pub const fn from_x(x: f64) -> Self {
        Self::new(x, 0., 0.)
    }

    /// Create a new translation with all values as zero except y
    pub const fn from_y(y: f64) -> Self {
        Self::new(0., y, 0.)
    }

    /// Create a new translation with all values as zero except z
    pub const fn from_z(z: f64) -> Self {
        Self::new(0., 0., z)
    }

    /// Create a new translation with x as a different value
    pub const fn with_x(&self, x: f64) -> Self {
        Self::new(x, self.0.y, self.0.z)
    }

    /// Create a new translation with y as a different value
    pub const fn with_y(&self, y: f64) -> Self {
        Self::new(self.0.x, y, self.0.z)
    }

    /// Create a new translation with z as a different value
    pub const fn with_z(&self, z: f64) -> Self {
        Self::new(self.0.x, self.0.y, z)
    }
}

impl From<Vec3> for Translation {
    fn from(value: Vec3) -> Self {
        Self::from_vec3(value)
    }
}

impl From<Translation> for Vec3 {
    fn from(value: Translation) -> Self {
        value.0
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
    fn new() {
        let v = Translation::new(1.0, 2.0, 3.0);
        assert_ulps_eq!(v.0, Vec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn splat() {
        let v = Translation::splat(2.0);
        assert_ulps_eq!(v.0, Vec3::new(2.0, 2.0, 2.0));
    }

    #[test]
    fn from_x() {
        let v = Translation::from_x(3.0);
        assert_ulps_eq!(v.0, Vec3::new(3.0, 0.0, 0.0));
    }

    #[test]
    fn from_y() {
        let v = Translation::from_y(4.0);
        assert_ulps_eq!(v.0, Vec3::new(0.0, 4.0, 0.0));
    }

    #[test]
    fn from_z() {
        let v = Translation::from_z(5.0);
        assert_ulps_eq!(v.0, Vec3::new(0.0, 0.0, 5.0));
    }

    mod constants {
        use super::*;
        use approx::assert_ulps_eq;

        #[test]
        fn zero() {
            assert_ulps_eq!(Translation::ZERO.0, Vec3::splat(0.));
        }

        #[test]
        fn cardinal() {
            assert_ulps_eq!(Translation::X.0, Vec3::new(1., 0., 0.));
            assert_ulps_eq!(Translation::Y.0, Vec3::new(0., 1., 0.));
            assert_ulps_eq!(Translation::Z.0, Vec3::new(0., 0., 1.));
        }

        #[test]
        fn cardinal_neg() {
            assert_ulps_eq!(Translation::NEG_X.0, Vec3::new(-1., 0., 0.));
            assert_ulps_eq!(Translation::NEG_Y.0, Vec3::new(0., -1., 0.));
            assert_ulps_eq!(Translation::NEG_Z.0, Vec3::new(0., 0., -1.));
        }
    }
}

#[cfg(test)]
mod arithmetic {
    use super::*;
    use approx::assert_ulps_eq;

    #[test]
    fn add() {
        let a = Translation::new(2.13, 97.05, 50.73);
        let b = Translation::new(41.55, 82.06, 50.94);
        let res = a + b;
        assert_ulps_eq!(res.0, Vec3::new(43.68, 179.11, 101.67));
    }

    #[test]
    fn sub() {
        let a = Translation::new(42.54, 74.96, 57.62);
        let b = Translation::new(43.16, 93.65, 68.58);
        let res = a - b;
        assert_ulps_eq!(res.0, Vec3::new(-0.62, -18.69, -10.96), epsilon = 1e-14);
    }

    #[test]
    fn neg() {
        let a = Translation::new(1.0, -2.0, 3.0);
        let res = -a;
        assert_ulps_eq!(res.0, Vec3::new(-1.0, 2.0, -3.0));
    }

    #[test]
    fn mul_scalar() {
        let a = Translation::new(1.0, -2.0, 3.0);
        let res = a * 2.0;
        assert_ulps_eq!(res.0, Vec3::new(2.0, -4.0, 6.0));
    }
}

#[cfg(test)]
mod assignment_arithmetic {
    use super::*;
    use approx::assert_ulps_eq;

    #[test]
    fn add_assign() {
        let mut a = Translation::new(1.0, 2.0, 3.0);
        let b = Translation::new(4.0, 5.0, 6.0);
        a += b;
        assert_ulps_eq!(a.0, Vec3::new(5.0, 7.0, 9.0));
    }

    #[test]
    fn sub_assign() {
        let mut a = Translation::new(4.0, 5.0, 6.0);
        let b = Translation::new(1.0, 2.0, 3.0);
        a -= b;
        assert_ulps_eq!(a.0, Vec3::new(3.0, 3.0, 3.0));
    }

    #[test]
    fn mul_assign_scalar() {
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
        let a = Translation::from_x(0.1) + Translation::from_x(0.2);
        let b = Translation::from_x(0.3);

        assert_ne!(a, b); // Normal compare should fail this

        // But using approx should work
        assert_abs_diff_eq!(a, b, epsilon = 1e-5);
        assert_relative_eq!(a, b, epsilon = 1e-5);
        assert_ulps_eq!(a, b);
    }
}
