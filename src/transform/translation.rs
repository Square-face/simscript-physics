use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

/// Represents a linear transformation in 3d space
///
/// The translation is being represented in meters
#[derive(Debug, Default, Clone, Copy, PartialEq, Approx)]
pub struct Translation(pub Vec3);

impl Translation {
    pub const ZERO: Self = Self::new(0., 0., 0.);

    pub const X: Self = Self::new(1., 0., 0.);
    pub const Y: Self = Self::new(0., 1., 0.);
    pub const Z: Self = Self::new(0., 0., 1.);

    pub const NEG_X: Self = Self::new(-1., 0., 0.);
    pub const NEG_Y: Self = Self::new(0., -1., 0.);
    pub const NEG_Z: Self = Self::new(0., 0., -1.);

    /// Create a new translation
    #[inline]
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vec3::new(x, y, z))
    }

    /// Create a new translation from a [Vec3]
    #[inline]
    #[must_use]
    pub const fn from_vec3(v: Vec3) -> Self {
        Self(v)
    }

    /// Create a new translation with all values as the same
    #[inline]
    #[must_use]
    pub const fn splat(v: f64) -> Self {
        Self::new(v, v, v)
    }

    /// Create a new translation with all values as zero except x
    #[inline]
    #[must_use]
    pub const fn from_x(x: f64) -> Self {
        Self::new(x, 0., 0.)
    }

    /// Create a new translation with all values as zero except y
    #[inline]
    #[must_use]
    pub const fn from_y(y: f64) -> Self {
        Self::new(0., y, 0.)
    }

    /// Create a new translation with all values as zero except z
    #[inline]
    #[must_use]
    pub const fn from_z(z: f64) -> Self {
        Self::new(0., 0., z)
    }

    /// Create a new translation with x as a different value
    #[inline]
    #[must_use]
    pub const fn with_x(&self, x: f64) -> Self {
        Self::new(x, self.0.y, self.0.z)
    }

    /// Create a new translation with y as a different value
    #[inline]
    #[must_use]
    pub const fn with_y(&self, y: f64) -> Self {
        Self::new(self.0.x, y, self.0.z)
    }

    /// Create a new translation with z as a different value
    #[inline]
    #[must_use]
    pub const fn with_z(&self, z: f64) -> Self {
        Self::new(self.0.x, self.0.y, z)
    }
}

impl From<Vec3> for Translation {
    #[inline]
    #[must_use]
    fn from(value: Vec3) -> Self {
        Self::from_vec3(value)
    }
}

impl From<Translation> for Vec3 {
    #[inline]
    #[must_use]
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
mod tests {
    use super::*;
    use approx::assert_ulps_eq;
    use rstest::*;
    use rstest_reuse::{self, *};

    #[template]
    #[rstest]
    #[case(Vec3::new(2.13, 97.05, 50.73), Vec3::new(41.55, 82.06, 50.94))]
    #[case(Vec3::new(42.54, 74.96, 57.62), Vec3::new(-43.16, -93.65, -68.58))]
    #[case(Vec3::new(33.36, 27.72, 45.08), Vec3::new(81.12, 85.85, 22.69))]
    #[case(Vec3::new(88.09, 50.55, 95.59), Vec3::new(6.32, 76.81, 62.23))]
    #[case(Vec3::new(43.87, 63.16, 30.39), Vec3::new(27.08, 66.96, 79.64))]
    fn double_cases(#[case] a: Vec3, #[case] b: Vec3) {}

    #[template]
    #[rstest]
    #[case(Vec3::new(47.98, 89.59, 93.40), 59.55)]
    #[case(Vec3::new(55.85, 29.37, 12.67), 60.21)]
    #[case(Vec3::new(1.96, 35.66, 63.39), 96.07)]
    fn scalar_cases(#[case] a: Translation, #[case] scalar: f64) {}

    #[rstest]
    #[case(Translation::new(45.08, 48.91, 1.47), Vec3::new(45.08, 48.91, 1.47))]
    #[case(Translation::new(81.81, 29.43, 68.61), Vec3::new(81.81, 29.43, 68.61))]
    #[case(Translation::splat(96.99), Vec3::new(96.99, 96.99, 96.99))]
    #[case(Translation::splat(41.72), Vec3::new(41.72, 41.72, 41.72))]
    #[case(Translation::from_x(65.37), Vec3::new(65.37, 0.0, 0.0))]
    #[case(Translation::from_x(72.70), Vec3::new(72.70, 0.0, 0.0))]
    #[case(Translation::from_y(94.59), Vec3::new(0.0, 94.59, 0.0))]
    #[case(Translation::from_y(22.66), Vec3::new(0.0, 22.66, 0.0))]
    #[case(Translation::from_z(42.38), Vec3::new(0.0, 0.0, 42.38))]
    #[case(Translation::from_z(26.30), Vec3::new(0.0, 0.0, 26.30))]
    fn constructors(#[case] t: Translation, #[case] v: Vec3) {
        assert_ulps_eq!(t.0, v);
    }

    #[rstest]
    #[case(Translation::ZERO, Vec3::splat(0.))]
    #[case(Translation::X, Vec3::new(1.0, 0.0, 0.0))]
    #[case(Translation::Y, Vec3::new(0.0, 1.0, 0.0))]
    #[case(Translation::Z, Vec3::new(0.0, 0.0, 1.0))]
    #[case(Translation::NEG_X, Vec3::new(-1.0, 0.0, 0.0))]
    #[case(Translation::NEG_Y, Vec3::new(0.0, -1.0, 0.0))]
    #[case(Translation::NEG_Z, Vec3::new(0.0, 0.0, -1.0))]
    fn constants(#[case] t: Translation, #[case] v: Vec3) {
        assert_ulps_eq!(t.0, v);
    }

    mod with {
        use super::*;

        #[apply(scalar_cases)]
        fn with_x(#[case] v: Vec3, #[case] scalar: f64) {
            let t = Translation::from_vec3(v);
            let actual = t.with_x(scalar);

            assert_ulps_eq!(actual.0.x, scalar);
            assert_ulps_eq!(actual.0.y, v.y);
            assert_ulps_eq!(actual.0.z, v.z);
        }

        #[apply(scalar_cases)]
        fn with_y(#[case] v: Vec3, #[case] scalar: f64) {
            let t = Translation::from_vec3(v);
            let actual = t.with_y(scalar);

            assert_ulps_eq!(actual.0.x, v.x);
            assert_ulps_eq!(actual.0.y, scalar);
            assert_ulps_eq!(actual.0.z, v.z);
        }

        #[apply(scalar_cases)]
        fn with_z(#[case] v: Vec3, #[case] scalar: f64) {
            let t = Translation::from_vec3(v);
            let actual = t.with_z(scalar);

            assert_ulps_eq!(actual.0.x, v.x);
            assert_ulps_eq!(actual.0.y, v.y);
            assert_ulps_eq!(actual.0.z, scalar);
        }
    }

    #[cfg(test)]
    mod arithmetic {
        use super::*;

        #[apply(double_cases)]
        fn add(#[case] a: Vec3, #[case] b: Vec3) {
            let t1 = Translation::from(a);
            let t2 = Translation::from(b);
            assert_ulps_eq!((t1 + t2).0, a + b, epsilon = 1e-14);
        }

        #[apply(double_cases)]
        fn sub(#[case] a: Vec3, #[case] b: Vec3) {
            let t1 = Translation::from(a);
            let t2 = Translation::from(b);
            assert_ulps_eq!((t1 - t2).0, a - b, epsilon = 1e-14);
        }

        #[rstest]
        #[case(Translation::new(1.0, -2.0, 3.0), -Translation::new(1.0, -2.0, 3.0))]
        fn neg(#[case] a: Translation, #[case] expected: Translation) {
            assert_ulps_eq!((-a).0, expected.0);
        }

        #[apply(scalar_cases)]
        fn mul_scalar(#[case] a: Vec3, #[case] scalar: f64) {
            let t = Translation::from_vec3(a);
            assert_ulps_eq!((t * scalar).0, a * scalar);
        }
    }

    #[cfg(test)]
    mod assignment_arithmetic {
        use super::*;

        #[apply(double_cases)]
        fn add_assign(#[case] a: Vec3, #[case] b: Vec3) {
            let mut t1 = Translation::from(a);
            let t2 = Translation::from(b);
            t1 += t2;
            assert_ulps_eq!(t1.0, a + b);
        }

        #[apply(double_cases)]
        fn sub_assign(#[case] a: Vec3, #[case] b: Vec3) {
            let mut t1 = Translation::from(a);
            let t2 = Translation::from(b);
            t1 -= t2;
            assert_ulps_eq!(t1.0, a - b);
        }

        #[apply(scalar_cases)]
        fn mul_assign_scalar(#[case] a: Vec3, #[case] scalar: f64) {
            let mut t = Translation::from_vec3(a);
            t *= scalar;
            assert_ulps_eq!(t.0, a * scalar);
        }
    }

    #[cfg(test)]
    mod traits {
        use super::*;

        #[apply(double_cases)]
        fn vec3_to_trans(#[case] a: Vec3, #[case] b: Vec3) {
            let t2: Translation = b.into();

            assert_ulps_eq!(Translation::from(a), Translation::from_vec3(a));
            assert_ulps_eq!(t2, Translation::from_vec3(b));
        }

        #[apply(double_cases)]
        fn trans_to_vec3(#[case] a: Vec3, #[case] b: Vec3) {
            let t1 = Translation::from_vec3(a);
            let t2 = Translation::from_vec3(b);

            let v3: Vec3 = t2.into();

            assert_ulps_eq!(Vec3::from(t1), a);
            assert_ulps_eq!(v3, b);
        }
    }
}
