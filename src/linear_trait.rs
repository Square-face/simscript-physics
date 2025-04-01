use glam::DVec3 as Vec3;

pub trait Vec3Wrap {
    /// A zero vector.
    const ZERO: Self;

    /// Vector of magnitude one in all directions.
    const ONE: Self;

    /// Unit vector in the positive X direction.
    const X: Self;
    /// Unit vector in the positive Y direction.
    const Y: Self;
    /// Unit vector in the positive Z direction.
    const Z: Self;

    /// Unit vector in the negative X direction.
    const NEG_X: Self;
    /// Unit vector in the negative Y direction.
    const NEG_Y: Self;
    /// Unit vector in the negative Z direction.
    const NEG_Z: Self;

    /// Creates a new instance with the specified `x`, `y`, and `z` components.
    #[inline]
    #[must_use]
    fn new(x: f64, y: f64, z: f64) -> Self
    where
        Self: std::marker::Sized,
    {
        Self::from_vec3(Vec3::new(x, y, z))
    }

    /// Creates an instance from an existing vector type.
    #[must_use]
    fn from_vec3(v: Vec3) -> Self;

    /// Creates an instance where all components are set to `v`.
    #[inline]
    #[must_use]
    fn splat(v: f64) -> Self
    where
        Self: std::marker::Sized,
    {
        Self::new(v, v, v) // Default implementation calls `new`
    }

    /// Creates an instance with only the X component set.
    #[inline]
    #[must_use]
    fn with_x(x: f64) -> Self
    where
        Self: std::marker::Sized,
    {
        Self::new(x, 0., 0.) // Default implementation calls `new`
    }

    /// Creates an instance with only the Y component set.
    #[inline]
    #[must_use]
    fn with_y(y: f64) -> Self
    where
        Self: std::marker::Sized,
    {
        Self::new(0., y, 0.) // Default implementation calls `new`
    }

    /// Creates an instance with only the Z component set.
    #[inline]
    #[must_use]
    fn with_z(z: f64) -> Self
    where
        Self: std::marker::Sized,
    {
        Self::new(0., 0., z) // Default implementation calls `new`
    }
}

#[cfg(test)]
mod tests {
    use std::f64;

    use super::*;
    use approx::{assert_ulps_eq, AbsDiffEq, RelativeEq, UlpsEq};
    use derives::Approx;
    use rstest::*;

    #[derive(Debug, Default, Clone, Copy, PartialEq, Approx)]
    struct MockVec(pub Vec3);

    // Mock implementation
    impl Vec3Wrap for MockVec {
        const ZERO: Self = MockVec(Vec3::new(0.0, 0.0, 0.0));
        const ONE: Self = MockVec(Vec3::new(1.0, 1.0, 1.0));
        const X: Self = MockVec(Vec3::new(1.0, 0.0, 0.0));
        const Y: Self = MockVec(Vec3::new(0.0, 1.0, 0.0));
        const Z: Self = MockVec(Vec3::new(0.0, 0.0, 1.0));
        const NEG_X: Self = MockVec(Vec3::new(-1.0, 0.0, 0.0));
        const NEG_Y: Self = MockVec(Vec3::new(0.0, -1.0, 0.0));
        const NEG_Z: Self = MockVec(Vec3::new(0.0, 0.0, -1.0));

        fn from_vec3(v: Vec3) -> Self {
            Self(v)
        }
    }

    #[rstest]
    #[case(MockVec::ZERO,   MockVec::new(0.0, 0.0, 0.0))]
    #[case(MockVec::ONE,    MockVec::new(1.0, 1.0, 1.0))]
    #[case(MockVec::X,      MockVec::new(1.0, 0.0, 0.0))]
    #[case(MockVec::Y,      MockVec::new(0.0, 1.0, 0.0))]
    #[case(MockVec::Z,      MockVec::new(0.0, 0.0, 1.0))]
    #[case(MockVec::NEG_X,  MockVec::new(-1.0, 0.0, 0.0))]
    #[case(MockVec::NEG_Y,  MockVec::new(0.0, -1.0, 0.0))]
    #[case(MockVec::NEG_Z,  MockVec::new(0.0, 0.0, -1.0))]
    fn constants(#[case] actual: MockVec, #[case] expected: MockVec) {
        assert_eq!(actual, expected);
    }

    #[rstest]
    #[case(50.82, 6.19, -58.04)]
    #[case(91.23, 18.32, -67.24)]
    fn from_vec(#[case] x: f64, #[case] y: f64, #[case] z: f64) {
        let v = Vec3::new(x, y, z);
        let m = MockVec::from_vec3(v);
        assert_ulps_eq!(m.0, v);
    }

    #[rstest]
    #[case::new(MockVec::new(0.0, 0.0, 0.0))]
    #[case::splat(MockVec::splat(0.0))]
    #[case::with_z(MockVec::with_z(0.0))]
    #[case::with_z(MockVec::with_z(0.0))]
    #[case::with_z(MockVec::with_z(0.0))]
    #[case::vec3(MockVec::from_vec3(Vec3::ZERO))]
    fn constructors_zero(#[case] actual: MockVec) {
        assert_ulps_eq!(actual.0.x, 0.0);
        assert_ulps_eq!(actual.0.y, 0.0);
        assert_ulps_eq!(actual.0.z, 0.0);
    }

    #[rstest]
    #[case::new(MockVec::new(37.51, 21.97, 61.60),      (37.51, 21.97, 61.60))]
    #[case::new(MockVec::new(-62.59, -79.50, 69.27),    (-62.59, -79.50, 69.27))]
    #[case::splat(MockVec::splat(-73.49),               (-73.49, -73.49, -73.49))]
    #[case::splat(MockVec::splat(-63.90),               (-63.90, -63.90, -63.90))]
    #[case::with_x(MockVec::with_x(-12.76),             (-12.76, 0.0, 0.0))]
    #[case::with_x(MockVec::with_x(-89.77),             (-89.77, 0.0, 0.0))]
    #[case::with_y(MockVec::with_y(-34.87),             (0.0, -34.87, 0.0))]
    #[case::with_y(MockVec::with_y(65.40),              (0.0, 65.40, 0.0))]
    #[case::with_z(MockVec::with_z(-12.60),             (0.0, 0.0, -12.60))]
    #[case::with_z(MockVec::with_z(-81.09),             (0.0, 0.0, -81.09))]
    fn constructors_random(#[case] actual: MockVec, #[case] (x, y, z): (f64, f64, f64)) {
        assert_ulps_eq!(actual.0.x, x);
        assert_ulps_eq!(actual.0.y, y);
        assert_ulps_eq!(actual.0.z, z);
    }
}
