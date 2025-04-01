#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
#[cfg(feature = "approx")]
use derives::Approx;

use glam::DVec3 as Vec3;
use overload::overload;
use std::iter::Sum;
use std::ops;

use crate::{inertia_mass::Inertia, velocity::AngVel};

use super::Momentum;

/// Angular momentum struct with a 3D vector.
#[cfg_attr(feature = "approx", derive(Approx))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct AngMom(pub Vec3);

impl AngMom {
    /// Zero momentum constant.
    pub const ZERO: Self = Self::splat(0.);
    /// Unit momentum constant (all components 1).
    pub const ONE: Self = Self::splat(1.);

    /// Unit momentum along X axis.
    pub const X: Self = Self::with_x(1.);
    /// Unit momentum along Y axis.
    pub const Y: Self = Self::with_y(1.);
    /// Unit momentum along Z axis.
    pub const Z: Self = Self::with_z(1.);

    /// Negative unit momentum along X axis.
    pub const NEG_X: Self = Self::with_x(-1.);
    /// Negative unit momentum along Y axis.
    pub const NEG_Y: Self = Self::with_y(-1.);
    /// Negative unit momentum along Z axis.
    pub const NEG_Z: Self = Self::with_z(-1.);

    /// Creates a new angular momentum from x, y, z components.
    #[inline]
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vec3::new(x, y, z))
    }

    /// Creates a new angular momentum from a [Vec3].
    #[inline]
    #[must_use]
    pub const fn from_vec3(v: Vec3) -> Self {
        Self(v)
    }

    /// Creates a new angular momentum with all components set to `v`.
    #[inline]
    #[must_use]
    pub const fn splat(v: f64) -> Self {
        Self::new(v, v, v)
    }

    /// Creates a new angular momentum with x component set to `x`, others zero.
    #[inline]
    #[must_use]
    pub const fn with_x(x: f64) -> Self {
        Self::new(x, 0., 0.)
    }

    /// Creates a new angular momentum with y component set to `y`, others zero.
    #[inline]
    #[must_use]
    pub const fn with_y(y: f64) -> Self {
        Self::new(0., y, 0.)
    }

    /// Creates a new angular momentum with z component set to `z`, others zero.
    #[inline]
    #[must_use]
    pub const fn with_z(z: f64) -> Self {
        Self::new(0., 0., z)
    }
}

/// Converts a [Vec3] into an [AngMom].
impl From<Vec3> for AngMom {
    #[inline]
    #[must_use]
    fn from(value: Vec3) -> Self {
        Self::from_vec3(value)
    }
}

/// Converts an [AngMom] into a [Vec3].
impl From<AngMom> for Vec3 {
    #[inline]
    #[must_use]
    fn from(value: AngMom) -> Self {
        value.0
    }
}

/// Extracts angular momentum from a [Momentum].
impl From<Momentum> for AngMom {
    #[inline]
    #[must_use]
    fn from(value: Momentum) -> Self {
        value.angular
    }
}

/// Sums an iterator of [AngMom] values.
impl Sum for AngMom {
    #[inline]
    #[must_use]
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::ZERO, |a, b| a + b)
    }
}

overload!((a: ?AngMom) + (b: ?AngMom) -> AngMom{ AngMom( a.0 + b.0 ) });
overload!((a: ?AngMom) - (b: ?AngMom) -> AngMom{ AngMom( a.0 - b.0 ) });
overload!((a: &mut AngMom) += (b: ?AngMom) { a.0 += b.0 });
overload!((a: &mut AngMom) -= (b: ?AngMom) { a.0 -= b.0 });

overload!((a: ?AngMom) / (b: ?Inertia) -> AngVel{ AngVel(b.0.mul_vec3(a.0)) });

overload!((a: ?AngMom) * (b: f64) -> AngMom{ AngMom( a.0 * b ) });
overload!((a: ?AngMom) / (b: f64) -> AngMom{ AngMom( a.0 / b ) });
overload!((a: &mut AngMom) *= (b: f64) { a.0 *= b });
overload!((a: &mut AngMom) /= (b: f64) { a.0 /= b });

overload!(-(a: ?AngMom) -> AngMom{ AngMom( -a.0 ) });
