#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
#[cfg(feature = "approx")]
use approx_derive::Approx;

use glam::DVec3 as Vec3;
use overload::overload;
use std::{iter::Sum, ops};

use crate::{inertia_mass::Mass, velocity::LinVel};

use super::{AngMom, Momentum};

/// Linear momentum struct with a 3D vector.
#[cfg_attr(feature = "approx", derive(Approx))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct LinMom(pub Vec3);

impl LinMom {
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

    /// Creates a new linear momentum from x, y, z components.
    #[inline]
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vec3::new(x, y, z))
    }

    /// Creates a new linear momentum from a [`Vec3`].
    #[inline]
    #[must_use]
    pub const fn from_vec3(v: Vec3) -> Self {
        Self(v)
    }

    /// Creates a new linear momentum with all components set to `v`.
    #[inline]
    #[must_use]
    pub const fn splat(v: f64) -> Self {
        Self::new(v, v, v)
    }

    /// Creates a new linear momentum with x component set to `x`, others zero.
    #[inline]
    #[must_use]
    pub const fn with_x(x: f64) -> Self {
        Self::new(x, 0., 0.)
    }

    /// Creates a new linear momentum with y component set to `y`, others zero.
    #[inline]
    #[must_use]
    pub const fn with_y(y: f64) -> Self {
        Self::new(0., y, 0.)
    }

    /// Creates a new linear momentum with z component set to `z`, others zero.
    #[inline]
    #[must_use]
    pub const fn with_z(z: f64) -> Self {
        Self::new(0., 0., z)
    }
}

impl LinMom {
    /// Converts to a [`Momentum`] with zero angular momentum.
    #[inline]
    #[must_use]
    pub const fn to_vel(self) -> Momentum {
        Momentum::new(self, AngMom::ZERO)
    }

    /// Combines with angular momentum to create a [`Momentum`].
    #[inline]
    #[must_use]
    pub const fn with_angular(self, ang: AngMom) -> Momentum {
        Momentum::new(self, ang)
    }
}

/// Converts a [`Vec3`] into a [`LinMom`].
impl From<Vec3> for LinMom {
    #[inline]
    #[must_use]
    fn from(value: Vec3) -> Self {
        Self::from_vec3(value)
    }
}

/// Converts a [`LinMom`] into a [`Vec3`].
impl From<LinMom> for Vec3 {
    #[inline]
    #[must_use]
    fn from(value: LinMom) -> Self {
        value.0
    }
}

/// Extracts linear momentum from a [`Momentum`].
impl From<Momentum> for LinMom {
    #[inline]
    #[must_use]
    fn from(value: Momentum) -> Self {
        value.linear
    }
}

/// Sums an iterator of [`LinMom`] values.
impl Sum for LinMom {
    #[inline]
    #[must_use]
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::ZERO, |a, b| a + b)
    }
}

overload!((a: ?LinMom) + (b: ?LinMom) -> LinMom{ LinMom( a.0 + b.0 ) });
overload!((a: ?LinMom) - (b: ?LinMom) -> LinMom{ LinMom( a.0 - b.0 ) });
overload!((a: &mut LinMom) += (b: ?LinMom) { a.0 += b.0 });
overload!((a: &mut LinMom) -= (b: ?LinMom) { a.0 -= b.0 });

overload!((a: ?LinMom) / (b: ?Mass) -> LinVel{ LinVel( a.0 / b.0 ) });

overload!((a: ?LinMom) * (b: f64) -> LinMom{ LinMom( a.0 * b ) });
overload!((a: ?LinMom) / (b: f64) -> LinMom{ LinMom( a.0 / b ) });
overload!((a: &mut LinMom) *= (b: f64) { a.0 *= b });
overload!((a: &mut LinMom) /= (b: f64) { a.0 /= b });

overload!(-(a: ?LinMom) -> LinMom{ LinMom( -a.0 ) });
