#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
#[cfg(feature = "approx")]
use derives::Approx;

use glam::DVec3 as Vec3;
use overload::overload;
use std::{iter::Sum, ops};

use crate::{inertia_mass::Mass, linear_trait::Vec3Wrap, velocity::LinVel};

use super::{AngMom, Momentum};

/// Linear momentum struct with a 3D vector.
#[cfg_attr(feature = "approx", derive(Approx))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct LinMom(pub Vec3);

impl Vec3Wrap for LinMom {
    const ZERO: Self = Self(Vec3::ZERO);
    const ONE: Self = Self(Vec3::ONE);

    const X: Self = Self(Vec3::X);
    const Y: Self = Self(Vec3::Y);
    const Z: Self = Self(Vec3::Z);

    const NEG_X: Self = Self(Vec3::NEG_X);
    const NEG_Y: Self = Self(Vec3::NEG_Y);
    const NEG_Z: Self = Self(Vec3::NEG_Z);

    #[inline]
    #[must_use]
    fn from_vec3(v: Vec3) -> Self {
        Self(v)
    }
}

impl LinMom {
    /// Converts to a [Momentum] with zero angular momentum.
    #[inline]
    #[must_use]
    pub const fn to_vel(self) -> Momentum {
        Momentum::new(self, AngMom::ZERO)
    }

    /// Combines with angular momentum to create a [Momentum].
    #[inline]
    #[must_use]
    pub const fn with_angular(self, ang: AngMom) -> Momentum {
        Momentum::new(self, ang)
    }
}

/// Converts a [Vec3] into a [LinMom].
impl From<Vec3> for LinMom {
    #[inline]
    #[must_use]
    fn from(value: Vec3) -> Self {
        Self::from_vec3(value)
    }
}

/// Converts a [LinMom] into a [Vec3].
impl From<LinMom> for Vec3 {
    #[inline]
    #[must_use]
    fn from(value: LinMom) -> Self {
        value.0
    }
}

/// Extracts linear momentum from a [Momentum].
impl From<Momentum> for LinMom {
    #[inline]
    #[must_use]
    fn from(value: Momentum) -> Self {
        value.linear
    }
}

/// Sums an iterator of [LinMom] values.
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
