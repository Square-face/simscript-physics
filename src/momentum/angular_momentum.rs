#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
#[cfg(feature = "approx")]
use derives::Approx;

use glam::DVec3 as Vec3;
use overload::overload;
use std::iter::Sum;
use std::ops;

use crate::{inertia_mass::Inertia, linear_trait::Vec3Wrap, velocity::AngVel};

use super::Momentum;

/// Angular momentum struct with a 3D vector.
#[cfg_attr(feature = "approx", derive(Approx))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct AngMom(pub Vec3);

impl Vec3Wrap for AngMom {
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
