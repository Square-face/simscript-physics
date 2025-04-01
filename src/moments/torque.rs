#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
#[cfg(feature = "approx")]
use derives::Approx;

use glam::DVec3 as Vec3;
use overload::overload;
use std::{iter::Sum, ops, time::Duration};

use crate::momentum::AngMom;

/// Torque in 3D space.
///
/// This struct wraps a [Vec3] to provide a strongly typed representation of torque,
/// making operations and transformations explicit.
#[cfg_attr(feature = "approx", derive(Approx))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Torque(pub Vec3);

impl Torque {
    /// A zero torque vector.
    pub const ZERO: Self = Self::splat(0.);

    /// Torque of magnitude one in all directions.
    pub const ONE: Self = Self::splat(1.);

    /// Unit torque in the positive X direction.
    pub const X: Self = Self::with_x(1.);
    /// Unit torque in the positive Y direction.
    pub const Y: Self = Self::with_y(1.);
    /// Unit torque in the positive Z direction.
    pub const Z: Self = Self::with_z(1.);

    /// Unit torque in the negative X direction.
    pub const NEG_X: Self = Self::with_x(-1.);
    /// Unit torque in the negative Y direction.
    pub const NEG_Y: Self = Self::with_y(-1.);
    /// Unit torque in the negative Z direction.
    pub const NEG_Z: Self = Self::with_z(-1.);

    /// Creates a new [Torque] with the specified `x`, `y`, and `z` components.
    #[inline]
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vec3::new(x, y, z))
    }

    /// Creates a [Torque] from an existing [Vec3].
    #[inline]
    #[must_use]
    pub const fn from_vec3(v: Vec3) -> Self {
        Self(v)
    }

    /// Creates a [Torque] where all components are set to `v`.
    #[inline]
    #[must_use]
    pub const fn splat(v: f64) -> Self {
        Self::new(v, v, v)
    }

    /// Creates a [Torque] with only the X component set.
    #[inline]
    #[must_use]
    pub const fn with_x(x: f64) -> Self {
        Self::new(x, 0., 0.)
    }

    /// Creates a [Torque] with only the Y component set.
    #[inline]
    #[must_use]
    pub const fn with_y(y: f64) -> Self {
        Self::new(0., y, 0.)
    }

    /// Creates a [Torque] with only the Z component set.
    #[inline]
    #[must_use]
    pub const fn with_z(z: f64) -> Self {
        Self::new(0., 0., z)
    }
}

impl Torque {
    /// Scales the torque by a time duration in seconds, returning an [AngMom].
    #[inline]
    #[must_use]
    pub fn mul_secs(&self, rhs: f64) -> AngMom {
        AngMom(self.0 * rhs)
    }

    /// Scales the torque by a [Duration], returning an [AngMom].
    ///
    /// Note: this function uses [Torque::mul_secs] internally. For performance-critical code,
    /// consider using [Torque::mul_secs] directly to avoid [Duration::as_secs_f64] overhead.
    #[inline]
    #[must_use]
    pub fn mul_dur(&self, rhs: &Duration) -> AngMom {
        self.mul_secs(rhs.as_secs_f64())
    }
}

impl From<Vec3> for Torque {
    #[inline]
    #[must_use]
    fn from(value: Vec3) -> Self {
        Self::from_vec3(value)
    }
}

impl From<Torque> for Vec3 {
    #[inline]
    #[must_use]
    fn from(value: Torque) -> Self {
        value.0
    }
}

impl Sum for Torque {
    #[inline]
    #[must_use]
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::ZERO, |a, b| a + b)
    }
}

overload!((a: ?Torque) + (b: ?Torque) -> Torque { Torque(a.0 + b.0) });
overload!((a: ?Torque) - (b: ?Torque) -> Torque { Torque(a.0 - b.0) });
overload!((a: &mut Torque) += (b: ?Torque) { a.0 += b.0 });
overload!((a: &mut Torque) -= (b: ?Torque) { a.0 -= b.0 });

overload!((a: ?Torque) * (b: f64) -> Torque { Torque(a.0 * b) });
overload!((a: ?Torque) / (b: f64) -> Torque { Torque(a.0 / b) });
overload!((a: &mut Torque) *= (b: f64) { a.0 *= b });
overload!((a: &mut Torque) /= (b: f64) { a.0 /= b });

overload!((a: ?Torque) * (b: Duration) -> AngMom { a.mul_dur(&b) });
overload!((a: ?Torque) * (b: &Duration) -> AngMom { a.mul_dur(b) });

overload!(-(a: ?Torque) -> Torque { Torque(-a.0) });
