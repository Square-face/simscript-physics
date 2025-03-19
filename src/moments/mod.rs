#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "approx")]
use {
    approx::{AbsDiffEq, RelativeEq, UlpsEq},
    approx_derive::Approx,
};

use crate::momentum::Momentum;
use glam::DVec3 as Vec3;
use overload::overload;
use std::{iter::Sum, ops, time::Duration};

pub use force::Force;
pub use torque::Torque;

mod force;
mod torque;

/// Represents a moment with force and torque components.
///
/// Encapsulates translational force [Moment::force] and rotational torque
/// [Moment::torque] for a strongly typed representation of moment.
#[cfg_attr(feature = "approx", derive(Approx))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Moment {
    /// Linear force component.
    pub force: Force,
    /// Angular torque component.
    pub torque: Torque,
}

impl Moment {
    /// Zero moment constant (no force or torque).
    pub const ZERO: Self = Self::new(Force::ZERO, Torque::ZERO);

    /// Constructs a new [Moment] from given force and torque.
    #[inline]
    #[must_use]
    pub const fn new(force: Force, torque: Torque) -> Self {
        Self { force, torque }
    }

    /// Creates a moment with only a force component.
    #[inline]
    #[must_use]
    pub const fn from_force(force: Force) -> Self {
        Self::new(force, Torque::ZERO)
    }

    /// Creates a moment with only a torque component.
    #[inline]
    #[must_use]
    pub const fn from_torque(torque: Torque) -> Self {
        Self::new(Force::ZERO, torque)
    }

    /// Constructs a [Moment] from raw vector representations of force and torque.
    #[inline]
    #[must_use]
    pub const fn from_vec3s(force: Vec3, torque: Vec3) -> Self {
        Self::new(Force::from_vec3(force), Torque::from_vec3(torque))
    }

    /// Constructs a [Moment] from a raw vector representing only force.
    #[inline]
    #[must_use]
    pub const fn from_force_vec3(v: Vec3) -> Self {
        Self::from_vec3s(v, Vec3::ZERO)
    }

    /// Constructs a [Moment] from a raw vector representing only torque.
    #[inline]
    #[must_use]
    pub const fn from_torque_vec3(v: Vec3) -> Self {
        Self::from_vec3s(Vec3::ZERO, v)
    }

    /// Constructs a [Moment] from a force and an offset, computing torque as their cross product.
    #[inline]
    #[must_use]
    pub fn from_force_and_offset(force: Force, offset: Vec3) -> Self {
        let torque = Torque::from_vec3(offset.cross(force.0));
        Self::new(force, torque)
    }

    /// Returns the magnitude of the moment (square root of force and torque squared lengths product).
    #[inline]
    #[must_use]
    pub fn magnitude(&self) -> f64 {
        (self.force.0.length_squared() * self.torque.0.length_squared()).sqrt()
    }
}

impl Moment {
    /// Scales the moment by a time duration in seconds, returning a [Momentum].
    #[inline]
    #[must_use]
    pub fn mul_secs(&self, rhs: f64) -> Momentum {
        Momentum::new(self.force.mul_secs(rhs), self.torque.mul_secs(rhs))
    }

    /// Scales the moment by a [Duration], returning a [Momentum].
    #[inline]
    #[must_use]
    pub fn mul_dur(&self, rhs: &Duration) -> Momentum {
        self.mul_secs(rhs.as_secs_f64())
    }
}

/// Conversion implementations to create [Moment] from individual components.
impl From<Force> for Moment {
    #[inline]
    #[must_use]
    fn from(value: Force) -> Self {
        Self::from_force(value)
    }
}

impl From<Torque> for Moment {
    #[inline]
    #[must_use]
    fn from(value: Torque) -> Self {
        Self::from_torque(value)
    }
}

/// Implements summation over an iterator of [Moment] values.
impl Sum for Moment {
    #[inline]
    #[must_use]
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::ZERO, |a, b| a + b)
    }
}

overload!((a: ?Moment) + (b: ?Moment) -> Moment { Moment::new(a.force + b.force, a.torque + b.torque) });
overload!((a: ?Moment) - (b: ?Moment) -> Moment { Moment::new(a.force - b.force, a.torque - b.torque) });
overload!((a: &mut Moment) += (b: ?Moment) { a.force += b.force; a.torque += b.torque; });
overload!((a: &mut Moment) -= (b: ?Moment) { a.force -= b.force; a.torque -= b.torque; });

overload!((a: ?Moment) * (b: f64) -> Moment { Moment::new(a.force * b, a.torque * b) });
overload!((a: ?Moment) / (b: f64) -> Moment { Moment::new(a.force / b, a.torque / b) });
overload!((a: &mut Moment) *= (b: f64) { a.force *= b; a.torque *= b; });
overload!((a: &mut Moment) /= (b: f64) { a.force /= b; a.torque /= b; });

overload!(-(a: ?Moment) -> Moment { Moment::new(-a.force, -a.torque) });

overload!((a: ?Moment) * (b: Duration) -> Momentum { a.mul_dur(&b) });
overload!((a: ?Moment) * (b: &Duration) -> Momentum { a.mul_dur(b) });
