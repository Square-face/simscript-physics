#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
#[cfg(feature = "approx")]
use approx_derive::Approx;

use glam::DVec3 as Vec3;
use overload::overload;
use std::{ops, time::Duration};

pub use force::Force;
pub use torque::Torque;

use crate::momentum::Momentum;

mod force;
mod torque;

#[cfg_attr(feature = "approx", derive(Approx))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Moment(pub Vec3, pub Vec3);

impl Moment {
    pub const ZERO: Self = Self::new(Vec3::ZERO, Vec3::ZERO);

    pub const fn new(offset: Vec3, force: Vec3) -> Self {
        Self(offset, force)
    }

    pub const fn from_force(force: Vec3) -> Self {
        Self(Vec3::ZERO, force)
    }

    pub const fn force(&self) -> Force {
        Force::new(self.1)
    }

    pub fn torque(&self) -> Torque {
        Torque::new(self.0.cross(self.1))
    }

    pub fn magnitude(&self) -> f64 {
        (self.0.length_squared() * self.1.length_squared()).sqrt()
    }
}

overload!((a: ?Moment) + (b: ?Moment) -> Moment{ Moment::new( a.0 + b.0, a.1 + b.1 ) });
overload!((a: ?Moment) - (b: ?Moment) -> Moment{ Moment::new( a.0 - b.0, a.1 - b.1 ) });
overload!((a: &mut Moment) += (b: ?Moment) { a.0 += b.0; a.1 += b.1; });
overload!((a: &mut Moment) -= (b: ?Moment) { a.0 -= b.0; a.1 -= b.1; });

overload!((a: ?Moment) * (b: f64) -> Moment{ Moment::new(a.0 * b, a.1 * b) });
overload!((a: ?Moment) / (b: f64) -> Moment{ Moment::new(a.0 / b, a.1 * b) });
overload!((a: &mut Moment) *= (b: f64) { a.0 *= b; a.1 *= b; });
overload!((a: &mut Moment) /= (b: f64) { a.0 /= b; a.1 /= b; });

overload!(-(a: ?Moment) -> Moment{ Moment::new(-a.0, -a.1) });

overload!((a: ?Moment) * (b: Duration) -> Momentum{ Momentum::new(a.force() * b, a.torque() * b) });
