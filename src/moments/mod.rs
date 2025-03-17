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
pub struct Moment {
    pub force: Force,
    pub torque: Torque,
}

impl Moment {
    pub const ZERO: Self = Self::new(Force::ZERO, Torque::ZERO);

    pub const fn new(force: Force, torque: Torque) -> Self {
        Self { force, torque }
    }

    pub const fn from_force(force: Force) -> Self {
        Self::new(force, Torque::ZERO)
    }

    pub fn from_force_and_offset(force: Force, offset: Vec3) -> Self {
        let torque = Torque::new(offset.cross(force.0));
        Self::new(force, torque)
    }

    pub fn magnitude(&self) -> f64 {
        (self.force.0.length_squared() * self.torque.0.length_squared()).sqrt()
    }
}

overload!((a: ?Moment) + (b: ?Moment) -> Moment{ Moment::new( a.force + b.force, a.torque + b.torque ) });
overload!((a: ?Moment) - (b: ?Moment) -> Moment{ Moment::new( a.force - b.force, a.torque - b.torque ) });
overload!((a: &mut Moment) += (b: ?Moment) { a.force += b.force; a.torque += b.torque; });
overload!((a: &mut Moment) -= (b: ?Moment) { a.force -= b.force; a.torque -= b.torque; });

overload!((a: ?Moment) * (b: f64) -> Moment{ Moment::new(a.force * b, a.torque * b) });
overload!((a: ?Moment) / (b: f64) -> Moment{ Moment::new(a.force / b, a.torque * b) });
overload!((a: &mut Moment) *= (b: f64) { a.force *= b; a.torque *= b; });
overload!((a: &mut Moment) /= (b: f64) { a.force /= b; a.torque /= b; });

overload!(-(a: ?Moment) -> Moment{ Moment::new(-a.force, -a.torque) });

overload!((a: ?Moment) * (b: Duration) -> Momentum{ Momentum::new(a.force * b, a.torque * b) });
