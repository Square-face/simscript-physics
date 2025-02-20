use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

pub use force::Force;
pub use torque::Torque;

mod force;
mod torque;

#[derive(Debug, Clone, Copy, PartialEq, Approx)]
pub struct Moment {
    pub offset: Vec3,
    pub force: Vec3,
}

impl Moment {
    pub const ZERO: Self = Self::new(Vec3::ZERO, Vec3::ZERO);

    pub const fn new(offset: Vec3, force: Vec3) -> Self {
        Self { offset, force }
    }

    pub const fn force(&self) -> Force {
        Force::new(self.force)
    }

    pub fn torque(&self) -> Torque {
        Torque::new(self.offset.cross(self.force))
    }
}

overload!((a: ?Moment) + (b: ?Moment) -> Moment{ Moment::new( a.offset + b.offset, a.force + b.force ) });
overload!((a: ?Moment) - (b: ?Moment) -> Moment{ Moment::new( a.offset - b.offset, a.force - b.force ) });
overload!((a: &mut Moment) += (b: ?Moment) { a.offset += b.offset; a.force += b.force; });
overload!((a: &mut Moment) -= (b: ?Moment) { a.offset -= b.offset; a.force -= b.force; });

overload!((a: ?Moment) * (b: f64) -> Moment{ Moment::new(a.offset * b, a.force * b) });
overload!((a: ?Moment) / (b: f64) -> Moment{ Moment::new(a.offset / b, a.force * b) });
overload!((a: &mut Moment) *= (b: f64) { a.offset *= b; a.force *= b; });
overload!((a: &mut Moment) /= (b: f64) { a.offset /= b; a.force /= b; });

overload!(-(a: ?Moment) -> Moment{ Moment::new(-a.offset, -a.force) });
