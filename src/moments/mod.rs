use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DVec3 as Vec3;

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
