use glam::{DMat3 as Mat3, DQuat as Quat};
pub use intertia::Inertia;
pub use mass::Mass;

mod intertia;
mod mass;

/// Represents the mass and its distribution in an entity
///
/// # Units
/// mass: kg
/// inertia: tensor based on kg
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InnertiaMass {
    pub mass: Mass,
    pub inertia: Inertia,
    pub inv_inertia: Inertia,
}

impl InnertiaMass {
    pub fn new(mass: Mass, inertia: Inertia) -> Self {
        Self {
            mass,
            inertia,
            inv_inertia: Inertia(inertia.0.inverse()),
        }
    }

    pub fn rotated(&self, rot: Quat) -> Self {
        Self::new(self.mass, self.inertia.rotated(rot))
    }

    pub fn rot_mat(&self, rot: Mat3) -> Self {
        Self::new(self.mass, self.inertia.rot_mat(rot))
    }
}
