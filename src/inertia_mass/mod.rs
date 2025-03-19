use glam::{DMat3 as Mat3, DQuat as Quat};
pub use intertia::Inertia;
pub use mass::Mass;

mod intertia;
mod mass;

/// An entity's mass and how its distributed
///
/// Contains one [Mass] and two [Inertia], one of which is always inverted to eliminate the need
/// to recalculate every time the inverse tensor is required
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InertiaMass {
    pub mass: Mass,
    pub inertia: Inertia,
    pub inv_inertia: Inertia,
}

impl InertiaMass {
    pub fn new(mass: Mass, inertia: Inertia) -> Self {
        Self {
            mass,
            inertia,
            inv_inertia: Inertia(inertia.0.inverse()),
        }
    }

    /// Rotates the mass using a [Quat]
    ///
    /// If performance is critical, directly calling [InertiaMass::rot_mat] may be preferable
    pub fn rotated(&self, rot: Quat) -> Self {
        Self::new(self.mass, self.inertia.rotated(rot))
    }

    /// Rotates the mass using a [Mat3]
    pub fn rot_mat(&self, rot: Mat3) -> Self {
        Self::new(self.mass, self.inertia.rot_mat(rot))
    }
}
