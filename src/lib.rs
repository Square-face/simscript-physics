use std::time::Duration;

use inertia_mass::InertiaMass;
use momentum::Momentum;
use transform::Transform;

pub mod inertia_mass;
pub mod moments;
pub mod momentum;
pub mod panels;
pub mod transform;
pub mod velocity;

/// Represents the kinetic state of a simulated entity
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub mass: InertiaMass,
    pub transform: Transform,
    pub momentum: Momentum,
}

impl State {
    pub const fn new(mass: InertiaMass, transform: Transform, momentum: Momentum) -> Self {
        Self {
            mass,
            transform,
            momentum,
        }
    }

    pub const fn new_stationary(mass: InertiaMass, transform: Transform) -> Self {
        Self::new(mass, transform, Momentum::ZERO)
    }

    pub const fn new_zeroed(mass: InertiaMass) -> Self {
        Self::new(mass, Transform::ZERO, Momentum::ZERO)
    }

    /// Steps the state forward by a [Duration]
    ///
    /// Updates the translation and rotation of the simulated entity by using its stored momentum and
    /// mass
    pub fn step_movement(&mut self, time: Duration) {
        let velocity = self.momentum / self.mass.rotated(self.transform.rotation.0);
        self.transform += velocity * time;
    }
}
