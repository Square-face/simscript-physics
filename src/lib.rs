use std::time::Duration;

use inertia_mass::InnertiaMass;
use momentum::Momentum;
use transform::Transform;

pub mod inertia_mass;
pub mod momentum;
pub mod transform;
pub mod velocity;

/// Represents the kinetic state of a simulated entity
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub mass: InnertiaMass,
    pub transform: Transform,
    pub momentum: Momentum,
}

impl State {
    pub const fn new(mass: InnertiaMass, transform: Transform, momentum: Momentum) -> Self {
        Self {
            mass,
            transform,
            momentum,
        }
    }

    pub const fn new_stationary(mass: InnertiaMass, transform: Transform) -> Self {
        Self::new(mass, transform, Momentum::ZERO)
    }

    pub const fn new_zeroed(mass: InnertiaMass) -> Self {
        Self::new(mass, Transform::ZERO, Momentum::ZERO)
    }

    /// Steps the state forward by a [Duration]
    ///
    /// Updates the translation and rotation of the simulated entity by using its stored momentum and
    /// mass
    pub fn step_movement(&mut self, time: Duration) {
        // Both linear and angular velocity can be calculated by dividing [Momentum]
        // with [InnertiaMass]
        let velocity = self.momentum / self.mass;
        self.transform += velocity * time;
    }
}
