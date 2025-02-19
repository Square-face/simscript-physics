use std::time::Duration;

use inertia_mass::InnertiaMass;
use momentum::Momentum;
use position::Position;

pub mod inertia_mass;
pub mod momentum;
pub mod position;
pub mod velocity;

/// Represents the kinetic state of a simulated entity
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub mass: InnertiaMass,
    pub position: Position,
    pub momentum: Momentum,
}

impl State {
    pub const fn new(mass: InnertiaMass, position: Position, momentum: Momentum) -> Self {
        Self {
            mass,
            position,
            momentum,
        }
    }

    pub const fn new_stationary(mass: InnertiaMass, position: Position) -> Self {
        Self::new(mass, position, Momentum::ZERO)
    }

    pub const fn new_zeroed(mass: InnertiaMass) -> Self {
        Self::new(mass, Position::ZERO, Momentum::ZERO)
    }

    /// Steps the state forward by a [Duration]
    ///
    /// Updates the position and rotation of the simulated entity by using its stored momentum and
    /// mass
    pub fn step_movement(&mut self, time: Duration) {
        // Both linear and angular velocity can be calculated by dividing [Momentum]
        // with [InnertiaMass]
        let velocity = self.momentum / self.mass;
        self.position += velocity * time;
    }
}
