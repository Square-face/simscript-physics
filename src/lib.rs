use std::time::Duration;

use inertia_mass::InertiaMass;
use momentum::Momentum;
use panels::Panel;
use transform::Transform;

pub mod inertia_mass;
pub mod moments;
pub mod momentum;
pub mod panels;
pub mod transform;
pub mod velocity;

/// Represents the kinetic state of a simulated entity
#[derive(Debug, Clone, PartialEq)]
pub struct State {
    pub mass: InertiaMass,
    pub transform: Transform,
    pub momentum: Momentum,
    pub panels: Vec<Panel>,
}

impl State {
    pub fn new(
        mass: InertiaMass,
        transform: Transform,
        momentum: Momentum,
        panels: Vec<Panel>,
    ) -> Self {
        Self {
            mass,
            transform,
            momentum,
            panels,
        }
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
