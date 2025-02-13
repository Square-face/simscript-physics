use std::time::Duration;

use inertia_mass::InnertiaMass;
use momentum::Momentum;
use position::Position;

mod inertia_mass;
mod momentum;
mod position;
mod velocity;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub mass: InnertiaMass,
    pub position: Position,
    pub momentum: Momentum,
}

impl State {
    pub fn step(&mut self, time: Duration) {
        let velocity = self.momentum / self.mass;
        self.position += velocity * time;
    }
}
