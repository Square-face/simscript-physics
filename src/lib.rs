use inertia_mass::InnertiaMass;
use momentum::Momentum;
use position::Position;

mod inertia_mass;
mod momentum;
mod position;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub mass: InnertiaMass,
    pub position: Position,
    pub momentum: Momentum,
}
