use mass::{Innertia, Mass};
use momentum::{AngMom, LinMom};
use position::{LinMove, AngMove};

mod mass;
mod momentum;
mod position;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InnertiaMass {
    pub mass: Mass,
    pub inertia: Innertia,
    pub inv_inertia: Innertia,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position {
    pub translation: LinMove,
    pub rotation: AngMove,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Momentum {
    pub linear: LinMom,
    pub rotation: AngMom,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub mass: InnertiaMass,
    pub position: Position,
    pub momentum: Momentum,
}
