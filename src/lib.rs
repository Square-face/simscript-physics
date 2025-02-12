use mass::{Innertia, Mass};
use momentum::{AngMom, LinMom};
use position::{LinMove, AngMove};

mod mass;
mod momentum;
mod position;

pub struct InnertiaMass {
    pub mass: Mass,
    pub inertia: Innertia,
    pub inv_inertia: Innertia,
}

pub struct Position {
    pub translation: LinMove,
    pub rotation: AngMove,
}

pub struct Momentum {
    pub linear: LinMom,
    pub rotation: AngMom,
}

pub struct State {
    pub mass: InnertiaMass,
    pub position: Position,
    pub momentum: Momentum,
}
