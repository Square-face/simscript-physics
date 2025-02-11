use glam::{DMat3, DQuat, DVec3};

pub struct Mass {
    pub mass: f64,
    pub inertia: DMat3,
    pub inv_inertia: DMat3,
}

pub struct Position {
    pub translation: DVec3,
    pub rotation: DQuat,
}

pub struct Momentum {
    pub linear: DVec3,
    pub rotation: DVec3,
}

pub struct State {
    pub mass: Mass,
    pub position: Position,
    pub momentum: Momentum,
}
