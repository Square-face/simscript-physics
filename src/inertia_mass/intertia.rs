use glam::DMat3 as Mat3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inertia(pub Mat3);

impl Inertia {
    pub const fn new(inertia: Mat3) -> Self {
        Self(inertia)
    }
}

