use glam::DMat3 as Mat3;

use super::Inertia;


impl Inertia {
    pub const fn new(inertia: Mat3) -> Self {
        Self(inertia)
    }
}

