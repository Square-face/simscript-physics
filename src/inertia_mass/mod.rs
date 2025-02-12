use glam::DMat3 as Mat3;

use crate::InnertiaMass;

pub mod intertia;
pub mod mass;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mass(pub f64);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inertia(pub Mat3);

impl InnertiaMass {
    pub fn new(mass: Mass, inertia: Inertia) -> Self {
        Self {
            mass,
            inertia,
            inv_inertia: Inertia(inertia.0.inverse()),
        }
    }
}
