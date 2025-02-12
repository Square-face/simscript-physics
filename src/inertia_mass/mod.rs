use glam::DMat3 as Mat3;

pub mod mass;
pub mod intertia;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mass(pub f64);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inertia(pub Mat3);
