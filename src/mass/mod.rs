use glam::DMat3 as Mat3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mass(pub f64);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Innertia(pub Mat3);
