use glam::{DQuat as Quat, DVec3 as Vec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinMove(pub Vec3);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngMove(pub Quat);
