use glam::{DQuat as Quat, DVec3 as Vec3};

pub mod linear_movement;
pub mod angular_movement;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinMove(pub Vec3);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngMove(pub Quat);
