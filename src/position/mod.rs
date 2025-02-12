use glam::{DQuat, DVec3};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinMove(pub DVec3);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngMove(pub DQuat);
