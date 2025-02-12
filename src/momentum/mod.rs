use glam::DVec3 as Vec3;

pub mod linear_momentum;
pub mod angular_momentum;

/// Represents the linear momentum an object has in all cardinal directions
///
/// While no unit is used explicitly, it is recomended to use this struct as if it is represented
/// in Ns (NewtonSeconds)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinMom(pub Vec3);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngMom(pub Vec3);
