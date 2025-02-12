use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

use crate::Momentum;

pub mod angular_momentum;
pub mod linear_momentum;

/// Represents the linear momentum an object has in all cardinal directions
///
/// While no unit is used explicitly, it is recomended to use this struct as if it is represented
/// in Ns (NewtonSeconds)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinMom(pub Vec3);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngMom(pub Vec3);

impl Momentum {
    pub const fn new(lin: LinMom, ang: AngMom) -> Self {
        Self {
            linear: lin,
            rotation: ang,
        }
    }
}

overload!((a: ?Momentum) + (b: ?Momentum) -> Momentum{Momentum{ linear: a.linear + b.linear, rotation: a.rotation + b.rotation }});
overload!((a: ?Momentum) - (b: ?Momentum) -> Momentum{Momentum{ linear: a.linear - b.linear, rotation: a.rotation - b.rotation }});

overload!((a: &mut Momentum) += (b: ?Momentum) { a.linear += b.linear; a.rotation += b.rotation; });
overload!((a: &mut Momentum) -= (b: ?Momentum) { a.linear -= b.linear; a.rotation -= b.rotation; });

overload!(-(a: ?Momentum) -> Momentum{Momentum{ linear: -a.linear, rotation: -a.rotation }});
