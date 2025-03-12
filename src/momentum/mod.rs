use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

pub use angular_momentum::AngMom;
pub use linear_momentum::LinMom;

use crate::{inertia_mass::InertiaMass, velocity::Velocity};

mod angular_momentum;
mod linear_momentum;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Momentum {
    pub linear: LinMom,
    pub rotation: AngMom,
}

impl Momentum {
    pub const ZERO: Self = Self::new(LinMom::ZERO, AngMom::ZERO);

    pub const fn new(lin: LinMom, ang: AngMom) -> Self {
        Self {
            linear: lin,
            rotation: ang,
        }
    }

    pub const fn from_lin(lin: Vec3) -> Self {
        Self::new(LinMom::new(lin), AngMom::ZERO)
    }

    pub const fn from_ang(ang: Vec3) -> Self {
        Self::new(LinMom::ZERO, AngMom::new(ang))
    }
}

overload!((a: ?Momentum) + (b: ?Momentum) -> Momentum{Momentum{ linear: a.linear + b.linear, rotation: a.rotation + b.rotation }});
overload!((a: ?Momentum) - (b: ?Momentum) -> Momentum{Momentum{ linear: a.linear - b.linear, rotation: a.rotation - b.rotation }});

overload!((a: &mut Momentum) += (b: ?Momentum) { a.linear += b.linear; a.rotation += b.rotation; });
overload!((a: &mut Momentum) -= (b: ?Momentum) { a.linear -= b.linear; a.rotation -= b.rotation; });

overload!((a: ?Momentum) * (b: f64) -> Momentum{Momentum{ linear: a.linear * b, rotation: a.rotation * b }});
overload!((a: ?Momentum) / (b: f64) -> Momentum{Momentum{ linear: a.linear / b, rotation: a.rotation / b }});

overload!((a: &mut Momentum) *= (b: f64) { a.linear *= b; a.rotation *= b; });
overload!((a: &mut Momentum) /= (b: f64) { a.linear /= b; a.rotation /= b; });

overload!((a: ?Momentum) / (b: ?InertiaMass) -> Velocity{ Velocity::new(a.linear / b.mass, a.rotation / b.inv_inertia) });

overload!(-(a: ?Momentum) -> Momentum{Momentum{ linear: -a.linear, rotation: -a.rotation }});
