use overload::overload;
use std::ops;

use angular_momentum::AngMom;
use linear_momentum::LinMom;

pub mod angular_momentum;
pub mod linear_momentum;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Momentum {
    pub linear: LinMom,
    pub rotation: AngMom,
}

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
