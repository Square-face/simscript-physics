use overload::overload;
use std::ops;

pub use angular_velocity::AngVel;
pub use linear_velocity::LinVel;

mod angular_velocity;
mod linear_velocity;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Velocity {
    pub linear: LinVel,
    pub angular: AngVel,
}

impl Velocity {
    pub fn new(lin: LinVel, ang: AngVel) -> Self {
        Self {
            linear: lin,
            angular: ang,
        }
    }
}

overload!((a: ?Velocity) + (b: ?Velocity) -> Velocity{Velocity{ linear: a.linear + b.linear, angular: a.angular + b.angular }});
overload!((a: ?Velocity) - (b: ?Velocity) -> Velocity{Velocity{ linear: a.linear - b.linear, angular: a.angular - b.angular }});

overload!((a: &mut Velocity) += (b: ?Velocity) { a.linear += b.linear; a.angular += b.angular; });
overload!((a: &mut Velocity) -= (b: ?Velocity) { a.linear -= b.linear; a.angular -= b.angular; });

overload!(-(a: ?Velocity) -> Velocity{Velocity{ linear: -a.linear, angular: -a.angular }});
