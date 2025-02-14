use glam::{DQuat as Quat, DVec3 as Vec3, EulerRot};
use overload::overload;
use std::{ops, time::Duration};

use crate::position::AngMove;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngVel(pub Vec3);

impl AngVel {
    pub fn mul_time(&self, secs: f64) -> AngMove {
        let delta = self.0 * secs;
        let rotation = Quat::from_euler(EulerRot::YXZ, delta.x, delta.y, delta.z);

        AngMove(rotation)
    }
}

overload!((a: ?AngVel) + (b: ?AngVel) -> AngVel{ AngVel( a.0 + b.0 ) });
overload!((a: ?AngVel) - (b: ?AngVel) -> AngVel{ AngVel( a.0 - b.0 ) });
overload!((a: &mut AngVel) += (b: ?AngVel) { a.0 += b.0 });
overload!((a: &mut AngVel) -= (b: ?AngVel) { a.0 -= b.0 });

overload!((a: ?AngVel) * (b: ?Duration) -> AngMove{ a.mul_time(b.as_secs_f64()) });

overload!((a: ?AngVel) * (b: f64) -> AngVel{ AngVel( a.0 * b ) });
overload!((a: &mut AngVel) *= (b: f64) { a.0 *= b });

overload!(-(a: ?AngVel) -> AngVel{ AngVel( -a.0 ) });
