use glam::{DQuat, DVec3 as Vec3, EulerRot};
use overload::overload;
use std::{ops, time::Duration};

use crate::position::AngMove;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngVel(pub Vec3);

overload!((a: ?AngVel) + (b: ?AngVel) -> AngVel{ AngVel( a.0 + b.0 ) });
overload!((a: ?AngVel) - (b: ?AngVel) -> AngVel{ AngVel( a.0 - b.0 ) });
overload!((a: &mut AngVel) += (b: ?AngVel) { a.0 += b.0 });
overload!((a: &mut AngVel) -= (b: ?AngVel) { a.0 -= b.0 });

overload!((a: ?AngVel) * (b: ?Duration) -> AngMove{ let d = a.0*b.as_secs_f64(); AngMove(DQuat::from_euler(EulerRot::YXZ, d.x, d.y, d.z)) });

overload!((a: ?AngVel) * (b: f64) -> AngVel{ AngVel( a.0 * b ) });
overload!((a: &mut AngVel) *= (b: f64) { a.0 *= b });

overload!(-(a: ?AngVel) -> AngVel{ AngVel( -a.0 ) });
