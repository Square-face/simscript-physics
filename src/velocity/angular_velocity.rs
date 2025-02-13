use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngVel (pub Vec3);

overload!((a: ?AngVel) + (b: ?AngVel) -> AngVel{ AngVel( a.0 + b.0 ) });
overload!((a: ?AngVel) - (b: ?AngVel) -> AngVel{ AngVel( a.0 - b.0 ) });
overload!((a: &mut AngVel) += (b: ?AngVel) { a.0 += b.0 });
overload!((a: &mut AngVel) -= (b: ?AngVel) { a.0 -= b.0 });

overload!((a: ?AngVel) * (b: f64) -> AngVel{ AngVel( a.0 * b ) });
overload!((a: &mut AngVel) *= (b: f64) { a.0 *= b });

overload!(-(a: ?AngVel) -> AngVel{ AngVel( -a.0 ) });
