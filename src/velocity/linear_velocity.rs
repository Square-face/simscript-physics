use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinVel(pub Vec3);

overload!((a: ?LinVel) + (b: ?LinVel) -> LinVel{ LinVel( a.0 + b.0 ) });
overload!((a: ?LinVel) - (b: ?LinVel) -> LinVel{ LinVel( a.0 - b.0 ) });
overload!((a: &mut LinVel) += (b: ?LinVel) { a.0 += b.0 });
overload!((a: &mut LinVel) -= (b: ?LinVel) { a.0 -= b.0 });

overload!((a: ?LinVel) * (b: f64) -> LinVel{ LinVel( a.0 * b ) });
overload!((a: &mut LinVel) *= (b: f64) { a.0 *= b });

overload!(-(a: ?LinVel) -> LinVel{ LinVel( -a.0 ) });
