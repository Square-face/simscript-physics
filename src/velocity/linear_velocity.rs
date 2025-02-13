use glam::DVec3 as Vec3;
use overload::overload;
use std::{ops, time::Duration};

use crate::position::LinMove;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinVel(pub Vec3);

overload!((a: ?LinVel) + (b: ?LinVel) -> LinVel{ LinVel( a.0 + b.0 ) });
overload!((a: ?LinVel) - (b: ?LinVel) -> LinVel{ LinVel( a.0 - b.0 ) });
overload!((a: &mut LinVel) += (b: ?LinVel) { a.0 += b.0 });
overload!((a: &mut LinVel) -= (b: ?LinVel) { a.0 -= b.0 });

overload!((a: ?LinVel) * (b: f64) -> LinVel{ LinVel( a.0 * b ) });
overload!((a: &mut LinVel) *= (b: f64) { a.0 *= b });

overload!((a: ?LinVel) * (b: ?Duration) -> LinMove{ LinMove(a.0 * b.as_secs_f64()) });

overload!(-(a: ?LinVel) -> LinVel{ LinVel( -a.0 ) });
