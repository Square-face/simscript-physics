use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinMove(pub Vec3);

impl LinMove {
    pub const ZERO: Self = Self::new(Vec3::ZERO);

    /// Create a new [LinMove] with the given movement
    pub const fn new(mom: Vec3) -> Self {
        Self(mom)
    }
}

overload!((a: ?LinMove) + (b: ?LinMove) -> LinMove{ LinMove( a.0 + b.0 ) });
overload!((a: ?LinMove) - (b: ?LinMove) -> LinMove{ LinMove( a.0 - b.0 ) });
overload!((a: &mut LinMove) += (b: ?LinMove) { a.0 += b.0 });
overload!((a: &mut LinMove) -= (b: ?LinMove) { a.0 -= b.0 });

overload!((a: ?LinMove) * (b: f64) -> LinMove{ LinMove( a.0 * b ) });
overload!((a: &mut LinMove) *= (b: f64) { a.0 *= b });

overload!(-(a: ?LinMove) -> LinMove{ LinMove( -a.0 ) });
