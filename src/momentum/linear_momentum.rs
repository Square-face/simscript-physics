use super::LinMom;
use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

impl LinMom {
    pub const ZERO: Self = Self::new(Vec3::ZERO);

    /// Create a new [LinMom] with the given starting momentum
    pub const fn new(mom: Vec3) -> Self {
        Self(mom)
    }
}

overload!((a: ?LinMom) + (b: ?LinMom) -> LinMom{ LinMom( a.0 + b.0 ) });
overload!((a: ?LinMom) - (b: ?LinMom) -> LinMom{ LinMom( a.0 - b.0 ) });
overload!((a: &mut LinMom) += (b: ?LinMom) { a.0 += b.0 });
overload!((a: &mut LinMom) -= (b: ?LinMom) { a.0 -= b.0 });

overload!((a: ?LinMom) * (b: f64) -> LinMom{ LinMom( a.0 * b ) });
overload!((a: &mut LinMom) *= (b: f64) { a.0 *= b });

overload!(-(a: ?LinMom) -> LinMom{ LinMom( -a.0 ) });
