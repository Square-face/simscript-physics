
use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngMom(pub Vec3);

impl AngMom {
    pub const ZERO: Self = Self::new(Vec3::ZERO);

    /// Create a new [AngMom] with the given starting momentum
    pub const fn new(mom: Vec3) -> Self {
        Self(mom)
    }
}

overload!((a: ?AngMom) + (b: ?AngMom) -> AngMom{ AngMom( a.0 + b.0 ) });
overload!((a: ?AngMom) - (b: ?AngMom) -> AngMom{ AngMom( a.0 - b.0 ) });
overload!((a: &mut AngMom) += (b: ?AngMom) { a.0 += b.0 });
overload!((a: &mut AngMom) -= (b: ?AngMom) { a.0 -= b.0 });

overload!((a: ?AngMom) * (b: f64) -> AngMom{ AngMom( a.0 * b ) });
overload!((a: &mut AngMom) *= (b: f64) { a.0 *= b });

overload!(-(a: ?AngMom) -> AngMom{ AngMom( -a.0 ) });
