use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

use crate::{inertia_mass::Mass, velocity::LinVel};

/// Represents the linear momentum an object has in all cardinal directions
///
/// While no unit is used explicitly, it is recommended to use this struct as if it is represented
/// in Ns (Newton-seconds)
#[derive(Debug, Clone, Copy, PartialEq, Approx)]
pub struct LinMom(pub Vec3);

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

overload!((a: ?LinMom) / (b: ?Mass) -> LinVel{ LinVel( a.0 / b.0 ) });

overload!((a: ?LinMom) * (b: f64) -> LinMom{ LinMom( a.0 * b ) });
overload!((a: ?LinMom) / (b: f64) -> LinMom{ LinMom( a.0 / b ) });
overload!((a: &mut LinMom) *= (b: f64) { a.0 *= b });
overload!((a: &mut LinMom) /= (b: f64) { a.0 /= b });

overload!(-(a: ?LinMom) -> LinMom{ LinMom( -a.0 ) });
