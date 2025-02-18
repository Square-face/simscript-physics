use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

#[derive(Debug, Clone, Copy, PartialEq, Approx)]
pub struct LinMove(pub Vec3);

impl LinMove {
    pub const ZERO: Self = Self::splat(0.);

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vec3::new(x, y, z))
    }

    pub const fn splat(v: f64) -> Self {
        Self(Vec3::splat(v))
    }

    pub const fn with_x(x: f64) -> Self {
        Self(Vec3::new(x, 0., 0.))
    }

    pub const fn with_y(y: f64) -> Self {
        Self(Vec3::new(0., y, 0.))
    }

    pub const fn with_z(z: f64) -> Self {
        Self(Vec3::new(0., 0., z))
    }
}

overload!((a: ?LinMove) + (b: ?LinMove) -> LinMove{ LinMove( a.0 + b.0 ) });
overload!((a: ?LinMove) - (b: ?LinMove) -> LinMove{ LinMove( a.0 - b.0 ) });
overload!((a: &mut LinMove) += (b: ?LinMove) { a.0 += b.0 });
overload!((a: &mut LinMove) -= (b: ?LinMove) { a.0 -= b.0 });

overload!((a: ?LinMove) * (b: f64) -> LinMove{ LinMove( a.0 * b ) });
overload!((a: &mut LinMove) *= (b: f64) { a.0 *= b });

overload!(-(a: ?LinMove) -> LinMove{ LinMove( -a.0 ) });
