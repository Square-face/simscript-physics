use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

#[derive(Debug, Clone, Copy, PartialEq, Approx)]
pub struct Force(pub Vec3);

impl Force {
    pub const ZERO: Self = Self::new(Vec3::ZERO);

    pub const fn new(force: Vec3) -> Self {
        Self(force)
    }

    pub const fn splat(f: f64) -> Self {
        Self(Vec3::splat(f))
    }

    pub fn magnitue(&self) -> f64 {
        self.0.length()
    }
}

overload!((a: ?Force) + (b: ?Force) -> Force{Force::new(a.0 + b.0)});
overload!((a: ?Force) - (b: ?Force) -> Force{Force::new(a.0 - b.0)});
overload!((a: &mut Force) += (b: ?Force) {a.0 += b.0});
overload!((a: &mut Force) -= (b: ?Force) {a.0 -= b.0});

overload!((a: ?Force) * (b: f64) -> Force{Force::new(a.0 * b)});
overload!((a: ?Force) / (b: f64) -> Force{Force::new(a.0 / b)});
overload!((a: &mut Force) *= (b: f64) {a.0 *= b});
overload!((a: &mut Force) /= (b: f64) {a.0 /= b});

overload!(-(a: ?Force) -> Force{Force::new(-a.0)});
