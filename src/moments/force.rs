#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
#[cfg(feature = "approx")]
use approx_derive::Approx;

use glam::DVec3 as Vec3;
use overload::overload;
use std::{ops, time::Duration};

use crate::momentum::LinMom;

#[cfg_attr(feature = "approx", derive(Approx))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
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

overload!((a: ?Force) * (b: ?Duration) -> LinMom{ LinMom(a.0 * b.as_secs_f64()) });
