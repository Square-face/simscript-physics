#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
#[cfg(feature = "approx")]
use approx_derive::Approx;

use glam::DVec3 as Vec3;
use overload::overload;
use std::{ops, time::Duration};

use crate::momentum::AngMom;

#[cfg_attr(feature = "approx", derive(Approx))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Torque(pub Vec3);

impl Torque {
    pub const ZERO: Self = Self::new(Vec3::ZERO);

    pub const fn new(force: Vec3) -> Self {
        Self(force)
    }

    pub const fn splat(f: f64) -> Self {
        Self(Vec3::splat(f))
    }
}

overload!((a: ?Torque) + (b: ?Torque) -> Torque{Torque::new(a.0 + b.0)});
overload!((a: ?Torque) - (b: ?Torque) -> Torque{Torque::new(a.0 - b.0)});
overload!((a: &mut Torque) += (b: ?Torque) {a.0 += b.0});
overload!((a: &mut Torque) -= (b: ?Torque) {a.0 -= b.0});

overload!((a: ?Torque) * (b: f64) -> Torque{Torque::new(a.0 * b)});
overload!((a: ?Torque) / (b: f64) -> Torque{Torque::new(a.0 / b)});
overload!((a: &mut Torque) *= (b: f64) {a.0 *= b});
overload!((a: &mut Torque) /= (b: f64) {a.0 /= b});

overload!(-(a: ?Torque) -> Torque{Torque::new(-a.0)});

overload!((a: ?Torque) * (b: ?Duration) -> AngMom{ AngMom(a.0 * b.as_secs_f64()) });
