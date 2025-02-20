use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DVec3 as Vec3;
use overload::overload;
use std::ops;

#[derive(Debug, Clone, Copy, PartialEq, Approx)]
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
