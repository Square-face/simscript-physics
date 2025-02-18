#![allow(clippy::suspicious_op_assign_impl)]
#![allow(clippy::suspicious_arithmetic_impl)]

use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::DQuat as Quat;
use overload::overload;
use std::ops;

#[derive(Debug, Clone, Copy, PartialEq, Approx)]
pub struct AngMove(pub Quat);

impl AngMove {
    pub const ZERO: Self = Self::new(Quat::IDENTITY);

    /// Create a new [AngMove] with the given starting rotation
    pub const fn new(ang: Quat) -> Self {
        Self(ang)
    }
}

overload!((a: ?AngMove) + (b: ?AngMove) -> AngMove{ AngMove( a.0 * b.0 ) });
overload!((a: ?AngMove) - (b: ?AngMove) -> AngMove{ AngMove( a.0 * (-b).0) });
overload!((a: &mut AngMove) += (b: ?AngMove) { a.0 *= b.0 });
overload!((a: &mut AngMove) -= (b: ?AngMove) { a.0 *= (-b).0 });

overload!(-(a: ?AngMove) -> AngMove{ AngMove(a.0.inverse()) });
