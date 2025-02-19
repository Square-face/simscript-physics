use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::{DQuat as Quat, DVec3 as Vec3, EulerRot};
use overload::overload;
use std::{ops, time::Duration};

use crate::position::AngMove;

#[derive(Debug, Clone, Copy, PartialEq, Approx)]
pub struct AngVel(pub Vec3);

impl AngVel {
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

    pub fn mul_time(&self, secs: f64) -> AngMove {
        let delta = self.0 * secs;
        let rotation = Quat::from_euler(EulerRot::YXZ, delta.x, delta.y, delta.z);

        AngMove(rotation)
    }
}

overload!((a: ?AngVel) + (b: ?AngVel) -> AngVel{ AngVel( a.0 + b.0 ) });
overload!((a: ?AngVel) - (b: ?AngVel) -> AngVel{ AngVel( a.0 - b.0 ) });
overload!((a: &mut AngVel) += (b: ?AngVel) { a.0 += b.0 });
overload!((a: &mut AngVel) -= (b: ?AngVel) { a.0 -= b.0 });

overload!((a: ?AngVel) * (b: ?Duration) -> AngMove{ a.mul_time(b.as_secs_f64()) });

overload!((a: ?AngVel) * (b: f64) -> AngVel{ AngVel( a.0 * b ) });
overload!((a: &mut AngVel) *= (b: f64) { a.0 *= b });

overload!(-(a: ?AngVel) -> AngVel{ AngVel( -a.0 ) });

#[cfg(test)]
mod arithmetic {
    use super::*;

    #[test]
    fn test_add() {
        let av1 = AngVel(Vec3::new(1.0, 2.0, 3.0));
        let av2 = AngVel(Vec3::new(4.0, 5.0, 6.0));
        let result = av1 + av2;
        assert_eq!(result, AngVel(Vec3::new(5.0, 7.0, 9.0)));
    }

    #[test]
    fn test_sub() {
        let av1 = AngVel(Vec3::new(5.0, 7.0, 9.0));
        let av2 = AngVel(Vec3::new(1.0, 2.0, 3.0));
        let result = av1 - av2;
        assert_eq!(result, AngVel(Vec3::new(4.0, 5.0, 6.0)));
    }

    #[test]
    fn test_mul_by_scalar() {
        let av = AngVel(Vec3::new(1.0, -2.0, 3.0));
        let result = av * 2.0;
        assert_eq!(result, AngVel(Vec3::new(2.0, -4.0, 6.0)));
    }

    #[test]
    fn test_neg() {
        let av = AngVel(Vec3::new(1.0, -2.0, 3.0));
        let result = -av;
        assert_eq!(result, AngVel(Vec3::new(-1.0, 2.0, -3.0)));
    }
}

#[cfg(test)]
mod to_angmove {
    use approx::assert_ulps_eq;
    use super::*;

    #[test]
    fn test_mul_by_duration() {
        let av = AngVel(Vec3::new(1.0, 2.0, 3.0));
        let duration = Duration::from_secs_f64(2.0);
        let result = av * duration;
        let expected_rotation = Quat::from_euler(EulerRot::YXZ, 2.0, 4.0, 6.0);
        assert_ulps_eq!(result.0, expected_rotation);
    }
}

#[cfg(test)]
mod assign_arithmetic {
    use super::*;

    #[test]
    fn test_add_assign() {
        let mut av = AngVel(Vec3::new(1.0, 1.0, 1.0));
        av += AngVel(Vec3::new(2.0, 3.0, 4.0));
        assert_eq!(av, AngVel(Vec3::new(3.0, 4.0, 5.0)));
    }

    #[test]
    fn test_sub_assign() {
        let mut av = AngVel(Vec3::new(5.0, 5.0, 5.0));
        av -= AngVel(Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(av, AngVel(Vec3::new(4.0, 3.0, 2.0)));
    }

    #[test]
    fn test_mul_assign() {
        let mut av = AngVel(Vec3::new(2.0, 3.0, 4.0));
        av *= 2.0;
        assert_eq!(av, AngVel(Vec3::new(4.0, 6.0, 8.0)));
    }
}

