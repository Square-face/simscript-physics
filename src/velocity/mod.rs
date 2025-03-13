#[cfg(feature = "approx")]
use approx::{AbsDiffEq, RelativeEq, UlpsEq};
#[cfg(feature = "approx")]
use approx_derive::Approx;

pub use angular_velocity::AngVel;
use glam::DVec3 as Vec3;
pub use linear_velocity::LinVel;
use overload::overload;
use std::{ops, time::Duration};

use crate::transform::Transform;

mod angular_velocity;
mod linear_velocity;

#[cfg_attr(feature = "approx", derive(Approx))]
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Velocity {
    pub linear: LinVel,
    pub angular: AngVel,
}

impl Velocity {
    pub const ZERO: Self = Self::new(LinVel::ZERO, AngVel::ZERO);

    pub const fn new(lin: LinVel, ang: AngVel) -> Self {
        Self {
            linear: lin,
            angular: ang,
        }
    }

    pub const fn from_lin(v: LinVel) -> Self {
        Self::new(v, AngVel::ZERO)
    }

    pub const fn from_ang(v: AngVel) -> Self {
        Self::new(LinVel::ZERO, v)
    }

    pub const fn from_vec3s(lin: Vec3, ang: Vec3) -> Self {
        Self::new(LinVel::from_vec3(lin), AngVel::from_vec3(ang))
    }
}

impl From<LinVel> for Velocity {
    fn from(value: LinVel) -> Self {
        Self::from_lin(value)
    }
}

impl From<AngVel> for Velocity {
    fn from(value: AngVel) -> Self {
        Self::from_ang(value)
    }
}

overload!((a: ?Velocity) + (b: ?Velocity) -> Velocity{ Velocity::new(a.linear + b.linear, a.angular + b.angular) });
overload!((a: ?Velocity) - (b: ?Velocity) -> Velocity{ Velocity::new(a.linear - b.linear, a.angular - b.angular) });
overload!((a: &mut Velocity) += (b: ?Velocity) { a.linear += b.linear; a.angular += b.angular; });
overload!((a: &mut Velocity) -= (b: ?Velocity) { a.linear -= b.linear; a.angular -= b.angular; });

overload!((a: ?Velocity) * (b: f64) -> Velocity{ Velocity::new(a.linear * b, a.angular * b) });
overload!((a: ?Velocity) / (b: f64) -> Velocity{ Velocity::new(a.linear / b, a.angular / b) });
overload!((a: &mut Velocity) *= (b: f64) { a.linear *= b; a.angular *= b; });
overload!((a: &mut Velocity) /= (b: f64) { a.linear /= b; a.angular /= b; });

overload!((a: ?Velocity) * (b: ?Duration) -> Transform{ Transform::new(a.linear * b, a.angular * b) });

overload!(-(a: ?Velocity) -> Velocity{Velocity{ linear: -a.linear, angular: -a.angular }});

#[cfg(test)]
mod constructors {
    use super::*;
    use approx::assert_ulps_eq;

    #[test]
    fn new() {
        let lv = LinVel::new(81.70, 43.59, 63.66);
        let av = AngVel::new(88.71, 60.05, 5.34);
        assert_ulps_eq!(Velocity::new(lv, av).linear, lv);
        assert_ulps_eq!(Velocity::new(lv, av).angular, av);
    }

    #[test]
    fn from() {
        let lv = LinVel::new(54.76, 11.08, 92.23);
        let av = AngVel::new(82.86, 5.21, 61.19);

        let v1 = Velocity::from_lin(lv);
        let v2 = Velocity::from_ang(av);

        assert_ulps_eq!(v1.linear, lv);
        assert_ulps_eq!(v1.angular, AngVel::ZERO);
        assert_ulps_eq!(v2.linear, LinVel::ZERO);
        assert_ulps_eq!(v2.angular, av);
    }

    #[test]
    fn consts() {
        assert_ulps_eq!(LinVel::ZERO.0, Vec3::new(0., 0., 0.));

        assert_ulps_eq!(LinVel::X.0, Vec3::new(1., 0., 0.));
        assert_ulps_eq!(LinVel::Y.0, Vec3::new(0., 1., 0.));
        assert_ulps_eq!(LinVel::Z.0, Vec3::new(0., 0., 1.));

        assert_ulps_eq!(LinVel::NEG_X.0, Vec3::new(-1., 0., 0.));
        assert_ulps_eq!(LinVel::NEG_Y.0, Vec3::new(0., -1., 0.));
        assert_ulps_eq!(LinVel::NEG_Z.0, Vec3::new(0., 0., -1.));
    }
}

#[cfg(test)]
mod arithmetic {
    use super::*;
    use approx::assert_ulps_eq;

    mod same_type {
        use super::*;

        #[test]
        fn add() {
            let a = Vec3::new(12.73, 74.02, 59.29);
            let b = Vec3::new(21.07, 7.92, 65.18);
            let c = Vec3::new(-86.97, -70.44, 30.48);
            let d = Vec3::new(-15.66, 69.47, 81.93);

            let v1 = Velocity::from_vec3s(a, b);
            let v2 = Velocity::from_vec3s(c, d);

            assert_ulps_eq!(v1 + v2, Velocity::from_vec3s(a + c, b + d));
            assert_ulps_eq!(v2 + v1, Velocity::from_vec3s(a + c, b + d));
        }

        #[test]
        fn sub() {
            let v1 = LinVel::splat(10.2);
            let v2 = LinVel::new(4.3, -12., 0.1);

            assert_ulps_eq!(v1 - v2, LinVel::new(5.9, 22.2, 10.1));
            assert_ulps_eq!(v2 - v1, LinVel::new(-5.9, -22.2, -10.1));
        }

        #[test]
        fn neg() {
            let v1 = LinVel::with_z(176.45);
            assert_ulps_eq!(-v1, LinVel::new(0., 0., -176.45));
        }
    }

    #[cfg(test)]
    mod scalar {
        use super::*;

        #[test]
        fn mul() {
            let v1 = LinVel::splat(42.);
            let v2 = LinVel::with_y(0.1);

            assert_ulps_eq!(v1 * 2., LinVel::splat(84.));
            assert_ulps_eq!(v2 * 10., LinVel::with_y(1.));
        }

        #[test]
        fn div() {
            let v1 = LinVel::splat(63.);
            let v2 = LinVel::with_y(1.0);

            assert_ulps_eq!(v1 / 3., LinVel::splat(21.));
            assert_ulps_eq!(v2 / 10., LinVel::with_y(0.1));
        }
    }

    #[cfg(test)]
    mod time {
        use super::*;

        #[test]
        fn mul() {
            let v = LinVel::new(83.7, 47.1, 139.2);
            let t = Duration::from_secs_f64(5.);
            assert_ulps_eq!(
                v * t,
                crate::transform::Translation::new(418.5, 235.5, 696.)
            );
        }
    }
}

#[cfg(test)]
mod traits {
    use super::*;

    #[cfg(test)]
    mod from {
        use approx::assert_ulps_eq;

        use super::*;

        #[test]
        fn vec3_into_linvel() {
            let v1 = Vec3::new(33.59, 89.08, 46.54);
            let v2 = Vec3::new(44.45, 30.98, 65.68);
            let v3 = Vec3::new(80.58, 35.28, 83.22);

            let l1: LinVel = v1.into();
            let l2: LinVel = v2.into();
            let l3: LinVel = v3.into();

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }

        #[test]
        fn linvel_into_vec3() {
            let l1 = LinVel::new(15.98, 43.88, 47.36);
            let l2 = LinVel::new(75.68, 91.02, 12.66);
            let l3 = LinVel::new(10.82, 54.16, 52.37);

            let v1: Vec3 = l1.into();
            let v2: Vec3 = l2.into();
            let v3: Vec3 = l3.into();

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }

        #[test]
        fn vec3_from_linvel() {
            let v1 = Vec3::new(78.40, 50.84, 98.58);
            let v2 = Vec3::new(18.03, 10.49, 45.86);
            let v3 = Vec3::new(25.11, 12.18, 27.54);

            let l1 = LinVel::from(v1);
            let l2 = LinVel::from(v2);
            let l3 = LinVel::from(v3);

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }

        #[test]
        fn linvel_from_vec3() {
            let l1 = LinVel::new(66.96, 23.12, 49.18);
            let l2 = LinVel::new(16.67, 40.61, 19.60);
            let l3 = LinVel::new(61.88, 12.68, 26.26);

            let v1 = Vec3::from(l1);
            let v2 = Vec3::from(l2);
            let v3 = Vec3::from(l3);

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }
    }
}
