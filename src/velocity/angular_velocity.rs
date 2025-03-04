use approx::{AbsDiffEq, RelativeEq, UlpsEq};
use approx_derive::Approx;
use glam::{DQuat as Quat, DVec3 as Vec3};
use overload::overload;
use std::{ops, time::Duration};

use crate::transform::Rotation;

#[derive(Debug, Default, Clone, Copy, PartialEq, Approx)]
pub struct AngVel(pub Vec3);

impl AngVel {
    pub const ZERO: Self = Self::from_vec3(Vec3::ZERO);
    pub const X: Self = Self::from_vec3(Vec3::X);
    pub const Y: Self = Self::from_vec3(Vec3::Y);
    pub const Z: Self = Self::from_vec3(Vec3::Z);

    pub const NEG_X: Self = Self::from_vec3(Vec3::NEG_X);
    pub const NEG_Y: Self = Self::from_vec3(Vec3::NEG_Y);
    pub const NEG_Z: Self = Self::from_vec3(Vec3::NEG_Z);

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

    pub const fn from_vec3(v: Vec3) -> Self {
        Self(v)
    }

    pub fn mul_secs(&self, secs: f64) -> Rotation {
        let delta = self.0 * secs;
        let rotation = Quat::from_scaled_axis(delta);

        Rotation(rotation)
    }
}

impl From<Vec3> for AngVel {
    fn from(value: Vec3) -> Self {
        Self::from_vec3(value)
    }
}

impl From<AngVel> for Vec3 {
    fn from(value: AngVel) -> Self {
        value.0
    }
}

overload!((a: ?AngVel) + (b: ?AngVel) -> AngVel{ AngVel( a.0 + b.0 ) });
overload!((a: ?AngVel) - (b: ?AngVel) -> AngVel{ AngVel( a.0 - b.0 ) });
overload!((a: &mut AngVel) += (b: ?AngVel) { a.0 += b.0 });
overload!((a: &mut AngVel) -= (b: ?AngVel) { a.0 -= b.0 });

overload!((a: ?AngVel) * (b: ?Duration) -> Rotation{ a.mul_secs(b.as_secs_f64()) });

overload!((a: ?AngVel) * (b: f64) -> AngVel{ AngVel( a.0 * b ) });
overload!((a: ?AngVel) / (b: f64) -> AngVel{ AngVel( a.0 / b ) });
overload!((a: &mut AngVel) *= (b: f64) { a.0 *= b });
overload!((a: &mut AngVel) /= (b: f64) { a.0 /= b });

overload!(-(a: ?AngVel) -> AngVel{ AngVel( -a.0 ) });

#[cfg(test)]
mod constructors {
    use approx::assert_ulps_eq;

    use super::*;

    #[test]
    fn new_splat_fromvec3() {
        assert_ulps_eq!(
            AngVel::new(64.54, 77.86, 2.77).0,
            Vec3::new(64.54, 77.86, 2.77)
        );
        assert_ulps_eq!(AngVel::splat(61.07).0, Vec3::new(61.07, 61.07, 61.07));

        let v = Vec3::new(12.57, 22.66, 58.90);
        assert_ulps_eq!(AngVel::from_vec3(v).0, v);
    }

    #[test]
    fn with() {
        assert_ulps_eq!(AngVel::with_x(66.60).0, Vec3::new(66.60, 0., 0.));
        assert_ulps_eq!(AngVel::with_y(76.75).0, Vec3::new(0., 76.75, 0.));
        assert_ulps_eq!(AngVel::with_z(21.58).0, Vec3::new(0., 0., 21.58));
    }

    #[test]
    fn consts() {
        assert_ulps_eq!(AngVel::ZERO, AngVel::splat(0.));

        assert_ulps_eq!(AngVel::X, AngVel::with_x(1.));
        assert_ulps_eq!(AngVel::Y, AngVel::with_y(1.));
        assert_ulps_eq!(AngVel::Z, AngVel::with_z(1.));

        assert_ulps_eq!(AngVel::NEG_X, AngVel::with_x(-1.));
        assert_ulps_eq!(AngVel::NEG_Y, AngVel::with_y(-1.));
        assert_ulps_eq!(AngVel::NEG_Z, AngVel::with_z(-1.));
    }
}

#[cfg(test)]
mod arithmetic {
    use super::*;
    use approx::assert_ulps_eq;

    #[cfg(test)]
    mod same_type {
        use super::*;

        #[test]
        fn add() {
            let av1 = AngVel::new(1.0, 2.0, 3.0);
            let av2 = AngVel::new(4.0, 5.0, 6.0);
            let result = av1 + av2;
            assert_ulps_eq!(result, AngVel::new(5.0, 7.0, 9.0));
        }

        #[test]
        fn sub() {
            let av1 = AngVel::new(5.0, 7.0, 9.0);
            let av2 = AngVel::new(1.0, 2.0, 3.0);
            let result = av1 - av2;
            assert_ulps_eq!(result, AngVel::new(4.0, 5.0, 6.0));
        }

        #[test]
        fn neg() {
            let av = AngVel::new(33.50, -88.84, 75.72);
            let result = -av;
            assert_ulps_eq!(result, AngVel::new(-33.50, 88.84, -75.72));
        }
    }

    #[cfg(test)]
    mod scalar {
        use super::*;

        #[test]
        fn mul() {
            let av = AngVel::new(1.0, -2.0, 3.0);
            assert_ulps_eq!(av * 2.0, AngVel::new(2.0, -4.0, 6.0));
        }

        #[test]
        fn div() {
            let av = AngVel::new(9.0, -3.0, 3.0);
            assert_ulps_eq!(av / 3.0, AngVel::new(3.0, -1.0, 1.0));
        }
    }

    #[cfg(test)]
    mod time {
        use super::*;

        #[test]
        fn mul() {
            let av = AngVel::new(84.48, 48.38, 25.55);
            let dur = Duration::from_secs_f64(1.6);
            assert_ulps_eq!(av * dur, av.mul_secs(1.6));
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

            let l1: AngVel = v1.into();
            let l2: AngVel = v2.into();
            let l3: AngVel = v3.into();

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }

        #[test]
        fn linvel_into_vec3() {
            let l1 = AngVel::new(15.98, 43.88, 47.36);
            let l2 = AngVel::new(75.68, 91.02, 12.66);
            let l3 = AngVel::new(10.82, 54.16, 52.37);

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

            let l1 = AngVel::from(v1);
            let l2 = AngVel::from(v2);
            let l3 = AngVel::from(v3);

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }

        #[test]
        fn linvel_from_vec3() {
            let l1 = AngVel::new(66.96, 23.12, 49.18);
            let l2 = AngVel::new(16.67, 40.61, 19.60);
            let l3 = AngVel::new(61.88, 12.68, 26.26);

            let v1 = Vec3::from(l1);
            let v2 = Vec3::from(l2);
            let v3 = Vec3::from(l3);

            assert_ulps_eq!(l1.0, v1);
            assert_ulps_eq!(l2.0, v2);
            assert_ulps_eq!(l3.0, v3);
        }
    }
}

#[cfg(test)]
mod to_rotation {
    use std::f64::consts::PI;

    use super::*;
    use approx::assert_ulps_eq;

    #[test]
    fn no_time() {
        let av = AngVel::X;
        let dur = Duration::ZERO;
        let res = av * dur;
        let expected = Quat::IDENTITY;
        assert_ulps_eq!(res.0, expected);
    }

    #[test]
    fn single_axis() {
        let ax = AngVel::X;
        let ay = AngVel::Y;
        let az = AngVel::Z;

        let ex = Quat::from_rotation_x(1.);
        let ey = Quat::from_rotation_y(1.);
        let ez = Quat::from_rotation_z(1.);

        assert_ulps_eq!(ax.mul_secs(1.).0, ex);
        assert_ulps_eq!(ay.mul_secs(1.).0, ey);
        assert_ulps_eq!(az.mul_secs(1.).0, ez);

        assert_ulps_eq!(ax.mul_secs(2.).0, ex * ex);
        assert_ulps_eq!(ay.mul_secs(2.).0, ey * ey);
        assert_ulps_eq!(az.mul_secs(2.).0, ez * ez);
    }

    #[test]
    fn compound_axis() {
        let axy: AngVel = Vec3::new(1., 1., 0.).normalize().into();
        let ayz: AngVel = Vec3::new(0., 1., 1.).normalize().into();
        let azx: AngVel = Vec3::new(1., 0., 1.).normalize().into();

        let ex = Quat::from_rotation_x(PI/2.);
        let ey = Quat::from_rotation_y(PI/2.);
        let ez = Quat::from_rotation_z(PI/2.);

        assert_ulps_eq!(axy.mul_secs(PI).0, ex * ey * ex);
        assert_ulps_eq!(ayz.mul_secs(PI).0, ey * ez * ey);
        assert_ulps_eq!(azx.mul_secs(PI).0, ez * ex * ez);
    }
}
