use glam::{DQuat as Quat, DVec3 as Vec3};

use crate::{
    moments::{Force, Moment},
    velocity::{LinVel, Velocity},
    State,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Panel {
    pub offset: Vec3,
    pub normal: Vec3,
    pub area: f64,
}

const DENSITY: f64 = 1.293;
const HALF_C_D: f64 = 1.28 / 2.;

impl Panel {
    pub fn new(offset: Vec3, normal: Vec3, area: f64) -> Self {
        Self {
            offset,
            normal,
            area,
        }
    }

    pub fn to_force(&self, rel_vel: &LinVel) -> Force {
        let area = self.normal.dot(rel_vel.0.normalize_or_zero()) * self.area;
        let v_sq = rel_vel.0 * rel_vel.0;

        Force::new(DENSITY * -v_sq * HALF_C_D * area)
    }

    pub fn get_rel_vel(&self, rot: &Quat, vel: &Velocity) -> LinVel {
        let rotated = rot.mul_vec3(self.offset);
        let rot_vel = vel.angular.0.cross(rotated);
        LinVel(rot.mul_vec3(vel.linear.0) + rot_vel)
    }

    pub fn to_moment(&self, state: &State) -> Moment {
        let rot = state.transform.rotation.0;
        let vel = self.get_rel_vel(&rot, &(state.momentum / state.mass));

        let force = Force::new(rot.mul_vec3(self.to_force(&vel).0));


        Moment::new(rot.mul_vec3(self.offset), force.0)
    }
}

#[cfg(test)]
mod relative_velocity {
    use super::*;
    use crate::velocity::AngVel;
    use approx::assert_ulps_eq;
    use std::f64::consts::PI;

    #[test]
    fn stationary() {
        let panel = Panel::new(Vec3::X, Vec3::Y, 1.0);
        let rot = Quat::IDENTITY;
        let vel = Velocity::ZERO;

        assert_ulps_eq!(panel.get_rel_vel(&rot, &vel), LinVel::ZERO);
    }

    #[test]
    fn rotated() {
        let panel = Panel::new(Vec3::X, Vec3::Y, 1.0);
        let rot = Quat::from_rotation_x(PI / 2.);
        let vel = Velocity::ZERO;

        assert_ulps_eq!(panel.get_rel_vel(&rot, &vel), LinVel::ZERO);
    }

    #[test]
    fn moving() {
        let panel = Panel::new(Vec3::X, Vec3::Y, 1.0);
        let rot = Quat::IDENTITY;
        let vel = Velocity::new(LinVel::with_y(1.0), AngVel::ZERO);

        assert_ulps_eq!(panel.get_rel_vel(&rot, &vel), LinVel::with_y(1.0));
    }

    #[test]
    fn rotated_moving() {
        let panel = Panel::new(Vec3::X, Vec3::Y, 1.0);
        let rot = Quat::from_rotation_z(PI / 2.);
        let vel = Velocity::new(LinVel::with_y(1.0), AngVel::ZERO);

        assert_ulps_eq!(panel.get_rel_vel(&rot, &vel), LinVel::with_x(-1.0));
    }

    #[test]
    fn rotating() {
        let panel = Panel::new(Vec3::X, Vec3::Y, 1.0);
        let rot = Quat::IDENTITY;
        let vel = Velocity::new(LinVel::ZERO, AngVel::with_z(1.));

        assert_ulps_eq!(panel.get_rel_vel(&rot, &vel), LinVel::with_y(1.0));
    }
}

#[cfg(test)]
mod to_force {
    use std::f64::consts::PI;

    use super::*;
    use approx::assert_ulps_eq;

    const EXP: f64 = DENSITY * HALF_C_D;

    #[test]
    fn stationary() {
        let panel = Panel::new(Vec3::X, Vec3::Y, 1.);
        let vel = LinVel::ZERO;
        assert_ulps_eq!(panel.to_force(&vel), Force::ZERO);
    }

    #[test]
    fn parallel() {
        let p1 = Panel::new(Vec3::ZERO, Vec3::Y, 1.);

        let v1 = LinVel::with_z(1.);
        let v2 = LinVel::with_x(1.);

        assert_ulps_eq!(p1.to_force(&v1), Force::ZERO);
        assert_ulps_eq!(p1.to_force(&v2), Force::ZERO);
    }

    #[test]
    fn inline() {
        let p1 = Panel::new(Vec3::ZERO, Vec3::X, 1.);
        let p2 = Panel::new(Vec3::ZERO, Vec3::Y, 1.);
        let p3 = Panel::new(Vec3::ZERO, Vec3::Z, 1.);

        let v1 = LinVel::with_x(1.);
        let v2 = LinVel::with_y(1.);
        let v3 = LinVel::with_z(1.);

        assert_ulps_eq!(p1.to_force(&v1), Force::new(Vec3::NEG_X * EXP));
        assert_ulps_eq!(p2.to_force(&v2), Force::new(Vec3::NEG_Y * EXP));
        assert_ulps_eq!(p3.to_force(&v3), Force::new(Vec3::NEG_Z * EXP));
    }

    #[test]
    fn inline_offset() {
        let v1 = LinVel::with_y(1.);

        let p1 = Panel::new(Vec3::Y, Vec3::Y, 1.);
        let p2 = Panel::new(Vec3::X, Vec3::Y, 1.);
        let p3 = Panel::new(Vec3::Z, Vec3::Y, 1.);

        assert_ulps_eq!(p1.to_force(&v1), Force::new(Vec3::NEG_Y * EXP));
        assert_ulps_eq!(p2.to_force(&v1), Force::new(Vec3::NEG_Y * EXP));
        assert_ulps_eq!(p3.to_force(&v1), Force::new(Vec3::NEG_Y * EXP));
    }

    #[test]
    fn angled() {
        let exp = EXP * (PI / 4.).sin();

        let p1 = Panel::new(Vec3::ZERO, Vec3::new(1., 1., 0.).normalize(), 1.);

        let v1 = LinVel::with_x(1.);

        assert_ulps_eq!(p1.to_force(&v1), Force::new(Vec3::NEG_X * exp));
    }
}

#[cfg(test)]
mod to_moment {
    use super::*;
    use crate::{
        inertia_mass::{Inertia, InnertiaMass, Mass}, moments::Torque, momentum::{AngMom, LinMom, Momentum}, transform::Transform
    };
    use approx::assert_ulps_eq;

    const EXP: f64 = DENSITY * HALF_C_D;

    fn mass_x() -> InnertiaMass {
        InnertiaMass::new(Mass::new(1.), Inertia::cylinder_x(1., 1., 1.))
    }
    fn mass_y() -> InnertiaMass {
        InnertiaMass::new(Mass::new(1.), Inertia::cylinder_y(1., 1., 1.))
    }
    fn mass_z() -> InnertiaMass {
        InnertiaMass::new(Mass::new(1.), Inertia::cylinder_z(1., 1., 1.))
    }

    #[test]
    fn stationary() {
        let p1 = Panel::new(Vec3::ZERO, Vec3::X, 1.);

        let s1 = State::new(mass_x(), Transform::ZERO, Momentum::ZERO);

        assert_ulps_eq!(p1.to_moment(&s1), Moment::ZERO);
    }

    #[test]
    fn linear_movement() {
        let p1 = Panel::new(Vec3::ZERO, Vec3::X, 1.);
        let p2 = Panel::new(Vec3::ZERO, Vec3::Y, 1.);
        let p3 = Panel::new(Vec3::ZERO, Vec3::Z, 1.);

        let s1 = State::new(mass_x(), Transform::ZERO, Momentum::from_lin(Vec3::X));
        let s2 = State::new(mass_y(), Transform::ZERO, Momentum::from_lin(Vec3::Y));
        let s3 = State::new(mass_z(), Transform::ZERO, Momentum::from_lin(Vec3::Z));

        assert_ulps_eq!(p1.to_moment(&s1), Moment::from_force(Vec3::NEG_X * EXP));
        assert_ulps_eq!(p1.to_moment(&s2), Moment::ZERO);
        assert_ulps_eq!(p1.to_moment(&s3), Moment::ZERO);

        assert_ulps_eq!(p2.to_moment(&s1), Moment::ZERO);
        assert_ulps_eq!(p2.to_moment(&s2), Moment::from_force(Vec3::NEG_Y * EXP));
        assert_ulps_eq!(p2.to_moment(&s3), Moment::ZERO);

        assert_ulps_eq!(p3.to_moment(&s1), Moment::ZERO);
        assert_ulps_eq!(p3.to_moment(&s2), Moment::ZERO);
        assert_ulps_eq!(p3.to_moment(&s3), Moment::from_force(Vec3::NEG_Z * EXP));
    }

    #[test]
    fn angular_movement() {
        let pxy = Panel::new(Vec3::X, Vec3::Y, 1.);
        let pyz = Panel::new(Vec3::Y, Vec3::Z, 1.);
        let pzx = Panel::new(Vec3::Z, Vec3::X, 1.);

        // momentum /2 bcs inertia is a bit weird
        let sx = State::new(mass_x(), Transform::ZERO, Momentum::from_ang(Vec3::X / 2.));
        let sy = State::new(mass_y(), Transform::ZERO, Momentum::from_ang(Vec3::Y / 2.));
        let sz = State::new(mass_z(), Transform::ZERO, Momentum::from_ang(Vec3::Z / 2.));

        assert_ulps_eq!(pxy.to_moment(&sx).magnitude(), 0.);
        assert_ulps_eq!(pxy.to_moment(&sy).magnitude(), 0.);
        assert_ulps_eq!(pxy.to_moment(&sz), Moment::new(Vec3::X, -Vec3::Y * EXP));

        assert_ulps_eq!(pyz.to_moment(&sx), Moment::new(Vec3::Y, -Vec3::Z * EXP));
        assert_ulps_eq!(pyz.to_moment(&sy).magnitude(), 0.);
        assert_ulps_eq!(pyz.to_moment(&sz).magnitude(), 0.);

        assert_ulps_eq!(pzx.to_moment(&sx).magnitude(), 0.);
        assert_ulps_eq!(pzx.to_moment(&sy), Moment::new(Vec3::Z, -Vec3::X * EXP));
        assert_ulps_eq!(pzx.to_moment(&sz).magnitude(), 0.);
    }
}
