use glam::{DQuat as Quat, DVec3 as Vec3};

use crate::{
    moments::{Force, Moment},
    velocity::{AngVel, LinVel, Velocity},
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

        Force::new(DENSITY * rel_vel.0.length_squared() * HALF_C_D * area * -self.normal)
    }

    pub fn rotated(&self, rot: &Quat) -> Self {
        let offset = rot.mul_vec3(self.offset);
        let normal = rot.mul_vec3(self.normal);

        Self::new(offset, normal, self.area)
    }

    pub fn rotation_based_velocity(&self, vel: &AngVel) -> LinVel {
        LinVel(vel.0.cross(self.offset))
    }

    pub fn tip_velocity(&self, vel: &Velocity) -> LinVel {
        let linear = vel.linear.0;
        let angular = self.rotation_based_velocity(&vel.angular);
        LinVel(linear) + angular
    }

    pub fn to_moment(&self, state: &State) -> Moment {
        let rot = state.transform.rotation.0;
        let vel = state.momentum / state.mass;
        let rotated = self.rotated(&rot);
        let vel = rotated.tip_velocity(&vel);

        let force = rotated.to_force(&vel);

        Moment::new(rot.mul_vec3(self.offset), force.0)
    }
}

#[cfg(test)]
mod test_utils {

    use crate::inertia_mass::{Inertia, InnertiaMass, Mass};

    use super::*;
    use std::f64::consts::PI;

    pub const EXP: f64 = DENSITY * HALF_C_D;

    pub fn quarter_rotations() -> (Quat, Quat, Quat) {
        (
            Quat::from_rotation_x(PI / 2.),
            Quat::from_rotation_y(PI / 2.),
            Quat::from_rotation_z(PI / 2.),
        )
    }

    pub fn xyz_panels() -> (Panel, Panel, Panel) {
        (
            Panel::new(Vec3::X, Vec3::X, 1.),
            Panel::new(Vec3::Y, Vec3::Y, 1.),
            Panel::new(Vec3::Z, Vec3::Z, 1.),
        )
    }

    pub fn xyz_neg_panels() -> (Panel, Panel, Panel) {
        (
            Panel::new(-Vec3::X, Vec3::X, 1.),
            Panel::new(-Vec3::Y, Vec3::Y, 1.),
            Panel::new(-Vec3::Z, Vec3::Z, 1.),
        )
    }

    pub fn xyz_angvel() -> (AngVel, AngVel, AngVel) {
        (
            AngVel::with_x(1.0),
            AngVel::with_y(1.0),
            AngVel::with_z(1.0),
        )
    }

    pub fn xyz_linvel() -> (LinVel, LinVel, LinVel) {
        (
            LinVel::with_x(1.0),
            LinVel::with_y(1.0),
            LinVel::with_z(1.0),
        )
    }

    pub fn cyl_xyz() -> (InnertiaMass, InnertiaMass, InnertiaMass) {
        (
            InnertiaMass::new(Mass::new(1.), Inertia::cylinder_x(1., 1., 1.)),
            InnertiaMass::new(Mass::new(1.), Inertia::cylinder_y(1., 1., 1.)),
            InnertiaMass::new(Mass::new(1.), Inertia::cylinder_z(1., 1., 1.)),
        )
    }

    #[allow(clippy::type_complexity)]
    pub fn all_vel() -> (
        (Velocity, Velocity, Velocity),
        (Velocity, Velocity, Velocity),
        (Velocity, Velocity, Velocity),
    ) {
        let (ax, ay, az) = xyz_angvel();
        let (lx, ly, lz) = xyz_linvel();
        (
            (
                Velocity::new(lx, ax),
                Velocity::new(lx, ay),
                Velocity::new(lx, az),
            ),
            (
                Velocity::new(ly, ax),
                Velocity::new(ly, ay),
                Velocity::new(ly, az),
            ),
            (
                Velocity::new(lz, ax),
                Velocity::new(lz, ay),
                Velocity::new(lz, az),
            ),
        )
    }
}

#[cfg(test)]
mod rotated {
    use approx::assert_ulps_eq;
    use test_utils::*;

    use super::*;

    #[test]
    fn no_rotation() {
        let p1 = Panel::new(Vec3::X, Vec3::Y, 1.);
        let q1 = Quat::IDENTITY;

        assert_ulps_eq!(p1.rotated(&q1).offset, p1.offset);
    }

    #[test]
    fn quarter_turn_offset() {
        let (rx, ry, rz) = quarter_rotations();
        let (px, py, pz) = xyz_panels();

        assert_ulps_eq!(px.rotated(&rx).offset, px.offset);
        assert_ulps_eq!(px.rotated(&ry).offset, pz.offset);
        assert_ulps_eq!(px.rotated(&rz).offset, -py.offset);

        assert_ulps_eq!(py.rotated(&rx).offset, -pz.offset);
        assert_ulps_eq!(py.rotated(&ry).offset, py.offset);
        assert_ulps_eq!(py.rotated(&rz).offset, px.offset);

        assert_ulps_eq!(pz.rotated(&rx).offset, py.offset);
        assert_ulps_eq!(pz.rotated(&ry).offset, -px.offset);
        assert_ulps_eq!(pz.rotated(&rz).offset, pz.offset);
    }

    #[test]
    fn quarter_turn_normal() {
        let (rx, ry, rz) = quarter_rotations();
        let (px, py, pz) = xyz_panels();

        assert_ulps_eq!(px.rotated(&rx).normal, px.normal);
        assert_ulps_eq!(px.rotated(&ry).normal, pz.normal);
        assert_ulps_eq!(px.rotated(&rz).normal, -py.normal);

        assert_ulps_eq!(py.rotated(&rx).normal, -pz.normal);
        assert_ulps_eq!(py.rotated(&ry).normal, py.normal);
        assert_ulps_eq!(py.rotated(&rz).normal, px.normal);

        assert_ulps_eq!(pz.rotated(&rx).normal, py.normal);
        assert_ulps_eq!(pz.rotated(&ry).normal, -px.normal);
        assert_ulps_eq!(pz.rotated(&rz).normal, pz.normal);
    }
}

#[cfg(test)]
mod rotation_based_velocity {
    use super::*;
    use approx::assert_ulps_eq;
    use test_utils::*;

    #[test]
    fn stationary() {
        let p1 = Panel::new(Vec3::X, Vec3::X, 1.);
        let rot = Quat::IDENTITY;
        let vel = AngVel::ZERO;

        assert_ulps_eq!(p1.rotation_based_velocity(&rot, &vel), LinVel::ZERO);
    }

    #[test]
    fn inline() {
        let q1 = Quat::IDENTITY;

        let (px, py, pz) = xyz_panels();
        let (vx, vy, vz) = xyz_angvel();

        assert_ulps_eq!(px.rotation_based_velocity(&q1, &vx), LinVel::ZERO);
        assert_ulps_eq!(py.rotation_based_velocity(&q1, &vy), LinVel::ZERO);
        assert_ulps_eq!(pz.rotation_based_velocity(&q1, &vz), LinVel::ZERO);
    }

    #[test]
    fn perpendicular() {
        let q1 = Quat::IDENTITY;

        let (px, py, pz) = xyz_panels();
        let (ax, ay, az) = xyz_angvel();
        let (vx, vy, vz) = xyz_linvel();

        assert_ulps_eq!(px.rotation_based_velocity(&q1, &ay), -vz);
        assert_ulps_eq!(py.rotation_based_velocity(&q1, &az), -vx);
        assert_ulps_eq!(pz.rotation_based_velocity(&q1, &ax), -vy);
    }

    #[test]
    fn perpendicular_reverse() {
        let q1 = Quat::IDENTITY;

        let (px, py, pz) = xyz_neg_panels();
        let (ax, ay, az) = xyz_angvel();
        let (vx, vy, vz) = xyz_linvel();

        assert_ulps_eq!(px.rotation_based_velocity(&q1, &ay), vz);
        assert_ulps_eq!(py.rotation_based_velocity(&q1, &az), vx);
        assert_ulps_eq!(pz.rotation_based_velocity(&q1, &ax), vy);
    }

    #[test]
    fn perpendicular_backward() {
        let q1 = Quat::IDENTITY;

        let (px, py, pz) = xyz_panels();
        let (ax, ay, az) = xyz_angvel();
        let (vx, vy, vz) = xyz_linvel();

        assert_ulps_eq!(px.rotation_based_velocity(&q1, &-ay), vz);
        assert_ulps_eq!(py.rotation_based_velocity(&q1, &-az), vx);
        assert_ulps_eq!(pz.rotation_based_velocity(&q1, &-ax), vy);
    }

    #[test]
    fn perpendicular_backward_reverse() {
        let q1 = Quat::IDENTITY;

        let (px, py, pz) = xyz_neg_panels();
        let (ax, ay, az) = xyz_angvel();
        let (vx, vy, vz) = xyz_linvel();

        assert_ulps_eq!(px.rotation_based_velocity(&q1, &-ay), -vz);
        assert_ulps_eq!(py.rotation_based_velocity(&q1, &-az), -vx);
        assert_ulps_eq!(pz.rotation_based_velocity(&q1, &-ax), -vy);
    }

    #[cfg(test)]
    mod rotated {
        use super::*;

        #[test]
        fn inline() {
            let (qx, qy, qz) = quarter_rotations();
            let (px, py, pz) = xyz_panels();
            let (ax, ay, az) = xyz_angvel();
            //let (vx, vy, vz) = xyz_linvel();

            assert_ulps_eq!(px.rotation_based_velocity(&qy, &az), LinVel::ZERO);
            assert_ulps_eq!(py.rotation_based_velocity(&qz, &ax), LinVel::ZERO);
            assert_ulps_eq!(pz.rotation_based_velocity(&qx, &ay), LinVel::ZERO);
        }

        #[test]
        fn perpendicular() {
            let (qx, qy, qz) = quarter_rotations();
            let (px, py, pz) = xyz_panels();
            let (ax, ay, az) = xyz_angvel();
            let (vx, vy, vz) = xyz_linvel();

            assert_ulps_eq!(px.rotation_based_velocity(&qz, &ax), -vz);
            assert_ulps_eq!(py.rotation_based_velocity(&qx, &ay), -vx);
            assert_ulps_eq!(pz.rotation_based_velocity(&qy, &az), -vy);
        }

        #[test]
        fn perpendicular_inv() {
            let (qx, qy, qz) = quarter_rotations();
            let (px, py, pz) = xyz_panels();
            let (ax, ay, az) = xyz_angvel();
            let (vx, vy, vz) = xyz_linvel();

            assert_ulps_eq!(px.rotation_based_velocity(&qz.inverse(), &ax), vz);
            assert_ulps_eq!(py.rotation_based_velocity(&qx.inverse(), &ay), vx);
            assert_ulps_eq!(pz.rotation_based_velocity(&qy.inverse(), &az), vy);
        }
    }
}

#[cfg(test)]
mod relative_velocity {
    use super::*;
    use crate::velocity::AngVel;
    use approx::assert_ulps_eq;
    use std::f64::consts::PI;
    use test_utils::*;

    #[test]
    fn stationary() {
        let panel = Panel::new(Vec3::X, Vec3::Y, 1.0);
        let rot = Quat::IDENTITY;
        let vel = Velocity::ZERO;

        assert_ulps_eq!(panel.tip_velocity(&rot, &vel), LinVel::ZERO);
    }

    #[test]
    fn rotated_stationary() {
        let panel = Panel::new(Vec3::X, Vec3::Y, 1.0);
        let rot = Quat::from_rotation_x(PI / 2.);
        let vel = Velocity::ZERO;

        assert_ulps_eq!(panel.tip_velocity(&rot, &vel), LinVel::ZERO);
    }

    #[test]
    fn no_rotation_moving() {
        let q = Quat::IDENTITY;
        let (px, py, pz) = xyz_panels();
        let (lx, ly, lz) = xyz_linvel();
        let a = AngVel::ZERO;

        assert_ulps_eq!(px.tip_velocity(&q, &Velocity::new(lx, a)), lx);
        assert_ulps_eq!(py.tip_velocity(&q, &Velocity::new(ly, a)), ly);
        assert_ulps_eq!(pz.tip_velocity(&q, &Velocity::new(lz, a)), lz);

        assert_ulps_eq!(px.tip_velocity(&q, &Velocity::new(ly, a)), ly);
        assert_ulps_eq!(py.tip_velocity(&q, &Velocity::new(lz, a)), lz);
        assert_ulps_eq!(pz.tip_velocity(&q, &Velocity::new(lx, a)), lx);

        assert_ulps_eq!(px.tip_velocity(&q, &Velocity::new(lz, a)), lz);
        assert_ulps_eq!(py.tip_velocity(&q, &Velocity::new(lx, a)), lx);
        assert_ulps_eq!(pz.tip_velocity(&q, &Velocity::new(ly, a)), ly);
    }

    #[test]
    fn rotated_moving() {
        let (rx, ry, rz) = quarter_rotations();
        let (px, py, pz) = xyz_panels();
        let (lx, ly, lz) = xyz_linvel();
        let a = AngVel::ZERO;

        assert_ulps_eq!(px.tip_velocity(&rx, &Velocity::new(lx, a)), lx);
        assert_ulps_eq!(py.tip_velocity(&ry, &Velocity::new(ly, a)), ly);
        assert_ulps_eq!(pz.tip_velocity(&rz, &Velocity::new(lz, a)), lz);

        assert_ulps_eq!(px.tip_velocity(&ry, &Velocity::new(ly, a)), ly);
        assert_ulps_eq!(py.tip_velocity(&rz, &Velocity::new(lz, a)), lz);
        assert_ulps_eq!(pz.tip_velocity(&rx, &Velocity::new(lx, a)), lx);

        assert_ulps_eq!(px.tip_velocity(&rz, &Velocity::new(lz, a)), lz);
        assert_ulps_eq!(py.tip_velocity(&rx, &Velocity::new(lx, a)), lx);
        assert_ulps_eq!(pz.tip_velocity(&ry, &Velocity::new(ly, a)), ly);
    }

    #[test]
    fn rotating_moving() {
        let q = Quat::IDENTITY;
        let (px, py, pz) = xyz_panels();
        let (lx, ly, lz) = xyz_linvel();
        let ((xx, xy, xz), (yx, yy, yz), (zx, zy, zz)) = all_vel();

        assert_ulps_eq!(px.tip_velocity(&q, &xx), lx);
        assert_ulps_eq!(py.tip_velocity(&q, &xx), lx + lz);
        assert_ulps_eq!(pz.tip_velocity(&q, &xx), lx - ly);

        assert_ulps_eq!(px.tip_velocity(&q, &yx), ly);
        assert_ulps_eq!(py.tip_velocity(&q, &yx), ly + lz);
        assert_ulps_eq!(pz.tip_velocity(&q, &yx), ly - ly);

        assert_ulps_eq!(px.tip_velocity(&q, &zx), lz);
        assert_ulps_eq!(py.tip_velocity(&q, &zx), lz + lz);
        assert_ulps_eq!(pz.tip_velocity(&q, &zx), lz - ly);

        assert_ulps_eq!(px.tip_velocity(&q, &xy), lx - lz);
        assert_ulps_eq!(py.tip_velocity(&q, &xy), lx);
        assert_ulps_eq!(pz.tip_velocity(&q, &xy), lx + lx);

        assert_ulps_eq!(px.tip_velocity(&q, &yy), ly - lz);
        assert_ulps_eq!(py.tip_velocity(&q, &yy), ly);
        assert_ulps_eq!(pz.tip_velocity(&q, &yy), ly + lx);

        assert_ulps_eq!(px.tip_velocity(&q, &zy), lz - lz);
        assert_ulps_eq!(py.tip_velocity(&q, &zy), lz);
        assert_ulps_eq!(pz.tip_velocity(&q, &zy), lz + lx);

        assert_ulps_eq!(px.tip_velocity(&q, &xz), lx + ly);
        assert_ulps_eq!(py.tip_velocity(&q, &xz), lx - lx);
        assert_ulps_eq!(pz.tip_velocity(&q, &xz), lx);

        assert_ulps_eq!(px.tip_velocity(&q, &yz), ly + ly);
        assert_ulps_eq!(py.tip_velocity(&q, &yz), ly - lx);
        assert_ulps_eq!(pz.tip_velocity(&q, &yz), ly);

        assert_ulps_eq!(px.tip_velocity(&q, &zz), lz + ly);
        assert_ulps_eq!(py.tip_velocity(&q, &zz), lz - lx);
        assert_ulps_eq!(pz.tip_velocity(&q, &zz), lz);
    }
}

#[cfg(test)]
mod to_force {
    use std::f64::consts::PI;

    use super::*;
    use approx::assert_ulps_eq;
    use test_utils::*;

    #[test]
    fn stationary() {
        let panel = Panel::new(Vec3::X, Vec3::Y, 1.);
        let vel = LinVel::ZERO;
        assert_ulps_eq!(panel.to_force(&vel), Force::ZERO);
    }

    #[test]
    fn parallel() {
        let p1 = Panel::new(Vec3::ZERO, Vec3::Y, 1.);

        let v1 = LinVel::Z;
        let v2 = LinVel::X;

        assert_ulps_eq!(p1.to_force(&v1), Force::ZERO);
        assert_ulps_eq!(p1.to_force(&v2), Force::ZERO);
    }

    #[test]
    fn inline() {
        let p1 = Panel::new(Vec3::ZERO, Vec3::X, 1.);
        let p2 = Panel::new(Vec3::ZERO, Vec3::Y, 1.);
        let p3 = Panel::new(Vec3::ZERO, Vec3::Z, 1.);

        let (lx, ly, lz) = xyz_linvel();

        assert_ulps_eq!(p1.to_force(&lx), Force::new(Vec3::NEG_X * EXP));
        assert_ulps_eq!(p2.to_force(&ly), Force::new(Vec3::NEG_Y * EXP));
        assert_ulps_eq!(p3.to_force(&lz), Force::new(Vec3::NEG_Z * EXP));
    }

    #[test]
    fn inline_offset() {
        let v1 = LinVel::Y;

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

        let v1 = LinVel::X;

        assert_ulps_eq!(p1.to_force(&v1), Force::new(Vec3::new(-1., -1., 0.).normalize() * exp));
    }
}

#[cfg(test)]
mod to_moment {
    use super::*;
    use test_utils::*;

    use crate::{momentum::Momentum, transform::Transform};
    use approx::assert_ulps_eq;

    #[test]
    fn stationary() {
        let px = Panel::new(Vec3::ZERO, Vec3::X, 1.);
        let (cx, _, _) = cyl_xyz();

        let s1 = State::new(cx, Transform::ZERO, Momentum::ZERO);

        assert_ulps_eq!(px.to_moment(&s1), Moment::ZERO);
    }

    #[test]
    fn linear_movement() {
        let px = Panel::new(Vec3::ZERO, Vec3::X, 1.);
        let py = Panel::new(Vec3::ZERO, Vec3::Y, 1.);
        let pz = Panel::new(Vec3::ZERO, Vec3::Z, 1.);

        let (cx, cy, cz) = cyl_xyz();

        let sx = State::new(cx, Transform::ZERO, Momentum::from_lin(Vec3::X));
        let sy = State::new(cy, Transform::ZERO, Momentum::from_lin(Vec3::Y));
        let sz = State::new(cz, Transform::ZERO, Momentum::from_lin(Vec3::Z));

        assert_ulps_eq!(px.to_moment(&sx), Moment::from_force(Vec3::NEG_X * EXP));
        assert_ulps_eq!(px.to_moment(&sy), Moment::ZERO);
        assert_ulps_eq!(px.to_moment(&sz), Moment::ZERO);

        assert_ulps_eq!(py.to_moment(&sx), Moment::ZERO);
        assert_ulps_eq!(py.to_moment(&sy), Moment::from_force(Vec3::NEG_Y * EXP));
        assert_ulps_eq!(py.to_moment(&sz), Moment::ZERO);

        assert_ulps_eq!(pz.to_moment(&sx), Moment::ZERO);
        assert_ulps_eq!(pz.to_moment(&sy), Moment::ZERO);
        assert_ulps_eq!(pz.to_moment(&sz), Moment::from_force(Vec3::NEG_Z * EXP));
    }

    #[test]
    fn angular_movement() {
        let pxy = Panel::new(Vec3::X, Vec3::Y, 1.);
        let pyz = Panel::new(Vec3::Y, Vec3::Z, 1.);
        let pzx = Panel::new(Vec3::Z, Vec3::X, 1.);

        let (cx, cy, cz) = cyl_xyz();

        // Momentum /2 because inertia is a bit weird
        let sx = State::new(cx, Transform::ZERO, Momentum::from_ang(Vec3::X / 2.));
        let sy = State::new(cy, Transform::ZERO, Momentum::from_ang(Vec3::Y / 2.));
        let sz = State::new(cz, Transform::ZERO, Momentum::from_ang(Vec3::Z / 2.));

        assert_ulps_eq!(pxy.to_moment(&sx).magnitude(), 0.);
        assert_ulps_eq!(pxy.to_moment(&sy).magnitude(), 0.);
        assert_ulps_eq!(pxy.to_moment(&sz), Moment::new(Vec3::X, Vec3::NEG_Y * EXP));

        assert_ulps_eq!(pyz.to_moment(&sx), Moment::new(Vec3::Y, Vec3::NEG_Z * EXP));
        assert_ulps_eq!(pyz.to_moment(&sy).magnitude(), 0.);
        assert_ulps_eq!(pyz.to_moment(&sz).magnitude(), 0.);

        assert_ulps_eq!(pzx.to_moment(&sx).magnitude(), 0.);
        assert_ulps_eq!(pzx.to_moment(&sy), Moment::new(Vec3::Z, Vec3::NEG_X * EXP));
        assert_ulps_eq!(pzx.to_moment(&sz).magnitude(), 0.);
    }
}
