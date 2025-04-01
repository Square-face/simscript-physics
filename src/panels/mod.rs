use glam::{DQuat as Quat, DVec3 as Vec3};

use crate::{
    moments::{Force, Moment},
    velocity::{AngVel, LinVel, Velocity},
};

/// Represents a simulated "aerodynamic" panel.
///
/// Used to heavily approximate the effects of aerodynamics on a simulated entity
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Panel {
    /// Position relative to origin.
    pub offset: Vec3,
    /// Direction the panel faces.
    pub normal: Vec3,
    /// Surface area of the panel.
    pub area: f64,
}

/// Air density (kg/m³).
const DENSITY: f64 = 1.293;
/// Half of drag coefficient.
const HALF_C_D: f64 = 1.28 / 2.;

impl Panel {
    /// Creates a new panel with given offset, normal, and area.
    pub fn new(offset: Vec3, normal: Vec3, area: f64) -> Self {
        Self {
            offset,
            normal,
            area,
        }
    }

    /// Calculates aerodynamic force based on relative velocity.
    pub fn to_force(&self, rel_vel: &LinVel) -> Force {
        let area = self.normal.dot(rel_vel.0.normalize_or_zero()) * self.area;
        Force::from_vec3(DENSITY * rel_vel.0.length_squared() * HALF_C_D * area * -self.normal)
    }

    /// Returns a new panel rotated by the given quaternion.
    pub fn rotated(&self, rot: &Quat) -> Self {
        let offset = rot.mul_vec3(self.offset);
        let normal = rot.mul_vec3(self.normal);
        Self::new(offset, normal, self.area)
    }

    /// Computes linear velocity at the panel due to angular velocity.
    pub fn rotation_based_velocity(&self, vel: &AngVel) -> LinVel {
        LinVel(vel.0.cross(self.offset))
    }

    /// Calculates total velocity at the panel's tip from combined linear and angular velocity.
    pub fn tip_velocity(&self, vel: &Velocity) -> LinVel {
        let linear = vel.linear.0;
        let angular = self.rotation_based_velocity(&vel.angular);
        LinVel(linear) + angular
    }

    /// Computes the moment the panel would induce on the simulated entity given a certain
    /// orientation and relative wind speed
    pub fn to_moment(&self, vel: &Velocity, rot: &Quat) -> Moment {
        let rotated = self.rotated(rot);
        let vel = rotated.tip_velocity(vel);
        let force = rotated.to_force(&vel);
        Moment::from_force_and_offset(force, rotated.offset)
    }
}

#[cfg(test)]
mod test_utils {

    use crate::linear_trait::LinVec as _;

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
        assert_ulps_eq!(px.rotated(&ry).offset, -pz.offset);
        assert_ulps_eq!(px.rotated(&rz).offset, py.offset);

        assert_ulps_eq!(py.rotated(&rx).offset, pz.offset);
        assert_ulps_eq!(py.rotated(&ry).offset, py.offset);
        assert_ulps_eq!(py.rotated(&rz).offset, -px.offset);

        assert_ulps_eq!(pz.rotated(&rx).offset, -py.offset);
        assert_ulps_eq!(pz.rotated(&ry).offset, px.offset);
        assert_ulps_eq!(pz.rotated(&rz).offset, pz.offset);
    }

    #[test]
    fn quarter_turn_normal() {
        let (rx, ry, rz) = quarter_rotations();
        let (px, py, pz) = xyz_panels();

        assert_ulps_eq!(px.rotated(&rx).normal, px.normal);
        assert_ulps_eq!(px.rotated(&ry).normal, -pz.normal);
        assert_ulps_eq!(px.rotated(&rz).normal, py.normal);

        assert_ulps_eq!(py.rotated(&rx).normal, pz.normal);
        assert_ulps_eq!(py.rotated(&ry).normal, py.normal);
        assert_ulps_eq!(py.rotated(&rz).normal, -px.normal);

        assert_ulps_eq!(pz.rotated(&rx).normal, -py.normal);
        assert_ulps_eq!(pz.rotated(&ry).normal, px.normal);
        assert_ulps_eq!(pz.rotated(&rz).normal, pz.normal);
    }
}

#[cfg(test)]
mod rotation_based_velocity {

    use super::*;
    use crate::linear_trait::LinVec as _;
    use approx::assert_ulps_eq;
    use test_utils::*;

    #[test]
    fn stationary() {
        let p1 = Panel::new(Vec3::X, Vec3::X, 1.);
        let vel = AngVel::ZERO;

        assert_ulps_eq!(p1.rotation_based_velocity(&vel), LinVel::ZERO);
    }

    #[test]
    fn inline() {
        let (px, py, pz) = xyz_panels();
        let (vx, vy, vz) = xyz_angvel();

        assert_ulps_eq!(px.rotation_based_velocity(&vx), LinVel::ZERO);
        assert_ulps_eq!(py.rotation_based_velocity(&vy), LinVel::ZERO);
        assert_ulps_eq!(pz.rotation_based_velocity(&vz), LinVel::ZERO);
    }

    #[test]
    fn perpendicular() {
        let (px, py, pz) = xyz_panels();
        let (ax, ay, az) = xyz_angvel();
        let (vx, vy, vz) = xyz_linvel();

        assert_ulps_eq!(px.rotation_based_velocity(&ay), -vz);
        assert_ulps_eq!(py.rotation_based_velocity(&az), -vx);
        assert_ulps_eq!(pz.rotation_based_velocity(&ax), -vy);
    }

    #[test]
    fn perpendicular_reverse() {
        let (px, py, pz) = xyz_neg_panels();
        let (ax, ay, az) = xyz_angvel();
        let (vx, vy, vz) = xyz_linvel();

        assert_ulps_eq!(px.rotation_based_velocity(&ay), vz);
        assert_ulps_eq!(py.rotation_based_velocity(&az), vx);
        assert_ulps_eq!(pz.rotation_based_velocity(&ax), vy);
    }

    #[test]
    fn perpendicular_backward() {
        let (px, py, pz) = xyz_panels();
        let (ax, ay, az) = xyz_angvel();
        let (vx, vy, vz) = xyz_linvel();

        assert_ulps_eq!(px.rotation_based_velocity(&-ay), vz);
        assert_ulps_eq!(py.rotation_based_velocity(&-az), vx);
        assert_ulps_eq!(pz.rotation_based_velocity(&-ax), vy);
    }

    #[test]
    fn perpendicular_backward_reverse() {
        let (px, py, pz) = xyz_neg_panels();
        let (ax, ay, az) = xyz_angvel();
        let (vx, vy, vz) = xyz_linvel();

        assert_ulps_eq!(px.rotation_based_velocity(&-ay), -vz);
        assert_ulps_eq!(py.rotation_based_velocity(&-az), -vx);
        assert_ulps_eq!(pz.rotation_based_velocity(&-ax), -vy);
    }
}

#[cfg(test)]
mod relative_velocity {
    use super::*;
    use crate::linear_trait::LinVec as _;
    use crate::velocity::AngVel;
    use approx::assert_ulps_eq;
    use test_utils::*;

    #[test]
    fn stationary() {
        let panel = Panel::new(Vec3::X, Vec3::Y, 1.0);
        let vel = Velocity::ZERO;

        assert_ulps_eq!(panel.tip_velocity(&vel), LinVel::ZERO);
    }

    #[test]
    fn linear_motion() {
        let (px, py, pz) = xyz_panels();
        let (lx, ly, lz) = xyz_linvel();
        let a = AngVel::ZERO;

        assert_ulps_eq!(px.tip_velocity(&Velocity::new(lx, a)), lx);
        assert_ulps_eq!(py.tip_velocity(&Velocity::new(ly, a)), ly);
        assert_ulps_eq!(pz.tip_velocity(&Velocity::new(lz, a)), lz);

        assert_ulps_eq!(px.tip_velocity(&Velocity::new(ly, a)), ly);
        assert_ulps_eq!(py.tip_velocity(&Velocity::new(lz, a)), lz);
        assert_ulps_eq!(pz.tip_velocity(&Velocity::new(lx, a)), lx);

        assert_ulps_eq!(px.tip_velocity(&Velocity::new(lz, a)), lz);
        assert_ulps_eq!(py.tip_velocity(&Velocity::new(lx, a)), lx);
        assert_ulps_eq!(pz.tip_velocity(&Velocity::new(ly, a)), ly);
    }

    #[test]
    fn linear_and_angular_motion() {
        let (px, py, pz) = xyz_panels();
        let (lx, ly, lz) = xyz_linvel();
        let ((xx, xy, xz), (yx, yy, yz), (zx, zy, zz)) = all_vel();

        assert_ulps_eq!(px.tip_velocity(&xx), lx);
        assert_ulps_eq!(py.tip_velocity(&xx), lx + lz);
        assert_ulps_eq!(pz.tip_velocity(&xx), lx - ly);

        assert_ulps_eq!(px.tip_velocity(&yx), ly);
        assert_ulps_eq!(py.tip_velocity(&yx), ly + lz);
        assert_ulps_eq!(pz.tip_velocity(&yx), ly - ly);

        assert_ulps_eq!(px.tip_velocity(&zx), lz);
        assert_ulps_eq!(py.tip_velocity(&zx), lz + lz);
        assert_ulps_eq!(pz.tip_velocity(&zx), lz - ly);

        assert_ulps_eq!(px.tip_velocity(&xy), lx - lz);
        assert_ulps_eq!(py.tip_velocity(&xy), lx);
        assert_ulps_eq!(pz.tip_velocity(&xy), lx + lx);

        assert_ulps_eq!(px.tip_velocity(&yy), ly - lz);
        assert_ulps_eq!(py.tip_velocity(&yy), ly);
        assert_ulps_eq!(pz.tip_velocity(&yy), ly + lx);

        assert_ulps_eq!(px.tip_velocity(&zy), lz - lz);
        assert_ulps_eq!(py.tip_velocity(&zy), lz);
        assert_ulps_eq!(pz.tip_velocity(&zy), lz + lx);

        assert_ulps_eq!(px.tip_velocity(&xz), lx + ly);
        assert_ulps_eq!(py.tip_velocity(&xz), lx - lx);
        assert_ulps_eq!(pz.tip_velocity(&xz), lx);

        assert_ulps_eq!(px.tip_velocity(&yz), ly + ly);
        assert_ulps_eq!(py.tip_velocity(&yz), ly - lx);
        assert_ulps_eq!(pz.tip_velocity(&yz), ly);

        assert_ulps_eq!(px.tip_velocity(&zz), lz + ly);
        assert_ulps_eq!(py.tip_velocity(&zz), lz - lx);
        assert_ulps_eq!(pz.tip_velocity(&zz), lz);
    }
}

#[cfg(test)]
mod to_force {
    use std::f64::consts::PI;

    use crate::linear_trait::LinVec as _;
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

        assert_ulps_eq!(p1.to_force(&lx), Force::NEG_X * EXP);
        assert_ulps_eq!(p2.to_force(&ly), Force::NEG_Y * EXP);
        assert_ulps_eq!(p3.to_force(&lz), Force::NEG_Z * EXP);
    }

    #[test]
    fn inline_offset() {
        let v1 = LinVel::Y;

        let p1 = Panel::new(Vec3::Y, Vec3::Y, 1.);
        let p2 = Panel::new(Vec3::X, Vec3::Y, 1.);
        let p3 = Panel::new(Vec3::Z, Vec3::Y, 1.);

        assert_ulps_eq!(p1.to_force(&v1), Force::NEG_Y * EXP);
        assert_ulps_eq!(p2.to_force(&v1), Force::NEG_Y * EXP);
        assert_ulps_eq!(p3.to_force(&v1), Force::NEG_Y * EXP);
    }

    #[test]
    fn angled() {
        let exp = EXP * (PI / 4.).sin();

        let p1 = Panel::new(Vec3::ZERO, Vec3::new(1., 1., 0.).normalize(), 1.);

        let v1 = LinVel::X;

        assert_ulps_eq!(
            p1.to_force(&v1),
            Force::from_vec3(Vec3::new(-1., -1., 0.).normalize() * exp)
        );
    }
}

#[cfg(test)]
mod to_moment {
    use super::*;
    use test_utils::*;

    use crate::moments::Torque;
    use approx::assert_ulps_eq;

    #[test]
    fn stationary() {
        let px = Panel::new(Vec3::ZERO, Vec3::X, 1.);
        let v0 = Velocity::ZERO;
        let q0 = Quat::IDENTITY;

        assert_ulps_eq!(px.to_moment(&v0, &q0), Moment::ZERO);
    }

    #[test]
    fn linear_movement() {
        let q0 = Quat::IDENTITY;

        let px = Panel::new(Vec3::ZERO, Vec3::X, 1.);
        let py = Panel::new(Vec3::ZERO, Vec3::Y, 1.);
        let pz = Panel::new(Vec3::ZERO, Vec3::Z, 1.);

        let (lx, ly, lz) = xyz_linvel();
        let (vx, vy, vz) = (lx.to_vel(), ly.to_vel(), lz.to_vel());

        assert_ulps_eq!(
            px.to_moment(&vx, &q0),
            Moment::from_force(Force(Vec3::NEG_X * EXP))
        );
        assert_ulps_eq!(px.to_moment(&vy, &q0), Moment::ZERO);
        assert_ulps_eq!(px.to_moment(&vz, &q0), Moment::ZERO);

        assert_ulps_eq!(py.to_moment(&vx, &q0), Moment::ZERO);
        assert_ulps_eq!(
            py.to_moment(&vy, &q0),
            Moment::from_force(Force(Vec3::NEG_Y * EXP))
        );
        assert_ulps_eq!(py.to_moment(&vz, &q0), Moment::ZERO);

        assert_ulps_eq!(pz.to_moment(&vx, &q0), Moment::ZERO);
        assert_ulps_eq!(pz.to_moment(&vy, &q0), Moment::ZERO);
        assert_ulps_eq!(
            pz.to_moment(&vz, &q0),
            Moment::from_force(Force(Vec3::NEG_Z * EXP))
        );
    }

    #[test]
    fn angular_movement() {
        let q0 = Quat::IDENTITY;

        let pxy = Panel::new(Vec3::X, Vec3::Y, 1.);
        let pyz = Panel::new(Vec3::Y, Vec3::Z, 1.);
        let pzx = Panel::new(Vec3::Z, Vec3::X, 1.);

        let (lx, ly, lz) = xyz_linvel();
        let (vx, vy, vz) = (lx.to_vel(), ly.to_vel(), lz.to_vel());

        assert_ulps_eq!(pxy.to_moment(&vx, &q0).magnitude(), 0.);
        assert_ulps_eq!(pxy.to_moment(&vy, &q0).force, Force(Vec3::NEG_Y * EXP));
        assert_ulps_eq!(pxy.to_moment(&vy, &q0).torque, Torque(Vec3::NEG_Z * EXP));
        assert_ulps_eq!(pxy.to_moment(&vz, &q0).magnitude(), 0.);

        assert_ulps_eq!(pyz.to_moment(&vx, &q0).magnitude(), 0.);
        assert_ulps_eq!(pyz.to_moment(&vy, &q0).magnitude(), 0.);
        assert_ulps_eq!(pyz.to_moment(&vz, &q0).force, Force(Vec3::NEG_Z * EXP));
        assert_ulps_eq!(pyz.to_moment(&vz, &q0).torque, Torque(Vec3::NEG_X * EXP));

        assert_ulps_eq!(pzx.to_moment(&vx, &q0).force, Force(Vec3::NEG_X * EXP));
        assert_ulps_eq!(pzx.to_moment(&vx, &q0).torque, Torque(Vec3::NEG_Y * EXP));
        assert_ulps_eq!(pzx.to_moment(&vy, &q0).magnitude(), 0.);
        assert_ulps_eq!(pzx.to_moment(&vz, &q0).magnitude(), 0.);
    }
}
