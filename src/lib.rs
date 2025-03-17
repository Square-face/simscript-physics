use std::time::Duration;

use inertia_mass::InertiaMass;
use moments::Moment;
use momentum::Momentum;
use transform::Transform;
use velocity::Velocity;

pub mod inertia_mass;
pub mod moments;
pub mod momentum;
pub mod panels;
pub mod transform;
pub mod velocity;

/// Represents the kinetic state of a simulated entity
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub mass: InertiaMass,
    pub transform: Transform,
    pub momentum: Momentum,
}

impl State {
    pub const fn new(mass: InertiaMass, transform: Transform, momentum: Momentum) -> Self {
        Self {
            mass,
            transform,
            momentum,
        }
    }

    pub const fn new_stationary(mass: InertiaMass, transform: Transform) -> Self {
        Self::new(mass, transform, Momentum::ZERO)
    }

    pub const fn new_zeroed(mass: InertiaMass) -> Self {
        Self::new(mass, Transform::ZERO, Momentum::ZERO)
    }

    pub fn velocity(&self) -> Velocity {
        self.momentum / self.mass.rotated(self.transform.rotation.0)
    }

    /// Steps the state forward by a [Duration] using the Runge-Kutta Algorithm
    ///
    pub fn step_movement(&mut self, delta: Duration, moment: Moment) {
        let half_delta = delta / 2;

        let int1 = &self;
        let (k1_dx, k1_dp) = (int1.velocity(), moment);

        let int2 = Self::new(
            self.mass,
            self.transform + k1_dx * delta,
            self.momentum + k1_dp * delta,
        );
        let (k2_dx, k2_dp) = (int2.velocity(), moment);

        let int3 = Self::new(
            int2.mass,
            int2.transform + k2_dx * half_delta,
            int2.momentum + k2_dp * half_delta,
        );
        let (k3_dx, k3_dp) = (int3.velocity(), moment);

        let int4 = Self::new(
            int3.mass,
            int3.transform + k3_dx * half_delta,
            int3.momentum + k3_dp * half_delta,
        );
        let (k4_dx, k4_dp) = (int4.velocity(), moment);

        self.transform += (k1_dx + k2_dx * 2.0 + k3_dx * 2.0 + k4_dx) * (delta / 6);
        self.momentum += (k1_dp + k2_dp * 2.0 + k3_dp * 2.0 + k4_dp) * (delta / 6);
    }
}
