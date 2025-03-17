use std::time::Duration;

use inertia_mass::{Inertia, InertiaMass, Mass};
use moments::Moment;
use momentum::Momentum;
use panels::Panel;
use transform::{Rotation, Transform, Translation};
use velocity::Velocity;

pub mod inertia_mass;
pub mod moments;
pub mod momentum;
pub mod panels;
pub mod transform;
pub mod velocity;
pub mod builder;

/// Represents the kinetic state of a simulated entity
///
/// Can be constructed manually or using [`StateBuilder`]
#[derive(Debug, Clone, PartialEq)]
pub struct State {
    pub mass: InertiaMass,
    pub transform: Transform,
    pub momentum: Momentum,
    panels: Vec<Panel>,
}

impl State {
    pub const fn new(
        mass: InertiaMass,
        transform: Transform,
        momentum: Momentum,
        panels: Vec<Panel>,
    ) -> Self {
        Self {
            mass,
            transform,
            momentum,
            panels,
        }
    }

    pub const fn new_stationary(mass: InertiaMass, transform: Transform) -> Self {
        Self::without_panels(mass, transform, Momentum::ZERO)
    }

    pub const fn new_zeroed(mass: InertiaMass) -> Self {
        Self::without_panels(mass, Transform::ZERO, Momentum::ZERO)
    }

    pub const fn without_panels(mass: InertiaMass, transform: Transform, momentum: Momentum) -> State {
        Self::new(mass, transform, momentum, Vec::new())
    }

    /// Steps the state forward by a [Duration] using the Runge-Kutta Algorithm
    ///
    /// # Arguments
    /// delta: time difference for this iteration
    /// moment: any external forces that are effectively constant
    pub fn simulation_step(&mut self, delta: Duration, moment: Moment) {
        let half_delta = delta / 2;

        let int1 = &self;
        let (k1_dx, k1_dp) = (int1.velocity(), moment);

        let int2 = Self::without_panels(
            self.mass,
            self.transform + k1_dx * delta,
            self.momentum + k1_dp * delta,
        );
        let (k2_dx, k2_dp) = (int2.velocity(), moment);

        let int3 = Self::without_panels(
            int2.mass,
            int2.transform + k2_dx * half_delta,
            int2.momentum + k2_dp * half_delta,
        );
        let (k3_dx, k3_dp) = (int3.velocity(), moment);

        let int4 = Self::without_panels(
            int3.mass,
            int3.transform + k3_dx * half_delta,
            int3.momentum + k3_dp * half_delta,
        );
        let (k4_dx, k4_dp) = (int4.velocity(), moment);

        self.transform += (k1_dx + k2_dx * 2.0 + k3_dx * 2.0 + k4_dx) * (delta / 6);
        self.momentum += (k1_dp + k2_dp * 2.0 + k3_dp * 2.0 + k4_dp) * (delta / 6);
    }
}

impl State {
    /// Returns the inertia mass of this [`State`] where the inertia tensors are rotated to be
    /// aligned with the rest of the [`State`]
    ///
    /// # Performance
    ///
    /// Note: This function has to do 2 sets of matrix multiplications and one quaternion to matrix
    /// conversion, it is recommended to use [`State::rotated_inertia`] or
    /// [`State::rotated_inv_inertia`] instead as they only do one matrix multiplication
    ///
    /// If you are only using the actual mass of the entity, use [`State::mass`] to not have to do
    /// any extra calculations at all.
    #[inline]
    #[must_use]
    pub fn rotated_mass(&self) -> InertiaMass {
        self.mass.rotated(self.transform.rotation.0)
    }

    /// Returns the inertia of this [`State`] rotated to be aligned with the rest of the [`State`]
    #[inline]
    #[must_use]
    pub fn rotated_inertia(&self) -> Inertia {
        self.mass.inertia.rotated(self.transform.rotation.0)
    }

    /// Returns the inverse inertia of this [`State`] rotated to be aligned with the rest of the
    /// [`State`]
    #[inline]
    #[must_use]
    pub fn rotated_inv_inertia(&self) -> Inertia {
        self.mass.inv_inertia.rotated(self.transform.rotation.0)
    }

    /// Calculates the velocity that this [`State`] has
    #[inline]
    #[must_use]
    pub fn velocity(&self) -> Velocity {
        let linvel = self.momentum.linear / self.mass();
        let angvel = self.momentum.rotation / self.rotated_inv_inertia();
        Velocity::new(linvel, angvel)
    }
}

/// Getters for all the internal fields
impl State {
    /// Returns a reference to the translation of this [`State`].
    #[inline]
    #[must_use]
    pub const fn translation(&self) -> &Translation {
        &self.transform.translation
    }

    /// Returns a reference to the rotation of this [`State`].
    #[inline]
    #[must_use]
    pub const fn rotation(&self) -> &Rotation {
        &self.transform.rotation
    }

    /// Returns a reference to the mass of this [`State`].
    #[inline]
    #[must_use]
    pub const fn mass(&self) -> &Mass {
        &self.mass.mass
    }

    /// Returns a reference to the inertia of this [`State`].
    #[inline]
    #[must_use]
    pub const fn inertia(&self) -> &Inertia {
        &self.mass.inertia
    }

    /// Returns a reference to the inverse inertia of this [`State`].
    #[inline]
    #[must_use]
    pub const fn inv_inertia(&self) -> &Inertia {
        &self.mass.inv_inertia
    }
}
