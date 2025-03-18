use std::time::Duration;

use inertia_mass::InertiaMass;
use moments::Moment;
use momentum::Momentum;
use panels::Panel;
use transform::Transform;

pub mod inertia_mass;
pub mod moments;
pub mod momentum;
pub mod panels;
pub mod transform;
pub mod velocity;

mod builder;
pub use builder::StateBuilder;
use velocity::Velocity;

/// Represents the kinetic state of a simulated entity
#[derive(Debug, Clone, PartialEq)]
pub struct State {
    pub mass: InertiaMass,
    pub transform: Transform,
    pub momentum: Momentum,
    pub panels: Vec<Panel>,
}

impl State {
    pub fn new(
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

    pub fn panel_moment(&self) -> Moment {
        let rot = self.transform.rotation.0;
        let vel = self.momentum / self.mass.rotated(rot);

        self.panels
            .iter()
            .map(|panel| panel.to_moment(&vel, &rot))
            .fold(Moment::ZERO, |acc, e| acc + e)
    }

    pub fn velocity(&self) -> Velocity {
        self.momentum / self.mass.rotated(self.transform.rotation.0)
    }
}

/// Time step functions
impl State{

    /// Steps the state forward by a [Duration] using the Forward Euler method
    ///
    /// The Euler method is much simpler than Runge Kutta 4 and requires less compute per
    /// iteration, it does however result in more error over time and can become unstable easier
    pub fn forward_euler(&mut self, time: Duration) {
        self.momentum += self.panel_moment() * time;

        let velocity = self.momentum / self.mass.rotated(self.transform.rotation.0);
        self.transform += velocity * time;
    }


    /// Steps the state forward by a [Duration] using the Runge Kutta 4 method
    ///
    /// Runge Kutta 4 is a more robust way to step forward a simulation, compared to the Forward
    /// Euler method, it requires more compute, but the results are more accurate and stable.
    pub fn runge_kutta_4(&mut self, delta: Duration) {
        let half_delta = delta / 2;

        let mut int = self.clone();
        let (k1_x, k1_p) = (int.velocity(), self.panel_moment());
        int.momentum += k1_p * half_delta;
        int.transform += k1_x * half_delta;

        let (k2_x, k2_p) = (int.velocity(), self.panel_moment());
        int.momentum += k2_p * half_delta;
        int.transform += k2_x * half_delta;

        let (k3_x, k3_p) = (int.velocity(), self.panel_moment());
        int.momentum += k3_p * delta;
        int.transform += k3_x * delta;

        let (k4_x, k4_p) = (int.velocity(), self.panel_moment());

        self.momentum += (k1_p + k2_p * 2. + k3_p * 2. + k4_p) * (delta / 6);
        self.transform += (k1_x + k2_x * 2. + k3_x * 2. + k4_x) * (delta / 6);
    }
}
