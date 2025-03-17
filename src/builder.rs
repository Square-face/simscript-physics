use crate::State;
use crate::inertia_mass::InertiaMass;
use crate::momentum::Momentum;
use crate::panels::Panel;
use crate::transform::Transform;

/// Builder for `State`
#[derive(Debug, Default, Clone, PartialEq)]
pub struct StateBuilder {
    mass: Option<InertiaMass>,
    transform: Option<Transform>,
    momentum: Option<Momentum>,
    panels: Vec<Panel>,
}

impl StateBuilder {
    /// Creates a new `StateBuilder`
    pub const fn new() -> Self {
        Self {
            mass: None,
            transform: None,
            momentum: None,
            panels: Vec::new(),
        }
    }

    /// Sets the mass
    pub const fn mass(mut self, mass: InertiaMass) -> Self {
        self.mass = Some(mass);
        self
    }

    /// Sets the transform
    pub const fn transform(mut self, transform: Transform) -> Self {
        self.transform = Some(transform);
        self
    }

    /// Sets the momentum
    pub const fn momentum(mut self, momentum: Momentum) -> Self {
        self.momentum = Some(momentum);
        self
    }

    /// Adds a panel
    pub fn add_panel(mut self, panel: Panel) -> Self {
        self.panels.push(panel);
        self
    }

    /// Adds multiple panels
    pub fn add_panels(mut self, panels: Vec<Panel>) -> Self {
        self.panels.extend(panels);
        self
    }

    /// Sets all panels
    pub fn panels(mut self, panels: Vec<Panel>) -> Self {
        self.panels = panels;
        self
    }

    /// Builds the `State`, panicking if required fields are missing
    pub fn build(self) -> State {
        State {
            mass: self.mass.expect("mass must be set"),
            transform: self.transform.unwrap_or(Transform::ZERO),
            momentum: self.momentum.unwrap_or(Momentum::ZERO),
            panels: self.panels,
        }
    }
}
