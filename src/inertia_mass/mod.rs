pub use intertia::Inertia;
pub use mass::Mass;

mod intertia;
mod mass;


/// Represents the mass and its distribution in an entity
///
/// # Units
/// mass: kg
/// inertia: tensor based on kg
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InnertiaMass {
    pub mass: Mass,
    pub inertia: Inertia,
    pub inv_inertia: Inertia,
}

impl InnertiaMass {
    pub fn new(mass: Mass, inertia: Inertia) -> Self {
        Self {
            mass,
            inertia,
            inv_inertia: Inertia(inertia.0.inverse()),
        }
    }
}
