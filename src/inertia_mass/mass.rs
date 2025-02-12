#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mass(pub f64);

impl Mass {
    pub const fn new(mass: f64) -> Self {
        Self(mass)
    }
}
