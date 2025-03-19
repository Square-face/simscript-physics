#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// The mass an object.
///
/// This struct only exists to allow strongly typed equations with mass to be possible.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mass(pub f64);

impl Mass {
    pub const ZERO: Self = Self::new(0.);

    pub const fn new(mass: f64) -> Self {
        Self(mass)
    }
}

impl From<f64> for Mass {
    fn from(value: f64) -> Self {
        Self::new(value)
    }
}

impl From<Mass> for f64 {
    fn from(value: Mass) -> Self {
        value.0
    }
}
