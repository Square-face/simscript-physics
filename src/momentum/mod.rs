use glam::DVec3 as Vec3;
use overload::overload;
use std::{iter::Sum, ops, time::Duration};

pub use angular_momentum::AngMom;
pub use linear_momentum::LinMom;

use crate::{inertia_mass::InertiaMass, velocity::Velocity};

mod angular_momentum;
mod linear_momentum;

/// Represents momentum with linear and angular components.
///
/// Encapsulates translational momentum [Momentum::linear] and rotational momentum
/// [Momentum::angular] for a strongly typed representation of momentum.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct Momentum {
    /// Linear momentum component.
    pub linear: LinMom,
    /// Angular momentum component.
    pub angular: AngMom,
}

impl Momentum {
    /// Zero momentum constant (no momentum).
    pub const ZERO: Self = Self::new(LinMom::ZERO, AngMom::ZERO);

    /// Constructs a new [Momentum] from given linear and angular momenta.
    #[inline]
    #[must_use]
    pub const fn new(lin: LinMom, ang: AngMom) -> Self {
        Self { linear: lin, angular: ang }
    }

    /// Creates momentum with only a linear component.
    #[inline]
    #[must_use]
    pub const fn from_linear(v: LinMom) -> Self {
        Self::new(v, AngMom::ZERO)
    }

    /// Creates momentum with only an angular component.
    #[inline]
    #[must_use]
    pub const fn from_angular(v: AngMom) -> Self {
        Self::new(LinMom::ZERO, v)
    }

    /// Constructs a [Momentum] from raw vector representations of linear and angular momentum.
    #[inline]
    #[must_use]
    pub const fn from_vec3s(lin: Vec3, ang: Vec3) -> Self {
        Self::new(LinMom::from_vec3(lin), AngMom::from_vec3(ang))
    }

    /// Constructs a [Momentum] from a raw vector representing only linear momentum.
    #[inline]
    #[must_use]
    pub const fn from_linear_vec3(v: Vec3) -> Self {
        Self::from_vec3s(v, Vec3::ZERO)
    }

    /// Constructs a [Momentum] from a raw vector representing only angular momentum.
    #[inline]
    #[must_use]
    pub const fn from_angular_vec3(v: Vec3) -> Self {
        Self::from_vec3s(Vec3::ZERO, v)
    }
}

/// Conversion implementations to create [Momentum] from individual components.
impl From<LinMom> for Momentum {
    #[inline]
    #[must_use]
    fn from(value: LinMom) -> Self {
        Self::from_linear(value)
    }
}

impl From<AngMom> for Momentum {
    #[inline]
    #[must_use]
    fn from(value: AngMom) -> Self {
        Self::from_angular(value)
    }
}

/// Implements summation over an iterator of [Momentum] values.
impl Sum for Momentum {
    #[inline]
    #[must_use]
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::ZERO, |a, b| a + b)
    }
}

overload!((a: ?Momentum) + (b: ?Momentum) -> Momentum { Momentum::new(a.linear + b.linear, a.angular + b.angular) });
overload!((a: ?Momentum) - (b: ?Momentum) -> Momentum { Momentum::new(a.linear - b.linear, a.angular - b.angular) });
overload!((a: &mut Momentum) += (b: ?Momentum) { a.linear += b.linear; a.angular += b.angular; });
overload!((a: &mut Momentum) -= (b: ?Momentum) { a.linear -= b.linear; a.angular -= b.angular; });

overload!((a: ?Momentum) * (b: f64) -> Momentum { Momentum::new(a.linear * b, a.angular * b) });
overload!((a: ?Momentum) / (b: f64) -> Momentum { Momentum::new(a.linear / b, a.angular / b) });
overload!((a: &mut Momentum) *= (b: f64) { a.linear *= b; a.angular *= b; });
overload!((a: &mut Momentum) /= (b: f64) { a.linear /= b; a.angular /= b; });

overload!((a: ?Momentum) / (b: ?InertiaMass) -> Velocity { Velocity::new(a.linear / b.mass, a.angular / b.inv_inertia) });

overload!(-(a: ?Momentum) -> Momentum { Momentum { linear: -a.linear, angular: -a.angular } });
