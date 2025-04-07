#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "approx")]
use {
    approx::{AbsDiffEq, RelativeEq, UlpsEq},
    approx_derive::Approx,
};

use glam::{DQuat as Quat, DVec3 as Vec3};
use overload::overload;
use std::{iter::Sum, ops};

mod rotation;
mod translation;

pub use rotation::Rotation;
pub use translation::Translation;

/// Represents a 3D transformation consisting of a translation and rotation component.
///
/// This struct combines a position ([Translation]) and orientation ([Rotation]) in 3D space.
/// It provides various constructors and operations for working with transformations.
#[cfg_attr(feature = "approx", derive(Approx))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    /// The translation component of the transformation
    pub translation: Translation,
    /// The rotation component of the transformation
    pub rotation: Rotation,
}

impl Transform {
    /// A constant representing a zero transformation (no translation, identity rotation).
    pub const ZERO: Self = Self::new(Translation::ZERO, Rotation::ZERO);

    /// Creates a new transformation from translation and rotation components.
    ///
    /// # Arguments
    /// * `lin` - The translation component ([Translation])
    /// * `ang` - The rotation component ([Rotation])
    ///
    /// # Returns
    /// A new [Transform] instance with the specified components
    #[inline]
    #[must_use]
    pub const fn new(lin: Translation, ang: Rotation) -> Self {
        Self {
            translation: lin,
            rotation: ang,
        }
    }

    /// Creates a transformation with only a translation component (zero rotation).
    ///
    /// # Arguments
    /// * `v` - The translation component ([Translation])
    ///
    /// # Returns
    /// A new [Transform] with the specified translation and identity rotation
    #[inline]
    #[must_use]
    pub const fn from_translation(v: Translation) -> Self {
        Self::new(v, Rotation::ZERO)
    }

    /// Creates a transformation with only a rotation component (zero translation).
    ///
    /// # Arguments
    /// * `v` - The rotation component ([Rotation])
    ///
    /// # Returns
    /// A new [Transform] with the specified rotation and zero translation
    #[inline]
    #[must_use]
    pub const fn from_rotation(v: Rotation) -> Self {
        Self::new(Translation::ZERO, v)
    }

    /// Creates a transformation from raw [Vec3] and [Quat] values.
    ///
    /// # Arguments
    /// * `pos` - The position as a 3D vector ([Vec3])
    /// * `rot` - The rotation as a quaternion ([Quat])
    ///
    /// # Returns
    /// A new [Transform] with the specified position and rotation
    #[inline]
    #[must_use]
    pub const fn from_inner(pos: Vec3, rot: Quat) -> Self {
        Self::new(Translation::from_vec3(pos), Rotation::new(rot))
    }

    /// Creates a transformation from a position vector (identity rotation).
    ///
    /// # Arguments
    /// * `v` - The position as a 3D vector ([Vec3])
    ///
    /// # Returns
    /// A new [Transform] with the specified position and identity rotation
    #[inline]
    #[must_use]
    pub const fn from_vec3(v: Vec3) -> Self {
        Self::from_inner(v, Quat::IDENTITY)
    }

    /// Creates a transformation from a rotation quaternion (zero translation).
    ///
    /// # Arguments
    /// * `v` - The rotation as a quaternion ([Quat])
    ///
    /// # Returns
    /// A new [Transform] with the specified rotation and zero translation
    #[inline]
    #[must_use]
    pub const fn from_quat(v: Quat) -> Self {
        Self::from_inner(Vec3::ZERO, v)
    }
}

/// Conversion from [Translation] to [Transform]
impl From<Translation> for Transform {
    /// Converts a [Translation] into a [Transform] with zero rotation.
    #[inline]
    #[must_use]
    fn from(value: Translation) -> Self {
        Self::from_translation(value)
    }
}

/// Conversion from [Rotation] to [Transform]
impl From<Rotation> for Transform {
    /// Converts a [Rotation] into a [Transform] with zero translation.
    #[inline]
    #[must_use]
    fn from(value: Rotation) -> Self {
        Self::from_rotation(value)
    }
}

/// Implementation of summing transformations
impl Sum for Transform {
    /// Computes the sum of an iterator of transformations.
    ///
    /// The translations and rotations are summed separately.
    ///
    /// # Arguments
    /// * `iter` - An iterator over [Transform] values
    ///
    /// # Returns
    /// The sum of all transformations in the iterator as a [Transform]
    #[inline]
    #[must_use]
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::ZERO, |a, b| a + b)
    }
}

// Operator overloads for transformation arithmetic
overload!((a: ?Transform) + (b: ?Transform) -> Transform {
    Transform {
        translation: a.translation + b.translation,
        rotation: a.rotation + b.rotation
    }
});

overload!((a: ?Transform) - (b: ?Transform) -> Transform {
    Transform {
        translation: a.translation - b.translation,
        rotation: a.rotation - b.rotation
    }
});

overload!((a: &mut Transform) += (b: ?Transform) {
    a.translation += b.translation;
    a.rotation += b.rotation;
});

overload!((a: &mut Transform) -= (b: ?Transform) {
    a.translation -= b.translation;
    a.rotation -= b.rotation;
});

overload!(-(a: ?Transform) -> Transform {
    Transform {
        translation: -a.translation,
        rotation: -a.rotation
    }
});
