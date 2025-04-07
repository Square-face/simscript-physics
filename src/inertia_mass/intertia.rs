#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "approx")]
use {
    approx::{AbsDiffEq, RelativeEq, UlpsEq},
    approx_derive::Approx,
};

use glam::{DMat3 as Mat3, DQuat as Quat};

/// The mass distribution of an object.
///
/// Represents how the mass of an object is distributed, used to calculate rotational velocity from
/// momentum. Uses a [Mat3] internally.
#[cfg_attr(feature = "approx", derive(Approx))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inertia(pub Mat3);

impl Inertia {
    pub const fn new(inertia: Mat3) -> Self {
        Self(inertia)
    }

    /// Creates an inertia tensor for a uniform cylinder with its height along the x-axis.
    ///
    /// # Arguments
    /// * `height` - The height of the cylinder.
    /// * `radius` - The radius of the cylinder's cross-section.
    /// * `mass` - The mass of the cylinder.
    ///
    /// # Returns
    /// An [Inertia] object representing the cylinder.
    #[inline]
    #[must_use]
    pub const fn cylinder_x(height: f64, radius: f64, mass: f64) -> Self {
        let h2 = height * height;
        let r2 = radius * radius;
        let m = mass;

        let side = m * h2 / 12.0 + m * r2 / 4.0;
        let front = m * r2 / 2.0;

        Self::new(Mat3::from_cols_array_2d(&[
            [front, 0.0, 0.0],
            [0.0, side, 0.0],
            [0.0, 0.0, side],
        ]))
    }

    /// Creates an inertia tensor for a uniform cylinder with its height along the y-axis.
    ///
    /// # Arguments
    /// * `height` - The height of the cylinder.
    /// * `radius` - The radius of the cylinder's cross-section.
    /// * `mass` - The mass of the cylinder.
    ///
    /// # Returns
    /// An [Inertia] object representing the cylinder.
    #[inline]
    #[must_use]
    pub const fn cylinder_y(height: f64, radius: f64, mass: f64) -> Self {
        let h2 = height * height;
        let r2 = radius * radius;
        let m = mass;

        let side = m * h2 / 12.0 + m * r2 / 4.0;
        let front = m * r2 / 2.0;

        Self::new(Mat3::from_cols_array_2d(&[
            [side, 0.0, 0.0],
            [0.0, front, 0.0],
            [0.0, 0.0, side],
        ]))
    }

    /// Creates an inertia tensor for a uniform cylinder with its height along the z-axis.
    ///
    /// # Arguments
    /// * `height` - The height of the cylinder.
    /// * `radius` - The radius of the cylinder's cross-section.
    /// * `mass` - The mass of the cylinder.
    ///
    /// # Returns
    /// An [Inertia] object representing the cylinder.
    #[inline]
    #[must_use]
    pub const fn cylinder_z(height: f64, radius: f64, mass: f64) -> Self {
        let h2 = height * height;
        let r2 = radius * radius;
        let m = mass;

        let side = m * h2 / 12.0 + m * r2 / 4.0;
        let front = m * r2 / 2.0;

        Self::new(Mat3::from_cols_array_2d(&[
            [side, 0.0, 0.0],
            [0.0, side, 0.0],
            [0.0, 0.0, front],
        ]))
    }

    /// Rotates the inertia using a quaternion
    ///
    /// # Arguments
    /// * `rot` - How to rotate the tensor
    ///
    /// # Returns
    /// An [Inertia] that has been rotated
    pub fn rotated(&self, rot: Quat) -> Self {
        self.rot_mat(Mat3::from_quat(rot))
    }

    /// Rotates the inertia using a rotation matrix
    ///
    /// # Arguments
    /// * `rot` - How to rotate the tensor
    ///
    /// # Returns
    /// An [Inertia] that has been rotated
    pub fn rot_mat(&self, rot: Mat3) -> Self {
        Self::new(rot * self.0 * rot.transpose())
    }
}

impl From<Mat3> for Inertia {
    fn from(value: Mat3) -> Self {
        Self::new(value)
    }
}

impl From<Inertia> for Mat3 {
    fn from(value: Inertia) -> Self {
        value.0
    }
}
