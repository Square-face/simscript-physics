use glam::DMat3 as Mat3;

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
    /// An [`Inertia<Local>`] object representing the cylinder.
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
    /// An [`Inertia<Local>`] object representing the cylinder.
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
    /// An [`Inertia<Local>`] object representing the cylinder.
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
}
