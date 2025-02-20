use glam::DVec3 as Vec3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Force(pub Vec3);

impl Force {

    pub const ZERO: Self = Self::new(Vec3::ZERO);

    pub const fn new(force: Vec3) -> Self {
        Self(force)
    }

    pub const fn splat(f: f64) -> Self {
        Self(Vec3::splat(f))
    }

    pub fn magnitue(&self) -> f64 {
        self.0.length()
    }
}
