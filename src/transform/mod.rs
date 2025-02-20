pub use rotation::Rotation;
pub use translation::Translation;
use overload::overload;
use std::ops;

mod rotation;
mod translation;

/// Represents an entity's translation and rotation
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    pub translation: Translation,
    pub rotation: Rotation,
}

impl Transform {
    pub const ZERO: Self = Self::new(Translation::ZERO, Rotation::ZERO);
    pub const fn new(lin: Translation, ang: Rotation) -> Self {
        Self {
            translation: lin,
            rotation: ang,
        }
    }
}

overload!((a: ?Transform) + (b: ?Transform) -> Transform{Transform{ translation: a.translation + b.translation, rotation: a.rotation + b.rotation }});
overload!((a: ?Transform) - (b: ?Transform) -> Transform{Transform{ translation: a.translation - b.translation, rotation: a.rotation - b.rotation }});

overload!((a: &mut Transform) += (b: ?Transform) { a.translation += b.translation; a.rotation += b.rotation; });
overload!((a: &mut Transform) -= (b: ?Transform) { a.translation -= b.translation; a.rotation -= b.rotation; });

overload!(-(a: ?Transform) -> Transform{Transform{ translation: -a.translation, rotation: -a.rotation }});
