pub use angular_movement::AngMove;
pub use linear_movement::LinMove;
use overload::overload;
use std::ops;

mod angular_movement;
mod linear_movement;

/// Represents an entity's position and rotation
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position {
    pub translation: LinMove,
    pub rotation: AngMove,
}

impl Position {
    pub const fn new(lin: LinMove, ang: AngMove) -> Self {
        Self {
            translation: lin,
            rotation: ang,
        }
    }
}

overload!((a: ?Position) + (b: ?Position) -> Position{Position{ translation: a.translation + b.translation, rotation: a.rotation + b.rotation }});
overload!((a: ?Position) - (b: ?Position) -> Position{Position{ translation: a.translation - b.translation, rotation: a.rotation - b.rotation }});

overload!((a: &mut Position) += (b: ?Position) { a.translation += b.translation; a.rotation += b.rotation; });
overload!((a: &mut Position) -= (b: ?Position) { a.translation -= b.translation; a.rotation -= b.rotation; });

overload!(-(a: ?Position) -> Position{Position{ translation: -a.translation, rotation: -a.rotation }});
