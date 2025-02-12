use glam::{DQuat as Quat, DVec3 as Vec3};
use overload::overload;
use std::ops;

use crate::Position;

pub mod angular_movement;
pub mod linear_movement;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinMove(pub Vec3);

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngMove(pub Quat);

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
