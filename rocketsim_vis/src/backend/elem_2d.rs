use glam::Vec2;

use crate::backend::Color;

#[derive(Debug, Clone)]
pub enum Elem2D {
    Circle {
        pos: Vec2,
        radius: f32,
        color: Color,
    },
    Rect {
        a: Vec2,
        b: Vec2,
        color: Color,
        rounding: f32,
    },
    Text {
        string: String,
        pos: Vec2,
        centered: bool,
        color: Color,
    },
}
