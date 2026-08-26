use crate::backend::Color;
use glam::Vec2;

#[derive(Debug, Copy, Clone)]
pub enum Shape2D {
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
}
