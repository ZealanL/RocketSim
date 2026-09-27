use glam::Vec2;

use crate::backend::Color;

/// 2D HUD shape in normalized overlay coordinates.
///
/// Positions are centered: `(0, 0)` is screen center, `y` spans -1 to 1
/// (scaled by window height). Used for the boost meter and other overlays
/// in [`VisRenderState::shapes_2d`](crate::backend::VisRenderState::shapes_2d).
#[derive(Debug, Clone)]
pub enum Elem2D {
    /// Filled circle.
    Circle {
        /// Center in overlay coordinates.
        pos: Vec2,
        /// Radius in overlay units.
        radius: f32,
        /// Fill color.
        color: Color,
    },
    /// Filled rectangle from `a` to `b`.
    Rect {
        /// First corner in overlay coordinates.
        a: Vec2,
        /// Opposite corner in overlay coordinates.
        b: Vec2,
        /// Fill color.
        color: Color,
        /// Corner rounding in overlay units.
        rounding: f32,
    },
    /// Text label.
    Text {
        /// Text to draw.
        string: String,
        /// Anchor position in overlay coordinates.
        pos: Vec2,
        /// Center the text on `pos` instead of drawing from the top-left.
        centered: bool,
        /// Text color.
        color: Color,
    },
}
