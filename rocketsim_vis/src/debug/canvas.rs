//! Fluent helpers for debug drawings.
//!
//! Use [`DebugRenderer::draw_frame`] for marks that remain through the next
//! simulation update and expire at the following update. Use
//! [`GroupHandle::draw`] for persistent marks. Each helper adds one item and
//! returns it for styling. Three-dimensional positions use group-local Unreal
//! units. [`Canvas::text_2d`] uses viewport fractions, with the origin at the
//! top-left. Use [`DrawList`] to build items for explicit submission.
//!
//! # Example
//!
//! ```
//! use rocketsim_vis::debug::{Color, DebugRenderer, DepthMode};
//!
//! let renderer = DebugRenderer::new_detached();
//! let revision = renderer.draw_frame(|draw| {
//!     draw.line([0.0, 0.0, 0.0], [10.0, 0.0, 0.0])
//!         .color(Color::RED)
//!         .width_px(2.0)
//!         .depth(DepthMode::Xray);
//!     draw.text_2d([0.1, 0.1], "origin");
//! }).unwrap();
//! assert!(revision > 0);
//! ```

use super::{
    DebugRenderer, GroupHandle, GroupOptions,
    types::{
        Color, DebugItem, DebugResult, DepthMode, DrawList, LabelItem, LabelPos, Lifetime,
        LineItem, PointItem, PolylineItem, RectBorder, RectItem, TextAlign, Width,
    },
};

/// A drawing builder supplied to frame and group drawing closures.
///
/// Add items to the canvas supplied by [`DebugRenderer::draw_frame`] or
/// [`GroupHandle::draw`]. The method submits the finished drawing after the
/// closure returns. Styling methods do not validate immediately. Submission
/// returns an error if a value is invalid.
///
/// New lines and polylines are white, 2 pixels wide, and depth-tested. New
/// points are white, 4 pixels wide, and depth-tested. New labels are white,
/// 16 logical points high, left-aligned, and have no background. New rectangles
/// have a white fill and no border.
#[derive(Default)]
pub struct Canvas {
    list: DrawList,
}

impl Canvas {
    /// Create an empty canvas.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a line between two group-local points, in Unreal units.
    pub fn line(&mut self, start: [f32; 3], end: [f32; 3]) -> &mut LineItem {
        self.list.items.push(DebugItem::Line(LineItem {
            start,
            end,
            color: Color::WHITE,
            width: Width::Pixels(2.0),
            depth: DepthMode::Test,
        }));
        match self.list.items.last_mut().unwrap() {
            DebugItem::Line(v) => v,
            _ => unreachable!(),
        }
    }

    /// Add a connected line through group-local points, in Unreal units.
    ///
    /// The default line is open. Supply 2 through
    /// [`MAX_POLYLINE_POINTS`](super::MAX_POLYLINE_POINTS) points.
    pub fn polyline(&mut self, points: impl Into<Vec<[f32; 3]>>) -> &mut PolylineItem {
        self.list.items.push(DebugItem::Polyline(PolylineItem {
            points: points.into(),
            color: Color::WHITE,
            width: Width::Pixels(2.0),
            depth: DepthMode::Test,
            closed: false,
        }));
        match self.list.items.last_mut().unwrap() {
            DebugItem::Polyline(v) => v,
            _ => unreachable!(),
        }
    }

    /// Add a point at a group-local position, in Unreal units.
    pub fn point(&mut self, pos: [f32; 3]) -> &mut PointItem {
        self.list.items.push(DebugItem::Point(PointItem {
            pos,
            color: Color::WHITE,
            size: Width::Pixels(4.0),
            depth: DepthMode::Test,
        }));
        match self.list.items.last_mut().unwrap() {
            DebugItem::Point(v) => v,
            _ => unreachable!(),
        }
    }

    /// Add a screen label at normalized coordinates; `[0, 0]` is top-left.
    pub fn text_2d(&mut self, pos: [f32; 2], text: impl Into<String>) -> &mut LabelItem {
        self.label(LabelPos::Screen(pos), text)
    }

    /// Add a projected label at a group-local position, in Unreal units.
    ///
    /// The viewer draws it as an overlay without depth testing.
    pub fn text_3d(&mut self, pos: [f32; 3], text: impl Into<String>) -> &mut LabelItem {
        self.label(LabelPos::World(pos), text)
    }

    fn label(&mut self, pos: LabelPos, text: impl Into<String>) -> &mut LabelItem {
        self.list.items.push(DebugItem::Label(LabelItem {
            text: text.into(),
            pos,
            size: 16.0,
            align: TextAlign::Left,
            color: Color::WHITE,
            bg: None,
        }));
        match self.list.items.last_mut().unwrap() {
            DebugItem::Label(v) => v,
            _ => unreachable!(),
        }
    }

    /// Add a screen rectangle from its normalized top-left and bottom-right.
    pub fn rect(&mut self, min: [f32; 2], max: [f32; 2]) -> &mut RectItem {
        self.list.items.push(DebugItem::Rect(RectItem {
            min,
            max,
            fill: Color::WHITE,
            border: None,
            texture: None,
        }));
        match self.list.items.last_mut().unwrap() {
            DebugItem::Rect(v) => v,
            _ => unreachable!(),
        }
    }

    pub(crate) fn into_list(self) -> DrawList {
        self.list
    }
}

impl DebugRenderer {
    /// Replace the reserved `debug/frame` group with a temporary drawing.
    ///
    /// The drawing remains through the next simulation update and expires at
    /// the following update. Call this once per tick to refresh moving marks.
    /// Return the new group revision. The closure runs synchronously.
    ///
    /// # Errors
    ///
    /// Return the validation, resource, or scene-limit errors from
    /// [`Self::replace_group`]. The previous drawing remains if replacement
    /// fails, unless its existing lifetime has expired.
    pub fn draw_frame(&self, draw: impl FnOnce(&mut Canvas)) -> DebugResult<u64> {
        let mut canvas = Canvas::new();
        draw(&mut canvas);
        self.replace_group(
            "debug/frame",
            canvas.into_list(),
            GroupOptions {
                lifetime: Lifetime::Frame,
                ..GroupOptions::default()
            },
        )
    }
}

impl GroupHandle {
    /// Replace this group with a persistent drawing.
    ///
    /// Reset all options to their defaults, including visibility, world
    /// anchor, identity transform, and persistent lifetime. This does not
    /// preserve a previous transform. Use [`Self::draw_with`] for custom
    /// options. Return the new revision or a submission error.
    pub fn draw(&self, draw: impl FnOnce(&mut Canvas)) -> DebugResult<u64> {
        self.draw_with(GroupOptions::default(), draw)
    }

    /// Replace this group with a drawing and custom group options.
    ///
    /// Apply the supplied options and start a new lifetime. Return the new
    /// revision or the errors described by [`DebugRenderer::replace_group`].
    pub fn draw_with(
        &self,
        options: GroupOptions,
        draw: impl FnOnce(&mut Canvas),
    ) -> DebugResult<u64> {
        let mut canvas = Canvas::new();
        draw(&mut canvas);
        self.replace(canvas.into_list(), options)
    }
}

impl LineItem {
    /// Set the line color.
    pub fn color(&mut self, value: Color) -> &mut Self {
        self.color = value;
        self
    }
    /// Set line width in screen pixels.
    pub fn width_px(&mut self, value: f32) -> &mut Self {
        self.width = Width::Pixels(value);
        self
    }
    /// Set line width in Unreal world units.
    pub fn width_world(&mut self, value: f32) -> &mut Self {
        self.width = Width::World(value);
        self
    }
    /// Set the line depth mode.
    pub fn depth(&mut self, value: DepthMode) -> &mut Self {
        self.depth = value;
        self
    }
}
impl PolylineItem {
    /// Set the polyline color.
    pub fn color(&mut self, value: Color) -> &mut Self {
        self.color = value;
        self
    }
    /// Set line width in screen pixels.
    pub fn width_px(&mut self, value: f32) -> &mut Self {
        self.width = Width::Pixels(value);
        self
    }
    /// Set line width in Unreal world units.
    pub fn width_world(&mut self, value: f32) -> &mut Self {
        self.width = Width::World(value);
        self
    }
    /// Set the polyline depth mode.
    pub fn depth(&mut self, value: DepthMode) -> &mut Self {
        self.depth = value;
        self
    }
    /// Set whether the final point connects to the first point.
    pub fn closed(&mut self, value: bool) -> &mut Self {
        self.closed = value;
        self
    }
}
impl PointItem {
    /// Set the point color.
    pub fn color(&mut self, value: Color) -> &mut Self {
        self.color = value;
        self
    }
    /// Set point size in screen pixels.
    pub fn size_px(&mut self, value: f32) -> &mut Self {
        self.size = Width::Pixels(value);
        self
    }
    /// Set point size in Unreal world units.
    pub fn size_world(&mut self, value: f32) -> &mut Self {
        self.size = Width::World(value);
        self
    }
    /// Set the point depth mode.
    pub fn depth(&mut self, value: DepthMode) -> &mut Self {
        self.depth = value;
        self
    }
}
impl LabelItem {
    /// Set the label color.
    pub fn color(&mut self, value: Color) -> &mut Self {
        self.color = value;
        self
    }
    /// Set font size in viewer logical points. Valid values are in `4..=128`.
    pub fn size(&mut self, value: f32) -> &mut Self {
        self.size = value;
        self
    }
    /// Set a background color, or `None` for no background.
    pub fn background(&mut self, value: Option<Color>) -> &mut Self {
        self.bg = value;
        self
    }
    /// Set horizontal text alignment.
    pub fn align(&mut self, value: TextAlign) -> &mut Self {
        self.align = value;
        self
    }
}
impl RectItem {
    /// Set the rectangle fill color.
    pub fn fill(&mut self, value: Color) -> &mut Self {
        self.fill = value;
        self
    }
    /// Set border color and width. Use [`Width::Pixels`] for logical points.
    ///
    /// The current viewer uses a fixed one-point border for [`Width::World`].
    pub fn border(&mut self, color: Color, width: Width) -> &mut Self {
        self.border = Some(RectBorder { color, width });
        self
    }
    /// Remove the rectangle border.
    pub fn no_border(&mut self) -> &mut Self {
        self.border = None;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fluent_defaults_and_styles_build_valid_drawing() {
        let mut draw = Canvas::new();
        draw.line([0.0; 3], [1.0; 3])
            .color(Color::RED)
            .width_px(3.0)
            .depth(DepthMode::Xray);
        draw.point([2.0; 3]).size_world(5.0);
        draw.text_2d([0.1, 0.1], "Test")
            .size(20.0)
            .background(Some(Color::BLACK));
        let list = draw.into_list();
        list.validate("test").unwrap();
        let DebugItem::Line(line) = &list.items[0] else {
            panic!("expected line")
        };
        assert_eq!(line.color, Color::RED);
        assert_eq!(line.width, Width::Pixels(3.0));
        assert_eq!(line.depth, DepthMode::Xray);
    }

    #[test]
    fn frame_lasts_through_next_tick_and_group_replacement_is_independent() {
        let renderer = DebugRenderer::new_detached();
        renderer
            .group("persistent")
            .draw(|draw| {
                draw.point([0.0; 3]);
            })
            .unwrap();
        renderer
            .draw_frame(|draw| {
                draw.line([0.0; 3], [1.0; 3]);
            })
            .unwrap();
        renderer.advance_tick();
        assert!(renderer.snapshot().groups.contains_key("debug/frame"));
        renderer.advance_tick();
        let scene = renderer.snapshot();
        assert!(!scene.groups.contains_key("debug/frame"));
        assert!(scene.groups.contains_key("persistent"));
    }
}
