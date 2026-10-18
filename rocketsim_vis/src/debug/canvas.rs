//! Ergonomic drawing helpers for retained debug groups.
//!
//! `text_2d` positions use normalized screen coordinates in `0..=1`, with
//! the origin at the top-left. The raw [`DrawList`] API remains available.

use super::{
    DebugRenderer, GroupHandle, GroupOptions,
    types::{
        Color, DebugItem, DebugResult, DepthMode, DrawList, LabelItem, LabelPos, Lifetime,
        LineItem, PointItem, PolylineItem, RectBorder, RectItem, TextAlign, Width,
    },
};

/// A short-lived builder that submits its items as one retained group.
#[derive(Default)]
pub struct Canvas {
    list: DrawList,
}

impl Canvas {
    pub fn new() -> Self {
        Self::default()
    }

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

    pub fn text_2d(&mut self, pos: [f32; 2], text: impl Into<String>) -> &mut LabelItem {
        self.label(LabelPos::Screen(pos), text)
    }

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
    /// Replace the reserved `debug/frame` group for one physics frame.
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
    pub fn draw(&self, draw: impl FnOnce(&mut Canvas)) -> DebugResult<u64> {
        self.draw_with(GroupOptions::default(), draw)
    }

    /// Replace this group with a drawing and custom group options.
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
    pub fn color(&mut self, value: Color) -> &mut Self {
        self.color = value;
        self
    }
    pub fn width_px(&mut self, value: f32) -> &mut Self {
        self.width = Width::Pixels(value);
        self
    }
    pub fn width_world(&mut self, value: f32) -> &mut Self {
        self.width = Width::World(value);
        self
    }
    pub fn depth(&mut self, value: DepthMode) -> &mut Self {
        self.depth = value;
        self
    }
}
impl PolylineItem {
    pub fn color(&mut self, value: Color) -> &mut Self {
        self.color = value;
        self
    }
    pub fn width_px(&mut self, value: f32) -> &mut Self {
        self.width = Width::Pixels(value);
        self
    }
    pub fn width_world(&mut self, value: f32) -> &mut Self {
        self.width = Width::World(value);
        self
    }
    pub fn depth(&mut self, value: DepthMode) -> &mut Self {
        self.depth = value;
        self
    }
    pub fn closed(&mut self, value: bool) -> &mut Self {
        self.closed = value;
        self
    }
}
impl PointItem {
    pub fn color(&mut self, value: Color) -> &mut Self {
        self.color = value;
        self
    }
    pub fn size_px(&mut self, value: f32) -> &mut Self {
        self.size = Width::Pixels(value);
        self
    }
    pub fn size_world(&mut self, value: f32) -> &mut Self {
        self.size = Width::World(value);
        self
    }
    pub fn depth(&mut self, value: DepthMode) -> &mut Self {
        self.depth = value;
        self
    }
}
impl LabelItem {
    pub fn color(&mut self, value: Color) -> &mut Self {
        self.color = value;
        self
    }
    pub fn size(&mut self, value: f32) -> &mut Self {
        self.size = value;
        self
    }
    pub fn background(&mut self, value: Option<Color>) -> &mut Self {
        self.bg = value;
        self
    }
    pub fn align(&mut self, value: TextAlign) -> &mut Self {
        self.align = value;
        self
    }
}
impl RectItem {
    pub fn fill(&mut self, value: Color) -> &mut Self {
        self.fill = value;
        self
    }
    pub fn border(&mut self, color: Color, width: Width) -> &mut Self {
        self.border = Some(RectBorder { color, width });
        self
    }
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
