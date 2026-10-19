use std::collections::{HashMap, HashSet};

use egui::{Color32, Pos2, Rect, TextureHandle};
use glam::Mat4;
use rocketsim_vis::debug::{
    Color, DebugItem, DebugScene, LabelPos, RectItem, TextAlign, TextureResource, Width,
};

/// Cached egui textures for retained debug-scene assets.
#[derive(Default)]
pub struct OverlayCache {
    textures: HashMap<String, (u64, bool, TextureHandle)>,
}

impl OverlayCache {
    pub fn new() -> Self {
        Self::default()
    }

    /// Update textures referenced by visible rectangles and release stale handles.
    pub fn sync(&mut self, ctx: &egui::Context, scene: &DebugScene) {
        let mut referenced = HashSet::new();
        for group in scene.groups.values().filter(|group| group.visible) {
            for item in group.items.iter() {
                if let DebugItem::Rect(rect) = item
                    && let Some(id) = &rect.texture
                {
                    referenced.insert(id);
                    self.sync_texture(ctx, scene, id);
                }
            }
        }
        self.textures.retain(|id, _| referenced.contains(id));
    }

    fn sync_texture(&mut self, ctx: &egui::Context, scene: &DebugScene, id: &str) {
        if let Some(resource) = scene.textures.get(id) {
            self.load_texture(ctx, id, resource, false);
        } else if self.textures.get(id).is_none_or(|(revision, builtin, _)| {
            *revision != super::cache::BUILTIN_REVISION || !*builtin
        }) && let Some(resource) = super::cache::builtin_texture(id)
        {
            self.load_texture(ctx, id, &resource, true);
        }
    }

    fn load_texture(
        &mut self,
        ctx: &egui::Context,
        id: &str,
        resource: &TextureResource,
        builtin: bool,
    ) {
        let revision = resource.revision;
        if self
            .textures
            .get(id)
            .is_some_and(|(cached_revision, cached_builtin, _)| {
                *cached_revision == revision && *cached_builtin == builtin
            })
        {
            return;
        }
        let image = egui::ColorImage::from_rgba_unmultiplied(
            [resource.width as usize, resource.height as usize],
            &resource.rgba,
        );
        let handle = ctx.load_texture(id, image, egui::TextureOptions::LINEAR);
        self.textures
            .insert(id.to_owned(), (revision, builtin, handle));
    }

    /// Paint visible overlay items. World labels are projected but are not depth-tested.
    pub fn paint(
        &mut self,
        ui: &egui::Ui,
        rect: Rect,
        scene: &DebugScene,
        state: Option<&rocketsim::ArenaState>,
        view: &Mat4,
        proj: &Mat4,
    ) {
        let mut referenced = HashSet::new();
        let painter = ui.painter().with_clip_rect(rect);
        for (name, group) in &scene.groups {
            if !group.visible {
                continue;
            }
            let group_matrix = super::transform::resolve_group_matrix(group, state);
            for item in group.items.iter() {
                match item {
                    DebugItem::Rect(item) => {
                        if let Some(id) = &item.texture {
                            referenced.insert(id);
                            self.sync_texture(ui.ctx(), scene, id);
                        }
                        paint_rect(&painter, rect, item, &self.textures);
                    }
                    DebugItem::Label(item) => {
                        let pos = match item.pos {
                            LabelPos::Screen(p) => Some(normalized_pos(rect, p)),
                            LabelPos::World(p) => group_matrix.and_then(|matrix| {
                                project_world(
                                    rect,
                                    view,
                                    proj,
                                    matrix.transform_point3(glam::Vec3::from(p)),
                                )
                            }),
                        };
                        if let Some(pos) = pos {
                            let font = egui::FontId::proportional(item.size);
                            let color = color32(item.color);
                            let galley = painter.layout_no_wrap(item.text.clone(), font, color);
                            let left = match item.align {
                                TextAlign::Left => pos.x,
                                TextAlign::Center => pos.x - galley.size().x * 0.5,
                                TextAlign::Right => pos.x - galley.size().x,
                            };
                            let text_rect = Rect::from_min_size(
                                Pos2::new(left, pos.y - galley.size().y * 0.5),
                                galley.size(),
                            );
                            if let Some(bg) = item.bg {
                                painter.rect_filled(text_rect.expand(3.0), 0.0, color32(bg));
                            }
                            painter.galley(text_rect.min, galley, color);
                        }
                    }
                    _ => {
                        let _ = name;
                    }
                }
            }
        }
        self.textures.retain(|id, _| referenced.contains(id));
    }
}

fn normalized_pos(rect: Rect, p: [f32; 2]) -> Pos2 {
    Pos2::new(
        rect.left() + p[0] * rect.width(),
        rect.top() + p[1] * rect.height(),
    )
}

fn project_world(rect: Rect, view: &Mat4, proj: &Mat4, point: glam::Vec3) -> Option<Pos2> {
    let clip = *proj * *view * point.extend(1.0);
    if !clip.is_finite() || clip.w <= 0.0 || clip.z < -clip.w || clip.z > clip.w {
        return None;
    }
    let ndc = clip.truncate() / clip.w;
    if !ndc.x.is_finite() || !ndc.y.is_finite() {
        return None;
    }
    let pos = Pos2::new(
        rect.left() + (ndc.x + 1.0) * 0.5 * rect.width(),
        rect.top() + (1.0 - ndc.y) * 0.5 * rect.height(),
    );
    rect.contains(pos).then_some(pos)
}

fn paint_rect(
    painter: &egui::Painter,
    viewport: Rect,
    item: &RectItem,
    textures: &HashMap<String, (u64, bool, TextureHandle)>,
) {
    let min = normalized_pos(viewport, item.min);
    let max = normalized_pos(viewport, item.max);
    let target = Rect::from_two_pos(min, max).intersect(viewport);
    if !target.is_positive() {
        return;
    }
    if let Some(id) = &item.texture {
        if let Some((_, _, texture)) = textures.get(id) {
            painter.image(
                texture.id(),
                target,
                Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
                color32(item.fill),
            );
        } else {
            painter.rect_filled(target, 0.0, color32(item.fill));
        }
    } else {
        painter.rect_filled(target, 0.0, color32(item.fill));
    }
    if let Some(border) = item.border {
        let width = match border.width {
            Width::Pixels(px) => px,
            Width::World(_) => 1.0,
        };
        painter.rect_stroke(
            target,
            0.0,
            egui::Stroke::new(width.max(0.0), color32(border.color)),
            egui::StrokeKind::Inside,
        );
    }
}

fn color32(color: Color) -> Color32 {
    let channel = |value: f32| (value.clamp(0.0, 1.0) * 255.0).round() as u8;
    Color32::from_rgba_unmultiplied(
        channel(color.r),
        channel(color.g),
        channel(color.b),
        channel(color.a),
    )
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use egui::Vec2;
    use rocketsim_vis::debug::{
        BuiltinTexture, DebugGroup, DebugItem, DebugScene, DrawList, GroupOptions, RectItem,
    };

    use super::*;

    fn paint_frame(context: &egui::Context, cache: &mut OverlayCache, scene: &DebugScene) {
        let mut output = context.run_ui(egui::RawInput::default(), |ui| {
            egui::CentralPanel::default().show(ui, |ui| {
                let rect = ui.max_rect();
                cache.paint(ui, rect, scene, None, &Mat4::IDENTITY, &Mat4::IDENTITY);
            });
        });
        output.textures_delta.clear();
    }

    #[test]
    fn overlay_coordinates_and_builtin_rectangles_share_one_paint_scenario() {
        let viewport = Rect::from_min_size(Pos2::new(10.0, 20.0), Vec2::new(200.0, 100.0));
        assert_eq!(
            normalized_pos(viewport, [0.5, 0.25]),
            Pos2::new(110.0, 45.0)
        );
        assert_eq!(
            project_world(viewport, &Mat4::IDENTITY, &Mat4::IDENTITY, glam::Vec3::ZERO),
            Some(Pos2::new(110.0, 70.0))
        );
        assert_eq!(
            project_world(
                viewport,
                &Mat4::IDENTITY,
                &Mat4::IDENTITY,
                glam::Vec3::new(0.0, 0.0, 2.0)
            ),
            None
        );

        let mut draw = DrawList::new();
        draw.items.push(DebugItem::Rect(RectItem {
            min: [0.1, 0.1],
            max: [0.9, 0.9],
            fill: Color::WHITE,
            border: None,
            texture: Some(BuiltinTexture::Ball.id().to_owned()),
        }));
        let group = Arc::new(DebugGroup::new("rect", draw, GroupOptions::default()).unwrap());
        let mut scene = DebugScene::empty();
        scene.groups.insert(group.name.clone(), group);
        let scene = Arc::new(scene);
        let context = egui::Context::default();
        let mut cache = OverlayCache::default();
        paint_frame(&context, &mut cache, &scene);
        assert!(
            cache
                .textures
                .get(BuiltinTexture::Ball.id())
                .is_some_and(|(_, builtin, _)| *builtin)
        );

        let empty = Arc::new(DebugScene::empty());
        paint_frame(&context, &mut cache, &empty);
        assert!(cache.textures.is_empty());
    }
}
