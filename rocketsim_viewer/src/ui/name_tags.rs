//! Name tags anchored above cars in the 3D view.
//!
//! A tag is placed at a world-space point above the car. The offset scales
//! with camera distance so the tag keeps a constant screen gap above the car:
//! approaching a car does not make the tag float unusually high.

use glam::{Mat4, Vec3, Vec3A};
use rocketsim::{CarInfo, CarState, Team};
use rocketsim_vis::proto::CarNameDecl;

/// Screen gap between a car and its tag, in points.
const TAG_PIXELS_ABOVE: f32 = 30.0;
/// Blue team tag text. Closely matches the arena's blue, brightened for text.
const BLUE_TEXT: egui::Color32 = egui::Color32::from_rgb(90, 165, 255);
/// Orange team tag text. Closely matches the arena's orange.
const ORANGE_TEXT: egui::Color32 = egui::Color32::from_rgb(255, 155, 70);

/// Tag text color for a car's team.
pub(crate) fn team_color(team: Team) -> egui::Color32 {
    if team == Team::Blue {
        BLUE_TEXT
    } else {
        ORANGE_TEXT
    }
}

/// World-space offset that projects to `pixels` of screen separation.
///
/// Use the vertical field of view and viewport height in pixels. The result is
/// proportional to distance, so the projected gap stays constant as the camera
/// moves toward or away from the car.
pub(crate) fn world_offset_for_pixels(
    distance: f32,
    vfov_deg: f32,
    viewport_height: f32,
    pixels: f32,
) -> f32 {
    let half_tan = (0.5 * vfov_deg.to_radians()).tan().max(1e-3);
    distance.max(1.0) * 2.0 * half_tan * (pixels / viewport_height.max(1.0))
}

/// Return the tag text for one car, or `None` when it must not be shown.
///
/// The spectated car and demoed cars have no tag. An unnamed car has no tag.
pub(crate) fn tag_name(
    names: &CarNameDecl,
    hidden_car: Option<u32>,
    car_index: usize,
    demoed: bool,
) -> Option<&str> {
    if demoed {
        return None;
    }
    let index = u32::try_from(car_index).ok()?;
    if hidden_car == Some(index) {
        return None;
    }
    names.get(index)
}

/// Per-frame inputs for the name tag pass.
pub(crate) struct NameTagView<'a> {
    pub(crate) cars: &'a [(CarInfo, CarState)],
    pub(crate) names: &'a CarNameDecl,
    /// Car to hide, normally the one the viewer is spectating.
    pub(crate) hidden_car: Option<u32>,
    pub(crate) camera_position: Vec3A,
    /// Horizontal field of view in degrees, in viewer convention.
    pub(crate) camera_fov: f32,
    pub(crate) view: &'a Mat4,
    pub(crate) proj: &'a Mat4,
}

/// Paint all declared name tags over the 3D view.
pub(crate) fn paint(ui: &egui::Ui, rect: egui::Rect, tags: &NameTagView<'_>) {
    if tags.names.is_empty() {
        return;
    }
    let vfov = crate::render::view::hfov_to_vfov_deg(tags.camera_fov, rect.aspect_ratio());
    // World-space direction of view-space up. The transpose maps a view-space
    // axis back to world space because the view matrix is orthogonal.
    let up_world = (tags.view.transpose() * glam::Vec4::new(0.0, 1.0, 0.0, 0.0))
        .truncate()
        .normalize();
    let camera = Vec3::from(tags.camera_position);
    let painter = ui.painter().with_clip_rect(rect);

    for (info, state) in tags.cars {
        let Some(name) = tag_name(tags.names, tags.hidden_car, info.idx, state.is_demoed) else {
            continue;
        };
        let car_pos = Vec3::from(state.phys.pos);
        let distance = car_pos.distance(camera);
        let offset = world_offset_for_pixels(distance, vfov, rect.height(), TAG_PIXELS_ABOVE);
        let tag_world = car_pos + up_world * offset;
        let clip = *tags.proj * *tags.view * tag_world.extend(1.0);
        if clip.w <= 0.0 {
            continue;
        }
        let ndc = clip.truncate() / clip.w;
        if !ndc.is_finite() || ndc.z < -1.0 || ndc.z > 1.0 {
            continue;
        }
        let pos = egui::pos2(
            rect.left() + (ndc.x * 0.5 + 0.5) * rect.width(),
            rect.top() + (0.5 - ndc.y * 0.5) * rect.height(),
        );
        if !rect.contains(pos) {
            continue;
        }
        let color = team_color(info.team);
        let galley =
            painter.layout_no_wrap(name.to_string(), egui::FontId::proportional(15.0), color);
        let text_rect = egui::Rect::from_center_size(pos, galley.size());
        painter.rect_filled(
            text_rect.expand(3.0),
            4.0,
            egui::Color32::from_black_alpha(150),
        );
        painter.galley(text_rect.min, galley, color);
    }
}

#[cfg(test)]
mod tests {
    use rocketsim_vis::proto::CarNameDecl;

    use super::*;

    #[test]
    fn tag_text_uses_team_colors() {
        assert_eq!(team_color(Team::Blue), BLUE_TEXT);
        assert_eq!(team_color(Team::Orange), ORANGE_TEXT);
        assert_ne!(BLUE_TEXT, ORANGE_TEXT);
    }

    #[test]
    fn spectated_and_unnamed_cars_have_no_tag() {
        let names = CarNameDecl {
            rev: 1,
            names: vec![(0, "Alice".into()), (1, "Bob".into())],
        };
        assert_eq!(tag_name(&names, None, 0, false), Some("Alice"));
        assert_eq!(tag_name(&names, Some(0), 0, false), None);
        assert_eq!(tag_name(&names, Some(0), 1, false), Some("Bob"));
        assert_eq!(tag_name(&names, None, 1, true), None);
        assert_eq!(tag_name(&names, None, 7, false), None);
    }

    #[test]
    fn world_offset_keeps_screen_gap_constant_with_distance() {
        let near = world_offset_for_pixels(500.0, 60.0, 720.0, 30.0);
        let far = world_offset_for_pixels(1000.0, 60.0, 720.0, 30.0);
        assert!((far / near - 2.0).abs() < 1e-5);
        // Recomputing the pixel gap from the offset gives the requested value.
        let half_tan = (30.0_f32.to_radians()).tan();
        for distance in [200.0, 2000.0] {
            let offset = world_offset_for_pixels(distance, 60.0, 720.0, 30.0);
            let pixels = offset / (distance * 2.0 * half_tan) * 720.0;
            assert!((pixels - 30.0).abs() < 1e-3, "{distance}");
        }
    }
}
