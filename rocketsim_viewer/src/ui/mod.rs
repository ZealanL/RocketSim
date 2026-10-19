pub(crate) mod hud;

mod settings;

pub(crate) use settings::{SettingsInput, SettingsView, show_settings};

use egui::Rect;

use crate::{
    input::{Action, Capture, InputSource},
    settings::BindingDevice,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CameraChoice {
    Follow,
    Director,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum UiAction {
    ResetPlay,
    SetCamera(CameraChoice),
}

pub(crate) struct ViewerUiState {
    pub(crate) menu_open: bool,
    pub(crate) rebind_action: Option<(Action, usize)>,
    pub(crate) rebind_capture: Option<Capture>,
    pub(crate) capture_cancel_rect: Option<Rect>,
    pub(crate) capture_cancel_hold: f32,
    pub(crate) capture_start_back: Option<InputSource>,
    pub(crate) capture_pending_back: Option<InputSource>,
    pub(crate) binding_filter: String,
    pub(crate) binding_group: Option<String>,
    pub(crate) binding_device: BindingDevice,
    pub(crate) settings_tab: usize,
    pub(crate) reset_all_confirm: bool,
    pub(crate) reset_requested: bool,
    pub(crate) binding_warning: Option<String>,
    pub(crate) menu_focus: usize,
    pub(crate) menu_nav_active: bool,
    pub(crate) nav_repeat_cd: f32,
    pub(crate) nav_moved: bool,
    pub(crate) nav_ids: Vec<String>,
    pub(crate) applied_ui_scale: f32,
}

impl ViewerUiState {
    pub(crate) fn new(menu_open: bool) -> Self {
        Self {
            menu_open,
            rebind_action: None,
            rebind_capture: None,
            capture_cancel_rect: None,
            capture_cancel_hold: 0.0,
            capture_start_back: None,
            capture_pending_back: None,
            binding_filter: String::new(),
            binding_group: None,
            binding_device: BindingDevice::KeyboardMouse,
            settings_tab: 0,
            reset_all_confirm: false,
            reset_requested: false,
            binding_warning: None,
            menu_focus: 0,
            menu_nav_active: false,
            nav_repeat_cd: 0.0,
            nav_moved: false,
            nav_ids: Vec::new(),
            applied_ui_scale: 0.0,
        }
    }
}

pub(crate) fn draw_boost_gauge(
    ui: &egui::Ui,
    rect: egui::Rect,
    fraction: f32,
    label: &str,
    low: bool,
) {
    use std::f32::consts::TAU;
    let r = (rect.width().min(rect.height()) * 0.07).clamp(30.0, 64.0);
    let margin = 24.0;
    let center = rect.right_bottom() - egui::vec2(margin + r, margin + r);
    let painter = ui.painter();
    painter.circle_filled(center, r + 14.0, egui::Color32::from_black_alpha(110));
    painter.circle_stroke(
        center,
        r,
        egui::Stroke::new(9.0_f32, egui::Color32::from_white_alpha(70)),
    );
    if fraction > 0.001 {
        // Polyline arc (no egui arc primitive): 32 segments
        // clockwise from the top.
        let start = -std::f32::consts::FRAC_PI_2;
        let end = start + fraction.clamp(0.0, 1.0) * TAU;
        let pts: Vec<egui::Pos2> = (0..=32)
            .map(|k| {
                let a = start + (end - start) * (k as f32 / 32.0);
                center + egui::vec2(a.cos() * r, a.sin() * r)
            })
            .collect();
        let color = if low {
            egui::Color32::from_rgb(255, 70, 50)
        } else {
            egui::Color32::from_rgb(255, 158, 0)
        };
        painter.add(egui::epaint::PathShape::line(
            pts,
            egui::Stroke::new(9.0_f32, color),
        ));
    }
    painter.text(
        center,
        egui::Align2::CENTER_CENTER,
        label,
        egui::FontId::proportional(r * 0.55),
        egui::Color32::WHITE,
    );
}

pub(crate) fn nav_row(
    ui: &mut egui::Ui,
    resp: &egui::Response,
    ids: &mut Vec<String>,
    id: String,
    focus: usize,
    active: bool,
    moved: bool,
) {
    if !resp.enabled() {
        return;
    }
    let idx = ids.len();
    if active && idx == focus {
        ui.painter().rect_stroke(
            resp.rect.expand(2.0),
            4.0,
            egui::Stroke::new(2.0_f32, egui::Color32::YELLOW),
            egui::StrokeKind::Outside,
        );
        if moved {
            resp.scroll_to_me(Some(egui::Align::Center));
        }
    }
    ids.push(id);
}

pub(crate) fn menu_nav_step(
    nav_edge: [bool; 4],
    nav_held: [bool; 4],
    dt: f32,
    repeat_cd: &mut f32,
) -> (i32, i32) {
    let mut dy = i32::from(nav_edge[1]) - i32::from(nav_edge[0]);
    let mut dx = i32::from(nav_edge[3]) - i32::from(nav_edge[2]);
    if nav_edge.iter().any(|&edge| edge) {
        *repeat_cd = 0.35;
    } else if nav_held.iter().any(|&held| held) {
        *repeat_cd -= dt;
        if *repeat_cd <= 0.0 {
            *repeat_cd = 0.08;
            dy = i32::from(nav_held[1]) - i32::from(nav_held[0]);
            dx = i32::from(nav_held[3]) - i32::from(nav_held[2]);
        }
    } else {
        *repeat_cd = 0.0;
    }
    (dy, dx)
}

pub(crate) const MSAA_LEVELS: [u16; 4] = [0, 2, 4, 8];

pub(crate) fn step_msaa(current: u16, direction: i32) -> u16 {
    let index = MSAA_LEVELS
        .iter()
        .position(|&value| value == current)
        .unwrap_or(2);
    MSAA_LEVELS[(index as i32 + direction.signum()).clamp(0, 3) as usize]
}

pub(crate) fn step_stick_deadzone(current: f32, direction: i32) -> f32 {
    let current = if current.is_finite() { current } else { 0.0 };
    (current + direction as f32 * 0.01).clamp(0.0, 0.5)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn opposing_navigation_edges_start_the_repeat_delay() {
        let mut repeat_cd = 0.0;
        let edges = [true, true, false, false];
        let held = [true, true, false, false];
        assert_eq!(menu_nav_step(edges, held, 0.018, &mut repeat_cd), (0, 0));
        assert!((repeat_cd - 0.35).abs() < f32::EPSILON);
        assert_eq!(menu_nav_step([false; 4], held, 0.1, &mut repeat_cd), (0, 0));
        assert!((repeat_cd - 0.25).abs() < 1e-6);
    }
}
