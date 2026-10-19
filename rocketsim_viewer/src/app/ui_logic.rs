use super::{CamMode, CameraState, InputState};
use crate::{
    camera::CameraPreset,
    input::{ACTIONS, Action, Capture, InputSnapshot, InputSource},
    settings::{BindingDevice, ViewerSettings},
    ui::{ViewerUiState, step_msaa, step_stick_deadzone},
};

/// Pad menu action for one focused row: `dx` (±1 from ◀▶) adjusts the
/// focused slider or flips the focused toggle; `a` (select button)
/// activates buttons and flips toggles. Mirrors the mouse click
/// handlers one row at a time.
/// Mutable menu state for [`menu_pad_action`] (one struct beats nine args).
pub(super) struct MenuPadCtx<'a> {
    settings: &'a mut ViewerSettings,
    camera: &'a mut CameraState,
    ui: &'a mut ViewerUiState,
    input: &'a mut InputState,
    input_snapshot: &'a InputSnapshot,
}

impl<'a> MenuPadCtx<'a> {
    pub(super) fn new(
        settings: &'a mut ViewerSettings,
        camera: &'a mut CameraState,
        ui: &'a mut ViewerUiState,
        input: &'a mut InputState,
        input_snapshot: &'a InputSnapshot,
    ) -> Self {
        Self {
            settings,
            camera,
            ui,
            input,
            input_snapshot,
        }
    }
}

pub(super) fn menu_pad_action(id: String, dx: i32, a: bool, cx: &mut MenuPadCtx) {
    // Slider adjust helper: step, clamped to the widget range.
    fn slide_val(v: f32, dx: i32, step: f32, lo: f32, hi: f32) -> f32 {
        (v + dx as f32 * step).clamp(lo, hi)
    }
    // Adjust one float slider and mark dirty.
    macro_rules! slide {
        ($field:expr, $step:expr, $lo:expr, $hi:expr) => {{
            $field = slide_val($field, dx, $step, $lo, $hi);
            cx.settings.mark_dirty();
        }};
    }
    match id.as_str() {
        "bind.capture.cancel" if a => {
            cx.ui.rebind_action = None;
            cx.ui.rebind_capture = None;
        }
        "pad.select" if dx != 0 || a => {
            if let Some(gilrs) = cx.input.gilrs.as_ref() {
                let pads: Vec<_> = gilrs
                    .gamepads()
                    .filter(|(_, pad)| pad.is_connected())
                    .map(|(id, _)| id)
                    .collect();
                if !pads.is_empty() {
                    let index = pads
                        .iter()
                        .position(|id| Some(*id) == cx.input.active_pad)
                        .unwrap_or(0) as i32;
                    let direction = if dx == 0 { 1 } else { dx.signum() };
                    cx.input.active_pad =
                        Some(pads[(index + direction).rem_euclid(pads.len() as i32) as usize]);
                    cx.input.pad_selection_manual = true;
                    cx.input.raw_pad.clear();
                    cx.input.pad_buttons.clear();
                    cx.input.suppress_pad_on_next_frame = true;
                }
            }
        }
        "bind.group" if dx != 0 => {
            let groups = [
                None,
                Some("Driving".to_string()),
                Some("Simulation".to_string()),
                Some("Camera".to_string()),
                Some("Free camera".to_string()),
                Some("Menu".to_string()),
            ];
            let index = groups
                .iter()
                .position(|group| group == &cx.ui.binding_group)
                .unwrap_or(0) as i32;
            cx.ui.binding_group =
                groups[(index + dx.signum()).rem_euclid(groups.len() as i32) as usize].clone();
        }
        id if id.starts_with("settings.tab.") && (dx != 0 || a) => {
            if dx != 0 {
                cx.ui.settings_tab =
                    (cx.ui.settings_tab as i32 + dx.signum()).rem_euclid(4) as usize;
            } else if let Some(index) = id
                .rsplit('.')
                .next()
                .and_then(|value| value.parse::<usize>().ok())
                .filter(|index| *index < 4)
            {
                cx.ui.settings_tab = index;
            }
        }
        id if id.starts_with("bind.show_all.") && a => cx.ui.binding_device = BindingDevice::All,
        id if id.starts_with("bind.device.") && (dx != 0 || a) => {
            let devices = [
                BindingDevice::All,
                BindingDevice::KeyboardMouse,
                BindingDevice::Controller,
            ];
            if dx != 0 {
                let index = devices
                    .iter()
                    .position(|device| device == &cx.ui.binding_device)
                    .unwrap_or(0) as i32;
                cx.ui.binding_device =
                    devices[(index + dx.signum()).rem_euclid(devices.len() as i32) as usize];
            } else if let Some(device) = id
                .rsplit('.')
                .next()
                .and_then(|value| value.parse::<usize>().ok())
                .and_then(|index| devices.get(index))
            {
                cx.ui.binding_device = *device;
            }
        }
        "bind.filter.clear" if a => cx.ui.binding_filter.clear(),
        "cam.wide" if a => {
            cx.camera.mode = CamMode::Man;
            cx.camera.auto_cam = None;
            if let Some(man) = cx.camera.manager.as_mut() {
                man.set_wide_view();
            }
        }
        "cam.director" if a => {
            cx.camera.mode = CamMode::Director;
            cx.camera.auto_cam = None;
        }
        "cam.reset_play" if a => {
            cx.ui.reset_requested = true;
        }
        "cam.reset_camera" if a => {
            cx.settings.reset_camera();
        }
        "settings.reset_all" if a => cx.ui.reset_all_confirm = true,
        "reset.confirm.cancel" if a => cx.ui.reset_all_confirm = false,
        "reset.confirm.apply" if a => {
            cx.settings.reset_all();
            cx.ui.reset_all_confirm = false;
        }
        "input.spectate" if a || dx != 0 => {
            cx.settings.spectate_only = !cx.settings.spectate_only;
            cx.settings.mark_dirty();
        }
        "input.deadzone" if dx != 0 => {
            cx.settings.bindings.pad.stick_deadzone =
                step_stick_deadzone(cx.settings.bindings.pad.stick_deadzone, dx);
            cx.settings.mark_dirty();
        }
        "mut.boost" if a || dx != 0 => {
            cx.settings.mut_unlimited_boost = !cx.settings.mut_unlimited_boost;
            cx.settings.mark_dirty();
        }
        "mut.regen" if a || dx != 0 => {
            cx.settings.mut_recharge = !cx.settings.mut_recharge;
            cx.settings.mark_dirty();
        }
        "mut.flips" if a || dx != 0 => {
            cx.settings.mut_flips = !cx.settings.mut_flips;
            cx.settings.mark_dirty();
        }
        "mut.jumps" if a || dx != 0 => {
            cx.settings.mut_double_jumps = !cx.settings.mut_double_jumps;
            cx.settings.mark_dirty();
        }
        "ov.trail" if a || dx != 0 => {
            cx.settings.show_trail = !cx.settings.show_trail;
            cx.settings.mark_dirty();
        }
        "ov.boost" if a || dx != 0 => {
            cx.settings.show_boost = !cx.settings.show_boost;
            cx.settings.mark_dirty();
        }
        "ov.ground" if a || dx != 0 => {
            cx.settings.show_ground_line = !cx.settings.show_ground_line;
            cx.settings.mark_dirty();
        }
        "ov.pads" if a || dx != 0 => {
            cx.settings.show_pads = !cx.settings.show_pads;
            cx.settings.mark_dirty();
        }
        "disp.vsync" if a || dx != 0 => {
            cx.settings.vsync = !cx.settings.vsync;
            cx.settings.mark_dirty();
        }
        "cam.fov" if dx != 0 => {
            slide!(cx.settings.camera.fov_degrees, 1.0, 60.0, 130.0);
        }
        "cam.distance" if dx != 0 => {
            slide!(cx.settings.camera.car_cam.distance, 5.0, 100.0, 600.0);
        }
        "cam.height" if dx != 0 => {
            slide!(cx.settings.camera.car_cam.height, 2.0, 0.0, 300.0);
        }
        "cam.angle" if dx != 0 => {
            slide!(cx.settings.camera.car_cam.angle_deg, 0.5, -30.0, 30.0);
        }
        "cam.stiffness" if dx != 0 => {
            slide!(cx.settings.camera.car_cam.stiffness, 0.05, 0.0, 1.0);
        }
        "cam.transition" if dx != 0 => {
            slide!(cx.settings.camera.car_cam.transition_speed, 0.05, 1.0, 2.0);
        }
        "disp.fps" if dx != 0 => {
            cx.settings.fps_limit_hz =
                slide_val(cx.settings.fps_limit_hz as f32, dx, 10.0, 0.0, 240.0) as u32;
            cx.settings.mark_dirty();
        }
        "disp.uiscale" if dx != 0 => {
            slide!(cx.settings.ui_scale, 0.05, 0.5, 2.0);
        }
        "disp.msaa" if dx != 0 => {
            cx.settings.msaa = step_msaa(cx.settings.msaa, dx);
            cx.settings.mark_dirty();
        }

        _ => {
            if a {
                if let Some(rest) = id.strip_prefix("bind.action.") {
                    let mut parts = rest.split('.');
                    if let (Some(action_index), Some(slot)) = (
                        parts.next().and_then(|x| x.parse::<usize>().ok()),
                        parts.next().and_then(|x| x.parse::<usize>().ok()),
                    ) && let Some(action) = ACTIONS.get(action_index).copied()
                    {
                        cx.ui.rebind_action = Some((action, slot));
                        cx.ui.rebind_capture = Some(Capture::begin(cx.input_snapshot));

                        cx.ui.capture_start_back = cx
                            .settings
                            .action_map()
                            .sources(Action::MenuBack)
                            .iter()
                            .copied()
                            .find(|source| {
                                *source != InputSource::Unbound
                                    && cx.input_snapshot.value(*source).abs() >= 0.5
                            });
                        cx.ui.capture_cancel_hold = 0.0;
                        cx.ui.capture_pending_back = None;
                        cx.input.evaluator.suppress_until_release(cx.input_snapshot);
                    }
                } else if let Some(rest) = id.strip_prefix("bind.clear.") {
                    let mut parts = rest.split('.');
                    if let (Some(index), Some(slot)) = (
                        parts.next().and_then(|x| x.parse::<usize>().ok()),
                        parts.next().and_then(|x| x.parse::<usize>().ok()),
                    ) && let Some(action) = ACTIONS.get(index).copied()
                    {
                        let mut map = cx.settings.action_map().clone();
                        map.set(action, slot, InputSource::Unbound);
                        cx.settings.action_bindings = Some(map);
                        cx.settings.mark_dirty();
                    }
                } else if let Some(index) = id
                    .strip_prefix("bind.reset.")
                    .and_then(|x| x.parse::<usize>().ok())
                {
                    if let Some(action) = ACTIONS.get(index).copied() {
                        let mut map = cx.settings.action_map().clone();
                        cx.ui.binding_device.reset_action(&mut map, action);
                        cx.settings.action_bindings = Some(map);
                        cx.settings.mark_dirty();
                    }
                } else if let Some(n) = id
                    .strip_prefix("cam.preset.")
                    .and_then(|n| n.parse::<usize>().ok())
                    .filter(|&n| n < CameraPreset::ALL.len())
                {
                    let (cc, fov) = (
                        &mut cx.settings.camera.car_cam,
                        &mut cx.settings.camera.fov_degrees,
                    );
                    CameraPreset::ALL[n].apply(cc, fov);
                    cx.settings.mark_dirty();
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::camera::StartMode;
    use super::*;
    use crate::input::{ActionMap, sources::MouseButton};

    #[test]
    fn controller_menu_navigation_changes_clamped_deadzone_and_core_actions() {
        let shared_source = InputSource::Mouse(MouseButton::Extra2);
        let mut shared_bindings = ActionMap::default();
        shared_bindings.set(Action::Jump, 0, shared_source);
        shared_bindings.set(Action::FreeReset, 1, shared_source);
        assert!(
            shared_bindings
                .conflicts()
                .iter()
                .any(|conflict| conflict.0 == Action::Jump && conflict.2 == Action::FreeReset)
        );
        let mut settings = ViewerSettings::default();
        let mut accepted_bindings = settings.action_map().clone();
        accepted_bindings.set(Action::Jump, 0, shared_source);
        assert_eq!(accepted_bindings.sources(Action::Jump)[0], shared_source);

        settings.bindings.pad.stick_deadzone = 0.49;
        let mut camera = CameraState::new(StartMode::Birds);
        let mut ui_state = ViewerUiState::new(true);
        ui_state.binding_device = BindingDevice::All;
        let mut input = InputState::new();
        let input_snapshot = InputSnapshot::new();
        let mut cx = MenuPadCtx::new(
            &mut settings,
            &mut camera,
            &mut ui_state,
            &mut input,
            &input_snapshot,
        );

        menu_pad_action("input.deadzone".into(), 1, false, &mut cx);
        assert_eq!(cx.settings.bindings.pad.stick_deadzone, 0.5);
        menu_pad_action("input.deadzone".into(), 1, false, &mut cx);
        assert_eq!(cx.settings.bindings.pad.stick_deadzone, 0.5);
        menu_pad_action("input.deadzone".into(), -1, false, &mut cx);
        assert!((cx.settings.bindings.pad.stick_deadzone - 0.49).abs() < 1e-6);

        menu_pad_action("bind.group".into(), 1, false, &mut cx);
        assert_eq!(cx.ui.binding_group.as_deref(), Some("Driving"));
        menu_pad_action("settings.tab.3".into(), 0, true, &mut cx);
        assert_eq!(cx.ui.settings_tab, 3);
        menu_pad_action("bind.device.2".into(), 0, true, &mut cx);
        assert_eq!(cx.ui.binding_device, BindingDevice::Controller);
        menu_pad_action("bind.show_all.17".into(), 0, true, &mut cx);
        assert_eq!(cx.ui.binding_device, BindingDevice::All);
        cx.settings.show_trail = false;
        menu_pad_action("settings.reset_all".into(), 0, true, &mut cx);
        assert!(cx.ui.reset_all_confirm);
        assert!(!cx.settings.show_trail);
        menu_pad_action("reset.confirm.cancel".into(), 0, true, &mut cx);
        assert!(!cx.ui.reset_all_confirm);
        assert!(!cx.settings.show_trail);
        menu_pad_action("settings.reset_all".into(), 0, true, &mut cx);
        menu_pad_action("reset.confirm.apply".into(), 0, true, &mut cx);
        assert!(!cx.ui.reset_all_confirm);
        assert!(cx.settings.show_trail);
        assert_eq!(cx.settings.bindings.pad.stick_deadzone, 0.1);
    }
}
