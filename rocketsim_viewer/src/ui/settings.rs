use std::collections::HashMap;

use super::{CameraChoice, MSAA_LEVELS, UiAction, ViewerUiState, nav_row};
use crate::{
    camera::CameraPreset,
    input::{
        ACTIONS, Action, Capture, Evaluator, InputSnapshot, InputSource,
        sources::{PadButton, clamp_deadzone},
    },
    settings::{BindingDevice, ViewerSettings},
};

pub(crate) struct SettingsInput<'a> {
    pub(crate) evaluator: &'a mut Evaluator,
    pub(crate) active_pad: &'a mut Option<gilrs::GamepadId>,
    pub(crate) pad_selection_manual: &'a mut bool,
    pub(crate) raw_pad: &'a mut HashMap<u32, f32>,
    pub(crate) pad_buttons: &'a mut HashMap<PadButton, f32>,
    pub(crate) suppress_pad_on_next_frame: &'a mut bool,
}

pub(crate) struct SettingsView<'a> {
    pub(crate) ctx: &'a egui::Context,
    pub(crate) ui_state: &'a mut ViewerUiState,
    pub(crate) settings: &'a mut ViewerSettings,
    pub(crate) input: SettingsInput<'a>,
    pub(crate) input_snapshot: &'a InputSnapshot,
    pub(crate) connected_pads: &'a [(gilrs::GamepadId, String)],
    pub(crate) cam_line: &'a str,
    pub(crate) drive_line: &'a str,
    pub(crate) driven_car: u32,
    pub(crate) rebinding: bool,
}

pub(crate) fn show_settings(view: SettingsView<'_>) -> Vec<UiAction> {
    let SettingsView {
        ctx,
        ui_state,
        settings,
        mut input,
        input_snapshot,
        connected_pads,
        cam_line,
        drive_line,
        driven_car,
        rebinding,
    } = view;
    let mut menu_actions = Vec::new();
    if ui_state.menu_open && !rebinding && !ui_state.reset_all_confirm {
        let mut ui_action_map = settings.action_map().clone();
        let mut bindings_dirty = false;
        let binding_group = &mut ui_state.binding_group;
        let binding_filter = &mut ui_state.binding_filter;
        let binding_device = &mut ui_state.binding_device;
        let rebind_action = &mut ui_state.rebind_action;
        let rebind_capture = &mut ui_state.rebind_capture;

        let capture_start_back = &mut ui_state.capture_start_back;
        let action_evaluator = &mut *input.evaluator;
        let binding_warning = &mut ui_state.binding_warning;
        let active_pad = &mut *input.active_pad;
        let raw_pad = &mut input.raw_pad;
        let settings = &mut *settings;
        // Pad-nav focus snapshot for this draw (ids rebuilt below).
        let focus = ui_state.menu_focus;
        let active = ui_state.menu_nav_active;
        let moved = ui_state.nav_moved;

        ui_state.nav_ids.clear();
        egui::Window::new("Settings")
                .resizable(true)
                .default_size([760.0, 680.0])
                .min_size([540.0, 400.0])
                .max_size([1100.0, 900.0])
                .scroll([false, false])
                .show(ctx, |ui| {
                let (ids, focus, active, moved) = (&mut ui_state.nav_ids, focus, active, moved);
                ui.horizontal_wrapped(|ui| {
                    for (index, label) in ["Camera", "Bindings", "Display", "Simulation"].into_iter().enumerate() {
                        let response = ui.selectable_label(ui_state.settings_tab == index, label);
                        if response.clicked() { ui_state.settings_tab = index; }
                        nav_row(ui, &response, ids, format!("settings.tab.{index}"), focus, active, moved);
                    }
                });
                ui.separator();
                let content_height = (ui.available_height() - 36.0).max(100.0);
                egui::ScrollArea::vertical()
                    .id_salt(("settings-content", ui_state.settings_tab))
                    .max_height(content_height)
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                if ui_state.settings_tab == 0 {
                ui.scope(|ui| {
                        ui.horizontal(|ui| {
                            let r = ui.button("Wide view");
                            if r.clicked() {
                                menu_actions.push(UiAction::SetCamera(CameraChoice::Follow));
                            }
                            nav_row(ui, &r, ids, "cam.wide".to_string(), focus, active, moved);
                            let r = ui.button("Director");
                            if r.clicked() {
                                menu_actions.push(UiAction::SetCamera(CameraChoice::Director));
                            }
                            nav_row(ui, &r, ids, "cam.director".to_string(), focus, active, moved);
                        });
                        ui.label(format!("View: {cam_line}"));
                        ui.label(format!(
                            "Bindings: Follow 1 {}, Follow 8 {}, Director {}, Free camera {}, Next camera {}, Ball camera {}.",
                            ui_action_map.sources(Action::Follow1)[0].name(),
                            ui_action_map.sources(Action::Follow8)[0].name(),
                            ui_action_map.sources(Action::Director)[0].name(),
                            ui_action_map.sources(Action::FreeCam)[0].name(),
                            ui_action_map.sources(Action::NextCam)[0].name(),
                            ui_action_map.sources(Action::BallCam)[0].name(),
                        ));
                        ui.label(format!("You drive car {driven_car}."));
                        ui.label(drive_line);
                        // One focus stop per slider (pad ◀▶ adjusts).
                        let r = ui.add(
                            egui::Slider::new(&mut settings.camera.fov_degrees, 60.0..=130.0)
                                .text("FOV (h°)"),
                        );
                        if r.changed() {
                            settings.mark_dirty();
                        }
                        nav_row(ui, &r, ids, "cam.fov".to_string(), focus, active, moved);
                        let r = ui.add(
                            egui::Slider::new(&mut settings.camera.car_cam.distance, 100.0..=600.0).text("distance"),
                        );
                        if r.changed() {
                            settings.mark_dirty();
                        }
                        nav_row(ui, &r, ids, "cam.distance".to_string(), focus, active, moved);
                        let r =
                            ui.add(egui::Slider::new(&mut settings.camera.car_cam.height, 0.0..=300.0).text("height"));
                        if r.changed() {
                            settings.mark_dirty();
                        }
                        nav_row(ui, &r, ids, "cam.height".to_string(), focus, active, moved);
                        let r = ui.add(
                            egui::Slider::new(&mut settings.camera.car_cam.angle_deg, -30.0..=30.0).text("angle°"),
                        );
                        if r.changed() {
                            settings.mark_dirty();
                        }
                        nav_row(ui, &r, ids, "cam.angle".to_string(), focus, active, moved);
                        let r = ui.add(
                            egui::Slider::new(&mut settings.camera.car_cam.stiffness, 0.0..=1.0).text("stiffness"),
                        );
                        if r.changed() {
                            settings.mark_dirty();
                        }
                        nav_row(ui, &r, ids, "cam.stiffness".to_string(), focus, active, moved);
                        let r = ui.add(
                            egui::Slider::new(&mut settings.camera.car_cam.transition_speed, 1.0..=2.0)
                                .text("transition"),
                        );
                        if r.changed() {
                            settings.mark_dirty();
                        }
                        nav_row(
                            ui,
                            &r,
                            ids,
                            "cam.transition".to_string(),
                            focus,
                            active,
                            moved,
                        );
                        let r = ui.button("Reset camera (RL defaults)");
                        if r.clicked() {
                            settings.reset_camera();
                        }
                        nav_row(ui, &r, ids, "cam.reset_camera".to_string(), focus, active, moved);
                        ui.horizontal_wrapped(|ui| {
                            ui.label("Presets:");
                            for (pi, preset) in CameraPreset::ALL.iter().enumerate() {
                                let r = ui.button(preset.name());
                                if r.clicked() {
                                    let (cc, fov) = (
                                        &mut settings.camera.car_cam,
                                        &mut settings.camera.fov_degrees,
                                    );
                                    preset.apply(cc, fov);
                                    settings.mark_dirty();
                                }
                                nav_row(
                                    ui,
                                    &r,
                                    ids,
                                    format!("cam.preset.{pi}"),
                                    focus,
                                    active,
                                    moved,
                                );
                            }
                        });
                    });

                }

                if ui_state.settings_tab == 3 {
                ui.scope(|ui| {
                        ui.label("The simulation must apply controls and mutators. The drive example does this.");
                        let spectate = ui.checkbox(&mut settings.spectate_only, "Spectate only (no inputs)");
                        if spectate.changed() { settings.mark_dirty(); }
                        nav_row(ui, &spectate, ids, "input.spectate".into(), focus, active, moved);
                        ui.separator();
                        let reset_play = ui.button("Reset play");
                        if reset_play.clicked() { menu_actions.push(UiAction::ResetPlay); }
                        nav_row(ui, &reset_play, ids, "cam.reset_play".into(), focus, active, moved);
                        ui.separator();
                        // Live toggles: sent every frame, applied per tick.
                        for (id, label) in [
                            ("mut.boost", "Unlimited boost"),
                            ("mut.regen", "Boost regen"),
                            ("mut.flips", "Unlimited flips"),
                            ("mut.jumps", "Unlimited double jumps"),
                        ] {
                            let r = match id {
                                "mut.boost" => {
                                    ui.checkbox(&mut settings.mut_unlimited_boost, label)
                                }
                                "mut.regen" => {
                                    ui.checkbox(&mut settings.mut_recharge, label)
                                }
                                "mut.flips" => {
                                    ui.checkbox(&mut settings.mut_flips, label)
                                }
                                _ => ui.checkbox(&mut settings.mut_double_jumps, label),
                            };
                            if r.changed() {
                                settings.mark_dirty();
                            }
                            nav_row(ui, &r, ids, id.to_string(), focus, active, moved);
                        }
                    });

                }
                if ui_state.settings_tab == 2 {
                ui.strong("Overlays");
                ui.scope(|ui| {
                        for (id, label) in [
                            ("ov.trail", "Ball trail"),
                            ("ov.boost", "Boost flames"),
                            ("ov.ground", "Ground line"),
                            ("ov.pads", "Boost pads"),
                        ] {
                            let r = match id {
                                "ov.trail" => ui.checkbox(&mut settings.show_trail, label),
                                "ov.boost" => ui.checkbox(&mut settings.show_boost, label),
                                "ov.ground" => {
                                    ui.checkbox(&mut settings.show_ground_line, label)
                                }
                                _ => ui.checkbox(&mut settings.show_pads, label),
                            };
                            if r.changed() {
                                settings.mark_dirty();
                            }
                            nav_row(ui, &r, ids, id.to_string(), focus, active, moved);
                        }
                    });
                ui.separator();
                ui.strong("Rendering");
                ui.scope(|ui| {
                        let r = ui.add(
                            egui::Slider::new(&mut settings.fps_limit_hz, 0..=240)
                                .text("FPS cap (0=off)"),
                        );
                        if r.changed() {
                            settings.mark_dirty();
                        }
                        nav_row(ui, &r, ids, "disp.fps".to_string(), focus, active, moved);
                        let r = ui.add(
                            egui::Slider::new(&mut settings.ui_scale, 0.5..=2.0).text("UI scale"),
                        );
                        if r.changed() {
                            settings.mark_dirty();
                        }
                        nav_row(ui, &r, ids, "disp.uiscale".to_string(), focus, active, moved);
                        ui.horizontal(|ui| {
                            let r = ui.checkbox(&mut settings.vsync, "VSync");
                            if r.changed() {
                                settings.mark_dirty();
                            }
                            nav_row(ui, &r, ids, "disp.vsync".to_string(), focus, active, moved);
                            let mut samples = MSAA_LEVELS
                                .iter()
                                .position(|&value| value == settings.msaa)
                                .unwrap_or(2);
                            let r = ui.add(
                                egui::Slider::new(&mut samples, 0..=3)
                                    .text("MSAA")
                                    .custom_formatter(|value, _| {
                                        MSAA_LEVELS[value.round().clamp(0.0, 3.0) as usize].to_string()
                                    }),
                            );
                            if r.changed() {
                                settings.msaa = MSAA_LEVELS[samples];
                                settings.mark_dirty();
                            }
                            nav_row(ui, &r, ids, "disp.msaa".to_string(), focus, active, moved);
                        });
                        ui.label("(VSync/MSAA apply on relaunch)");
                    });

                }
                if ui_state.settings_tab == 1 {
                ui.scope(|ui| {
                        ui.horizontal(|ui| {
                            for (index, (device, label)) in [
                                (BindingDevice::All, "All"),
                                (BindingDevice::KeyboardMouse, "Keyboard & Mouse"),
                                (BindingDevice::Controller, "Controller"),
                            ]
                            .into_iter()
                            .enumerate()
                            {
                                let response = ui.selectable_label(*binding_device == device, label);
                                if response.clicked() {
                                    *binding_device = device;
                                }
                                nav_row(ui, &response, ids, format!("bind.device.{index}"), focus, active, moved);
                            }
                        });
                        if *binding_device == BindingDevice::Controller {
                            let selected_pad_name = (*active_pad)
                                .and_then(|id| connected_pads.iter().find(|(pad_id, _)| *pad_id == id))
                                .map(|(_, name)| name.as_str())
                                .unwrap_or("No controller");
                            ui.horizontal(|ui| {
                                ui.label(format!("Controller: {selected_pad_name}"));
                                let pad_cycle = ui.button("Change");
                                if pad_cycle.clicked() && !connected_pads.is_empty() {
                                    let index = connected_pads.iter().position(|(id, _)| Some(*id) == *active_pad).unwrap_or(0);
                                    *active_pad = Some(connected_pads[(index + 1) % connected_pads.len()].0);
                                    *input.pad_selection_manual = true;
                                    raw_pad.clear();
                                    input.pad_buttons.clear();
                                    *input.suppress_pad_on_next_frame = true;
                                }
                                nav_row(ui, &pad_cycle, ids, "pad.select".into(), focus, active, moved);
                            });
                            let deadzone = ui.add(egui::Slider::new(&mut settings.bindings.pad.stick_deadzone, 0.0..=0.5).text("Stick deadzone"));
                            if deadzone.changed() { settings.bindings.pad.stick_deadzone = clamp_deadzone(settings.bindings.pad.stick_deadzone).min(0.5); settings.mark_dirty(); }
                            nav_row(ui, &deadzone, ids, "input.deadzone".into(), focus, active, moved);
                        }
                        ui.label("Two shared slots per action; capture accepts any input type.");
                        let groups = ["Driving", "Simulation", "Camera", "Free camera", "Menu"];
                        let group_picker = egui::ComboBox::from_id_salt("binding-group")
                            .selected_text(binding_group.as_deref().unwrap_or("All actions"))
                            .show_ui(ui, |ui| {
                                ui.selectable_value(binding_group, None, "All actions");
                                for group in groups { ui.selectable_value(binding_group, Some(group.to_string()), group); }
                            });
                        nav_row(ui, &group_picker.response, ids, "bind.group".into(), focus, active, moved);
                        ui.horizontal(|ui| {
                            let filter_response = ui.add_sized([240.0, 24.0], egui::TextEdit::singleline(binding_filter).hint_text("Search actions…").id(egui::Id::unique("viewer.binding-filter")));
                            nav_row(ui, &filter_response, ids, "bind.filter".into(), focus, active, moved);
                            let clear_filter = ui.small_button("Clear");
                            if clear_filter.clicked() { binding_filter.clear(); }
                            nav_row(ui, &clear_filter, ids, "bind.filter.clear".into(), focus, active, moved);
                        });
                        let action_map = &mut ui_action_map;
                        let mut last_group = "";
                        egui::Grid::new("viewer.binding-grid").striped(true).num_columns(3).show(ui, |ui| {
                            ui.strong("Action");
                            ui.strong("Bindings");
                            ui.strong("Reset");
                            ui.end_row();
                            for (action_index, action) in ACTIONS.into_iter().enumerate() {
                                if binding_group.as_ref().is_some_and(|g| g.as_str() != action.group()) || (!binding_filter.is_empty() && !action.label().to_lowercase().contains(&binding_filter.to_lowercase())) { continue; }
                                if last_group != action.group() {
                                    last_group = action.group();
                                    ui.label(egui::RichText::new(last_group).strong());
                                    ui.label("");
                                    ui.label("");
                                    ui.end_row();
                                }
                                ui.add_sized([150.0, 24.0], egui::Label::new(action.label()));
                                let visible = binding_device.visible_slots(*action_map.sources(action));
                                let can_reset = !visible.is_empty();
                                let hidden = match binding_device {
                                    BindingDevice::KeyboardMouse => action_map.sources(action).iter().any(|source| matches!(source, InputSource::PadButton(_) | InputSource::PadAxis { .. } | InputSource::PadRaw { .. })),
                                    BindingDevice::Controller => action_map.sources(action).iter().any(|source| matches!(source, InputSource::Key(_) | InputSource::Mouse(_) | InputSource::MouseAxis { .. } | InputSource::Wheel { .. })),
                                    BindingDevice::All => false,
                                };
                                ui.horizontal(|ui| {
                                    if visible.is_empty() && hidden {
                                        let hint = if *binding_device == BindingDevice::Controller { "Assigned on Keyboard & Mouse" } else { "Assigned on Controller" };
                                        ui.colored_label(egui::Color32::GRAY, hint);
                                        let show_all = ui.small_button("Show All");
                                        if show_all.clicked() { *binding_device = BindingDevice::All; }
                                        nav_row(ui, &show_all, ids, format!("bind.show_all.{action_index}"), focus, active, moved);
                                    } else {
                                        for slot in visible {
                                            let source = action_map.sources(action)[slot];
                                            let text = if *rebind_action == Some((action, slot)) { "Press input…".to_string() } else { source.name() };
                                            let bind = ui.add_sized([132.0, 24.0], egui::Button::new(text));
                                            if bind.clicked() {
                                                *rebind_action = Some((action, slot));
                                                *rebind_capture = Some(Capture::begin(input_snapshot));
                                                *capture_start_back = action_map.sources(Action::MenuBack).iter().copied().find(|source| *source != InputSource::Unbound && input_snapshot.value(*source).abs() >= 0.5);
                                                ui_state.capture_pending_back = None;
                                                ui_state.capture_cancel_hold = 0.0;
                                                action_evaluator.suppress_until_release(input_snapshot);
                                            }
                                            nav_row(ui, &bind, ids, format!("bind.action.{action_index}.{slot}"), focus, active, moved);
                                            let clear = ui.add_enabled(source != InputSource::Unbound, egui::Button::new("×").small()).on_hover_text("Clear this slot");
                                            if clear.clicked() { action_map.set(action, slot, InputSource::Unbound); bindings_dirty = true; }
                                            nav_row(ui, &clear, ids, format!("bind.clear.{action_index}.{slot}"), focus, active, moved);
                                        }
                                    }
                                });
                                let reset = ui.add_enabled(can_reset, egui::Button::new("Reset").small()).on_hover_text("Restore default bindings in this device view");
                                if reset.clicked() { binding_device.reset_action(action_map, action); bindings_dirty = true; }
                                nav_row(ui, &reset, ids, format!("bind.reset.{action_index}"), focus, active, moved);
                                ui.end_row();
                            }
                        });

                        if let Some(warning) = &*binding_warning { ui.colored_label(egui::Color32::YELLOW, warning); }
                        let conflicts = action_map.conflicts();
                        if !conflicts.is_empty() { ui.colored_label(egui::Color32::YELLOW, format!("{} shared binding conflict(s); actions may trigger together", conflicts.len())); }
                    });
                }
                });
                ui.separator();
                let reset_all = ui.button("Reset all settings…");
                if reset_all.clicked() {
                    ui_state.reset_all_confirm = true;
                    ui_state.menu_focus = 0;
                }
                nav_row(ui, &reset_all, ids, "settings.reset_all".into(), focus, active, moved);
                ui_state.nav_moved = false;
            });
        if bindings_dirty {
            settings.action_bindings = Some(ui_action_map);
            settings.mark_dirty();
        }
    }

    menu_actions
}
