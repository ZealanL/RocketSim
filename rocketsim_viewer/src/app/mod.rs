mod camera;
mod effects;
mod input_state;
mod session_state;
mod startup;
mod ui_capture;
mod ui_logic;

use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use camera::{CamMode, CameraFrameInput, CameraState, StartMode};
use clap::Parser;
use effects::EffectState;
use input_state::*;
use rocketsim_vis::{
    game_mode_name, parse_game_mode,
    proto::{ControlMsg, DEFAULT_PORT},
    transport::viewer::{ConnStatus, ViewerListener, ViewerSnapshot},
};
use session_state::SessionState;
use startup::dropshot_tile_states;
pub(crate) use startup::run;
use ui_logic::{MenuPadCtx, menu_pad_action};

use crate::{
    debug_metrics,
    effects::dropshot::DropshotTileCache,
    geometry::{lines::Segment, mesh::ArenaMesh},
    input::{Action, sources::clamp_deadzone, spectate},
    render::{
        debug::{DebugStats, overlay::OverlayCache},
        scene::SceneGl,
        view,
    },
    session,
    settings::ViewerSettings,
    ui::{
        CameraChoice, SettingsInput, SettingsView, UiAction, ViewerUiState,
        hud::{boost_meter, spectated_boost},
        menu_nav_step, nav_row, show_settings,
    },
};

struct ViewerApp {
    frames: u64,
    debug_info: Arc<Mutex<DebugStats>>,
    debug_overlay: OverlayCache,
    debug_metrics: Option<debug_metrics::DebugMetrics>,
    hide_ball: bool,
    scene: Arc<Mutex<Option<SceneGl>>>,
    listener: ViewerListener,
    last_draw_ms: Arc<Mutex<f64>>,
    gpu_ms: Arc<Mutex<f64>>,
    cpu_ms: f64,
    measure_gpu: bool,
    pads_info: Arc<Mutex<(usize, usize)>>,
    lines_info: Arc<Mutex<(usize, usize)>>,
    effects: EffectState,
    dropshot_tiles: DropshotTileCache,
    session: SessionState,
    pending_segs: Arc<Mutex<Vec<Segment>>>,
    camera: CameraState,
    ctrl_seq: u64,
    driven_car: u32,

    settings: ViewerSettings,
    ui_state: ViewerUiState,
    input: InputState,
    last_frame: Instant,
    fps_ema: f32,
}

impl ViewerApp {
    fn new(
        mesh: Arc<ArenaMesh>,
        listener: ViewerListener,
        init_mode: StartMode,
        settings: ViewerSettings,
        menu_open: bool,
        measure_gpu: bool,
    ) -> Self {
        Self {
            frames: 0,
            debug_info: Arc::new(Mutex::new(DebugStats::default())),
            debug_overlay: OverlayCache::new(),
            debug_metrics: None,
            hide_ball: false,
            scene: Arc::new(Mutex::new(None)),
            listener,
            last_draw_ms: Arc::new(Mutex::new(0.0)),
            gpu_ms: Arc::new(Mutex::new(0.0)),
            cpu_ms: 0.0,
            measure_gpu,
            pads_info: Arc::new(Mutex::new((0, 0))),
            lines_info: Arc::new(Mutex::new((0, 0))),
            effects: EffectState::default(),
            dropshot_tiles: DropshotTileCache::default(),
            session: SessionState::new(mesh),
            pending_segs: Arc::new(Mutex::new(Vec::new())),
            camera: CameraState::new(init_mode),
            ctrl_seq: 0,
            driven_car: 0,

            settings,
            ui_state: ViewerUiState::new(menu_open),
            input: InputState::new(),
            last_frame: Instant::now(),
            fps_ema: 60.0,
        }
    }
}

impl eframe::App for ViewerApp {
    fn on_exit(&mut self, gl: Option<&glow::Context>) {
        if let Some(gl) = gl
            && let Ok(scene) = self.scene.lock()
            && let Some(scene) = scene.as_ref()
        {
            scene.destroy(gl);
        }
        if let Err(e) = self.settings.flush() {
            eprintln!("rocketsim-viewer: cannot flush settings ({e})");
        }
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // eframe hands us the root Ui; the body below works on the Context.
        let ctx = &ui.ctx().clone();
        // Use repaint scheduling for the frame cap. Do not block the UI thread.
        let budget = if self.settings.fps_limit_hz == 0 {
            Duration::ZERO
        } else {
            Duration::from_secs_f32(1.0 / self.settings.fps_limit_hz as f32)
        };
        // UI scale applies live.
        if (self.settings.ui_scale - self.ui_state.applied_ui_scale).abs() > 1e-6 {
            ctx.set_pixels_per_point(self.settings.ui_scale);
            self.ui_state.applied_ui_scale = self.settings.ui_scale;
        }
        let focused = ctx.input(|i| i.focused);
        let mut nav_edge = [false; 4];
        let mut nav_held = [false; 4];
        let pad_pulses = self.input.poll_gamepads();
        let input_snapshot = self.input.snapshot(ctx, &pad_pulses, focused);
        if self.input.suppress_pad_on_next_frame {
            self.input.suppress_until_release(&input_snapshot);
            self.input.suppress_pad_on_next_frame = false;
        }
        let capture_map = self
            .ui_state
            .rebind_action
            .map(|_| self.settings.action_map().clone());

        let cancel_rect = self.ui_state.capture_cancel_rect;
        let cancel_hover = ctx.input(|i| {
            i.pointer
                .hover_pos()
                .is_some_and(|p| cancel_rect.is_some_and(|r| r.contains(p)))
        });
        let cancel_click = ctx.input(|i| {
            cancel_hover
                && [
                    egui::PointerButton::Primary,
                    egui::PointerButton::Secondary,
                    egui::PointerButton::Middle,
                    egui::PointerButton::Extra1,
                    egui::PointerButton::Extra2,
                ]
                .into_iter()
                .any(|button| i.pointer.button_pressed(button))
        });
        let capture_update = if let Some(action_map) = &capture_map {
            ui_capture::update_capture(ui_capture::CaptureContext {
                ui: &mut self.ui_state,
                settings: &mut self.settings,
                input: &mut self.input,
                action_map,
                snapshot: &input_snapshot,
                cancel_hover,
                cancel_click,
                dt: self.last_frame.elapsed().as_secs_f32(),
            })
        } else {
            ui_capture::CaptureUpdate::default()
        };
        self.input.update_focus(focused, &input_snapshot);
        let rebinding = self.ui_state.rebind_action.is_some()
            || capture_update.completed
            || capture_update.cancelled;
        let mut menu_toggled = false;
        let action_map = self.settings.action_map();
        self.input.evaluator.evaluate_into(
            action_map,
            &input_snapshot,
            clamp_deadzone(self.settings.bindings.pad.stick_deadzone),
            &mut self.input.action_values,
        );
        if self
            .input
            .action_values
            .get(&Action::MenuToggle)
            .is_some_and(|v| v.just_pressed)
            && !rebinding
        {
            if self.ui_state.reset_all_confirm {
                self.ui_state.reset_all_confirm = false;
            } else {
                self.ui_state.menu_open = !self.ui_state.menu_open;
            }
            menu_toggled = true;
            self.input.suppress_until_release(&input_snapshot);
            if !self.ui_state.menu_open {
                ctx.memory_mut(|memory| {
                    memory.surrender_focus(egui::Id::unique("viewer.binding-filter"));
                });
            }
            self.ui_state.menu_nav_active = true;
        }
        let action_suppressed = rebinding || menu_toggled;
        for (i, action) in [
            Action::MenuUp,
            Action::MenuDown,
            Action::MenuLeft,
            Action::MenuRight,
        ]
        .into_iter()
        .enumerate()
        {
            let value = self
                .input
                .action_values
                .get(&action)
                .copied()
                .unwrap_or_default();
            nav_edge[i] = value.just_pressed;
            nav_held[i] = value.pressed;
        }
        let keyboard_editing = ctx.egui_wants_keyboard_input();
        let nav_a_edge = !keyboard_editing
            && self
                .input
                .action_values
                .get(&Action::MenuAccept)
                .is_some_and(|v| v.just_pressed);
        let nav_b_edge = !keyboard_editing
            && self
                .input
                .action_values
                .get(&Action::MenuBack)
                .is_some_and(|v| v.just_pressed);
        if keyboard_editing {
            nav_edge = [false; 4];
            nav_held = [false; 4];
        }

        // Mouse use hides the pad highlight again.
        if ctx.input(|i| i.pointer.delta() != egui::Vec2::ZERO) {
            self.ui_state.menu_nav_active = false;
        }
        // Keep the cursor locked for camera look. Menus and unfocused windows release it.
        // menu open or focus lost releases all. Sent on change.
        let (want_grab, want_visible) =
            if !focused || self.ui_state.menu_open || self.camera.mode != CamMode::Free {
                (egui::CursorGrab::None, true)
            } else {
                (egui::CursorGrab::Locked, false)
            };
        // Full-state track so grab changes send once.
        static LAST_CURSOR: std::sync::Mutex<Option<(egui::CursorGrab, bool)>> =
            std::sync::Mutex::new(Some((egui::CursorGrab::None, true)));
        {
            let mut last = LAST_CURSOR.lock().expect("cursor state poisoned");
            if *last != Some((want_grab, want_visible)) {
                *last = Some((want_grab, want_visible));
                self.input.cursor_grabbed = want_grab != egui::CursorGrab::None;
                ctx.send_viewport_cmd(egui::ViewportCommand::CursorGrab(want_grab));
                ctx.send_viewport_cmd(egui::ViewportCommand::CursorVisible(want_visible));
            }
        }
        // CPU clock starts after the limiter sleep (measures work, not wait).
        let cpu_t0 = Instant::now();
        let dt = self.last_frame.elapsed().as_secs_f32();
        self.last_frame = Instant::now();
        if dt > 0.0 {
            self.fps_ema += (1.0 / dt - self.fps_ema) * 0.05;
        }
        self.frames += 1;
        let frames = self.frames;
        let ViewerSnapshot {
            generation,
            status,
            frame: client_frame,
            scene: mut scene_metadata,
            debug: retained_debug,
            received,
            debug_transfers,
        } = self.listener.snapshot();
        let scene_update = self.session.update_scene(
            generation,
            client_frame.as_deref(),
            &mut scene_metadata,
            &mut self.camera,
        );
        let generation_changed = scene_update.generation_changed;

        let mode_changed = scene_update.mode_changed;
        let rebuild_arena = scene_update.rebuild_arena;
        if generation_changed {
            self.effects.reset_session();
            self.driven_car = 0;
            if let Ok(mut segs) = self.pending_segs.lock() {
                segs.clear();
            }
            self.debug_overlay = OverlayCache::new();
        }
        let (scene, mesh, last_draw_ms, pads_info, pending_segs, lines_info) = (
            self.scene.clone(),
            self.session.mesh.clone(),
            self.last_draw_ms.clone(),
            self.pads_info.clone(),
            self.pending_segs.clone(),
            self.lines_info.clone(),
        );
        let (gpu_ms, measure_gpu) = (self.gpu_ms.clone(), self.measure_gpu);
        let (soup, welded, indices, fps) = (
            self.session.soup_verts,
            self.session.welded_verts,
            self.session.n_indices,
            self.fps_ema,
        );
        let tick = client_frame.as_ref().map(|frame| frame.state.tick_count);

        let debug_info = self.debug_info.clone();
        let profile_debug = self.debug_metrics.is_some();
        let show_pads = self.settings.show_pads;
        let show_ball = !self.hide_ball;
        let ball_radius_scale = scene_metadata
            .as_ref()
            .map_or(1.0, |m| (m.ball_radius / 91.25).max(0.01));
        let dropshot_mesh = self.dropshot_tiles.get(
            client_frame
                .as_ref()
                .and_then(|frame| dropshot_tile_states(&frame.state)),
        );

        if let Some(frame) = &client_frame {
            let update = self.effects.update(frame, &self.settings);
            if update.rewound {
                self.camera.reset_after_rewind();
            }
            *self.pending_segs.lock().expect("viewer segs poisoned") = update.segments;
        }

        let pose = self.camera.update(CameraFrameInput {
            state: client_frame.as_deref().map(|frame| &frame.state),
            metadata_mode: scene_metadata.as_ref().map(|metadata| metadata.game_mode),
            mode_changed,
            settings: &self.settings,
            action_values: &self.input.action_values,
            action_map,
            evaluator: &self.input.evaluator,
            input_snapshot: &input_snapshot,
            dt,
            focused,
            menu_open: self.ui_state.menu_open,
            action_suppressed,
            pointer_busy: ctx.egui_is_using_pointer(),
        });
        let (cam_pos, cam_dir, cam_fov, cam_line) =
            (pose.position, pose.direction, pose.fov, pose.label);

        // Roof rule: below z 2100 the bowl draws enclosed; above it the
        // cutaway stays open. Covers car views, keeps overviews open.
        let enclosed = cam_pos.z < 2100.0;

        // Driving inputs come from this window (focused only). The driven
        // car is the directly watched car only: other views send neutral,
        // so inputs never jump between cars on a director cut. A frame is
        // sent every tick (seq keeps rising) so the sim sees a live channel.
        // Unfocused windows send neutral (no driving from a stale held key).
        // An open menu disables ALL movement (no driving while configuring).
        let spectated = if self.camera.mode == CamMode::Man {
            self.camera
                .manager
                .as_ref()
                .and_then(|m| m.car_cam())
                .map(|c| c.car_idx as u32)
        } else {
            None
        };
        if let Some(idx) = spectated {
            self.driven_car = idx;
        }
        // Unfocused, rebinding, or menu-open: neutral controls, never a
        // stale key. (The menu's own Reset-play button bypasses this below.)
        let driving = can_drive(
            spectated.is_some(),
            focused,
            action_suppressed,
            self.ui_state.menu_open,
        );
        let (controls, buttons) = action_controls(
            &self.input.action_values,
            action_map,
            &self.input.evaluator,
            &input_snapshot,
            clamp_deadzone(self.settings.bindings.pad.stick_deadzone),
            driving,
            ctx.egui_wants_pointer_input(),
            self.ui_state.reset_requested,
        );
        // Spectate-only beats everything: neutral + zeroed buttons, but the
        // frame below still sends every tick so the channel stays live.
        let (controls, buttons) =
            spectate::gate(self.settings.spectate_only, driving, controls, buttons);
        // The menu's Reset-play button is a deliberate click, not a held
        // movement key: it survives the menu-open neutral above (but never
        // spectate-only, which zeroes all inputs by definition).
        let buttons = if !self.settings.spectate_only && self.ui_state.reset_requested {
            buttons | 0b001
        } else {
            buttons
        };
        // Boost meter source: raw boost of the watched car; None (no follow
        // view, bad index, demoed car) hides the meter.
        let meter = client_frame
            .as_ref()
            .and_then(|frame| spectated_boost(&frame.state.cars, spectated))
            .map(boost_meter);
        self.ctrl_seq += 1;
        self.listener.send_controls_for_generation(
            generation,
            ControlMsg::new(self.driven_car, controls, buttons, self.ctrl_seq).with_mutators(
                if self.settings.spectate_only {
                    0
                } else {
                    self.settings.mutators_mask()
                },
            ),
        );
        // The menu reset fires once, on the next send.
        self.ui_state.reset_requested = false;

        self.settings.save_if_due();

        // Player-facing status lines (all builds).
        let sim_line = match &status {
            ConnStatus::Waiting => "Sim: waiting for a sim…".to_string(),
            ConnStatus::Connected { .. } => {
                if let Some(frame) = &client_frame {
                    format!(
                        "Sim: live · {} · {} cars · ball {:.0} uu/s · tick {}",
                        game_mode_name(frame.state.game_mode()),
                        frame.state.num_cars(),
                        frame.state.ball.phys.vel.length(),
                        tick.unwrap_or(0)
                    )
                } else {
                    format!("Sim: live (tick {})", tick.unwrap_or(0))
                }
            }
        };
        let drive_line = if self.settings.spectate_only {
            spectate::status_text(true, driving, self.driven_car)
        } else if !focused {
            "Input: paused — click this window to drive".to_string()
        } else if self.ui_state.menu_open {
            "Input: paused — close the menu to drive".to_string()
        } else if self.camera.mode == CamMode::Free {
            "Input: free camera — the car coasts".to_string()
        } else if spectated.is_some() {
            if matches!(&status, ConnStatus::Connected { .. }) {
                format!("Input: sending to car {} · connected", self.driven_car)
            } else {
                format!(
                    "Input: car {} · no sim (start the drive demo)",
                    self.driven_car
                )
            }
        } else {
            "Input: watching — follow a car (1-8) to drive it".to_string()
        };

        // Options window (starts open; Escape toggles).
        let connected_pads: Vec<_> = self
            .input
            .gilrs
            .as_ref()
            .map(|g| {
                g.gamepads()
                    .filter(|(_, pad)| pad.is_connected())
                    .map(|(id, pad)| (id, pad.name().to_string()))
                    .collect()
            })
            .unwrap_or_default();
        let settings_input = SettingsInput {
            evaluator: &mut self.input.evaluator,
            active_pad: &mut self.input.active_pad,
            pad_selection_manual: &mut self.input.pad_selection_manual,
            raw_pad: &mut self.input.raw_pad,
            pad_buttons: &mut self.input.pad_buttons,
            suppress_pad_on_next_frame: &mut self.input.suppress_pad_on_next_frame,
        };
        let menu_actions = show_settings(SettingsView {
            ctx,
            ui_state: &mut self.ui_state,
            settings: &mut self.settings,
            input: settings_input,
            input_snapshot: &input_snapshot,
            connected_pads: &connected_pads,
            cam_line: &cam_line,
            drive_line: &drive_line,
            driven_car: self.driven_car,
            rebinding,
        });

        for action in menu_actions {
            match action {
                UiAction::ResetPlay => self.ui_state.reset_requested = true,
                UiAction::SetCamera(CameraChoice::Follow) => {
                    self.camera.mode = CamMode::Man;
                    self.camera.auto_cam = None;
                    if let Some(manager) = self.camera.manager.as_mut() {
                        manager.set_wide_view();
                    }
                }
                UiAction::SetCamera(CameraChoice::Director) => {
                    self.camera.mode = CamMode::Director;
                    self.camera.auto_cam = None;
                }
            }
        }

        if rebinding {
            egui::Modal::new(egui::Id::unique("rebind-input-modal")).show(ctx, |ui| {
                ui.set_min_width(300.0);
                ui.heading("Rebind input");
                ui.label("Press any key, mouse input, or controller input.");
                if let Some((action, slot)) = self.ui_state.rebind_action {
                    let bound = self.settings.action_map().sources(action)[slot];
                    ui.label(format!(
                        "{} · slot {} · currently {}",
                        action.label(),
                        slot + 1,
                        bound.name()
                    ));
                }
                ui.label("Hold the current Back binding for 0.75 seconds to cancel.");
                let cancel = ui.button("Cancel capture");
                self.ui_state.capture_cancel_rect = Some(cancel.rect);
                if cancel.clicked() {
                    self.ui_state.rebind_action = None;
                    self.ui_state.rebind_capture = None;
                    self.input.suppress_until_release(&input_snapshot);
                    self.ui_state.capture_cancel_rect = None;
                }
            });
        } else {
            self.ui_state.capture_cancel_rect = None;
        }

        if self.ui_state.reset_all_confirm {
            self.ui_state.nav_ids.clear();
            egui::Modal::new(egui::Id::unique("reset-all-confirm")).show(ctx, |ui| {
                ui.set_min_width(320.0);
                ui.heading("Reset all settings?");
                ui.label("This restores all viewer and binding defaults.");
                ui.horizontal(|ui| {
                    let cancel = ui.button("Cancel");
                    if cancel.clicked() {
                        self.ui_state.reset_all_confirm = false;
                    }
                    nav_row(
                        ui,
                        &cancel,
                        &mut self.ui_state.nav_ids,
                        "reset.confirm.cancel".into(),
                        self.ui_state.menu_focus,
                        self.ui_state.menu_nav_active,
                        self.ui_state.nav_moved,
                    );
                    let confirm = ui.button("Reset all");
                    if confirm.clicked() {
                        self.settings.reset_all();
                        self.ui_state.reset_all_confirm = false;
                    }
                    nav_row(
                        ui,
                        &confirm,
                        &mut self.ui_state.nav_ids,
                        "reset.confirm.apply".into(),
                        self.ui_state.menu_focus,
                        self.ui_state.menu_nav_active,
                        self.ui_state.nav_moved,
                    );
                });
            });
        }

        // Pad menu navigation (needs the fresh id list from the draw).
        // Up/down move with press + hold-repeat; left/right adjust the
        // focused slider or flip the focused toggle; A activates; B closes.
        if !self.ui_state.nav_ids.is_empty() {
            self.ui_state.menu_focus %= self.ui_state.nav_ids.len();
        }
        // Pad nav stays dead while rebinding (a select press binds
        // instead of activating the focused row).
        if focused
            && self.ui_state.menu_open
            && !action_suppressed
            && !self.ui_state.nav_ids.is_empty()
        {
            let n = self.ui_state.nav_ids.len() as i32;
            let (dy, dx) = menu_nav_step(nav_edge, nav_held, dt, &mut self.ui_state.nav_repeat_cd);
            if dy != 0 {
                self.ui_state.menu_focus =
                    (self.ui_state.menu_focus as i32 + dy).rem_euclid(n) as usize;
                self.ui_state.nav_moved = true;
                self.ui_state.menu_nav_active = true;
            }
            if nav_b_edge {
                if self.ui_state.reset_all_confirm {
                    self.ui_state.reset_all_confirm = false;
                } else {
                    self.ui_state.menu_open = false;
                    ctx.memory_mut(|memory| {
                        memory.surrender_focus(egui::Id::unique("viewer.binding-filter"));
                    });
                }
            } else if dx != 0 || nav_a_edge {
                self.ui_state.menu_nav_active = true;
                let confirming_reset = self.ui_state.reset_all_confirm;
                menu_pad_action(
                    self.ui_state
                        .nav_ids
                        .get(self.ui_state.menu_focus)
                        .cloned()
                        .unwrap_or_default(),
                    dx,
                    nav_a_edge,
                    &mut MenuPadCtx::new(
                        &mut self.settings,
                        &mut self.camera,
                        &mut self.ui_state,
                        &mut self.input,
                        &input_snapshot,
                    ),
                );
                if !confirming_reset && self.ui_state.reset_all_confirm {
                    self.ui_state.menu_focus = 0;
                    self.ui_state.nav_moved = true;
                }
            }
        }

        egui::Window::new("rocketsim viewer").show(ctx, |ui| {
            ui.label(sim_line);
            ui.label(format!("Camera: {cam_line}"));
            ui.label(drive_line);
            ui.label(format!("{fps:.0} fps"));
            if !retained_debug.is_empty() {
                let stats = self.debug_info.lock().map(|s| *s).unwrap_or_default();
                ui.monospace(format!(
                    "Debug: {} groups / {} draws / {} lines / {} points / {} triangles",
                    retained_debug.group_count(),
                    stats.draw_calls,
                    stats.segments,
                    stats.points,
                    stats.primitives
                ));
            }
            if cfg!(debug_assertions) {
                ui.monospace(format!("mesh soup={soup} welded={welded} idx={indices}"));
                ui.monospace(format!(
                    "frames={frames} net={received} tick={} fps={fps:.0}",
                    tick.unwrap_or(0)
                ));
                let ms = last_draw_ms.lock().map(|g| *g).unwrap_or(-1.0);
                ui.monospace(format!("draw: {ms:.3} ms cpu: {:.3} ms", self.cpu_ms));
                if self.measure_gpu {
                    let gpu = gpu_ms.lock().map(|g| *g).unwrap_or(-1.0);
                    ui.monospace(format!("gpu (stall): {gpu:.3} ms"));
                }
                ui.monospace(format!(
                    "seq: {} connected: {}",
                    self.ctrl_seq,
                    matches!(&status, ConnStatus::Connected { .. })
                ));
                ui.monospace(format!("roof: {}", if enclosed { "on" } else { "off" }));
                let (drawn, culled) = pads_info.lock().map(|g| *g).unwrap_or((0, 0));
                ui.monospace(format!("pads: {drawn} drawn, {culled} culled"));
                let (nlines, ltrunc) = lines_info.lock().map(|g| *g).unwrap_or((0, 0));
                ui.monospace(format!("lines: {nlines} ({ltrunc} truncated)"));
                ui.monospace(format!(
                    "cam: {:.0},{:.0},{:.0}",
                    cam_pos.x, cam_pos.y, cam_pos.z
                ));
            }
        });

        egui::CentralPanel::default().show(ui, |ui| {
            let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
            if rect.width() > 1.0 && rect.height() > 1.0 {
                let cb = crate::render::frame::paint_callback(
                    rect,
                    crate::render::frame::PreparedRenderFrame {
                        scene: scene.clone(),
                        mesh: mesh.clone(),
                        frame: client_frame.clone(),
                        debug: retained_debug.clone(),
                        pending_segments: pending_segs.clone(),
                        dropshot_mesh: dropshot_mesh.clone(),
                        camera_position: cam_pos,
                        camera_direction: cam_dir,
                        camera_fov: cam_fov,
                        enclosed,
                        rebuild_arena,
                        generation_changed,
                        show_ball,
                        show_pads,
                        ball_radius_scale,
                        profiling: profile_debug,
                        measure_gpu,
                        debug_info: debug_info.clone(),
                        last_draw_ms: last_draw_ms.clone(),
                        gpu_ms: gpu_ms.clone(),
                        pads_info: pads_info.clone(),
                        lines_info: lines_info.clone(),
                    },
                );
                ui.painter().add(cb);
                let (debug_view, debug_proj) = view::view_proj(
                    glam::Vec3::from(cam_pos),
                    glam::Vec3::from(cam_pos + cam_dir),
                    cam_fov,
                    rect.aspect_ratio(),
                );
                self.debug_overlay.paint(
                    ui,
                    rect,
                    &retained_debug,
                    client_frame.as_deref().map(|frame| &frame.state),
                    &debug_view,
                    &debug_proj,
                );
                // RL-style boost gauge over the 3D view: dim dial, value
                // arc clockwise from the top, number in the middle.
                if let Some(m) = &meter {
                    crate::ui::draw_boost_gauge(ui, rect, m.fraction, &m.label, m.low);
                }
            }
        });
        self.cpu_ms = cpu_t0.elapsed().as_secs_f64() * 1000.0;
        if let Some(metrics) = self.debug_metrics.as_mut() {
            let stats = self.debug_info.lock().map(|s| *s).unwrap_or_default();
            let draw_ms = self.last_draw_ms.lock().map(|s| *s).unwrap_or_default();
            metrics.record(
                self.cpu_ms,
                draw_ms,
                fps as f64,
                stats,
                retained_debug.group_count(),
                debug_transfers,
            );
        }
        // Repaint follows the frame limiter so idle frames are not pinned
        // to 60Hz: uncapped repaints immediately (busy loop), capped waits
        // out the rest of this frame's budget (the limiter sleep above
        // already consumed part of it). Input events still repaint at once,
        // which is why activity used to look faster than idling.
        if self.settings.fps_limit_hz == 0 {
            ctx.request_repaint();
        } else {
            ctx.request_repaint_after(budget.saturating_sub(cpu_t0.elapsed()));
        }
    }
}

#[cfg(test)]
mod tests {
    use rocketsim::GameMode;

    use super::*;

    #[test]
    fn only_dropshot_frames_supply_tile_geometry() {
        let mut state = rocketsim::ArenaState::new_empty(GameMode::Soccar);
        state.tile_states = Some(rocketsim::TileStates::DEFAULT);
        assert!(dropshot_tile_states(&state).is_none());
        let mut state = rocketsim::ArenaState::new_empty(GameMode::Dropshot);
        assert!(dropshot_tile_states(&state).is_none());
        state.tile_states = Some(rocketsim::TileStates::DEFAULT);
        assert_eq!(dropshot_tile_states(&state), state.tile_states.as_ref());
    }
}
