/// Viewer for live 3D simulation frames streamed over TCP.
///
/// Usage: `rocketsim-viewer [PORT] [CAM] [-g MODE] [--bind ADDRESS] [--gpushot]`.
/// The viewer uses stream metadata to build the arena.
mod session;

use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use clap::Parser;
use glam::{Mat3, Vec3, Vec3A};
use glow::HasContext;
use rocketsim::{CarControls, GameMode};
use rocketsim_viewer::{
    bindings::{Key, MouseButton, PadAxis, PadButton, clamp_deadzone, key_down},
    camera::{
        ArenaBoundsCache, AutoCam, CameraConfig, CameraMan, CameraPreset, CarCamSmoothing,
        DirectorState, DirectorTarget, FreeCam,
    },
    dropshot::DropshotTileCache,
    frustum::frustum_planes,
    fx::CarFx,
    hud::{boost_meter, spectated_boost},
    input::{ACTIONS, Action, ActionMap, Axis2, Capture, Evaluator, InputSnapshot, InputSource},
    lines::{Color, RibbonConfig, RibbonEmitter, Segment, ground_line},
    main_gl::{
        BALL_MODEL, BALL_TEXTURE, PUCK_MODEL, car_model_name, car_texture_name, group_car_draws,
        model_matrix,
    },
    mesh::ArenaMesh,
    pads::build_instances,
    scene::SceneGl,
    settings::ViewerSettings,
    spectate, view,
};
use rocketsim_vis::{
    game_mode_name,
    net_client::{ConnStatus, FrameClient},
    parse_game_mode,
    proto::{ControlMsg, DEFAULT_PORT},
};

struct ViewerApp {
    frames: u64,
    mesh: Arc<ArenaMesh>,
    scene: Arc<Mutex<Option<SceneGl>>>,
    client: FrameClient,
    last_draw_ms: Arc<Mutex<f64>>,
    /// GPU stall time in ms (only when `measure_gpu`; serializes pipeline).
    gpu_ms: Arc<Mutex<f64>>,
    /// Last update-body CPU time in ms (excludes the fps-limiter sleep).
    cpu_ms: f64,
    /// GPU-stall timing on/off (`gpushot` CLI).
    measure_gpu: bool,
    /// (pads drawn, pads culled) from the GL callback.
    pads_info: Arc<Mutex<(usize, usize)>>,
    /// (line segments drawn, truncated) from the GL callback.
    lines_info: Arc<Mutex<(usize, usize)>>,
    /// Trail state (CPU-side, updated per viewer frame from sim ticks).
    ball_ribbon: RibbonEmitter,
    boost_ribbons: Vec<RibbonEmitter>,
    /// Effect state (jump/dodge/reset, replayed in sim time).
    car_fx: CarFx,
    dropshot_tiles: DropshotTileCache,
    /// Last sim tick consumed (missed ticks replay into the ribbons).
    last_tick: Option<u64>,
    last_generation: Option<u64>,
    last_ball_pos: Option<Vec3A>,
    last_car_positions: Vec<Vec3A>,
    /// Line segments for the GL callback (built on the UI thread).
    pending_segs: Arc<Mutex<Vec<Segment>>>,
    /// Camera manager (built on the first frame; birds-eye default).
    cam_man: Option<CameraMan>,
    /// CLI start mode (`birds` default).
    init_mode: StartMode,
    /// Active camera mode (CLI initial state, then remappable actions).
    cam_mode: CamMode,
    /// Director rig (selection and camera smoothing).
    director: DirectorRig,
    /// Arena containment hulls, one per game mode (lazy; Soccar warmed).
    bounds: ArenaBoundsCache,
    bounds_for_scene: Option<Arc<rocketsim_viewer::camera::ArenaBounds>>,
    last_metadata: Option<Arc<rocketsim_vis::proto::SceneMetadata>>,
    /// Auto-camera state for the followed car.
    auto_cam: Option<AutoCam>,
    /// Free-camera state. It stays available when camera modes change.
    freecam: FreeCam,
    /// Last followed car (autocam target + drive default).
    last_followed: usize,

    /// Control seq number (+1 per send; sim drops stale).
    ctrl_seq: u64,
    /// Arena car now driven (the directly watched car, else unchanged).
    driven_car: u32,
    /// Reset play requested from the menu (sent once as buttons bit 0b001).
    reset_requested: bool,
    /// Cursor grab state (only sent on change, never each frame).
    cursor_grabbed: bool,
    /// Saved settings (TOML, debounced save).
    settings: ViewerSettings,
    /// Options window state. `MenuToggle` opens or closes it.
    menu_open: bool,
    /// Action currently being rebound.
    rebind_action: Option<(Action, usize)>,
    rebind_capture: Option<Capture>,
    capture_cancel_rect: Option<egui::Rect>,
    capture_cancel_hold: f32,
    capture_start_back: Option<InputSource>,
    capture_pending_back: Option<InputSource>,
    action_evaluator: Evaluator,
    last_input_focused: bool,
    action_values: HashMap<Action, rocketsim_viewer::input::ActionValue>,
    binding_filter: String,
    binding_group: Option<String>,

    headers_open: [bool; 6],
    binding_warning: Option<String>,
    /// Selected local gamepad for viewer actions and rebinding.
    gilrs: Option<gilrs::Gilrs>,
    /// Raw ev-code state for unmapped inputs on the active pad.
    raw_pad: HashMap<u32, f32>,
    /// One selected controller owns all controller input.
    active_pad: Option<gilrs::GamepadId>,
    pad_selection_manual: bool,
    suppress_pad_on_next_frame: bool,
    /// Last effect segments remain visible on repeated sim frames.
    last_fx_segs: Vec<Segment>,
    /// Menu focus index for controller navigation (into the current row list).
    menu_focus: usize,
    /// Controller navigation highlights the focused row.
    menu_nav_active: bool,
    /// Menu navigation repeat cooldown (s).
    nav_repeat_cd: f32,
    /// Focus row changed this frame (drives scroll-to-me).
    nav_moved: bool,
    /// Per-frame menu nav id list, rebuilt during the settings draw.
    nav_ids: Vec<String>,
    /// Right-stick look offsets, smoothed (yaw/pitch rad).
    swivel_yaw: f32,
    swivel_pitch: f32,
    /// UI scale already applied (re-apply on change).
    applied_ui_scale: f32,
    last_frame: Instant,
    fps_ema: f32,
    soup_verts: usize,
    welded_verts: usize,
    n_indices: usize,
}

/// Active viewer camera mode. User actions select camera modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CamMode {
    /// Birds-eye + follow cars.
    Man,
    /// Broadcast director (overview / car cuts).
    Director,
    /// Free camera. User actions control movement and look.
    Free,
}

/// CLI initial camera (`birds` default, `followN`, `director`, `autoN`, `free`).
#[derive(Debug, Clone)]
enum StartMode {
    Birds,
    Follow(usize),
    Director,
    Auto(usize),
    Free,
}

impl StartMode {
    fn try_parse(s: &str) -> Option<Self> {
        let s = s.to_ascii_lowercase();
        let index = |prefix: &str| s.strip_prefix(prefix).and_then(|n| n.parse::<usize>().ok());
        if let Some(n) = index("follow") {
            Some(Self::Follow(n))
        } else if s == "director" {
            Some(Self::Director)
        } else if let Some(n) = index("auto") {
            Some(Self::Auto(n))
        } else if s == "free" {
            Some(Self::Free)
        } else if s == "birds" {
            Some(Self::Birds)
        } else {
            None
        }
    }
}

fn draw_boost_gauge(ui: &egui::Ui, rect: egui::Rect, fraction: f32, label: &str, low: bool) {
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

fn to_gilrs_button(b: PadButton) -> gilrs::Button {
    use gilrs::Button as G;
    match b {
        PadButton::South => G::South,
        PadButton::East => G::East,
        PadButton::North => G::North,
        PadButton::West => G::West,
        PadButton::C => G::C,
        PadButton::Z => G::Z,
        PadButton::LeftTrigger => G::LeftTrigger,
        PadButton::LeftTrigger2 => G::LeftTrigger2,
        PadButton::RightTrigger => G::RightTrigger,
        PadButton::RightTrigger2 => G::RightTrigger2,
        PadButton::Select => G::Select,
        PadButton::Start => G::Start,
        PadButton::Mode => G::Mode,
        PadButton::LeftThumb => G::LeftThumb,
        PadButton::RightThumb => G::RightThumb,
        PadButton::DPadUp => G::DPadUp,
        PadButton::DPadDown => G::DPadDown,
        PadButton::DPadLeft => G::DPadLeft,
        PadButton::DPadRight => G::DPadRight,
        PadButton::Unknown => G::Unknown,
    }
}

fn to_gilrs_axis(a: PadAxis) -> gilrs::Axis {
    use gilrs::Axis as G;
    match a {
        PadAxis::LeftStickX => G::LeftStickX,
        PadAxis::LeftStickY => G::LeftStickY,
        PadAxis::LeftZ => G::LeftZ,
        PadAxis::RightStickX => G::RightStickX,
        PadAxis::RightStickY => G::RightStickY,
        PadAxis::RightZ => G::RightZ,
        PadAxis::DPadX => G::DPadX,
        PadAxis::DPadY => G::DPadY,
        PadAxis::Unknown => G::Unknown,
    }
}

/// Reads the selected controller only.
fn nav_row(
    ui: &mut egui::Ui,
    resp: &egui::Response,
    ids: &mut Vec<String>,
    id: String,
    focus: usize,
    active: bool,
    moved: bool,
) {
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

fn controller_priority(
    connected: bool,
    stick_axes: usize,
    face_buttons: usize,
    mapping: gilrs::MappingSource,
) -> i32 {
    if !connected {
        return i32::MIN;
    }
    let mapping_bonus = match mapping {
        gilrs::MappingSource::SdlMappings => 20,
        gilrs::MappingSource::Driver => 10,
        gilrs::MappingSource::None => 0,
    };
    stick_axes as i32 * 20 + face_buttons as i32 * 5 + mapping_bonus
}

fn pad_priority(pad: gilrs::Gamepad<'_>) -> i32 {
    let stick_axes = [
        gilrs::Axis::LeftStickX,
        gilrs::Axis::LeftStickY,
        gilrs::Axis::RightStickX,
        gilrs::Axis::RightStickY,
    ]
    .into_iter()
    .filter(|axis| pad.axis_data(*axis).is_some())
    .count();
    let face_buttons = [
        gilrs::Button::South,
        gilrs::Button::East,
        gilrs::Button::North,
        gilrs::Button::West,
    ]
    .into_iter()
    .filter(|button| pad.button_data(*button).is_some())
    .count();
    controller_priority(
        pad.is_connected(),
        stick_axes,
        face_buttons,
        pad.mapping_source(),
    )
}

const SWIVEL_MAX_YAW: f32 = 2.0;
/// Right-stick look range, pitch (rad).
const SWIVEL_MAX_PITCH: f32 = 0.9;

fn follow_look_targets(
    values: &HashMap<Action, rocketsim_viewer::input::ActionValue>,
) -> (f32, f32) {
    let value = |action| values.get(&action).map_or(0.0, |v| v.value.clamp(0.0, 1.0));
    (
        (value(Action::SwivelRight) - value(Action::SwivelLeft)) * SWIVEL_MAX_YAW,
        (value(Action::SwivelUp) - value(Action::SwivelDown)) * SWIVEL_MAX_PITCH,
    )
}

fn apply_follow_look(cam_dir: Vec3A, yaw: f32, pitch: f32) -> Vec3A {
    let theta = cam_dir.y.atan2(cam_dir.x) + yaw;
    let elev = (cam_dir.z.clamp(-1.0, 1.0).asin() + pitch).clamp(-1.55, 1.55);
    Vec3A::new(
        elev.cos() * theta.cos(),
        elev.cos() * theta.sin(),
        elev.sin(),
    )
}
fn can_drive(spectated: bool, focused: bool, rebinding: bool, menu_open: bool) -> bool {
    spectated && focused && !rebinding && !menu_open
}

#[allow(clippy::too_many_arguments)]
fn look_delta(
    map: &ActionMap,
    evaluator: &Evaluator,
    snapshot: &InputSnapshot,
    negative: Action,
    positive: Action,
    dt: f32,
    mouse_sensitivity: f32,
    deadzone: f32,
    mouse_busy: bool,
) -> f32 {
    let delta_for = |action| {
        map.sources(action)
            .iter()
            .copied()
            .filter(|source| {
                !mouse_busy
                    || !matches!(
                        source,
                        InputSource::MouseAxis { .. } | InputSource::Wheel { .. }
                    )
            })
            .fold(0.0, |sum, source| {
                let value = evaluator.source_value(snapshot, source, deadzone).max(0.0);
                sum + match source {
                    InputSource::MouseAxis { .. } => value * mouse_sensitivity,
                    InputSource::Wheel { .. } => value * mouse_sensitivity * 8.0,
                    _ => value.min(1.0) * 2.5 * dt,
                }
            })
    };
    delta_for(negative) - delta_for(positive)
}

#[allow(clippy::too_many_arguments)]
fn freecam_dolly_delta(
    map: &ActionMap,
    evaluator: &Evaluator,
    snapshot: &InputSnapshot,
    dt: f32,
    wheel_scale: f32,
    speed: f32,
    deadzone: f32,
    mouse_busy: bool,
) -> f32 {
    let action_delta = |action| {
        map.sources(action)
            .iter()
            .copied()
            .filter(|source| !mouse_busy || !matches!(source, InputSource::Wheel { .. }))
            .map(|source| {
                let value = evaluator.source_value(snapshot, source, deadzone).max(0.0);
                if matches!(source, InputSource::Wheel { .. }) {
                    value * wheel_scale
                } else {
                    value.min(1.0) * speed * dt
                }
            })
            .sum::<f32>()
    };
    action_delta(Action::FreeDollyForward) - action_delta(Action::FreeDollyBack)
}

fn mouse_motion_delta(events: &[egui::Event], pointer_delta: egui::Vec2) -> egui::Vec2 {
    let mut saw_raw_motion = false;
    let raw_delta = events
        .iter()
        .filter_map(|event| match event {
            egui::Event::MouseMoved(delta) => {
                saw_raw_motion = true;
                Some(*delta)
            }
            _ => None,
        })
        .fold(egui::Vec2::ZERO, |sum, delta| sum + delta);
    if events
        .iter()
        .any(|event| matches!(event, egui::Event::PointerMoved(_)))
    {
        pointer_delta
    } else if saw_raw_motion {
        raw_delta
    } else {
        pointer_delta
    }
}

#[derive(Debug, Default)]
struct PadFramePulses {
    buttons: HashMap<PadButton, f32>,
    raw: HashMap<u32, f32>,
}

impl PadFramePulses {
    fn record_press(&mut self, button: gilrs::Button, code: u32) {
        self.raw
            .entry(code)
            .and_modify(|value| *value = value.max(1.0))
            .or_insert(1.0);
        let button = from_gilrs_button(button);
        if button != PadButton::Unknown {
            self.buttons
                .entry(button)
                .and_modify(|value| *value = value.max(1.0))
                .or_insert(1.0);
        }
    }
}

fn update_raw_button_value(raw: &mut HashMap<u32, f32>, code: u32, value: f32) {
    raw.insert(code, value);
}

fn pad_frame_value(current: f32, pulse: f32) -> f32 {
    current.max(pulse)
}

fn from_gilrs_button(button: gilrs::Button) -> PadButton {
    use gilrs::Button as G;
    match button {
        G::South => PadButton::South,
        G::East => PadButton::East,
        G::North => PadButton::North,
        G::West => PadButton::West,
        G::C => PadButton::C,
        G::Z => PadButton::Z,
        G::LeftTrigger => PadButton::LeftTrigger,
        G::LeftTrigger2 => PadButton::LeftTrigger2,
        G::RightTrigger => PadButton::RightTrigger,
        G::RightTrigger2 => PadButton::RightTrigger2,
        G::Select => PadButton::Select,
        G::Start => PadButton::Start,
        G::Mode => PadButton::Mode,
        G::LeftThumb => PadButton::LeftThumb,
        G::RightThumb => PadButton::RightThumb,
        G::DPadUp => PadButton::DPadUp,
        G::DPadDown => PadButton::DPadDown,
        G::DPadLeft => PadButton::DPadLeft,
        G::DPadRight => PadButton::DPadRight,
        G::Unknown => PadButton::Unknown,
    }
}

fn update_raw_pad(raw: &mut HashMap<u32, f32>, event: gilrs::EventType) {
    match event {
        gilrs::EventType::ButtonPressed(_, code) | gilrs::EventType::ButtonRepeated(_, code) => {
            update_raw_button_value(raw, code.into_u32(), 1.0);
        }
        gilrs::EventType::ButtonReleased(_, code) => {
            update_raw_button_value(raw, code.into_u32(), 0.0);
        }
        gilrs::EventType::ButtonChanged(_, value, code)
        | gilrs::EventType::AxisChanged(_, value, code) => {
            update_raw_button_value(raw, code.into_u32(), value);
        }
        _ => {}
    }
}

fn action_input_snapshot(
    ctx: &egui::Context,
    gilrs: Option<&gilrs::Gilrs>,
    active_pad: Option<gilrs::GamepadId>,
    raw: &HashMap<u32, f32>,
    pulses: &PadFramePulses,
    _map: &ActionMap,
    focused: bool,
) -> InputSnapshot {
    let mut snapshot = InputSnapshot::new();
    snapshot.focused = focused;
    ctx.input(|i| {
        for key in Key::ALL {
            let source = InputSource::Key(*key);
            snapshot.set_down(
                source,
                key_down(i, *key) || key.to_egui().is_some_and(|k| i.key_pressed(k)),
            );
        }
        for (button, pointer) in [
            (MouseButton::Primary, egui::PointerButton::Primary),
            (MouseButton::Secondary, egui::PointerButton::Secondary),
            (MouseButton::Middle, egui::PointerButton::Middle),
            (MouseButton::Extra1, egui::PointerButton::Extra1),
            (MouseButton::Extra2, egui::PointerButton::Extra2),
        ] {
            snapshot.set_down(
                InputSource::Mouse(button),
                i.pointer.button_down(pointer) || i.pointer.button_pressed(pointer),
            );
        }
        let pointer_delta = i.pointer.delta();
        let delta = mouse_motion_delta(&i.events, pointer_delta);

        for axis in [Axis2::X, Axis2::Y] {
            let movement = if axis == Axis2::X { delta.x } else { delta.y };
            let wheel = if axis == Axis2::X {
                i.smooth_scroll_delta.x
            } else {
                i.smooth_scroll_delta.y
            };
            for sign in [-1, 1] {
                snapshot.set(
                    InputSource::MouseAxis { axis, sign },
                    (movement * sign as f32).max(0.0),
                );
                snapshot.set(
                    InputSource::Wheel { axis, sign },
                    (wheel * sign as f32).max(0.0),
                );
            }
        }
    });
    if let (Some(gilrs), Some(id)) = (gilrs, active_pad) {
        let pad = gilrs.gamepad(id);
        for button in [
            PadButton::South,
            PadButton::East,
            PadButton::North,
            PadButton::West,
            PadButton::C,
            PadButton::Z,
            PadButton::LeftTrigger,
            PadButton::LeftTrigger2,
            PadButton::RightTrigger,
            PadButton::RightTrigger2,
            PadButton::Select,
            PadButton::Start,
            PadButton::Mode,
            PadButton::LeftThumb,
            PadButton::RightThumb,
            PadButton::DPadUp,
            PadButton::DPadDown,
            PadButton::DPadLeft,
            PadButton::DPadRight,
        ] {
            let value = pad
                .button_data(to_gilrs_button(button))
                .map_or(0.0, |data| data.value());
            let value = pad_frame_value(value, pulses.buttons.get(&button).copied().unwrap_or(0.0));
            snapshot.set(InputSource::PadButton(button), value);
        }
        for axis in [
            PadAxis::LeftStickX,
            PadAxis::LeftStickY,
            PadAxis::LeftZ,
            PadAxis::RightStickX,
            PadAxis::RightStickY,
            PadAxis::RightZ,
            PadAxis::DPadX,
            PadAxis::DPadY,
        ] {
            let value = pad
                .axis_data(to_gilrs_axis(axis))
                .map_or(0.0, |data| data.value());
            for sign in [-1, 1] {
                snapshot.set(
                    InputSource::PadAxis { axis, sign },
                    (value * sign as f32).max(0.0),
                );
            }
        }
    }
    for (&code, &value) in raw {
        let value = pad_frame_value(value, pulses.raw.get(&code).copied().unwrap_or(0.0));
        for sign in [-1, 1] {
            snapshot.set(
                InputSource::PadRaw { code, sign },
                (value * sign as f32).max(0.0),
            );
        }
    }
    for (&code, &value) in &pulses.raw {
        if !raw.contains_key(&code) {
            for sign in [-1, 1] {
                snapshot.set(
                    InputSource::PadRaw { code, sign },
                    (value * sign as f32).max(0.0),
                );
            }
        }
    }
    snapshot
}

#[allow(clippy::too_many_arguments)]
fn action_controls(
    values: &HashMap<Action, rocketsim_viewer::input::ActionValue>,
    map: &ActionMap,
    evaluator: &Evaluator,
    snapshot: &InputSnapshot,
    deadzone: f32,
    enabled: bool,
    pointer_busy: bool,
    reset_requested: bool,
) -> (CarControls, u8) {
    let value = |a| values.get(&a).map_or(0.0, |v| v.value);
    let pressed = |a| values.get(&a).is_some_and(|v| v.pressed);
    let just_pressed = |a| values.get(&a).is_some_and(|v| v.just_pressed);
    let mut controls = CarControls::default();
    let mut buttons = 0;
    if enabled {
        controls.throttle =
            (value(Action::ThrottleForward) - value(Action::ThrottleBack)).clamp(-1.0, 1.0);
        controls.steer = (value(Action::SteerLeft) - value(Action::SteerRight)).clamp(-1.0, 1.0);
        controls.pitch = (value(Action::PitchBack) - value(Action::PitchForward)).clamp(-1.0, 1.0);
        controls.yaw = (value(Action::YawLeft) - value(Action::YawRight)).clamp(-1.0, 1.0);
        controls.roll = (value(Action::RollLeft) - value(Action::RollRight)).clamp(-1.0, 1.0);
        controls.handbrake = pressed(Action::Handbrake);
        let non_mouse_pressed = |action| {
            map.sources(action).iter().copied().any(|source| {
                !matches!(
                    source,
                    InputSource::Mouse(_)
                        | InputSource::MouseAxis { .. }
                        | InputSource::Wheel { .. }
                ) && evaluator.source_value(snapshot, source, deadzone) >= 0.5
            })
        };
        controls.jump = pressed(Action::Jump) && (!pointer_busy || non_mouse_pressed(Action::Jump));
        controls.boost =
            pressed(Action::Boost) && (!pointer_busy || non_mouse_pressed(Action::Boost));
        if pressed(Action::AirRoll) {
            controls.roll += controls.yaw;
            controls.yaw = 0.0;
        }
        if just_pressed(Action::KickoffReset) || reset_requested {
            buttons |= 1;
        }
        if just_pressed(Action::Dribble) {
            buttons |= 2;
        }
        if just_pressed(Action::Launch) {
            buttons |= 4;
        }
    }
    (controls, buttons)
}

/// Pad menu action for one focused row: `dx` (±1 from ◀▶) adjusts the
/// focused slider or flips the focused toggle; `a` (select button)
/// activates buttons and flips toggles. Mirrors the mouse click
/// handlers one row at a time.
/// Mutable menu state for [`menu_pad_action`] (one struct beats nine args).
const MSAA_LEVELS: [u16; 4] = [0, 2, 4, 8];

fn step_msaa(current: u16, direction: i32) -> u16 {
    let index = MSAA_LEVELS
        .iter()
        .position(|&value| value == current)
        .unwrap_or(2);
    MSAA_LEVELS[(index as i32 + direction.signum()).clamp(0, 3) as usize]
}

fn step_stick_deadzone(current: f32, direction: i32) -> f32 {
    let current = if current.is_finite() { current } else { 0.0 };
    (current + direction as f32 * 0.01).clamp(0.0, 0.5)
}

struct MenuPadCtx<'a> {
    settings: &'a mut ViewerSettings,
    cam_mode: &'a mut CamMode,
    cam_man: &'a mut Option<CameraMan>,
    auto_cam: &'a mut Option<AutoCam>,

    reset_requested: &'a mut bool,
    headers_open: &'a mut [bool; 6],
    rebind_action: &'a mut Option<(Action, usize)>,
    rebind_capture: &'a mut Option<Capture>,
    input_snapshot: &'a InputSnapshot,
    binding_group: &'a mut Option<String>,
    binding_filter: &'a mut String,
    capture_start_back: &'a mut Option<InputSource>,
    capture_cancel_hold: &'a mut f32,
    capture_pending_back: &'a mut Option<InputSource>,
    action_evaluator: &'a mut Evaluator,
    active_pad: &'a mut Option<gilrs::GamepadId>,
    pad_selection_manual: &'a mut bool,
    suppress_pad_on_next_frame: &'a mut bool,
    gilrs: Option<&'a gilrs::Gilrs>,
    raw_pad: &'a mut HashMap<u32, f32>,
}

fn menu_pad_action(id: String, dx: i32, a: bool, cx: &mut MenuPadCtx) {
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
        "header.camera" if a => cx.headers_open[0] = !cx.headers_open[0],
        "header.input" if a => cx.headers_open[1] = !cx.headers_open[1],
        "header.mutators" if a => cx.headers_open[2] = !cx.headers_open[2],
        "header.overlays" if a => cx.headers_open[3] = !cx.headers_open[3],
        "header.display" if a => cx.headers_open[4] = !cx.headers_open[4],
        "header.bindings" if a => cx.headers_open[5] = !cx.headers_open[5],
        "bind.capture.cancel" if a => {
            *cx.rebind_action = None;
            *cx.rebind_capture = None;
        }
        "pad.select" if dx != 0 || a => {
            if let Some(gilrs) = cx.gilrs {
                let pads: Vec<_> = gilrs
                    .gamepads()
                    .filter(|(_, pad)| pad.is_connected())
                    .map(|(id, _)| id)
                    .collect();
                if !pads.is_empty() {
                    let index = pads
                        .iter()
                        .position(|id| Some(*id) == *cx.active_pad)
                        .unwrap_or(0) as i32;
                    let direction = if dx == 0 { 1 } else { dx.signum() };
                    *cx.active_pad =
                        Some(pads[(index + direction).rem_euclid(pads.len() as i32) as usize]);
                    *cx.pad_selection_manual = true;
                    cx.raw_pad.clear();
                    *cx.suppress_pad_on_next_frame = true;
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
                .position(|group| group == cx.binding_group)
                .unwrap_or(0) as i32;
            *cx.binding_group =
                groups[(index + dx.signum()).rem_euclid(groups.len() as i32) as usize].clone();
        }
        "bind.filter.clear" if a => cx.binding_filter.clear(),
        "cam.wide" if a => {
            *cx.cam_mode = CamMode::Man;
            *cx.auto_cam = None;
            if let Some(man) = cx.cam_man.as_mut() {
                man.set_wide_view();
            }
        }
        "cam.director" if a => {
            *cx.cam_mode = CamMode::Director;
            *cx.auto_cam = None;
        }
        "cam.reset_play" if a => {
            *cx.reset_requested = true;
        }
        "cam.reset_camera" if a => {
            cx.settings.reset_camera();
        }
        "menu.reset_all" if a => {
            cx.settings.reset_all();
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
                        *cx.rebind_action = Some((action, slot));
                        *cx.rebind_capture = Some(Capture::begin(cx.input_snapshot));

                        *cx.capture_start_back = cx
                            .settings
                            .input_bindings()
                            .sources(Action::MenuBack)
                            .iter()
                            .copied()
                            .find(|source| {
                                *source != InputSource::Unbound
                                    && cx.input_snapshot.value(*source).abs() >= 0.5
                            });
                        *cx.capture_cancel_hold = 0.0;
                        *cx.capture_pending_back = None;
                        cx.action_evaluator
                            .suppress_until_release(cx.input_snapshot);
                    }
                } else if let Some(rest) = id.strip_prefix("bind.clear.") {
                    let mut parts = rest.split('.');
                    if let (Some(index), Some(slot)) = (
                        parts.next().and_then(|x| x.parse::<usize>().ok()),
                        parts.next().and_then(|x| x.parse::<usize>().ok()),
                    ) && let Some(action) = ACTIONS.get(index).copied()
                    {
                        let mut map = cx.settings.input_bindings();
                        map.set(action, slot, InputSource::Unbound);
                        cx.settings.action_bindings = Some(map);
                        cx.settings.mark_dirty();
                    }
                } else if let Some(index) = id
                    .strip_prefix("bind.reset.")
                    .and_then(|x| x.parse::<usize>().ok())
                {
                    if let Some(action) = ACTIONS.get(index).copied() {
                        let defaults = ActionMap::default();
                        let mut map = cx.settings.input_bindings();
                        for slot in 0..2 {
                            map.set(action, slot, defaults.sources(action)[slot]);
                        }
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DirectorIdentity {
    Overview,
    Car(usize),
}

fn director_identity(target: DirectorTarget, state: &rocketsim::ArenaState) -> DirectorIdentity {
    match target {
        DirectorTarget::Overview => DirectorIdentity::Overview,
        DirectorTarget::Car(index) => state
            .cars
            .get(index)
            .map(|(info, _)| DirectorIdentity::Car(info.idx))
            .unwrap_or(DirectorIdentity::Overview),
    }
}

struct DirectorRig {
    director: DirectorState,
    smooth: CarCamSmoothing,
    previous_identity: Option<DirectorIdentity>,
}

impl DirectorRig {
    fn new() -> Self {
        Self {
            director: DirectorState::new(),
            smooth: CarCamSmoothing::new(),
            previous_identity: None,
        }
    }
}

impl ViewerApp {
    fn new(
        mesh: Arc<ArenaMesh>,
        client: FrameClient,
        init_mode: StartMode,
        settings: ViewerSettings,
        menu_open: bool,
        measure_gpu: bool,
    ) -> Self {
        let (soup, welded, indices) = (mesh.soup_verts, mesh.welded_verts(), mesh.index_count());
        let cam_mode = match init_mode {
            StartMode::Director => CamMode::Director,
            StartMode::Free => CamMode::Free,
            _ => CamMode::Man,
        };
        Self {
            frames: 0,
            mesh,
            scene: Arc::new(Mutex::new(None)),
            client,
            settings,
            menu_open,
            applied_ui_scale: 0.0,
            last_draw_ms: Arc::new(Mutex::new(0.0)),
            gpu_ms: Arc::new(Mutex::new(0.0)),
            cpu_ms: 0.0,
            measure_gpu,
            pads_info: Arc::new(Mutex::new((0, 0))),
            lines_info: Arc::new(Mutex::new((0, 0))),
            ball_ribbon: RibbonEmitter::new(RibbonConfig::ball_trail()),
            boost_ribbons: Vec::new(),
            car_fx: CarFx::default(),
            dropshot_tiles: DropshotTileCache::default(),
            last_tick: None,
            last_generation: None,
            last_ball_pos: None,
            last_car_positions: Vec::new(),
            pending_segs: Arc::new(Mutex::new(Vec::new())),
            cam_man: None,
            init_mode,
            cam_mode,
            director: DirectorRig::new(),
            bounds: ArenaBoundsCache::new(),
            bounds_for_scene: None,
            last_metadata: None,
            auto_cam: None,
            freecam: FreeCam::new(),
            last_followed: 0,
            ctrl_seq: 0,
            driven_car: 0,
            reset_requested: false,
            cursor_grabbed: false,
            rebind_action: None,
            rebind_capture: None,
            capture_cancel_rect: None,
            capture_cancel_hold: 0.0,
            capture_start_back: None,
            capture_pending_back: None,
            action_evaluator: Evaluator::default(),
            last_input_focused: false,
            action_values: HashMap::new(),
            binding_filter: String::new(),
            binding_group: None,

            headers_open: [true, true, false, false, true, false],
            binding_warning: None,
            gilrs: gilrs::Gilrs::new().ok(),
            raw_pad: HashMap::new(),
            active_pad: None,
            pad_selection_manual: false,
            suppress_pad_on_next_frame: false,
            last_fx_segs: Vec::new(),
            menu_focus: 0,
            menu_nav_active: false,
            nav_repeat_cd: 0.0,
            nav_moved: false,
            nav_ids: Vec::new(),
            swivel_yaw: 0.0,
            swivel_pitch: 0.0,
            last_frame: Instant::now(),
            fps_ema: 60.0,
            soup_verts: soup,
            welded_verts: welded,
            n_indices: indices,
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
        if (self.settings.ui_scale - self.applied_ui_scale).abs() > 1e-6 {
            ctx.set_pixels_per_point(self.settings.ui_scale);
            self.applied_ui_scale = self.settings.ui_scale;
        }
        let focused = ctx.input(|i| i.focused);
        // Viewer-side pad reader (menu toggle, menu nav, camera look,
        // pad driving; the sim just takes the channel). Edges + held
        // state feed the menu block and the swivel below.
        let mut nav_edge = [false; 4];
        let mut nav_held = [false; 4];

        // Pump pad events EVERY frame first: state reads below go stale
        // otherwise (nothing else drains while playing). The rebind
        // capture later reuses this same batch.
        let mut pad_events: Vec<gilrs::Event> = Vec::new();
        let mut pad_device_changed = false;
        if let Some(g) = self.gilrs.as_mut() {
            while let Some(ev) = g.next_event() {
                pad_events.push(ev);
            }
            let old_active = self.active_pad;
            let connected_event = pad_events.iter().find_map(|event| {
                matches!(event.event, gilrs::EventType::Connected).then_some(event.id)
            });
            let disconnected_event = pad_events.iter().find_map(|event| {
                matches!(event.event, gilrs::EventType::Disconnected).then_some(event.id)
            });
            let connected_ids: Vec<_> = g
                .gamepads()
                .filter(|(_, pad)| pad.is_connected())
                .map(|(id, _)| id)
                .collect();
            let stable_fallback = session::choose_active_pad(
                self.active_pad,
                connected_event,
                disconnected_event,
                &connected_ids,
            );
            let mut ranked: Vec<_> = g
                .gamepads()
                .filter(|(_, pad)| pad.is_connected())
                .map(|(id, pad)| (pad_priority(pad), id))
                .collect();
            ranked.sort_by_key(|(priority, _)| *priority);
            let best = ranked.last().map(|(_, id)| *id).or(stable_fallback);
            let current_connected = self
                .active_pad
                .is_some_and(|id| g.gamepad(id).is_connected());
            let current_priority = self.active_pad.map(|id| pad_priority(g.gamepad(id)));
            if !current_connected {
                self.active_pad = best;
                self.pad_selection_manual = false;
            } else if !self.pad_selection_manual
                && let (Some(best), Some(best_priority), Some(current_priority)) = (
                    best,
                    ranked.last().map(|(priority, _)| *priority),
                    current_priority,
                )
                && best_priority > current_priority
            {
                self.active_pad = Some(best);
            }
            pad_device_changed = old_active != self.active_pad;
            if pad_device_changed {
                self.raw_pad.clear();
                self.suppress_pad_on_next_frame = true;
            }
        }
        // Keep final raw state and one-frame button press pulses. This keeps
        // quick press/release input visible to actions and capture.
        let suppress_pad_pulses = pad_device_changed || self.suppress_pad_on_next_frame;
        let mut pad_pulses = PadFramePulses::default();
        for event in &pad_events {
            if Some(event.id) != self.active_pad {
                continue;
            }
            match event.event {
                gilrs::EventType::ButtonPressed(button, code) => {
                    update_raw_pad(&mut self.raw_pad, event.event);
                    if !suppress_pad_pulses {
                        pad_pulses.record_press(button, code.into_u32());
                    }
                }
                gilrs::EventType::ButtonReleased(_, _)
                | gilrs::EventType::ButtonRepeated(_, _)
                | gilrs::EventType::ButtonChanged(_, _, _)
                | gilrs::EventType::AxisChanged(_, _, _) => {
                    update_raw_pad(&mut self.raw_pad, event.event);
                }
                _ => {}
            }
        }
        let action_map = self.settings.input_bindings();
        let input_snapshot = action_input_snapshot(
            ctx,
            self.gilrs.as_ref(),
            self.active_pad,
            &self.raw_pad,
            &pad_pulses,
            &action_map,
            focused,
        );
        if self.suppress_pad_on_next_frame {
            self.action_evaluator
                .suppress_until_release(&input_snapshot);
            self.suppress_pad_on_next_frame = false;
        }
        let cancel_rect = self.capture_cancel_rect;
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
        let mut capture_completed = false;
        let mut capture_cancelled = false;
        let mut controller_cancel_hold = false;
        if let Some((action, slot)) = self.rebind_action {
            if !self.menu_open {
                self.rebind_action = None;
                self.rebind_capture = None;
                self.action_evaluator
                    .suppress_until_release(&input_snapshot);
                capture_cancelled = true;
            } else {
                let back_held =
                    action_map
                        .sources(Action::MenuBack)
                        .iter()
                        .copied()
                        .find(|source| {
                            *source != InputSource::Unbound
                                && input_snapshot.value(*source).abs() >= 0.5
                        });
                if self
                    .capture_start_back
                    .is_some_and(|source| input_snapshot.value(source).abs() < 0.5)
                {
                    self.capture_start_back = None;
                }
                let cancel_source =
                    back_held.filter(|source| Some(*source) != self.capture_start_back);
                let mut bind_source = None;
                if let Some(source) = cancel_source {
                    self.capture_pending_back = Some(source);
                    self.capture_cancel_hold += self.last_frame.elapsed().as_secs_f32();
                    controller_cancel_hold = self.capture_cancel_hold >= 0.75;
                } else if self.capture_pending_back.is_some() {
                    bind_source = self.capture_pending_back.take();
                    self.capture_cancel_hold = 0.0;
                } else {
                    self.capture_cancel_hold = 0.0;
                }
                if controller_cancel_hold {
                    self.rebind_action = None;
                    self.rebind_capture = None;
                    self.capture_cancel_hold = 0.0;
                    self.capture_pending_back = None;
                    self.action_evaluator
                        .suppress_until_release(&input_snapshot);
                    capture_cancelled = true;
                } else if !cancel_click
                    && cancel_source.is_none()
                    && let Some(capture) = self.rebind_capture.as_mut()
                {
                    let mut capture_snapshot = input_snapshot.clone();
                    if cancel_hover {
                        for axis in [Axis2::X, Axis2::Y] {
                            for sign in [-1, 1] {
                                capture_snapshot.set(InputSource::MouseAxis { axis, sign }, 0.0);
                            }
                        }
                    }
                    let source = capture.poll(&capture_snapshot).or(bind_source);
                    if let Some(source) = source {
                        self.settings
                            .action_bindings
                            .get_or_insert_with(|| action_map.clone())
                            .set(action, slot, source);
                        self.settings.mark_dirty();
                        self.binding_warning = None;
                        self.rebind_action = None;
                        self.rebind_capture = None;
                        self.action_evaluator
                            .suppress_until_release(&input_snapshot);
                        capture_completed = true;
                    }
                }
            }
        }
        if focused && !self.last_input_focused {
            self.action_evaluator
                .suppress_until_release(&input_snapshot);
        }
        self.last_input_focused = focused;
        let rebinding = self.rebind_action.is_some() || capture_completed || capture_cancelled;
        let mut menu_toggled = false;
        self.action_values = self.action_evaluator.evaluate(
            &action_map,
            &input_snapshot,
            clamp_deadzone(self.settings.bindings.pad.stick_deadzone),
        );
        if self
            .action_values
            .get(&Action::MenuToggle)
            .is_some_and(|v| v.just_pressed)
            && !rebinding
        {
            self.menu_open = !self.menu_open;
            menu_toggled = true;
            self.action_evaluator
                .suppress_until_release(&input_snapshot);
            if !self.menu_open {
                ctx.memory_mut(|memory| {
                    memory.surrender_focus(egui::Id::unique("viewer.binding-filter"));
                });
            }
            self.menu_nav_active = true;
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
            let value = self.action_values.get(&action).copied().unwrap_or_default();
            nav_edge[i] = value.just_pressed;
            nav_held[i] = value.pressed;
        }
        let keyboard_editing = ctx.egui_wants_keyboard_input();
        let nav_a_edge = !keyboard_editing
            && self
                .action_values
                .get(&Action::MenuAccept)
                .is_some_and(|v| v.just_pressed);
        let nav_b_edge = !keyboard_editing
            && self
                .action_values
                .get(&Action::MenuBack)
                .is_some_and(|v| v.just_pressed);
        if keyboard_editing {
            nav_edge = [false; 4];
            nav_held = [false; 4];
        }

        // Mouse use hides the pad highlight again.
        if ctx.input(|i| i.pointer.delta() != egui::Vec2::ZERO) {
            self.menu_nav_active = false;
        }
        // Keep the cursor locked for camera look. Menus and unfocused windows release it.
        // menu open or focus lost releases all. Sent on change.
        let (want_grab, want_visible) =
            if !focused || self.menu_open || self.cam_mode != CamMode::Free {
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
                self.cursor_grabbed = want_grab != egui::CursorGrab::None;
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
        let snapshot = session::snapshot(&self.client);
        let generation = snapshot.generation;
        let client_frame = snapshot.frame;
        let mut scene_metadata = snapshot.scene;
        let generation_changed = session::generation_changed(self.last_generation, generation);
        self.last_generation = Some(generation);
        let metadata_mode_mismatch = matches!(
            (&client_frame, &scene_metadata),
            (Some(frame), Some(metadata)) if frame.state.game_mode() != metadata.game_mode
        );
        if metadata_mode_mismatch {
            scene_metadata = None;
        }
        if generation_changed {
            self.last_tick = None;
            self.last_ball_pos = None;
            self.last_car_positions.clear();
            self.ball_ribbon = RibbonEmitter::new(RibbonConfig::ball_trail());
            self.boost_ribbons.clear();
            self.car_fx = CarFx::default();
            self.last_fx_segs.clear();
            self.cam_man = None;
            self.auto_cam = None;
            self.director = DirectorRig::new();
            self.freecam = FreeCam::new();
            self.bounds.reset();
            self.bounds_for_scene = None;
            self.last_metadata = None;
            self.mesh = Arc::new(ArenaMesh::placeholder(self.mesh.mode));
            self.soup_verts = self.mesh.soup_verts;
            self.welded_verts = self.mesh.welded_verts();
            self.n_indices = self.mesh.index_count();
            self.last_followed = 0;
            self.driven_car = 0;
            self.swivel_yaw = 0.0;
            self.swivel_pitch = 0.0;
            if let Ok(mut segs) = self.pending_segs.lock() {
                segs.clear();
            }
        }
        let metadata_changed = scene_metadata
            .as_ref()
            .is_some_and(|metadata| metadata_changed(self.last_metadata.as_ref(), metadata));
        if let Some(metadata) = &scene_metadata
            && metadata_changed
        {
            let mesh = Arc::new(ArenaMesh::from_meshes(metadata.game_mode, &metadata.meshes));
            self.soup_verts = mesh.soup_verts;
            self.welded_verts = mesh.welded_verts();
            self.n_indices = mesh.index_count();
            self.mesh = mesh;
            self.bounds_for_scene = self
                .bounds
                .get_for_meshes(metadata.game_mode, &metadata.meshes);
            self.last_metadata = Some(metadata.clone());
        }
        let mut mode_changed = false;
        if let Some(frame) = &client_frame
            && frame.state.game_mode() != self.mesh.mode
        {
            let mode = frame.state.game_mode();
            self.mesh = Arc::new(
                scene_metadata
                    .as_ref()
                    .filter(|metadata| metadata.game_mode == mode)
                    .map(|metadata| ArenaMesh::from_meshes(mode, &metadata.meshes))
                    .unwrap_or_else(|| ArenaMesh::placeholder(mode)),
            );
            self.soup_verts = self.mesh.soup_verts;
            self.welded_verts = self.mesh.welded_verts();
            self.n_indices = self.mesh.index_count();
            if let Some(metadata) = scene_metadata
                .as_ref()
                .filter(|metadata| metadata.game_mode == mode)
            {
                self.bounds_for_scene = self.bounds.get_for_meshes(mode, &metadata.meshes);
                self.last_metadata = Some(metadata.clone());
            } else {
                self.bounds_for_scene = None;
                self.last_metadata = None;
            }
            mode_changed = true;
            eprintln!(
                "rocketsim-viewer: game mode switched to {}",
                game_mode_name(mode)
            );
        }
        let rebuild_arena = generation_changed || metadata_changed || mode_changed;
        let (scene, mesh, last_draw_ms, pads_info, pending_segs, lines_info) = (
            self.scene.clone(),
            self.mesh.clone(),
            self.last_draw_ms.clone(),
            self.pads_info.clone(),
            self.pending_segs.clone(),
            self.lines_info.clone(),
        );
        let (gpu_ms, measure_gpu) = (self.gpu_ms.clone(), self.measure_gpu);
        let (soup, welded, indices, fps) = (
            self.soup_verts,
            self.welded_verts,
            self.n_indices,
            self.fps_ema,
        );
        let (status, received) = (self.client.status(), self.client.received());
        let tick = client_frame.as_ref().map(|f| f.state.tick_count);
        let debug_draw = client_frame.as_ref().map(|f| f.debug.clone());
        let show_pads = self.settings.show_pads;
        let ball_radius_scale = scene_metadata
            .as_ref()
            .map_or(1.0, |m| (m.ball_radius / 91.25).max(0.01));
        let dropshot_mesh = self.dropshot_tiles.get(
            client_frame
                .as_ref()
                .and_then(|frame| dropshot_tile_states(&frame.state)),
        );

        // Advance trails in sim time: replay missed ticks (clamped, so a
        // reset jump cannot spiral the emitters).
        if let Some(frame) = &client_frame {
            let tick_now = frame.state.tick_count;
            let advance = session::tick_advance(self.last_tick, tick_now);
            let rewound = advance.reset;
            let steps = advance.steps;
            let ball_pos = frame.state.ball.phys.pos;
            let ball_teleport = self
                .last_ball_pos
                .is_some_and(|p| p.distance(ball_pos) > 1200.0);
            if rewound || ball_teleport {
                self.ball_ribbon = RibbonEmitter::new(RibbonConfig::ball_trail());
                self.boost_ribbons.clear();
                self.car_fx = CarFx::default();
                self.last_fx_segs.clear();
            }
            if rewound {
                self.cam_man = None;
                self.auto_cam = None;
                self.director = DirectorRig::new();
                self.freecam = FreeCam::new();
                self.swivel_yaw = 0.0;
                self.swivel_pitch = 0.0;
                self.last_followed = 0;
                self.last_car_positions.clear();
            }
            self.last_tick = Some(tick_now);
            self.last_ball_pos = Some(ball_pos);
            for _ in 0..steps {
                self.ball_ribbon
                    .update(true, ball_pos, Vec3A::ZERO, frame.dt);
            }
            let mut segs = if self.settings.show_trail {
                self.ball_ribbon.segments(ball_pos)
            } else {
                Vec::new()
            };
            let floor_z = rocketsim::consts::arena::get_aabb(frame.state.game_mode())
                .min
                .z;
            if self.settings.show_ground_line {
                segs.push(ground_line(ball_pos, floor_z));
            }
            // Effects replay missed ticks but keep only the last output
            // (rings draw at current age; replay would double-draw them).
            if steps > 0 {
                for _ in 0..steps {
                    self.last_fx_segs = self.car_fx.update(&frame.state, frame.dt);
                }
            }
            segs.extend(self.last_fx_segs.iter().cloned());
            segs.extend(frame.debug.lines.iter().map(|line| {
                Segment::flat(
                    Vec3A::from_array(line.start),
                    Vec3A::from_array(line.end),
                    Color::new(line.color[0], line.color[1], line.color[2], line.color[3]),
                    line.width,
                )
            }));
            for (i, (_, car)) in frame.state.cars.iter().enumerate() {
                while self.boost_ribbons.len() <= i {
                    self.boost_ribbons
                        .push(RibbonEmitter::new(RibbonConfig::boost()));
                }
                let car_pos = car.phys.pos;
                let car_teleport = self
                    .last_car_positions
                    .get(i)
                    .is_some_and(|p| p.distance(car_pos) > 1200.0);
                if car_teleport {
                    self.boost_ribbons[i] = RibbonEmitter::new(RibbonConfig::boost());
                }
                let fwd = car.phys.get_forward_dir();
                let emit_pos = car_pos - fwd * 60.0;
                let emit_vel = -fwd * 300.0;
                for _ in 0..steps {
                    self.boost_ribbons[i].update(car.is_boosting, emit_pos, emit_vel, frame.dt);
                }
                if self.settings.show_boost {
                    segs.extend(self.boost_ribbons[i].segments(emit_pos));
                }
            }
            self.last_car_positions = frame.state.cars.iter().map(|(_, c)| c.phys.pos).collect();
            *self.pending_segs.lock().expect("viewer segs poisoned") = segs;
        }

        // Camera: fixed pose until the first frame, then the active mode.
        // Camera shortcuts use the remappable action map. Driving controls
        // stay disabled in free-camera mode.
        let (cam_pos, mut cam_dir, cam_fov, cam_line) = match &client_frame {
            Some(frame) => {
                let state = &frame.state;
                if mode_changed && let Some(man) = self.cam_man.as_mut() {
                    man.set_game_mode(state.game_mode());
                }
                // Containment hull for this mode (lazy; TheVoid = None).
                let bounds = if scene_metadata
                    .as_ref()
                    .is_some_and(|metadata| metadata.game_mode == state.game_mode())
                {
                    self.bounds_for_scene.clone()
                } else {
                    None
                };
                if self.cam_man.is_none() {
                    let mut man = CameraMan::new(state.game_mode(), CameraConfig::default());
                    match self.init_mode {
                        StartMode::Follow(n) if n < state.num_cars() => man.follow_car(state, n),
                        StartMode::Auto(n) if n < state.num_cars() => {
                            man.follow_car(state, n);
                            self.auto_cam = Some(AutoCam::new(n));
                        }
                        StartMode::Follow(n) | StartMode::Auto(n) => {
                            eprintln!("rocketsim-viewer: car {n} is unavailable; using wide view");
                        }
                        _ => {}
                    }
                    self.cam_man = Some(man);
                }
                let previous_cam_mode = self.cam_mode;
                // All shortcuts stay dead while rebinding: the pressed key
                // rebinds instead of switching cameras or driving.
                for idx in 0..8 {
                    let action = [
                        Action::Follow1,
                        Action::Follow2,
                        Action::Follow3,
                        Action::Follow4,
                        Action::Follow5,
                        Action::Follow6,
                        Action::Follow7,
                        Action::Follow8,
                    ][idx];
                    if !action_suppressed
                        && !self.menu_open
                        && self
                            .action_values
                            .get(&action)
                            .is_some_and(|v| v.just_pressed)
                    {
                        self.cam_mode = CamMode::Man;
                        self.auto_cam = None;
                        if let Some(man) = self.cam_man.as_mut() {
                            man.follow_car(state, idx);
                        }
                    }
                }
                if !action_suppressed
                    && !self.menu_open
                    && self
                        .action_values
                        .get(&Action::Director)
                        .is_some_and(|v| v.just_pressed)
                {
                    self.cam_mode = CamMode::Director;
                    self.auto_cam = None;
                }
                if !action_suppressed
                    && !self.menu_open
                    && self
                        .action_values
                        .get(&Action::AutoCam)
                        .is_some_and(|v| v.just_pressed)
                {
                    self.cam_mode = CamMode::Man;
                    let idx = self.last_followed;
                    if let Some(man) = self.cam_man.as_mut() {
                        man.follow_car(state, idx);
                    }
                    self.auto_cam = Some(AutoCam::new(idx));
                }
                if !action_suppressed
                    && !self.menu_open
                    && self
                        .action_values
                        .get(&Action::FreeCam)
                        .is_some_and(|v| v.just_pressed)
                {
                    self.cam_mode = if self.cam_mode == CamMode::Free {
                        CamMode::Man
                    } else {
                        CamMode::Free
                    };
                    self.auto_cam = None;
                }
                if !action_suppressed
                    && !self.menu_open
                    && self
                        .action_values
                        .get(&Action::NextCam)
                        .is_some_and(|v| v.just_pressed)
                {
                    self.cam_mode = CamMode::Man;
                    self.auto_cam = None;
                    if let Some(man) = self.cam_man.as_mut() {
                        man.cycle_target(state);
                    }
                }
                if self.cam_mode != previous_cam_mode {
                    self.swivel_yaw = 0.0;
                    self.swivel_pitch = 0.0;
                }
                match self.cam_mode {
                    CamMode::Man => {
                        // Ball-cam uses its mapped action only in follow mode.
                        if !action_suppressed
                            && !self.menu_open
                            && self
                                .action_values
                                .get(&Action::BallCam)
                                .is_some_and(|v| v.just_pressed)
                            && let Some(man) = self.cam_man.as_mut()
                        {
                            man.try_toggle_ball_cam();
                        }
                        // Auto-cam drives ball-cam when armed on this car.
                        if let Some(auto) = self.auto_cam.as_mut() {
                            let idx = auto.car_idx();
                            let on = auto.update(state, dt);
                            if let Some(man) = self.cam_man.as_mut() {
                                let following = man.car_cam().is_some_and(|c| c.car_idx == idx);
                                if following {
                                    man.set_follow_face_ball(idx, on);
                                } else {
                                    self.auto_cam = None;
                                }
                            }
                        }
                        let man = self.cam_man.as_mut().expect("viewer camera built");
                        man.set_config(self.settings.camera);
                        man.update(state, dt);
                        if let Some(cam) = man.car_cam() {
                            self.last_followed = cam.car_idx;
                        }
                        // Arena containment: shorten the boom against the
                        // hull (focus stays; ball-cam re-aims at the ball
                        // from the pulled-in pose, car-cam keeps its look).
                        if let (Some(hull), Some(cam)) = (bounds.as_ref(), man.car_cam())
                            && let Some((_, car)) = state.cars.get(cam.car_idx)
                            && !car.is_demoed
                        {
                            let focus =
                                car.pos + Vec3A::new(0.0, 0.0, self.settings.camera.car_cam.height);
                            let (p, d) = hull.contain_follow(
                                focus,
                                state.ball.pos,
                                cam.face_ball,
                                man.cur_pos(),
                                man.cur_dir(),
                            );
                            man.override_pose(p, d);
                        }
                        let label = match (man.car_cam(), &self.auto_cam) {
                            (None, _) => "wide view".to_string(),
                            (Some(cam), None) => format!(
                                "car {} · {}",
                                cam.car_idx,
                                if cam.face_ball { "ball cam" } else { "car cam" }
                            ),
                            (Some(cam), Some(auto)) => format!(
                                "car {} · auto ({})",
                                cam.car_idx,
                                auto.off_reason().map(|r| r.as_str()).unwrap_or("ball cam")
                            ),
                        };
                        (man.cur_pos(), man.cur_dir(), man.cur_fov_degrees(), label)
                    }
                    CamMode::Director => {
                        let rig = &mut self.director;
                        let target = rig.director.update(state, dt);
                        let identity = director_identity(target, state);
                        if rig.previous_identity != Some(identity) {
                            rig.smooth.reset();
                            rig.previous_identity = Some(identity);
                        }
                        let (pos, dir, fov, label) = match target {
                            DirectorTarget::Overview => {
                                let (eye, target, fov) = view::arena_pose();
                                (
                                    Vec3A::from(eye),
                                    (Vec3A::from(target) - Vec3A::from(eye)).normalize_or_zero(),
                                    fov,
                                    "director · wide view".to_string(),
                                )
                            }
                            DirectorTarget::Car(index) => {
                                let mut pos = Vec3A::ZERO;
                                let mut dir = Vec3A::X;
                                rig.smooth.update(
                                    index,
                                    true,
                                    state,
                                    &self.settings.camera,
                                    &mut pos,
                                    &mut dir,
                                    dt,
                                );
                                if let (Some(hull), Some((_, car))) =
                                    (bounds.as_ref(), state.cars.get(index))
                                {
                                    let focus = car.pos
                                        + Vec3A::new(0.0, 0.0, self.settings.camera.car_cam.height);
                                    (pos, dir) =
                                        hull.contain_follow(focus, state.ball.pos, true, pos, dir);
                                }
                                (
                                    pos,
                                    dir,
                                    rig.smooth.cur_fov_h(),
                                    format!("director · car {}", state.cars[index].0.idx),
                                )
                            }
                        };
                        (pos, dir, fov, label)
                    }
                    CamMode::Free => {
                        // Free-camera mouse input is ignored over UI. Keyboard and
                        // controller look input remains active over UI.
                        const LOOK_SENS: f32 = 0.005;

                        if focused && !self.menu_open {
                            let pointer_busy = ctx.egui_is_using_pointer();
                            let deadzone =
                                clamp_deadzone(self.settings.bindings.pad.stick_deadzone);
                            let yaw_delta = look_delta(
                                &action_map,
                                &self.action_evaluator,
                                &input_snapshot,
                                Action::LookLeft,
                                Action::LookRight,
                                dt,
                                LOOK_SENS,
                                deadzone,
                                pointer_busy,
                            );
                            let pitch_delta = look_delta(
                                &action_map,
                                &self.action_evaluator,
                                &input_snapshot,
                                Action::LookUp,
                                Action::LookDown,
                                dt,
                                LOOK_SENS,
                                deadzone,
                                pointer_busy,
                            );
                            self.freecam.look(yaw_delta, pitch_delta);
                        }
                        // Flight reuses the car keys; up couples to the
                        // ball-cam key (Space by default).
                        // Flight keys stay dead while rebinding or with the
                        // menu open (no movement while configuring).
                        let (f, r, u) = if action_suppressed || self.menu_open || !focused {
                            (0.0, 0.0, 0.0)
                        } else {
                            let v = |a| self.action_values.get(&a).map_or(0.0, |v| v.value);
                            (
                                v(Action::FreeForward) - v(Action::FreeBack),
                                v(Action::FreeRight) - v(Action::FreeLeft),
                                v(Action::FreeUp) - v(Action::FreeDown),
                            )
                        };
                        if !self.menu_open
                            && self
                                .action_values
                                .get(&Action::FreeReset)
                                .is_some_and(|v| v.just_pressed)
                        {
                            self.freecam = FreeCam::new();
                        }
                        let fast = if !self.menu_open
                            && self
                                .action_values
                                .get(&Action::FreeFast)
                                .is_some_and(|v| v.pressed)
                        {
                            3.0
                        } else {
                            1.0
                        };
                        self.freecam
                            .fly_bounded(f, r, u, dt * fast, bounds.as_deref());
                        if focused && !self.menu_open && !action_suppressed {
                            let dolly = freecam_dolly_delta(
                                &action_map,
                                &self.action_evaluator,
                                &input_snapshot,
                                dt,
                                48.0,
                                220.0,
                                clamp_deadzone(self.settings.bindings.pad.stick_deadzone),
                                ctx.egui_is_using_pointer(),
                            );
                            self.freecam.dolly_bounded(dolly, bounds.as_deref());
                        }

                        let (p, d) = self.freecam.pos_dir();
                        (
                            p,
                            d,
                            self.settings.camera.fov_degrees,
                            "free camera".to_string(),
                        )
                    }
                }
            }
            None => {
                let (eye, target, fov) = view::arena_pose();
                (
                    Vec3A::from(eye),
                    (Vec3A::from(target) - Vec3A::from(eye)).normalize_or_zero(),
                    fov,
                    "waiting for sim".to_string(),
                )
            }
        };

        // Follow-view swivel uses its own remappable actions. Unlike freecam
        // look defaults, this does not make mouse movement alter car views.
        let following = self.cam_mode == CamMode::Man
            && self.cam_man.as_ref().and_then(|m| m.car_cam()).is_some();
        let (look_tx, look_ty) = if following && !self.menu_open {
            follow_look_targets(&self.action_values)
        } else {
            (0.0, 0.0)
        };
        let k = 1.0 - (-10.0 * dt).exp();
        self.swivel_yaw += (look_tx - self.swivel_yaw) * k;
        self.swivel_pitch += (look_ty - self.swivel_pitch) * k;
        if self.swivel_yaw.abs() + self.swivel_pitch.abs() > 1e-4 {
            cam_dir = apply_follow_look(cam_dir, self.swivel_yaw, self.swivel_pitch);
        }

        // Roof rule: below z 2100 the bowl draws enclosed; above it the
        // cutaway stays open. Covers car views, keeps overviews open.
        let enclosed = cam_pos.z < 2100.0;

        // Driving inputs come from this window (focused only). The driven
        // car is the directly watched car only: other views send neutral,
        // so inputs never jump between cars on a director cut. A frame is
        // sent every tick (seq keeps rising) so the sim sees a live channel.
        // Unfocused windows send neutral (no driving from a stale held key).
        // An open menu disables ALL movement (no driving while configuring).
        let spectated = if self.cam_mode == CamMode::Man {
            self.cam_man
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
            self.menu_open,
        );
        let (controls, buttons) = action_controls(
            &self.action_values,
            &action_map,
            &self.action_evaluator,
            &input_snapshot,
            clamp_deadzone(self.settings.bindings.pad.stick_deadzone),
            driving,
            ctx.egui_wants_pointer_input(),
            self.reset_requested,
        );
        // Spectate-only beats everything: neutral + zeroed buttons, but the
        // frame below still sends every tick so the channel stays live.
        let (controls, buttons) =
            spectate::gate(self.settings.spectate_only, driving, controls, buttons);
        // The menu's Reset-play button is a deliberate click, not a held
        // movement key: it survives the menu-open neutral above (but never
        // spectate-only, which zeroes all inputs by definition).
        let buttons = if !self.settings.spectate_only && self.reset_requested {
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
        session::send_controls(
            &self.client,
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
        self.reset_requested = false;

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
        } else if self.menu_open {
            "Input: paused — close the menu to drive".to_string()
        } else if self.cam_mode == CamMode::Free {
            "Input: free camera — the car coasts".to_string()
        } else if spectated.is_some() {
            if self.client.controls_connected() {
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
            .gilrs
            .as_ref()
            .map(|g| {
                g.gamepads()
                    .filter(|(_, pad)| pad.is_connected())
                    .map(|(id, pad)| (id, pad.name().to_string()))
                    .collect()
            })
            .unwrap_or_default();
        if self.menu_open && !rebinding {
            let mut ui_action_map = self.settings.input_bindings();
            let mut bindings_dirty = false;
            let binding_group = &mut self.binding_group;
            let binding_filter = &mut self.binding_filter;
            let rebind_action = &mut self.rebind_action;
            let rebind_capture = &mut self.rebind_capture;

            let capture_start_back = &mut self.capture_start_back;
            let action_evaluator = &mut self.action_evaluator;
            let binding_warning = &mut self.binding_warning;
            let active_pad = &mut self.active_pad;
            let raw_pad = &mut self.raw_pad;
            let settings = &mut self.settings;
            // Pad-nav focus snapshot for this draw (ids rebuilt below).
            let focus = self.menu_focus;
            let active = self.menu_nav_active;
            let moved = self.nav_moved;

            self.nav_ids.clear();
            egui::Window::new("settings")
                .scroll([false, true])
                .show(ctx, |ui| {
                let (ids, focus, active, moved) = (&mut self.nav_ids, focus, active, moved);
                let header = egui::CollapsingHeader::new("Camera")
                    .id_salt("viewer.header.camera")
                    .open(Some(self.headers_open[0]))
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            let r = ui.button("Wide view");
                            if r.clicked() {
                                self.cam_mode = CamMode::Man;
                                self.auto_cam = None;
                                if let Some(man) = self.cam_man.as_mut() {
                                    man.set_wide_view();
                                }
                            }
                            nav_row(ui, &r, ids, "cam.wide".to_string(), focus, active, moved);
                            let r = ui.button("Director");
                            if r.clicked() {
                                self.cam_mode = CamMode::Director;
                                self.auto_cam = None;
                            }
                            nav_row(ui, &r, ids, "cam.director".to_string(), focus, active, moved);
                        });
                        ui.label(format!("View: {cam_line}"));
                        ui.label(format!(
                            "Bindings: Follow 1 {}, Follow 8 {}, Director {}, Free camera {}, Next camera {}, Ball camera {}.",
                            action_map.sources(Action::Follow1)[0].name(),
                            action_map.sources(Action::Follow8)[0].name(),
                            action_map.sources(Action::Director)[0].name(),
                            action_map.sources(Action::FreeCam)[0].name(),
                            action_map.sources(Action::NextCam)[0].name(),
                            action_map.sources(Action::BallCam)[0].name(),
                        ));
                        ui.label(format!("You drive car {}.", self.driven_car));
                        ui.label(drive_line.clone());
                        let r = ui.button("Reset play");
                        if r.clicked() {
                            self.reset_requested = true;
                        }
                        nav_row(ui, &r, ids, "cam.reset_play".to_string(), focus, active, moved);
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
                if header.header_response.clicked() { self.headers_open[0] = !self.headers_open[0]; }
                nav_row(ui, &header.header_response, ids, "header.camera".into(), focus, active, moved);
                let header = egui::CollapsingHeader::new("Input")
                    .id_salt("viewer.header.input")
                    .open(Some(self.headers_open[1]))
                    .show(ui, |ui| {
                        let r = ui.checkbox(&mut settings.spectate_only, "Spectate only (no inputs)");
                        if r.changed() {
                            settings.mark_dirty();
                        }
                        nav_row(ui, &r, ids, "input.spectate".to_string(), focus, active, moved);
                        let r = ui.add(
                            egui::Slider::new(
                                &mut settings.bindings.pad.stick_deadzone,
                                0.0..=0.5,
                            )
                            .text("Controller stick deadzone"),
                        );
                        if r.changed() {
                            settings.bindings.pad.stick_deadzone =
                                clamp_deadzone(settings.bindings.pad.stick_deadzone).min(0.5);
                            settings.mark_dirty();
                        }
                        nav_row(ui, &r, ids, "input.deadzone".to_string(), focus, active, moved);
                    });
                if header.header_response.clicked() { self.headers_open[1] = !self.headers_open[1]; }
                nav_row(ui, &header.header_response, ids, "header.input".into(), focus, active, moved);
                let header = egui::CollapsingHeader::new("Mutators (drive demo)")
                    .id_salt("viewer.header.mutators")
                    .open(Some(self.headers_open[2]))
                    .show(ui, |ui| {
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
                if header.header_response.clicked() { self.headers_open[2] = !self.headers_open[2]; }
                nav_row(ui, &header.header_response, ids, "header.mutators".into(), focus, active, moved);
                let header = egui::CollapsingHeader::new("Overlays")
                    .id_salt("viewer.header.overlays")
                    .open(Some(self.headers_open[3]))
                    .show(ui, |ui| {
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
                if header.header_response.clicked() { self.headers_open[3] = !self.headers_open[3]; }
                nav_row(ui, &header.header_response, ids, "header.overlays".into(), focus, active, moved);
                let header = egui::CollapsingHeader::new("Display")
                    .id_salt("viewer.header.display")
                    .open(Some(self.headers_open[4]))
                    .show(ui, |ui| {
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
                if header.header_response.clicked() { self.headers_open[4] = !self.headers_open[4]; }
                nav_row(ui, &header.header_response, ids, "header.display".into(), focus, active, moved);
                let header = egui::CollapsingHeader::new("Bindings")
                    .id_salt("viewer.header.bindings")
                    .open(Some(self.headers_open[5]))
                    .show(ui, |ui| {
                        ui.label("Bind any input to any action. Shared bindings are allowed. Escape is a normal bind; use Cancel to stop capture.");
                        let selected_pad_name = (*active_pad)
                            .and_then(|id| connected_pads.iter().find(|(pad_id, _)| *pad_id == id))
                            .map(|(_, name)| name.as_str())
                            .unwrap_or("No controller");
                        ui.label(format!("Active controller: {selected_pad_name}"));
                        let pad_cycle = ui.button("Change controller");
                        if pad_cycle.clicked() && !connected_pads.is_empty() {
                            let index = connected_pads
                                .iter()
                                .position(|(id, _)| Some(*id) == *active_pad)
                                .unwrap_or(0);
                            *active_pad = Some(connected_pads[(index + 1) % connected_pads.len()].0);
                            self.pad_selection_manual = true;
                            raw_pad.clear();
                            self.suppress_pad_on_next_frame = true;
                        }
                        nav_row(ui, &pad_cycle, ids, "pad.select".into(), focus, active, moved);
                        let groups = ["Driving", "Simulation", "Camera", "Free camera", "Menu"];
                        let group_picker = egui::ComboBox::from_id_salt("binding-group")
                            .selected_text(binding_group.as_deref().unwrap_or("All actions"))
                            .show_ui(ui, |ui| {
                                ui.selectable_value(binding_group, None, "All actions");
                                for group in groups { ui.selectable_value(binding_group, Some(group.to_string()), group); }
                            });
                        nav_row(ui, &group_picker.response, ids, "bind.group".into(), focus, active, moved);
                        let filter_response = ui.add(
                            egui::TextEdit::singleline(binding_filter)
                                .id(egui::Id::unique("viewer.binding-filter")),
                        );
                        nav_row(ui, &filter_response, ids, "bind.filter".into(), focus, active, moved);
                        let clear_filter = ui.small_button("Clear filter");
                        if clear_filter.clicked() { binding_filter.clear(); }
                        nav_row(ui, &clear_filter, ids, "bind.filter.clear".into(), focus, active, moved);
                        let action_map = &mut ui_action_map;
                        for (action_index, action) in ACTIONS.into_iter().enumerate() {
                            if binding_group.as_ref().is_some_and(|g| g.as_str() != action.group()) || (!binding_filter.is_empty() && !action.label().to_lowercase().contains(&binding_filter.to_lowercase())) { continue; }
                            ui.horizontal(|ui| {
                                ui.label(action.label());
                                for slot in 0..2 {
                                    let source = action_map.sources(action)[slot];
                                    let text = if *rebind_action == Some((action, slot)) { "Press input…".to_string() } else { source.name() };
                                    let bind = ui.button(text);
                                    if bind.clicked() {
                                        *rebind_action = Some((action, slot));
                                        *rebind_capture = Some(Capture::begin(&input_snapshot));

                                        *capture_start_back = action_map.sources(Action::MenuBack).iter().copied().find(|source| *source != InputSource::Unbound && input_snapshot.value(*source).abs() >= 0.5);
                                        self.capture_pending_back = None;
                                        self.capture_cancel_hold = 0.0;
                                        action_evaluator.suppress_until_release(&input_snapshot);
                                    }
                                    nav_row(ui, &bind, ids, format!("bind.action.{action_index}.{slot}"), focus, active, moved);
                                    let clear = ui.small_button("Clear");
                                    if clear.clicked() { action_map.set(action, slot, InputSource::Unbound); bindings_dirty = true; }
                                    nav_row(ui, &clear, ids, format!("bind.clear.{action_index}.{slot}"), focus, active, moved);
                                }
                                let reset = ui.small_button("Reset");
                                if reset.clicked() { let defaults = ActionMap::default(); for slot in 0..2 { action_map.set(action, slot, defaults.sources(action)[slot]); } bindings_dirty = true; }
                                nav_row(ui, &reset, ids, format!("bind.reset.{action_index}"), focus, active, moved);
                            });

                        }

                        if let Some(warning) = &*binding_warning { ui.colored_label(egui::Color32::YELLOW, warning); }
                        let conflicts = action_map.conflicts();
                        if !conflicts.is_empty() { ui.colored_label(egui::Color32::YELLOW, format!("{} shared binding conflict(s); actions may trigger together", conflicts.len())); }
                    });
                if header.header_response.clicked() { self.headers_open[5] = !self.headers_open[5]; }
                nav_row(ui, &header.header_response, ids, "header.bindings".into(), focus, active, moved);
                ui.label("Use the Bindings section to configure keyboard, mouse, and controller input.");
                let r = ui.button("Reset everything");
                if r.clicked() {
                    settings.reset_all();
                }
                nav_row(ui, &r, ids, "menu.reset_all".to_string(), focus, active, moved);
                self.nav_moved = false;
            });
            if bindings_dirty {
                settings.action_bindings = Some(ui_action_map);
                settings.mark_dirty();
            }
        }

        if rebinding {
            egui::Modal::new(egui::Id::unique("rebind-input-modal")).show(ctx, |ui| {
                ui.set_min_width(300.0);
                ui.heading("Rebind input");
                ui.label("Press any key, mouse input, or controller input.");
                if let Some((action, slot)) = self.rebind_action {
                    let bound = self.settings.input_bindings().sources(action)[slot];
                    ui.label(format!(
                        "{} · slot {} · currently {}",
                        action.label(),
                        slot + 1,
                        bound.name()
                    ));
                }
                ui.label("Hold the current Back binding for 0.75 seconds to cancel.");
                let cancel = ui.button("Cancel capture");
                self.capture_cancel_rect = Some(cancel.rect);
                if cancel.clicked() {
                    self.rebind_action = None;
                    self.rebind_capture = None;
                    self.action_evaluator
                        .suppress_until_release(&input_snapshot);
                    self.capture_cancel_rect = None;
                }
            });
        } else {
            self.capture_cancel_rect = None;
        }

        // Pad menu navigation (needs the fresh id list from the draw).
        // Up/down move with press + hold-repeat; left/right adjust the
        // focused slider or flip the focused toggle; A activates; B closes.
        if !self.nav_ids.is_empty() {
            self.menu_focus %= self.nav_ids.len();
        }
        // Pad nav stays dead while rebinding (a select press binds
        // instead of activating the focused row).
        if focused && self.menu_open && !action_suppressed && !self.nav_ids.is_empty() {
            let n = self.nav_ids.len() as i32;
            let mut dy = 0i32;
            let mut dx = 0i32;
            if nav_edge[0] {
                dy -= 1;
            }
            if nav_edge[1] {
                dy += 1;
            }
            if nav_edge[2] {
                dx -= 1;
            }
            if nav_edge[3] {
                dx += 1;
            }
            if dy != 0 || dx != 0 {
                self.nav_repeat_cd = 0.35;
            } else if nav_held.iter().any(|&h| h) {
                self.nav_repeat_cd -= dt;
                if self.nav_repeat_cd <= 0.0 {
                    self.nav_repeat_cd = 0.08;
                    if nav_held[0] {
                        dy -= 1;
                    }
                    if nav_held[1] {
                        dy += 1;
                    }
                    if nav_held[2] {
                        dx -= 1;
                    }
                    if nav_held[3] {
                        dx += 1;
                    }
                }
            } else {
                self.nav_repeat_cd = 0.0;
            }
            if dy != 0 {
                self.menu_focus = (self.menu_focus as i32 + dy).rem_euclid(n) as usize;
                self.nav_moved = true;
                self.menu_nav_active = true;
            }
            if nav_b_edge {
                self.menu_open = false;
                ctx.memory_mut(|memory| {
                    memory.surrender_focus(egui::Id::unique("viewer.binding-filter"));
                });
            } else if dx != 0 || nav_a_edge {
                self.menu_nav_active = true;
                menu_pad_action(
                    self.nav_ids
                        .get(self.menu_focus)
                        .cloned()
                        .unwrap_or_default(),
                    dx,
                    nav_a_edge,
                    &mut MenuPadCtx {
                        settings: &mut self.settings,
                        cam_mode: &mut self.cam_mode,
                        cam_man: &mut self.cam_man,
                        auto_cam: &mut self.auto_cam,

                        reset_requested: &mut self.reset_requested,

                        headers_open: &mut self.headers_open,
                        rebind_action: &mut self.rebind_action,
                        rebind_capture: &mut self.rebind_capture,
                        input_snapshot: &input_snapshot,
                        binding_group: &mut self.binding_group,
                        binding_filter: &mut self.binding_filter,
                        capture_start_back: &mut self.capture_start_back,
                        capture_cancel_hold: &mut self.capture_cancel_hold,
                        capture_pending_back: &mut self.capture_pending_back,
                        action_evaluator: &mut self.action_evaluator,
                        active_pad: &mut self.active_pad,
                        pad_selection_manual: &mut self.pad_selection_manual,
                        suppress_pad_on_next_frame: &mut self.suppress_pad_on_next_frame,
                        gilrs: self.gilrs.as_ref(),
                        raw_pad: &mut self.raw_pad,
                    },
                );
            }
        }

        egui::Window::new("rocketsim viewer").show(ctx, |ui| {
            ui.label(sim_line);
            ui.label(format!("Camera: {cam_line}"));
            ui.label(drive_line);
            ui.label(format!("{fps:.0} fps"));
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
                    self.client.controls_connected()
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
                let cb = egui::PaintCallback {
                    rect,
                    callback: Arc::new(egui_glow::CallbackFn::new(move |info, painter| {
                        let gl = painter.gl();
                        let ppp = info.pixels_per_point;
                        let w_px = (info.viewport.width() * ppp).round() as i32;
                        let h_px = (info.viewport.height() * ppp).round() as i32;
                        if w_px <= 1 || h_px <= 1 {
                            return;
                        }
                        let x_px = (info.viewport.min.x * ppp).round() as i32;
                        let y_px = (info.screen_size_px[1] as f32 - info.viewport.max.y * ppp)
                            .round() as i32;
                        let aspect = w_px as f32 / h_px as f32;
                        let (view_mat, proj_mat) = view::view_proj(
                            glam::Vec3::from(cam_pos),
                            glam::Vec3::from(cam_pos + cam_dir),
                            cam_fov,
                            aspect,
                        );
                        let t0 = Instant::now();
                        let mut guard = scene.lock().expect("viewer scene poisoned");
                        // Rebuild on game-mode switches (mesh + bounds follow
                        // the stream; a switch hitches one frame).
                        if guard.is_none() {
                            *guard = Some(SceneGl::new(gl, &mesh));
                        } else if rebuild_arena && let Some(scene) = guard.as_mut() {
                            scene.rebuild_arena(gl, &mesh);
                        }
                        let scene: &mut SceneGl = guard.as_mut().expect("viewer scene built");
                        scene.arena.draw(
                            gl,
                            (x_px, y_px, w_px, h_px),
                            &glam::Mat4::IDENTITY,
                            &view_mat,
                            &proj_mat,
                            enclosed,
                        );
                        scene.prog_main.set_view_proj(gl, &view_mat, &proj_mat);
                        if let Some(frame) = &client_frame {
                            // Ball: textured, pos + rot.
                            let ball = &frame.state.ball.phys;
                            let ball_model = if frame.state.game_mode() == GameMode::Snowday {
                                PUCK_MODEL
                            } else {
                                BALL_MODEL
                            };
                            let ball_model = scene
                                .models
                                .get(ball_model)
                                .or_else(|| scene.models.get(BALL_MODEL))
                                .expect("scene has a ball or puck model");
                            scene.prog_main.draw(
                                gl,
                                ball_model,
                                &scene.textures[BALL_TEXTURE],
                                &(model_matrix(Vec3::from(ball.pos), Mat3::from(ball.rot_mat))
                                    * glam::Mat4::from_scale(
                                        if frame.state.game_mode() == GameMode::Snowday {
                                            Vec3::new(ball_radius_scale, ball_radius_scale, 1.0)
                                        } else {
                                            Vec3::splat(ball_radius_scale)
                                        },
                                    )),
                            );
                            for debug_model in &frame.debug.models {
                                let rotation = Mat3::from_cols_array_2d(&debug_model.rotation);
                                let transform =
                                    model_matrix(Vec3::from_array(debug_model.position), rotation)
                                        * glam::Mat4::from_scale(Vec3::from_array(
                                            debug_model.scale,
                                        ));
                                if let (Some(model), Some(texture)) = (
                                    scene.models.get(&debug_model.model),
                                    scene.textures.get(&debug_model.texture),
                                ) {
                                    scene.prog_main.draw(gl, model, texture, &transform);
                                }
                            }
                            // Cars: one instanced draw per (body, team).
                            let mut placements = Vec::new();
                            for (info, state) in &frame.state.cars {
                                if state.is_demoed {
                                    continue;
                                }
                                placements.push((
                                    car_model_name(&info.config),
                                    car_texture_name(info.team),
                                    model_matrix(
                                        Vec3::from(state.phys.pos),
                                        Mat3::from(state.phys.rot_mat),
                                    ),
                                ));
                            }

                            scene.prog_cars.set_view_proj(gl, &view_mat, &proj_mat);
                            for g in group_car_draws(&placements) {
                                let m = scene
                                    .car_models
                                    .get_mut(g.model)
                                    .expect("bundled car model");
                                m.upload(gl, &g.matrices);
                                scene
                                    .prog_cars
                                    .draw_group(gl, m, &scene.textures[g.texture]);
                            }
                            for debug_model in &frame.debug.models {
                                if let (Some(model), Some(texture)) = (
                                    scene.car_models.get_mut(&debug_model.model),
                                    scene.textures.get(&debug_model.texture),
                                ) {
                                    let rotation = Mat3::from_cols_array_2d(&debug_model.rotation);
                                    let transform = model_matrix(
                                        Vec3::from_array(debug_model.position),
                                        rotation,
                                    ) * glam::Mat4::from_scale(Vec3::from_array(
                                        debug_model.scale,
                                    ));
                                    model.upload(gl, &[transform]);
                                    scene.prog_cars.draw_group(gl, model, texture);
                                }
                            }
                            scene.prog_cars.unbind(gl);
                        }
                        scene.prog_main.unbind(gl);
                        // Pads: frustum-filtered instances (skipped if hidden).
                        let planes = frustum_planes(proj_mat * view_mat);
                        let empty: [Vec<glam::Mat4>; 4] = Default::default();
                        let (instances, culled) = match &client_frame {
                            Some(frame) if show_pads => {
                                build_instances(&frame.state.boost_pads, &planes, &scene.pad_radii)
                            }
                            _ => (empty, 0),
                        };
                        scene.pads.upload(gl, &instances);
                        scene.pads.draw(gl, &view_mat, &proj_mat);
                        scene.dropshot.upload_shared(gl, &dropshot_mesh);
                        scene.dropshot.draw(gl, &view_mat, &proj_mat);
                        scene
                            .arena
                            .draw_net(gl, &glam::Mat4::IDENTITY, &view_mat, &proj_mat);
                        if let Ok(mut info) = pads_info.lock() {
                            *info = (instances.iter().map(|v| v.len()).sum(), culled);
                        }
                        // Lines last (transparent): CPU-expanded quads.
                        let segs = std::mem::take(
                            &mut *pending_segs.lock().expect("viewer segs poisoned"),
                        );
                        // Eye for expansion: camera pos; proj11 + height set
                        // the min width.
                        let eye = cam_pos;
                        scene.lines.upload(
                            gl,
                            &segs,
                            eye,
                            &view_mat,
                            proj_mat.y_axis.y,
                            h_px as f32,
                        );
                        scene.lines.draw(gl, &view_mat, &proj_mat);
                        if let Ok(mut info) = lines_info.lock() {
                            *info = (scene.lines.drawn(), scene.lines.truncated);
                        }
                        // GPU-stall timing (profiling only: serializes the
                        // pipeline, never enable outside measurements).
                        if measure_gpu {
                            let gt0 = Instant::now();
                            unsafe {
                                gl.finish();
                            }
                            if let Ok(mut g) = gpu_ms.lock() {
                                *g = gt0.elapsed().as_secs_f64() * 1000.0;
                            }
                        }
                        drop(guard);
                        if let Ok(mut g) = last_draw_ms.lock() {
                            *g = t0.elapsed().as_secs_f64() * 1000.0;
                        }
                    })),
                };
                ui.painter().add(cb);
                if let Some(debug) = &debug_draw {
                    let painter = ui.painter();
                    for debug_rect in &debug.rects {
                        let point = |p: [f32; 2]| {
                            egui::pos2(
                                rect.left() + p[0].clamp(0.0, 1.0) * rect.width(),
                                rect.top() + p[1].clamp(0.0, 1.0) * rect.height(),
                            )
                        };
                        let color = egui::Color32::from_rgba_unmultiplied(
                            (debug_rect.color[0].clamp(0.0, 1.0) * 255.0) as u8,
                            (debug_rect.color[1].clamp(0.0, 1.0) * 255.0) as u8,
                            (debug_rect.color[2].clamp(0.0, 1.0) * 255.0) as u8,
                            (debug_rect.color[3].clamp(0.0, 1.0) * 255.0) as u8,
                        );
                        painter.rect_filled(
                            egui::Rect::from_min_max(point(debug_rect.min), point(debug_rect.max)),
                            0.0,
                            color,
                        );
                    }
                    for debug_text in &debug.text {
                        let color = egui::Color32::from_rgba_unmultiplied(
                            (debug_text.color[0].clamp(0.0, 1.0) * 255.0) as u8,
                            (debug_text.color[1].clamp(0.0, 1.0) * 255.0) as u8,
                            (debug_text.color[2].clamp(0.0, 1.0) * 255.0) as u8,
                            (debug_text.color[3].clamp(0.0, 1.0) * 255.0) as u8,
                        );
                        let pos = egui::pos2(
                            rect.left() + debug_text.position[0].clamp(0.0, 1.0) * rect.width(),
                            rect.top() + debug_text.position[1].clamp(0.0, 1.0) * rect.height(),
                        );
                        painter.text(
                            pos,
                            egui::Align2::LEFT_TOP,
                            &debug_text.text,
                            egui::FontId::proportional(14.0),
                            color,
                        );
                    }
                }
                // RL-style boost gauge over the 3D view: dim dial, value
                // arc clockwise from the top, number in the middle.
                if let Some(m) = &meter {
                    draw_boost_gauge(ui, rect, m.fraction, &m.label, m.low);
                }
            }
        });
        self.cpu_ms = cpu_t0.elapsed().as_secs_f64() * 1000.0;
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

/// Command line: `rocketsim-viewer [PORT] [CAM] [-g MODE] [--gpushot]`.
/// PORT/CAM stay positional (pre-clap usage keeps working).
#[derive(clap::Parser)]
#[command(name = "rocketsim-viewer", version)]
struct Cli {
    /// TCP port the viewer listens on (the sim dials this).
    #[arg(default_value_t = DEFAULT_PORT)]
    port: u16,
    /// Address the viewer binds (default: 127.0.0.1).
    #[arg(long, default_value = "127.0.0.1", value_parser = clap::value_parser!(std::net::IpAddr))]
    bind: std::net::IpAddr,
    /// Start camera: birds|followN|director|autoN|free.
    #[arg(default_value = "birds", value_parser = parse_camera)]
    cam: StartMode,
    /// Initial placeholder game mode. Stream metadata replaces it on connect.
    #[arg(short = 'g', long = "game-mode", default_value = "soccar", value_parser = parse_mode)]
    game_mode: GameMode,

    /// GPU-stall timing for profiling.
    #[arg(long)]
    gpushot: bool,
}

fn dropshot_tile_states(state: &rocketsim::ArenaState) -> Option<&rocketsim::TileStates> {
    if state.game_mode() == GameMode::Dropshot {
        state.tile_states.as_ref()
    } else {
        None
    }
}

fn parse_mode(s: &str) -> Result<GameMode, String> {
    parse_game_mode(s).ok_or_else(|| {
        format!("unknown game mode '{s}' (soccar|hoops|heatseeker|snowday|dropshot|thevoid|void)")
    })
}

fn parse_camera(s: &str) -> Result<StartMode, String> {
    StartMode::try_parse(s)
        .ok_or_else(|| format!("invalid camera '{s}' (birds|followN|director|autoN|free)"))
}

fn metadata_changed(
    previous: Option<&Arc<rocketsim_vis::proto::SceneMetadata>>,
    current: &Arc<rocketsim_vis::proto::SceneMetadata>,
) -> bool {
    previous.is_none_or(|previous| !Arc::ptr_eq(previous, current))
}

fn main() {
    let mut args: Vec<String> = std::env::args().collect();
    let measure_gpu_compat = args.iter().any(|a| a == "gpushot");
    args.retain(|a| a != "gpushot");
    let cli = Cli::parse_from(args);
    let port = cli.port;
    // The `CAM` option sets the start camera. Runtime camera and driving
    // inputs use the remappable action map. The options menu starts open.
    let init_mode = cli.cam;
    let menu_open = true;
    // `gpushot` also accepted bare (pre-clap usage) anywhere in argv.
    let measure_gpu = cli.gpushot || measure_gpu_compat;
    let settings = ViewerSettings::load();
    let addr = std::net::SocketAddr::new(cli.bind, port).to_string();
    eprintln!("rocketsim-viewer: listening on {addr} (sim should dial this addr)");

    let mesh = Arc::new(ArenaMesh::placeholder(cli.game_mode));
    let client = FrameClient::try_connect(&addr).unwrap_or_else(|e| {
        eprintln!("rocketsim-viewer: cannot listen on {addr}: {e}");
        std::process::exit(1);
    });

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1280.0, 720.0]),
        renderer: eframe::Renderer::Glow,
        // eframe defaults to zero depth bits; the 3D view needs real depth.
        depth_buffer: 24,
        multisampling: settings.msaa,
        glow_options: egui_glow::GlowConfiguration {
            vsync: settings.vsync,
            ..Default::default()
        },
        ..Default::default()
    };
    if let Err(e) = eframe::run_native(
        "rocketsim viewer",
        options,
        Box::new(move |_cc| {
            let app = ViewerApp::new(
                mesh.clone(),
                client,
                init_mode,
                settings,
                menu_open,
                measure_gpu,
            );
            Ok(Box::new(app) as _)
        }),
    ) {
        eprintln!("rocketsim-viewer: eframe::run_native failed: {e:?}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
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

    #[test]
    fn msaa_changes_use_supported_sample_counts() {
        assert_eq!(step_msaa(0, 1), 2);
        assert_eq!(step_msaa(2, 1), 4);
        assert_eq!(step_msaa(4, 1), 8);
        assert_eq!(step_msaa(8, 1), 8);
        assert_eq!(step_msaa(8, -1), 4);
        assert_eq!(step_msaa(0, -1), 0);
    }

    #[test]
    fn cli_help_and_version_are_parser_only() {
        let help = match Cli::try_parse_from(["rocketsim-viewer", "--help"]) {
            Err(error) => error,
            Ok(_) => panic!("--help must exit through clap"),
        };
        assert_eq!(help.kind(), clap::error::ErrorKind::DisplayHelp);
        let version = match Cli::try_parse_from(["rocketsim-viewer", "--version"]) {
            Err(error) => error,
            Ok(_) => panic!("--version must exit through clap"),
        };
        assert_eq!(version.kind(), clap::error::ErrorKind::DisplayVersion);
    }

    #[test]
    fn cli_initial_mode_parses_without_initializing_assets() {
        let cli = Cli::try_parse_from(["rocketsim-viewer", "-g", "hoops"]).unwrap();
        assert_eq!(cli.game_mode, GameMode::Hoops);
    }

    #[test]
    fn cli_bind_accepts_ipv6_socket_address() {
        let cli = Cli::try_parse_from(["rocketsim-viewer", "--bind", "::1", "1234"]).unwrap();
        assert_eq!(
            std::net::SocketAddr::new(cli.bind, cli.port).to_string(),
            "[::1]:1234"
        );
    }

    #[test]
    fn metadata_changes_are_arc_identity_based() {
        let metadata = Arc::new(rocketsim_vis::proto::SceneMetadata {
            game_mode: GameMode::Soccar,
            ball_radius: 91.25,
            meshes: Vec::new(),
        });
        let same = metadata.clone();
        assert!(!metadata_changed(Some(&metadata), &same));
        assert!(metadata_changed(None, &same));
    }

    #[test]
    fn camera_cli_rejects_huge_or_invalid_indices() {
        assert!(parse_camera("follow7").is_ok());
        assert!(parse_camera("auto0").is_ok());
        assert!(parse_camera("follow999999999999999999999999999999").is_err());
        assert!(parse_camera("follow8").is_ok());
        assert!(parse_camera("follow1000000000").is_ok());
        assert!(parse_camera("unknown").is_err());
    }

    #[test]
    fn mapped_stick_and_facepad_ranks_above_a_system_control_device() {
        let system_control = controller_priority(true, 0, 1, gilrs::MappingSource::Driver);
        let xbox = controller_priority(true, 4, 4, gilrs::MappingSource::SdlMappings);
        assert!(xbox > system_control);
        assert_eq!(
            controller_priority(false, 4, 4, gilrs::MappingSource::SdlMappings),
            i32::MIN
        );
    }

    #[test]
    fn shared_bindings_are_advisory_and_any_source_is_accepted() {
        let mut map = ActionMap::default();
        let key = InputSource::Key(Key::W);
        map.set(Action::Jump, 0, key);
        map.set(Action::FreeReset, 1, key);
        assert!(
            map.conflicts()
                .iter()
                .any(|c| c.0 == Action::Jump && c.2 == Action::FreeReset)
        );
        let settings = ViewerSettings::default();
        let mut map = settings.input_bindings();
        map.set(Action::Jump, 0, InputSource::Mouse(MouseButton::Extra2));
        assert_eq!(
            map.sources(Action::Jump)[0],
            InputSource::Mouse(MouseButton::Extra2)
        );
    }

    #[test]
    fn pad_press_release_pulses_survive_one_frame_for_mapped_and_raw_inputs() {
        let code = 196_608;
        let mut pulses = PadFramePulses::default();
        let mut raw = HashMap::new();
        pulses.record_press(gilrs::Button::South, code);
        update_raw_button_value(&mut raw, code, 0.0);

        assert_eq!(pad_frame_value(0.0, pulses.buttons[&PadButton::South]), 1.0);
        assert_eq!(pad_frame_value(raw[&code], pulses.raw[&code]), 1.0);

        let next_frame = PadFramePulses::default();
        assert_eq!(
            pad_frame_value(
                raw[&code],
                next_frame.raw.get(&code).copied().unwrap_or(0.0)
            ),
            0.0
        );
        assert!(next_frame.buttons.is_empty());

        let mut unknown_pulses = PadFramePulses::default();
        unknown_pulses.record_press(gilrs::Button::Unknown, code);
        assert!(!unknown_pulses.buttons.contains_key(&PadButton::Unknown));
        assert_eq!(unknown_pulses.raw[&code], 1.0);
    }

    #[test]
    fn raw_analog_button_changed_event_keeps_its_analog_value() {
        let mut raw = HashMap::new();
        update_raw_button_value(&mut raw, 196_608, 0.3);
        assert_eq!(raw[&196_608], 0.3);
        assert_eq!(pad_frame_value(raw[&196_608], 0.0), 0.3);
    }

    #[test]
    fn controller_menu_navigation_changes_clamped_deadzone_and_core_actions() {
        let mut settings = ViewerSettings::default();
        settings.bindings.pad.stick_deadzone = 0.49;
        let mut cam_mode = CamMode::Man;
        let mut cam_man = None;
        let mut auto_cam = None;
        let mut reset_requested = false;
        let mut headers_open = [true; 6];
        let mut rebind_action = None;
        let mut rebind_capture = None;
        let input_snapshot = InputSnapshot::new();
        let mut binding_group = None;
        let mut binding_filter = String::new();
        let mut capture_start_back = None;
        let mut capture_cancel_hold = 0.0;
        let mut capture_pending_back = None;
        let mut action_evaluator = Evaluator::default();
        let mut active_pad = None;
        let mut pad_selection_manual = false;
        let mut suppress_pad_on_next_frame = false;
        let mut raw_pad = HashMap::new();
        let mut cx = MenuPadCtx {
            settings: &mut settings,
            cam_mode: &mut cam_mode,
            cam_man: &mut cam_man,
            auto_cam: &mut auto_cam,
            reset_requested: &mut reset_requested,
            headers_open: &mut headers_open,
            rebind_action: &mut rebind_action,
            rebind_capture: &mut rebind_capture,
            input_snapshot: &input_snapshot,
            binding_group: &mut binding_group,
            binding_filter: &mut binding_filter,
            capture_start_back: &mut capture_start_back,
            capture_cancel_hold: &mut capture_cancel_hold,
            capture_pending_back: &mut capture_pending_back,
            action_evaluator: &mut action_evaluator,
            active_pad: &mut active_pad,
            pad_selection_manual: &mut pad_selection_manual,
            suppress_pad_on_next_frame: &mut suppress_pad_on_next_frame,
            gilrs: None,
            raw_pad: &mut raw_pad,
        };

        menu_pad_action("input.deadzone".into(), 1, false, &mut cx);
        assert_eq!(cx.settings.bindings.pad.stick_deadzone, 0.5);
        menu_pad_action("input.deadzone".into(), 1, false, &mut cx);
        assert_eq!(cx.settings.bindings.pad.stick_deadzone, 0.5);
        menu_pad_action("input.deadzone".into(), -1, false, &mut cx);
        assert!((cx.settings.bindings.pad.stick_deadzone - 0.49).abs() < 1e-6);

        menu_pad_action("header.input".into(), 0, true, &mut cx);
        assert!(!cx.headers_open[1]);
        menu_pad_action("bind.group".into(), 1, false, &mut cx);
        assert_eq!(cx.binding_group.as_deref(), Some("Driving"));
        cx.settings.show_trail = false;
        menu_pad_action("menu.reset_all".into(), 0, true, &mut cx);
        assert!(cx.settings.show_trail);
        assert_eq!(cx.settings.bindings.pad.stick_deadzone, 0.1);
    }

    #[test]
    fn action_controls_preserve_analog_values_and_gate_all_side_effects() {
        let mut values = HashMap::new();
        values.insert(
            Action::ThrottleForward,
            rocketsim_viewer::input::ActionValue {
                value: 0.4,
                pressed: false,
                just_pressed: false,
                just_released: false,
            },
        );
        values.insert(
            Action::SteerLeft,
            rocketsim_viewer::input::ActionValue {
                value: 0.7,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        );
        values.insert(
            Action::Launch,
            rocketsim_viewer::input::ActionValue {
                value: 1.0,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        );
        let map = ActionMap::default();
        let evaluator = Evaluator::default();
        let snapshot = InputSnapshot::new();
        let (controls, buttons) = action_controls(
            &values, &map, &evaluator, &snapshot, 0.1, true, false, false,
        );
        assert!((controls.throttle - 0.4).abs() < 1e-6);
        assert!((controls.steer - 0.7).abs() < 1e-6);
        assert_eq!(buttons, 0b100);
        let (neutral, no_buttons) = action_controls(
            &values, &map, &evaluator, &snapshot, 0.1, false, false, false,
        );
        assert_eq!(neutral, CarControls::default());
        assert_eq!(no_buttons, 0);
    }

    #[test]
    fn follow_look_signs_match_resulting_camera_direction_and_accept_remapped_sources() {
        use rocketsim_viewer::input::ActionValue;

        let mut values = HashMap::new();
        values.insert(
            Action::SwivelRight,
            ActionValue {
                value: 1.0,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        );
        let (yaw, pitch) = follow_look_targets(&values);
        let right = apply_follow_look(Vec3A::X, yaw, pitch);
        assert!(right.y > 0.0 && right.x < 1.0);

        values.clear();
        values.insert(
            Action::SwivelLeft,
            ActionValue {
                value: 1.0,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        );
        let (yaw, pitch) = follow_look_targets(&values);
        let left = apply_follow_look(Vec3A::X, yaw, pitch);
        assert!(left.y < 0.0 && left.x < 1.0);

        values.clear();
        values.insert(
            Action::SwivelUp,
            ActionValue {
                value: 1.0,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        );
        let (yaw, pitch) = follow_look_targets(&values);
        assert!(apply_follow_look(Vec3A::X, yaw, pitch).z > 0.0);
        values.clear();
        values.insert(
            Action::SwivelDown,
            ActionValue {
                value: 1.0,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        );
        let (yaw, pitch) = follow_look_targets(&values);
        assert!(apply_follow_look(Vec3A::X, yaw, pitch).z < 0.0);
    }

    #[test]
    fn freecam_look_signs_and_mouse_motion_ignore_frame_time() {
        let mut map = ActionMap::default();
        for action in [
            Action::LookLeft,
            Action::LookRight,
            Action::LookUp,
            Action::LookDown,
        ] {
            map.set(action, 0, InputSource::Unbound);
            map.set(action, 1, InputSource::Unbound);
        }
        map.set(
            Action::LookLeft,
            0,
            InputSource::MouseAxis {
                axis: Axis2::X,
                sign: 1,
            },
        );
        map.set(
            Action::LookDown,
            0,
            InputSource::MouseAxis {
                axis: Axis2::Y,
                sign: 1,
            },
        );
        let mut snapshot = InputSnapshot::new();
        snapshot.focused = true;
        snapshot.set(
            InputSource::MouseAxis {
                axis: Axis2::X,
                sign: 1,
            },
            10.0,
        );
        snapshot.set(
            InputSource::MouseAxis {
                axis: Axis2::Y,
                sign: 1,
            },
            10.0,
        );
        let mut evaluator = Evaluator::default();
        evaluator.evaluate(&map, &snapshot, 0.1);
        let yaw_30fps = look_delta(
            &map,
            &evaluator,
            &snapshot,
            Action::LookLeft,
            Action::LookRight,
            1.0 / 30.0,
            0.005,
            0.1,
            false,
        );
        let yaw_144fps = look_delta(
            &map,
            &evaluator,
            &snapshot,
            Action::LookLeft,
            Action::LookRight,
            1.0 / 144.0,
            0.005,
            0.1,
            false,
        );
        let pitch = look_delta(
            &map,
            &evaluator,
            &snapshot,
            Action::LookUp,
            Action::LookDown,
            1.0 / 60.0,
            0.005,
            0.1,
            false,
        );
        assert!((yaw_30fps - 0.05).abs() < 1e-6);
        assert_eq!(yaw_30fps, yaw_144fps);
        assert!((pitch + 0.05).abs() < 1e-6);
    }

    #[test]
    fn pointer_moved_input_is_mouse_motion_fallback() {
        let ctx = egui::Context::default();
        ctx.begin_pass(egui::RawInput {
            events: vec![egui::Event::PointerMoved(egui::pos2(100.0, 80.0))],
            ..Default::default()
        });
        let mut output = ctx.end_pass();
        output.textures_delta.clear();
        ctx.begin_pass(egui::RawInput {
            events: vec![egui::Event::PointerMoved(egui::pos2(117.0, 91.0))],
            ..Default::default()
        });
        let delta = ctx.input(|input| mouse_motion_delta(&input.events, input.pointer.delta()));
        output = ctx.end_pass();
        output.textures_delta.clear();
        assert_eq!(delta, egui::vec2(17.0, 11.0));
    }

    #[test]
    fn pointer_delta_wins_when_backend_emits_absolute_mouse_events() {
        let events = [
            egui::Event::MouseMoved(egui::vec2(800.0, 360.0)),
            egui::Event::PointerMoved(egui::pos2(800.0, 360.0)),
        ];
        assert_eq!(
            mouse_motion_delta(&events, egui::vec2(160.0, 0.0)),
            egui::vec2(160.0, 0.0)
        );
        let zero_raw_with_pointer = [
            egui::Event::MouseMoved(egui::Vec2::ZERO),
            egui::Event::PointerMoved(egui::pos2(900.0, 360.0)),
        ];
        assert_eq!(
            mouse_motion_delta(&zero_raw_with_pointer, egui::vec2(100.0, 0.0)),
            egui::vec2(100.0, 0.0)
        );
        let relative_raw_only = [egui::Event::MouseMoved(egui::vec2(4.0, -3.0))];
        assert_eq!(
            mouse_motion_delta(&relative_raw_only, egui::Vec2::ZERO),
            egui::vec2(4.0, -3.0)
        );
        let zero_raw_only = [egui::Event::MouseMoved(egui::Vec2::ZERO)];
        assert_eq!(
            mouse_motion_delta(&zero_raw_only, egui::Vec2::ZERO),
            egui::Vec2::ZERO
        );
    }

    #[test]
    fn pointer_hover_blocks_mouse_look_but_not_controller_look() {
        let mut map = ActionMap::default();
        for action in [Action::LookLeft, Action::LookRight] {
            map.set(action, 0, InputSource::Unbound);
            map.set(action, 1, InputSource::Unbound);
        }
        let mouse = InputSource::MouseAxis {
            axis: Axis2::X,
            sign: 1,
        };
        let stick = InputSource::PadAxis {
            axis: PadAxis::RightStickX,
            sign: 1,
        };
        map.set(Action::LookLeft, 0, mouse);
        map.set(Action::LookRight, 0, stick);
        let mut snapshot = InputSnapshot::new();
        snapshot.focused = true;
        snapshot.set(mouse, 12.0);
        snapshot.set(stick, 1.0);
        let mut evaluator = Evaluator::default();
        evaluator.evaluate(&map, &snapshot, 0.1);
        let delta = look_delta(
            &map,
            &evaluator,
            &snapshot,
            Action::LookLeft,
            Action::LookRight,
            1.0 / 60.0,
            0.005,
            0.1,
            true,
        );
        assert!((delta + 2.5 / 60.0).abs() < 1e-6);
    }

    #[test]
    fn freecam_dolly_wheel_delta_is_not_scaled_by_frame_time() {
        let mut map = ActionMap::default();
        let forward = InputSource::Wheel {
            axis: Axis2::Y,
            sign: 1,
        };
        map.set(Action::FreeDollyForward, 0, forward);
        map.set(Action::FreeDollyForward, 1, InputSource::Unbound);
        map.set(Action::FreeDollyBack, 0, InputSource::Unbound);
        map.set(Action::FreeDollyBack, 1, InputSource::Unbound);
        let mut snapshot = InputSnapshot::new();
        snapshot.focused = true;
        snapshot.set(forward, 2.0);
        let mut evaluator = Evaluator::default();
        evaluator.evaluate(&map, &snapshot, 0.1);
        let slow = freecam_dolly_delta(
            &map,
            &evaluator,
            &snapshot,
            1.0 / 30.0,
            48.0,
            220.0,
            0.1,
            false,
        );
        let fast = freecam_dolly_delta(
            &map,
            &evaluator,
            &snapshot,
            1.0 / 144.0,
            48.0,
            220.0,
            0.1,
            false,
        );
        assert_eq!(slow, 96.0);
        assert_eq!(slow, fast);
    }

    #[test]
    fn air_roll_adds_yaw_to_explicit_roll_without_using_handbrake() {
        use rocketsim_viewer::input::ActionValue;
        let value = |v, pressed| ActionValue {
            value: v,
            pressed,
            just_pressed: false,
            just_released: false,
        };
        let mut values = HashMap::new();
        values.insert(Action::YawLeft, value(0.4, true));
        values.insert(Action::RollLeft, value(0.3, true));
        values.insert(Action::AirRoll, value(1.0, true));
        let (controls, _) = action_controls(
            &values,
            &ActionMap::default(),
            &Evaluator::default(),
            &InputSnapshot::new(),
            0.1,
            true,
            false,
            false,
        );
        assert_eq!(controls.yaw, 0.0);
        assert!((controls.roll - 0.7).abs() < 1e-6);
        assert!(!controls.handbrake);
    }

    #[test]
    fn menu_open_disables_all_driving() {
        // Everything live but the menu open: no driving.
        assert!(!can_drive(true, true, false, true));
        // Menu closed restores it.
        assert!(can_drive(true, true, false, false));
        // Every other kill condition still kills, menu or not.
        for menu_open in [false, true] {
            assert!(!can_drive(false, true, false, menu_open));
            assert!(!can_drive(true, false, false, menu_open));
            assert!(!can_drive(true, true, true, menu_open));
        }
    }
}
