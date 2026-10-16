//! Detached viewer binary: live 3D scene streamed from the sim over TCP.
//!
//! Usage: `rocketsim-viewer [PORT] [CAM] [-g MODE] [--gpushot]` (defaults:
//! 39201, birds, soccar). CAM is `birds`|`followN`|`director`|`autoN`|`free`;
//! MODE is `soccar`|`hoops`|`heatseeker`|`snowday`|`dropshot`|`thevoid` and
//! sets the pre-connect mesh; once frames arrive the viewer follows the
//! sim's mode automatically. `gpushot` is GPU-stall timing for profiling.
//! The options menu starts open; Escape (or pad Start) toggles it.

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
    arena_gl::ArenaGl,
    bindings::{
        Bindings, Key, PadAnalog, PadAxis, PadBindings, PadButton, clamp_deadzone, key_down,
        key_pressed, mouse_down,
    },
    camera::{
        ArenaBoundsCache, AutoCam, CameraConfig, CameraMan, CameraPreset, CarCamSmoothing,
        DirectorState, DirectorTarget, FreeCam,
    },
    frustum::frustum_planes,
    fx::CarFx,
    hud::{boost_meter, spectated_boost},
    lines::{RibbonConfig, RibbonEmitter, Segment, ground_line},
    lines_gl::LinesGl,
    main_gl::{
        BALL_MODEL, BALL_TEXTURE, InstancedModelGl, MainInstProgram, MainProgram, ModelGl,
        car_model_name, car_texture_name, group_car_draws, model_matrix,
    },
    mesh::ArenaMesh,
    obj::ObjMesh,
    pads::{build_instances, load_pad_meshes, variant_radius},
    pads_gl::PadsGl,
    settings::ViewerSettings,
    spectate,
    textures::{PngImage, TextureGl},
    view,
};
use rocketsim_vis::{
    game_mode_name,
    net_client::{ConnStatus, ControlSender, FrameClient},
    parse_game_mode,
    photo::init_meshes,
    proto::{ControlMsg, DEFAULT_CONTROL_PORT, DEFAULT_PORT},
};

/// Owned GL scene: arena + main pipeline + pads + models/textures.
struct SceneGl {
    /// Mode the GL objects were uploaded for (rebuild on switches).
    mode: GameMode,
    arena: ArenaGl,
    prog_main: MainProgram,
    prog_cars: MainInstProgram,
    pads: PadsGl,
    pad_radii: [f32; 4],
    lines: LinesGl,
    models: HashMap<String, ModelGl>,
    car_models: HashMap<String, InstancedModelGl>,
    textures: HashMap<String, TextureGl>,
}

impl SceneGl {
    fn new(gl: &glow::Context, mesh: &ArenaMesh) -> Self {
        let arena = ArenaGl::new(gl, mesh);
        let prog_main = MainProgram::new(gl);
        let pad_meshes = load_pad_meshes();
        let pad_radii = [
            variant_radius(&pad_meshes[0]),
            variant_radius(&pad_meshes[1]),
            variant_radius(&pad_meshes[2]),
            variant_radius(&pad_meshes[3]),
        ];
        let pads = PadsGl::new(gl, &pad_meshes);
        let lines = LinesGl::new(gl);
        let mut models = HashMap::new();
        // One ball: plain draw is cheaper than instancing.
        models.insert(
            BALL_MODEL.to_string(),
            ModelGl::new(gl, &ObjMesh::load(include_str!("./models/ball.obj"))),
        );
        let mut textures = HashMap::new();
        for (name, png) in [
            (
                BALL_TEXTURE,
                include_bytes!("./textures/ball.png").as_slice(),
            ),
            (
                "car_blue",
                include_bytes!("./textures/car_blue.png").as_slice(),
            ),
            (
                "car_orange",
                include_bytes!("./textures/car_orange.png").as_slice(),
            ),
        ] {
            textures.insert(name.to_string(), TextureGl::new(gl, &PngImage::decode(png)));
        }
        let mut car_models = HashMap::new();
        for (name, obj) in [
            ("car_octane", include_str!("./models/car_octane.obj")),
            ("car_dominus", include_str!("./models/car_dominus.obj")),
            ("car_breakout", include_str!("./models/car_breakout.obj")),
            ("car_merc", include_str!("./models/car_merc.obj")),
        ] {
            car_models.insert(
                name.to_string(),
                InstancedModelGl::new(gl, &ObjMesh::load(obj)),
            );
        }
        Self {
            mode: mesh.mode,
            arena,
            prog_main,
            prog_cars: MainInstProgram::new(gl),
            pads,
            pad_radii,
            lines,
            models,
            textures,
            car_models,
        }
    }

    // NOTE: no destroy(): the scene lives as long as the process.
}

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
    /// Last sim tick consumed (missed ticks replay into the ribbons).
    last_tick: Option<u64>,
    /// Line segments for the GL callback (built on the UI thread).
    pending_segs: Arc<Mutex<Vec<Segment>>>,
    /// Camera manager (built on the first frame; birds-eye default).
    cam_man: Option<CameraMan>,
    /// CLI start mode (`birds` default).
    init_mode: StartMode,
    /// Active camera mode (CLI once, then keys/buttons).
    cam_mode: CamMode,
    /// Director rig (selection + smoothing + cut blend).
    director: DirectorRig,
    /// Arena containment hulls, one per game mode (lazy; Soccar warmed).
    bounds: ArenaBoundsCache,
    /// Auto-cam state for the followed car (`0` key arms it).
    auto_cam: Option<AutoCam>,
    /// Free-orbit camera state (`F` toggles; kept across switches).
    freecam: FreeCam,
    /// Last followed car (autocam target + drive default).
    last_followed: usize,
    /// Viewer-to-sim driving channel.
    controls: ControlSender,
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
    /// Options window open (starts open; Escape toggles).
    menu_open: bool,
    /// Key-rebind capture slot (bindings editor); None = not rebinding.
    rebind: Option<usize>,
    /// Pad-rebind capture slot (bindings editor); None = not rebinding.
    pad_rebind: Option<usize>,
    /// Bindings editor tab (session-only, not persisted).
    bind_tab: BindTab,
    /// Local gamepad for pad rebinding (menu only; driving pads live sim-side).
    gilrs: Option<gilrs::Gilrs>,
    /// Raw ev-code state for unmapped pads (buttons 1/0, axes deflection;
    /// cleared on connect). Feeds `Raw`/`RawNeg` analog bindings.
    raw_pad: HashMap<u32, f32>,
    /// Menu focus index for pad navigation (into the per-frame nav list).
    menu_focus: usize,
    /// Pad moved the menu recently (highlights the focused row).
    menu_nav_active: bool,
    /// D-pad/stick held state last frame (up/down/left/right, for repeat).
    nav_prev_dir: [bool; 4],
    /// Menu keys held last frame (select/back).
    nav_prev_a: bool,
    nav_prev_b: bool,
    /// Pad menu-toggle held last frame (edge detect).
    prev_menu_pad: bool,
    /// Pad ball-cam held last frame (edge detect).
    prev_ballcam_pad: bool,
    /// Pad action buttons held last frame (kickoff/dribble/launch).
    prev_pad_act: [bool; 3],
    /// Pad camera shortcuts held last frame (8 follow + director,
    /// autocam, free, cycle).
    prev_pad_cam: [bool; 12],
    /// Nav repeat cooldown (s).
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

/// Viewer camera mode (keys: `C` cycle, `Space` ball-cam, `1-8` follow
/// car, `9` director, `0` autocam, `F` free camera; WASD drives the car).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CamMode {
    /// Birds-eye + follow cars.
    Man,
    /// Broadcast director (overview / car cuts).
    Director,
    /// Free camera (mouse look, WASD fly, Space up, Shift down).
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
    fn parse(s: &str) -> Self {
        if let Some(n) = s.strip_prefix("follow").and_then(|n| n.parse().ok()) {
            Self::Follow(n)
        } else if s == "director" {
            Self::Director
        } else if let Some(n) = s.strip_prefix("auto").and_then(|n| n.parse().ok()) {
            Self::Auto(n)
        } else if s == "free" {
            Self::Free
        } else {
            Self::Birds
        }
    }
}

/// Bindings editor tab.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BindTab {
    Kbm,
    Pad,
}

/// Key-rebind slots for the bindings editor: (label, mutable key slot).
/// Mouse stays fixed (hand-edit settings.toml for it); pad has its own
/// rows below.
type KeySlot = fn(&mut Bindings) -> &mut Key;
const KEY_SLOTS: &[(&str, KeySlot)] = &[
    ("throttle forward", |b| &mut b.car.throttle_fwd),
    ("throttle back", |b| &mut b.car.throttle_back),
    ("steer left", |b| &mut b.car.steer_left),
    ("steer right", |b| &mut b.car.steer_right),
    ("roll left", |b| &mut b.car.roll_left),
    ("roll right", |b| &mut b.car.roll_right),
    ("handbrake", |b| &mut b.car.handbrake),
    ("kickoff reset", |b| &mut b.actions.kickoff_reset),
    ("dribble", |b| &mut b.actions.dribble),
    ("launch", |b| &mut b.actions.launch),
    ("director", |b| &mut b.camera.director),
    ("auto-cam", |b| &mut b.camera.autocam),
    ("free camera", |b| &mut b.camera.free),
    ("next car", |b| &mut b.camera.cycle),
    ("ball cam", |b| &mut b.camera.ball_cam),
    ("menu", |b| &mut b.ui.menu_toggle),
];

/// RL-style boost gauge painted over the viewport corner: dim full dial,
/// value arc clockwise from the top, number in the middle. Orange like
/// RL boost, red when low.
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

/// Pad-rebind slots for the bindings editor: (label, mutable slot).
/// Buttons capture the first pressed button; axes capture the first
/// stick/trigger moved past 60%.
enum PadSlotFn {
    Button(fn(&mut PadBindings) -> &mut PadButton),
    Axis(fn(&mut PadBindings) -> &mut PadAxis),
    Analog(fn(&mut PadBindings) -> &mut PadAnalog),
    OptButton(fn(&mut PadBindings) -> &mut PadButton),
}
const PAD_SLOTS: &[(&str, PadSlotFn)] = &[
    ("pad jump", PadSlotFn::Button(|b| &mut b.jump)),
    ("pad boost", PadSlotFn::Button(|b| &mut b.boost)),
    ("pad handbrake", PadSlotFn::Button(|b| &mut b.handbrake)),
    ("pad roll left", PadSlotFn::Button(|b| &mut b.roll_left)),
    ("pad roll right", PadSlotFn::Button(|b| &mut b.roll_right)),
    ("pad brake", PadSlotFn::Analog(|b| &mut b.brake_trigger)),
    (
        "pad throttle",
        PadSlotFn::Analog(|b| &mut b.throttle_trigger),
    ),
    ("pad steer", PadSlotFn::Axis(|b| &mut b.steer_axis)),
    ("pad pitch", PadSlotFn::Axis(|b| &mut b.pitch_axis)),
    ("pad ball cam", PadSlotFn::Button(|b| &mut b.ball_cam)),
    (
        "pad kickoff",
        PadSlotFn::OptButton(|b| &mut b.actions.kickoff_reset),
    ),
    (
        "pad dribble",
        PadSlotFn::OptButton(|b| &mut b.actions.dribble),
    ),
    (
        "pad launch",
        PadSlotFn::OptButton(|b| &mut b.actions.launch),
    ),
    (
        "pad follow 1",
        PadSlotFn::OptButton(|b| &mut b.camera.follow[0]),
    ),
    (
        "pad follow 2",
        PadSlotFn::OptButton(|b| &mut b.camera.follow[1]),
    ),
    (
        "pad follow 3",
        PadSlotFn::OptButton(|b| &mut b.camera.follow[2]),
    ),
    (
        "pad follow 4",
        PadSlotFn::OptButton(|b| &mut b.camera.follow[3]),
    ),
    (
        "pad follow 5",
        PadSlotFn::OptButton(|b| &mut b.camera.follow[4]),
    ),
    (
        "pad follow 6",
        PadSlotFn::OptButton(|b| &mut b.camera.follow[5]),
    ),
    (
        "pad follow 7",
        PadSlotFn::OptButton(|b| &mut b.camera.follow[6]),
    ),
    (
        "pad follow 8",
        PadSlotFn::OptButton(|b| &mut b.camera.follow[7]),
    ),
    (
        "pad director",
        PadSlotFn::OptButton(|b| &mut b.camera.director),
    ),
    (
        "pad auto-cam",
        PadSlotFn::OptButton(|b| &mut b.camera.autocam),
    ),
    ("pad free cam", PadSlotFn::OptButton(|b| &mut b.camera.free)),
    (
        "pad next car",
        PadSlotFn::OptButton(|b| &mut b.camera.cycle),
    ),
];

/// Maps a gilrs button event to a binding (mirrors drive.rs's reverse map).
fn from_gilrs_button(b: gilrs::Button) -> PadButton {
    use gilrs::Button as G;
    match b {
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

/// Maps a binding back to gilrs (drive.rs has its own copy; this crate
/// keeps bindings gilrs-free, so each binary maps at its edge).
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

/// True if any connected pad holds the bound button.
fn pad_button_down(gilrs: &gilrs::Gilrs, b: PadButton) -> bool {
    let target = to_gilrs_button(b);
    gilrs.gamepads().any(|(_, g)| g.is_pressed(target))
}

/// First connected pad's axis value, else 0.
fn pad_axis_value(gilrs: &gilrs::Gilrs, a: PadAxis) -> f32 {
    let target = to_gilrs_axis(a);
    gilrs
        .gamepads()
        .next()
        .map(|(_, g)| g.value(target))
        .unwrap_or(0.0)
}

/// Stick deadzone with rescale: drift below `dz` reads 0, full tilt 1.
fn pad_dz(v: f32, dz: f32) -> f32 {
    let dz = dz.clamp(0.0, 0.5);
    let m = v.abs();
    if m < dz || dz >= 1.0 {
        0.0
    } else {
        ((m - dz) / (1.0 - dz)).min(1.0) * v.signum()
    }
}

/// Short label for an analog slot (shared with the TOML shape).
fn pad_analog_label(a: PadAnalog) -> String {
    a.name()
}

/// Registers one menu row for pad navigation and highlights it when
/// focused. Highlight only shows after pad use (mouse users never see
/// it); scrolls the row into view on focus moves.
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

/// Maps a gilrs axis event to a binding.
fn from_gilrs_axis(a: gilrs::Axis) -> PadAxis {
    use gilrs::Axis as G;
    match a {
        G::LeftStickX => PadAxis::LeftStickX,
        G::LeftStickY => PadAxis::LeftStickY,
        G::LeftZ => PadAxis::LeftZ,
        G::RightStickX => PadAxis::RightStickX,
        G::RightStickY => PadAxis::RightStickY,
        G::RightZ => PadAxis::RightZ,
        G::DPadX => PadAxis::DPadX,
        G::DPadY => PadAxis::DPadY,
        G::Unknown => PadAxis::Unknown,
    }
}

/// Right-stick look range, yaw (rad): RL swivels wide, not full behind.
const SWIVEL_MAX_YAW: f32 = 2.0;
/// Right-stick look range, pitch (rad).
const SWIVEL_MAX_PITCH: f32 = 0.9;
/// Trigger deflection below this reads as released.
const TRIGGER_DEADZONE: f32 = 0.05;
/// Trigger gain (full pull overshoots 1.0; the sim clamps on apply).
const TRIGGER_SENSITIVITY: f32 = 1.25;

/// Reads a 0..1 analog drive input. Buttons read their trigger value;
/// axes read one half (positive for throttle, negative for brake), so a
/// single stick binds push-to-drive / pull-to-brake. Ported verbatim from
/// the drive demo (now the single owner of pad driving).
fn pad_analog_value(gamepad: &gilrs::Gamepad, raw: &HashMap<u32, f32>, src: &PadAnalog) -> f32 {
    // Sources already read as 0..1 magnitudes for their half (NegAxis
    // is the negative half); throttle vs brake needs no extra sign —
    // multiplying by field polarity again double-negates NegAxis and
    // turns push-up into brake.
    match src {
        PadAnalog::Button(b) => gamepad
            .button_data(to_gilrs_button(*b))
            .map_or(0.0, |trigger| trigger.value()),
        PadAnalog::Axis(a) => gamepad.value(to_gilrs_axis(*a)).max(0.0),
        PadAnalog::NegAxis(a) => (-gamepad.value(to_gilrs_axis(*a))).max(0.0),
        PadAnalog::Raw(c) => raw.get(c).copied().unwrap_or(0.0).max(0.0),
        PadAnalog::RawNeg(c) => (-raw.get(c).copied().unwrap_or(0.0)).max(0.0),
    }
}

/// Blends local gamepad driving over `controls` (viewer is now the single
/// owner of pad driving; the sim just takes the channel). Mirrors the old
/// drive-demo blend exactly: buttons OR in, sticks overwrite with
/// deadzone, triggers overwrite throttle, bumpers set roll. Callers gate
/// on `driving` so menu-open/unfocused/rebinding kills pads too.
fn blend_pad_driving(
    gilrs: &gilrs::Gilrs,
    raw: &HashMap<u32, f32>,
    mut controls: CarControls,
    pad: &PadBindings,
) -> CarControls {
    for (_, gamepad) in gilrs.gamepads() {
        controls.jump |= gamepad.is_pressed(to_gilrs_button(pad.jump));
        controls.boost |= gamepad.is_pressed(to_gilrs_button(pad.boost));
        controls.handbrake |= gamepad.is_pressed(to_gilrs_button(pad.handbrake));

        // Stick deadzone comes from settings (live, like the rest).
        let dz = clamp_deadzone(pad.stick_deadzone);
        let left_stick_x = gamepad.value(to_gilrs_axis(pad.steer_axis));
        if left_stick_x.abs() > dz {
            // Physics sign: steer=+1 yaws LEFT, so stick-right (+X)
            // sends -1.
            controls.steer = -left_stick_x;
            controls.yaw = -left_stick_x;
        }

        let left_stick_y = gamepad.value(to_gilrs_axis(pad.pitch_axis));
        if left_stick_y.abs() > dz {
            controls.pitch = -left_stick_y;
        }

        let left_trigger = pad_analog_value(&gamepad, raw, &pad.brake_trigger);
        let right_trigger = pad_analog_value(&gamepad, raw, &pad.throttle_trigger);

        if right_trigger > TRIGGER_DEADZONE {
            controls.throttle = right_trigger * TRIGGER_SENSITIVITY;
        }
        if left_trigger > TRIGGER_DEADZONE {
            controls.throttle = -left_trigger * TRIGGER_SENSITIVITY;
        }

        if gamepad.is_pressed(to_gilrs_button(pad.roll_right)) {
            controls.roll -= 1.0;
        }
        if gamepad.is_pressed(to_gilrs_button(pad.roll_left)) {
            controls.roll += 1.0;
        }
    }

    controls
}

/// Whether this frame sends live driver inputs: a followed car, a focused
/// window, no armed rebind, and no open menu. An open menu disables ALL
/// movement (configuring never drives); closing it restores driving.
/// Pure so the rule stays pinned by tests.
fn can_drive(spectated: bool, focused: bool, rebinding: bool, menu_open: bool) -> bool {
    spectated && focused && !rebinding && !menu_open
}

/// Pad menu action for one focused row: `dx` (±1 from ◀▶) adjusts the
/// focused slider or flips the focused toggle; `a` (select button)
/// activates buttons and flips toggles. Mirrors the mouse click
/// handlers one row at a time.
/// Mutable menu state for [`menu_pad_action`] (one struct beats nine args).
struct MenuPadCtx<'a> {
    settings: &'a mut ViewerSettings,
    cam_mode: &'a mut CamMode,
    auto_cam: &'a mut Option<AutoCam>,
    rebind: &'a mut Option<usize>,
    pad_rebind: &'a mut Option<usize>,
    reset_requested: &'a mut bool,
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
        "cam.wide" if a => {
            *cx.cam_mode = CamMode::Man;
            *cx.auto_cam = None;
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
            cx.settings.msaa = slide_val(cx.settings.msaa as f32, dx, 1.0, 0.0, 8.0) as u16;
            cx.settings.mark_dirty();
        }
        "bind.deadzone" if dx != 0 => {
            cx.settings.bindings.pad.stick_deadzone = clamp_deadzone(slide_val(
                cx.settings.bindings.pad.stick_deadzone,
                dx,
                0.05,
                0.0,
                0.5,
            ));
            cx.settings.mark_dirty();
        }
        _ => {
            if a {
                if let Some(n) = id
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
                } else if let Some(n) = id
                    .strip_prefix("bind.key.")
                    .and_then(|n| n.parse::<usize>().ok())
                    .filter(|&n| n < KEY_SLOTS.len())
                {
                    *cx.rebind = Some(n);
                    *cx.pad_rebind = None;
                } else if let Some(n) = id
                    .strip_prefix("bind.pad.")
                    .and_then(|n| n.parse::<usize>().ok())
                    .filter(|&n| n < PAD_SLOTS.len())
                {
                    *cx.pad_rebind = Some(n);
                    *cx.rebind = None;
                }
            }
        }
    }
}

/// Director rig: selection + follow smoothing + cut-blend pose mixing.
struct DirectorRig {
    director: DirectorState,
    smooth: CarCamSmoothing,
    prev_target: Option<DirectorTarget>,
    /// Pose snapshot at the last cut (blend start).
    from_pos: Vec3A,
    from_dir: Vec3A,
    /// Last output pose (becomes the next blend start).
    out_pos: Vec3A,
    out_dir: Vec3A,
    init: bool,
}

impl DirectorRig {
    fn new() -> Self {
        Self {
            director: DirectorState::new(),
            smooth: CarCamSmoothing::new(),
            prev_target: None,
            from_pos: Vec3A::ZERO,
            from_dir: Vec3A::X,
            out_pos: Vec3A::ZERO,
            out_dir: Vec3A::X,
            init: false,
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
        controls: ControlSender,
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
            last_tick: None,
            pending_segs: Arc::new(Mutex::new(Vec::new())),
            cam_man: None,
            init_mode,
            cam_mode,
            director: DirectorRig::new(),
            bounds: ArenaBoundsCache::new(),
            auto_cam: None,
            freecam: FreeCam::new(),
            last_followed: 0,
            controls,
            ctrl_seq: 0,
            driven_car: 0,
            reset_requested: false,
            cursor_grabbed: false,
            rebind: None,
            pad_rebind: None,
            bind_tab: BindTab::Kbm,
            gilrs: gilrs::Gilrs::new().ok(),
            raw_pad: HashMap::new(),
            menu_focus: 0,
            menu_nav_active: false,
            nav_prev_dir: [false; 4],
            nav_prev_a: false,
            nav_prev_b: false,
            prev_menu_pad: false,
            prev_ballcam_pad: false,
            prev_pad_act: [false; 3],
            prev_pad_cam: [false; 12],
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

    /// Builds the containment hull for `mode` up front (other modes build
    /// lazily on first sight). Logs tri count + build time.
    fn warm_bounds(&mut self, mode: GameMode) {
        let t0 = Instant::now();
        let n = self.bounds.get(mode).map(|b| b.tri_count()).unwrap_or(0);
        eprintln!(
            "rocketsim-viewer: {} bounds={n} tris ({:.0} ms)",
            game_mode_name(mode),
            t0.elapsed().as_secs_f32() * 1000.0
        );
    }
}

impl eframe::App for ViewerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // eframe hands us the root Ui; the body below works on the Context.
        let ctx = &ui.ctx().clone();
        // Frame limiter from settings (0 = uncapped).
        let budget = if self.settings.fps_limit_hz == 0 {
            Duration::from_secs(0)
        } else {
            Duration::from_secs_f32(1.0 / self.settings.fps_limit_hz as f32)
        };
        let now = Instant::now();
        let since_last = now.duration_since(self.last_frame);
        if since_last < budget {
            std::thread::sleep(budget - since_last);
        }
        // UI scale applies live.
        if (self.settings.ui_scale - self.applied_ui_scale).abs() > 1e-6 {
            ctx.set_pixels_per_point(self.settings.ui_scale);
            self.applied_ui_scale = self.settings.ui_scale;
        }
        // Frame-start bindings copy (rebinds apply next frame).
        let b = self.settings.bindings;
        // Esc cancels an armed rebind; otherwise it toggles the menu.
        // While either rebind is armed, bound keys stay dead so the
        // pressed key only rebinds (no camera/drive/menu side effects).
        let rebinding = self.rebind.is_some() || self.pad_rebind.is_some();
        if !rebinding && ctx.input(|i| key_pressed(i, b.ui.menu_toggle)) {
            self.menu_open = !self.menu_open;
        }
        // Viewer-side pad reader (menu toggle, menu nav, camera look,
        // pad driving; the sim just takes the channel). Edges + held
        // state feed the menu block and the swivel below.
        let mut pad_menu_edge = false;
        let mut nav_edge = [false; 4];
        let mut nav_held = [false; 4];
        let mut nav_a_edge = false;
        let mut nav_b_edge = false;
        let mut pad_ballcam_edge = false;
        let mut pad_act_edge = [false; 3];
        let mut pad_cam_edge = [false; 12];
        let mut look_raw = (0.0f32, 0.0f32);
        // Pump pad events EVERY frame first: state reads below go stale
        // otherwise (nothing else drains while playing). The rebind
        // capture later reuses this same batch.
        let mut pad_events: Vec<gilrs::Event> = Vec::new();
        if let Some(g) = self.gilrs.as_mut() {
            while let Some(ev) = g.next_event() {
                pad_events.push(ev);
            }
            let m = pad_button_down(g, b.ui.menu_toggle_pad);
            pad_menu_edge = m && !self.prev_menu_pad;
            self.prev_menu_pad = m;
            let sx = pad_axis_value(g, PadAxis::LeftStickX);
            let sy = pad_axis_value(g, PadAxis::LeftStickY);
            let dirs = [
                pad_button_down(g, PadButton::DPadUp) || sy > 0.6,
                pad_button_down(g, PadButton::DPadDown) || sy < -0.6,
                pad_button_down(g, PadButton::DPadLeft) || sx < -0.6,
                pad_button_down(g, PadButton::DPadRight) || sx > 0.6,
            ];
            for i in 0..4 {
                nav_edge[i] = dirs[i] && !self.nav_prev_dir[i];
                nav_held[i] = dirs[i];
                self.nav_prev_dir[i] = dirs[i];
            }
            let a = pad_button_down(g, b.ui.ui_activate);
            nav_a_edge = a && !self.nav_prev_a;
            self.nav_prev_a = a;
            let bb = pad_button_down(g, b.ui.ui_back);
            nav_b_edge = bb && !self.nav_prev_b;
            self.nav_prev_b = bb;
            let bc = pad_button_down(g, b.pad.ball_cam);
            pad_ballcam_edge = bc && !self.prev_ballcam_pad;
            self.prev_ballcam_pad = bc;
            // Rebindable action + camera buttons (viewer folds them
            // into the buttons bitmask and the camera shortcuts below).
            // Edges pause while rebinding, but held state still tracks
            // so no stale edge fires on disarm.
            let act_src = [
                b.pad.actions.kickoff_reset,
                b.pad.actions.dribble,
                b.pad.actions.launch,
            ];
            for i in 0..3 {
                let pb = act_src[i];
                if pb == PadButton::Unknown {
                    self.prev_pad_act[i] = false;
                    continue;
                }
                let cur = pad_button_down(g, pb);
                pad_act_edge[i] = cur && !self.prev_pad_act[i] && !rebinding;
                self.prev_pad_act[i] = cur;
            }
            let cam_bindings = [
                b.pad.camera.follow[0],
                b.pad.camera.follow[1],
                b.pad.camera.follow[2],
                b.pad.camera.follow[3],
                b.pad.camera.follow[4],
                b.pad.camera.follow[5],
                b.pad.camera.follow[6],
                b.pad.camera.follow[7],
                b.pad.camera.director,
                b.pad.camera.autocam,
                b.pad.camera.free,
                b.pad.camera.cycle,
            ];
            for i in 0..12 {
                let pb = cam_bindings[i];
                if pb == PadButton::Unknown {
                    self.prev_pad_cam[i] = false;
                    continue;
                }
                let cur = pad_button_down(g, pb);
                pad_cam_edge[i] = cur && !self.prev_pad_cam[i] && !rebinding;
                self.prev_pad_cam[i] = cur;
            }
            if !self.menu_open {
                let dz = clamp_deadzone(b.pad.stick_deadzone);
                look_raw = (
                    pad_dz(pad_axis_value(g, b.pad.look_x), dz),
                    pad_dz(pad_axis_value(g, b.pad.look_y), dz),
                );
            }
        }
        // Raw ev-code state for unmapped pads (buttons 1/0, axes
        // deflection; cleared on connect). Feeds Raw/RawNeg bindings.
        for ev in &pad_events {
            match ev.event {
                gilrs::EventType::ButtonPressed(_, code) => {
                    self.raw_pad.insert(code.into_u32(), 1.0);
                }
                gilrs::EventType::ButtonReleased(_, code) => {
                    self.raw_pad.insert(code.into_u32(), 0.0);
                }
                gilrs::EventType::AxisChanged(_, v, code) => {
                    self.raw_pad.insert(code.into_u32(), v);
                }
                gilrs::EventType::Connected => {
                    self.raw_pad.clear();
                }
                _ => {}
            }
        }
        if pad_menu_edge && !rebinding {
            self.menu_open = !self.menu_open;
            self.menu_nav_active = true;
        }
        // Mouse use hides the pad highlight again.
        if ctx.input(|i| i.pointer.delta() != egui::Vec2::ZERO) {
            self.menu_nav_active = false;
        }
        // Cursor: Free gets Confined + hidden so look deltas flow with
        // no cursor over the view; Man/Director get Locked + hidden;
        // menu open or focus lost releases all. Sent on change.
        let focused = ctx.input(|i| i.focused);
        let (want_grab, want_visible) = if !focused || self.menu_open {
            (egui::CursorGrab::None, true)
        } else if self.cam_mode == CamMode::Free {
            // Confined + hidden: deltas still flow, and no visible cursor
            // sits over the view.
            (egui::CursorGrab::Confined, false)
        } else {
            (egui::CursorGrab::Locked, false)
        };
        // Full-state track so Locked<->Confined flips send once.
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
        let (scene, mesh, client_frame, last_draw_ms, pads_info, pending_segs, lines_info) = (
            self.scene.clone(),
            self.mesh.clone(),
            self.client.latest(),
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
        let show_pads = self.settings.show_pads;

        // Advance trails in sim time: replay missed ticks (clamped, so a
        // reset jump cannot spiral the emitters).
        if let Some(frame) = &client_frame {
            let tick_now = frame.state.tick_count;
            let steps = match self.last_tick {
                Some(last) => tick_now.saturating_sub(last).clamp(1, 10),
                None => 1,
            } as usize;
            self.last_tick = Some(tick_now);
            let ball_pos = frame.state.ball.phys.pos;
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
            let mut fx_segs = Vec::new();
            for _ in 0..steps {
                fx_segs = self.car_fx.update(&frame.state, frame.dt);
            }
            segs.extend(fx_segs);
            for (i, (_, car)) in frame.state.cars.iter().enumerate() {
                while self.boost_ribbons.len() <= i {
                    self.boost_ribbons
                        .push(RibbonEmitter::new(RibbonConfig::boost()));
                }
                let fwd = car.phys.get_forward_dir();
                let emit_pos = car.phys.pos - fwd * 60.0;
                let emit_vel = -fwd * 300.0;
                for _ in 0..steps {
                    self.boost_ribbons[i].update(car.is_boosting, emit_pos, emit_vel, frame.dt);
                }
                if self.settings.show_boost {
                    segs.extend(self.boost_ribbons[i].segments(emit_pos));
                }
            }
            *self.pending_segs.lock().expect("viewer segs poisoned") = segs;
        }

        // Camera: fixed pose until the first frame, then the active mode.
        // Keys: 1-8 follow, 9 director, 0 autocam, F free camera,
        // C cycle, Space ball-cam. WASD always drives, never a camera key.
        let (cam_pos, mut cam_dir, cam_fov, cam_line) = match &client_frame {
            Some(frame) => {
                let state = &frame.state;
                // Game-mode switch: a restarted sim may stream a different
                // mode than the mesh was built for. Rebuild the mesh (the
                // GL scene follows it on the next paint) and re-point
                // camera + bounds; a switch hitches one frame.
                if state.game_mode() != self.mesh.mode {
                    let mode = state.game_mode();
                    let mesh = Arc::new(ArenaMesh::build(mode));
                    self.soup_verts = mesh.soup_verts;
                    self.welded_verts = mesh.welded_verts();
                    self.n_indices = mesh.index_count();
                    self.mesh = mesh;
                    self.warm_bounds(mode);
                    if let Some(man) = self.cam_man.as_mut() {
                        man.set_game_mode(mode);
                    }
                    eprintln!(
                        "rocketsim-viewer: game mode switched to {}",
                        game_mode_name(mode)
                    );
                }
                // Containment hull for this mode (lazy; TheVoid = None).
                let bounds = self.bounds.get(state.game_mode());
                if self.cam_man.is_none() {
                    let mut man = CameraMan::new(state.game_mode(), CameraConfig::default());
                    match self.init_mode {
                        StartMode::Follow(n) | StartMode::Auto(n) => {
                            for _ in 0..=n {
                                man.cycle_target(state);
                            }
                        }
                        _ => {}
                    }
                    // Auto CLI arms only if the cycle actually landed on n.
                    if let StartMode::Auto(n) = self.init_mode
                        && man.car_cam().is_some_and(|c| c.car_idx == n)
                    {
                        self.auto_cam = Some(AutoCam::new(n));
                    }
                    self.cam_man = Some(man);
                }
                // NOTE: driving keys steer the car, so they are not camera keys.
                // All shortcuts stay dead while rebinding: the pressed key
                // rebinds instead of switching cameras or driving.
                for (idx, key) in b.camera.follow.iter().enumerate() {
                    if (!rebinding && ctx.input(|inp| key_pressed(inp, *key))) || pad_cam_edge[idx]
                    {
                        self.cam_mode = CamMode::Man;
                        self.auto_cam = None;
                        if let Some(man) = self.cam_man.as_mut() {
                            man.follow_car(state, idx);
                        }
                    }
                }
                if (!rebinding && ctx.input(|i| key_pressed(i, b.camera.director)))
                    || pad_cam_edge[8]
                {
                    self.cam_mode = CamMode::Director;
                    self.auto_cam = None;
                }
                if (!rebinding && ctx.input(|i| key_pressed(i, b.camera.autocam)))
                    || pad_cam_edge[9]
                {
                    self.cam_mode = CamMode::Man;
                    let idx = self.last_followed;
                    if let Some(man) = self.cam_man.as_mut() {
                        man.follow_car(state, idx);
                    }
                    self.auto_cam = Some(AutoCam::new(idx));
                }
                if (!rebinding && ctx.input(|i| key_pressed(i, b.camera.free))) || pad_cam_edge[10]
                {
                    self.cam_mode = if self.cam_mode == CamMode::Free {
                        CamMode::Man
                    } else {
                        CamMode::Free
                    };
                    self.auto_cam = None;
                }
                if (!rebinding && ctx.input(|i| key_pressed(i, b.camera.cycle))) || pad_cam_edge[11]
                {
                    self.cam_mode = CamMode::Man;
                    self.auto_cam = None;
                    if let Some(man) = self.cam_man.as_mut() {
                        man.cycle_target(state);
                    }
                }
                match self.cam_mode {
                    CamMode::Man => {
                        // Ball-cam toggle, keyboard or pad (free camera
                        // has none; pad only with the menu closed).
                        if ((!rebinding && ctx.input(|i| key_pressed(i, b.camera.ball_cam)))
                            || (!self.menu_open && pad_ballcam_edge))
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
                        // Forced ball-cam follow for car targets; fixed pose
                        // for overview.
                        let (pos, dir, fov, label) = match target {
                            DirectorTarget::Overview => {
                                let (eye, tgt, fov) = view::arena_pose();
                                rig.smooth.reset();
                                (
                                    Vec3A::from(eye),
                                    (Vec3A::from(tgt) - Vec3A::from(eye)).normalize_or_zero(),
                                    fov,
                                    "director · wide view".to_string(),
                                )
                            }
                            DirectorTarget::Car(i) => {
                                let mut p = Vec3A::ZERO;
                                let mut d = Vec3A::X;
                                rig.smooth.update(
                                    i,
                                    true,
                                    state,
                                    &self.settings.camera,
                                    &mut p,
                                    &mut d,
                                    dt,
                                );
                                // Same boom shortening as the follow view
                                // (director cars always run ball-cam).
                                if let (Some(hull), Some((_, car))) =
                                    (bounds.as_ref(), state.cars.get(i))
                                {
                                    let focus = car.pos
                                        + Vec3A::new(0.0, 0.0, self.settings.camera.car_cam.height);
                                    (p, d) = hull.contain_follow(focus, state.ball.pos, true, p, d);
                                }
                                (p, d, rig.smooth.cur_fov_h(), format!("director · car {i}"))
                            }
                        };
                        // Cut blend: a target change restarts the blend from
                        // the last output pose — except cuts to/from the
                        // (outside) overview, which snap: blending would
                        // sweep the camera straight through the wall.
                        if rig.prev_target != Some(target) {
                            let overview_cut = rig.prev_target == Some(DirectorTarget::Overview)
                                || target == DirectorTarget::Overview;
                            rig.prev_target = Some(target);
                            if rig.init && !overview_cut {
                                rig.from_pos = rig.out_pos;
                                rig.from_dir = rig.out_dir;
                            } else {
                                rig.from_pos = pos;
                                rig.from_dir = dir;
                                rig.init = true;
                            }
                        }
                        let b = rig.director.blend();
                        let pos = rig.from_pos.lerp(pos, b);
                        let mut dir = rig.from_dir.lerp(dir, b).normalize_or_zero();
                        if dir.length_squared() < 1e-8 {
                            dir = rig.from_dir;
                        }
                        rig.out_pos = pos;
                        rig.out_dir = dir;
                        (pos, dir, fov, label)
                    }
                    CamMode::Free => {
                        // Mouse looks, WASD flies level, Space rises, Shift
                        // sinks. Mouse and wheel count only with the menu
                        // closed and off UI (an open menu never steers the
                        // camera). Free camera never drives (see below).
                        const LOOK_SENS: f32 = 0.005;
                        const DOLLY_SENS: f32 = -120.0;
                        if !self.menu_open && !ctx.egui_wants_pointer_input() {
                            let d = ctx.input(|i| i.pointer.delta());
                            self.freecam.look(-d.x * LOOK_SENS, -d.y * LOOK_SENS);
                            let wheel = ctx.input(|i| i.smooth_scroll_delta.y);
                            self.freecam
                                .dolly_bounded(wheel * DOLLY_SENS / 50.0, bounds.as_deref());
                        }
                        // Flight reuses the car keys; up couples to the
                        // ball-cam key (Space by default).
                        // Flight keys stay dead while rebinding or with the
                        // menu open (no movement while configuring).
                        let (f, r, u) = if rebinding || self.menu_open {
                            (0.0, 0.0, 0.0)
                        } else {
                            ctx.input(|i| {
                                let down = |k| key_down(i, k);
                                (
                                    (down(b.car.throttle_fwd) as i8
                                        - down(b.car.throttle_back) as i8)
                                        as f32,
                                    (down(b.car.steer_right) as i8 - down(b.car.steer_left) as i8)
                                        as f32,
                                    (down(b.camera.ball_cam) as i8 - down(b.car.handbrake) as i8)
                                        as f32,
                                )
                            })
                        };
                        self.freecam.fly_bounded(f, r, u, dt, bounds.as_deref());
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

        // Right-stick look (RL swivel): rotates the VIEW, boom stays.
        // Stick right looks right, stick up looks up; springs back on
        // release. Follow views only, menu closed.
        let following = self.cam_mode == CamMode::Man
            && self.cam_man.as_ref().and_then(|m| m.car_cam()).is_some();
        let (look_tx, look_ty) = if following && !self.menu_open {
            (look_raw.0 * SWIVEL_MAX_YAW, look_raw.1 * SWIVEL_MAX_PITCH)
        } else {
            (0.0, 0.0)
        };
        let k = 1.0 - (-10.0 * dt).exp();
        self.swivel_yaw += (look_tx - self.swivel_yaw) * k;
        self.swivel_pitch += (look_ty - self.swivel_pitch) * k;
        if self.swivel_yaw.abs() + self.swivel_pitch.abs() > 1e-4 {
            let theta = cam_dir.y.atan2(cam_dir.x) - self.swivel_yaw;
            let elev = (cam_dir.z.clamp(-1.0, 1.0).asin() + self.swivel_pitch).clamp(-1.55, 1.55);
            cam_dir = Vec3A::new(
                elev.cos() * theta.cos(),
                elev.cos() * theta.sin(),
                elev.sin(),
            );
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
        let driving = can_drive(spectated.is_some(), focused, rebinding, self.menu_open);
        let mut controls = CarControls::default();
        let mut buttons = 0u8;
        if driving {
            let c = b.car;
            let (w, s, a, d, q, e, primary, secondary) = ctx.input(|i| {
                (
                    key_down(i, c.throttle_fwd),
                    key_down(i, c.throttle_back),
                    key_down(i, c.steer_left),
                    key_down(i, c.steer_right),
                    key_down(i, c.roll_left),
                    key_down(i, c.roll_right),
                    mouse_down(i, c.jump),
                    mouse_down(i, c.boost),
                )
            });
            let shift = ctx.input(|i| key_down(i, c.handbrake));
            // Pointer input counts only outside UI (sliders keep their drag).
            let ui_busy = ctx.egui_wants_pointer_input();
            if w {
                controls.throttle += 1.0;
            }
            if s {
                controls.throttle -= 1.0;
            }
            // Probe ground truth: steer=+1 yaws the nose LEFT, so the
            // LEFT key sends +1 (RIGHT sends -1). Do not "fix" the signs.
            if a {
                controls.steer += 1.0;
            }
            if d {
                controls.steer -= 1.0;
            }
            // Roll probe (airborne, nose +Y): roll=+1 tilts up toward
            // −X = driver's left. LEFT sends +1 (RIGHT −1); do not flip.
            if q {
                controls.roll += 1.0;
            }
            if e {
                controls.roll -= 1.0;
            }
            controls.handbrake = shift;
            if !ui_busy {
                controls.jump = primary;
                controls.boost = secondary;
            }
            controls.yaw = controls.steer;
            controls.pitch = -controls.throttle;
            if controls.handbrake {
                controls.roll = controls.yaw;
                controls.yaw = 0.0;
            }
            controls.throttle = controls.throttle.clamp(-1.0, 1.0);
            controls.steer = controls.steer.clamp(-1.0, 1.0);
            controls.pitch = controls.pitch.clamp(-1.0, 1.0);
            controls.yaw = controls.yaw.clamp(-1.0, 1.0);
            controls.roll = controls.roll.clamp(-1.0, 1.0);
            // Local gamepad driving blends over KBM (viewer is the single
            // owner; gated on `driving` like everything else, so menu-open
            // kills pads too). Unclamped by design: trigger gain overshoots
            // 1.0 and the sim clamps on apply, exactly as before.
            if let Some(g) = self.gilrs.as_ref() {
                controls = blend_pad_driving(g, &self.raw_pad, controls, &b.pad);
            }
            // NOTE: a modifier bound here would never fire (no press edge).
            if ctx.input(|i| key_pressed(i, b.actions.kickoff_reset))
                || self.reset_requested
                || pad_act_edge[0]
            {
                buttons |= 0b001;
            }
            if ctx.input(|i| key_pressed(i, b.actions.dribble)) || pad_act_edge[1] {
                buttons |= 0b010;
            }
            if ctx.input(|i| key_pressed(i, b.actions.launch)) || pad_act_edge[2] {
                buttons |= 0b100;
            }
        }
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
        self.controls.send(
            ControlMsg::new(self.driven_car, controls, buttons, self.ctrl_seq)
                .with_mutators(self.settings.mutators_mask()),
        );
        // The menu reset fires once, on the next send.
        self.reset_requested = false;

        self.settings.save_if_due();

        // Player-facing status lines (all builds).
        let sim_line = match &status {
            ConnStatus::Waiting => "Sim: waiting for a sim…".to_string(),
            ConnStatus::Connected { .. } => {
                format!("Sim: live (tick {})", tick.unwrap_or(0))
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
            if self.controls.connected() {
                format!("Input: driving car {} · live", self.driven_car)
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
        if self.menu_open {
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
                egui::CollapsingHeader::new("Camera")
                    .default_open(true)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            let r = ui.button("Wide view");
                            if r.clicked() {
                                self.cam_mode = CamMode::Man;
                                self.auto_cam = None;
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
                            "Keys: {}-{} follow, {} director, {} auto, {} free, {} next car, {} ball cam.",
                            b.camera.follow[0].name(),
                            b.camera.follow[7].name(),
                            b.camera.director.name(),
                            b.camera.autocam.name(),
                            b.camera.free.name(),
                            b.camera.cycle.name(),
                            b.camera.ball_cam.name(),
                        ));
                        ui.label(format!("You drive car {}.", self.driven_car));
                        ui.label(drive_line.clone());
                        let r = ui.button("Reset play (Backspace)");
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
                egui::CollapsingHeader::new("Input")
                    .default_open(true)
                    .show(ui, |ui| {
                        let r = ui.checkbox(&mut settings.spectate_only, "Spectate only (no inputs)");
                        if r.changed() {
                            settings.mark_dirty();
                        }
                        nav_row(ui, &r, ids, "input.spectate".to_string(), focus, active, moved);
                    });
                egui::CollapsingHeader::new("Mutators (drive demo)")
                    .default_open(false)
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
                egui::CollapsingHeader::new("Overlays")
                    .default_open(false)
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
                egui::CollapsingHeader::new("Display")
                    .default_open(true)
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
                            let r =
                                ui.add(egui::Slider::new(&mut settings.msaa, 0..=8).text("MSAA"));
                            if r.changed() {
                                settings.mark_dirty();
                            }
                            nav_row(ui, &r, ids, "disp.msaa".to_string(), focus, active, moved);
                        });
                        ui.label("(VSync/MSAA apply on relaunch)");
                    });
                egui::CollapsingHeader::new("Bindings")
                    .default_open(false)
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.selectable_value(
                                &mut self.bind_tab,
                                BindTab::Kbm,
                                "Keyboard + mouse",
                            );
                            ui.selectable_value(&mut self.bind_tab, BindTab::Pad, "Controller");
                        });
                        match self.bind_tab {
                            BindTab::Kbm => {
                        // Keys rebind by capture; mouse stays fixed
                        // (hand-edit settings.toml for it).
                        for (n, (label, slot)) in KEY_SLOTS.iter().enumerate() {
                            let resp = ui.horizontal(|ui| {
                                ui.label(*label);
                                let text = if self.rebind == Some(n) {
                                    "press a key…".to_string()
                                } else {
                                    slot(&mut settings.bindings).name().to_string()
                                };
                                if ui.button(text).clicked() {
                                    self.rebind = Some(n);
                                    self.pad_rebind = None;
                                }
                            });
                            nav_row(
                                ui,
                                &resp.response,
                                ids,
                                format!("bind.key.{n}"),
                                focus,
                                active,
                                moved,
                            );
                        }
                        ui.label(format!(
                            "mouse: jump {:?} · boost {:?} (fixed)",
                            settings.bindings.car.jump, settings.bindings.car.boost
                        ));
                        if let Some(n) = self.rebind {
                            let hit = ctx.input(|i| {
                                Key::ALL.iter().copied().find(|k| {
                                    k.to_egui().is_some_and(|ek| i.key_pressed(ek))
                                })
                            });
                            if let Some(k) = hit {
                                // The menu key cancels instead of binding.
                                if k == settings.bindings.ui.menu_toggle {
                                    self.rebind = None;
                                    self.pad_rebind = None;
                                } else {
                                    *KEY_SLOTS[n].1(&mut settings.bindings) = k;
                                    settings.mark_dirty();
                                    self.rebind = None;
                                }
                            }
                            ui.label("Press a key… (menu key cancels)");
                        }
                            }
                            BindTab::Pad => {
                        // Stick deadzone: drive sticks + right-stick look.
                        let r = ui.add(
                            egui::Slider::new(
                                &mut settings.bindings.pad.stick_deadzone,
                                0.0..=0.5,
                            )
                            .text("stick deadzone"),
                        );
                        if r.changed() {
                            settings.bindings.pad.stick_deadzone =
                                clamp_deadzone(settings.bindings.pad.stick_deadzone);
                            settings.mark_dirty();
                        }
                        nav_row(ui, &r, ids, "bind.deadzone".to_string(), focus, active, moved);
                        // Controller rows: first button press / stick move
                        // past 60% while armed captures the slot. The menu
                        // key cancels; stale pad events drain when idle so
                        // an old press never binds the moment you arm.
                        if self.gilrs.is_none() {
                            ui.label("controller: no gamepad found");
                        } else {
                            for (n, (label, slot)) in PAD_SLOTS.iter().enumerate() {
                                let resp = ui.horizontal(|ui| {
                                    ui.label(*label);
                                    // PadBindings is Copy: read via a local.
                                    let mut pad = settings.bindings.pad;
                                    let cur = match slot {
                                        PadSlotFn::Button(f) => {
                                            format!("{:?}", f(&mut pad))
                                        }
                                        PadSlotFn::Axis(f) => {
                                            format!("{:?}", f(&mut pad))
                                        }
                                        PadSlotFn::Analog(f) => {
                                            pad_analog_label(*f(&mut pad))
                                        }
                                        PadSlotFn::OptButton(f) => {
                                            let v = *f(&mut pad);
                                            if v == PadButton::Unknown {
                                                "—".to_string()
                                            } else {
                                                format!("{v:?}")
                                            }
                                        }
                                    };
                                    let text = if self.pad_rebind == Some(n) {
                                        "press it…".to_string()
                                    } else {
                                        cur
                                    };
                                    if ui.button(text).clicked() {
                                        self.pad_rebind = Some(n);
                                        self.rebind = None;
                                    }
                                });
                                nav_row(
                                    ui,
                                    &resp.response,
                                    ids,
                                    format!("bind.pad.{n}"),
                                    focus,
                                    active,
                                    moved,
                                );
                            }
                        }
                        if let Some(n) = self.pad_rebind {
                            // Mapped hits plus raw-ev-code fallback: pads
                            // whose axes gilrs leaves unmapped (their pulls
                            // otherwise bind `Unknown`) bind by raw code.
                            // Unknown buttons never bind (unreproducible).
                            // Events come from the frame-start pump above.
                            let mut hit_b = None;
                            let mut hit_a = None;
                            let mut hit_raw = None;
                            for ev in &pad_events {
                                match ev.event {
                                    gilrs::EventType::ButtonPressed(b, _) => {
                                        let mapped = from_gilrs_button(b);
                                        if mapped != PadButton::Unknown {
                                            hit_b = Some(mapped);
                                        }
                                    }
                                    gilrs::EventType::AxisChanged(a, v, code) => {
                                        if v.abs() <= 0.6 {
                                            continue;
                                        }
                                        let mapped = from_gilrs_axis(a);
                                        if mapped != PadAxis::Unknown {
                                            hit_a = Some((mapped, v < 0.0));
                                        } else {
                                            hit_raw = Some((code.into_u32(), v < 0.0));
                                        }
                                    }
                                    _ => {}
                                }
                            }
                            if ctx.input(|i| key_pressed(i, settings.bindings.ui.menu_toggle)) {
                                self.pad_rebind = None;
                                self.rebind = None;
                            } else {
                                match &PAD_SLOTS[n].1 {
                                    PadSlotFn::Button(f) => {
                                        if let Some(b) = hit_b {
                                            *f(&mut settings.bindings.pad) = b;
                                            settings.mark_dirty();
                                            self.pad_rebind = None;
                                        }
                                    }
                                    PadSlotFn::Axis(f) => {
                                        if let Some((a, _)) = hit_a {
                                            *f(&mut settings.bindings.pad) = a;
                                            settings.mark_dirty();
                                            self.pad_rebind = None;
                                        }
                                    }
                                    PadSlotFn::OptButton(f) => {
                                        if let Some(b) = hit_b {
                                            *f(&mut settings.bindings.pad) = b;
                                            settings.mark_dirty();
                                            self.pad_rebind = None;
                                        }
                                    }
                                    PadSlotFn::Analog(f) => {
                                        // Push direction binds: up/right
                                        // takes the positive half.
                                        if let Some(b) = hit_b {
                                            *f(&mut settings.bindings.pad) =
                                                PadAnalog::Button(b);
                                            settings.mark_dirty();
                                            self.pad_rebind = None;
                                        } else if let Some((a, neg)) = hit_a {
                                            *f(&mut settings.bindings.pad) = if neg {
                                                PadAnalog::NegAxis(a)
                                            } else {
                                                PadAnalog::Axis(a)
                                            };
                                            settings.mark_dirty();
                                            self.pad_rebind = None;
                                        } else if let Some((c, neg)) = hit_raw {
                                            *f(&mut settings.bindings.pad) = if neg {
                                                PadAnalog::RawNeg(c)
                                            } else {
                                                PadAnalog::Raw(c)
                                            };
                                            settings.mark_dirty();
                                            self.pad_rebind = None;
                                        }
                                    }
                                }
                            }
                            let prompt = match &PAD_SLOTS[n].1 {
                                PadSlotFn::Button(_) | PadSlotFn::OptButton(_) => {
                                    "Press a button… (menu key cancels)"
                                }
                                PadSlotFn::Axis(_) => "Move a stick fully… (menu key cancels)",
                                PadSlotFn::Analog(_) => {
                                    "Press / move fully (direction binds)… (menu key cancels)"
                                }
                            };
                            ui.label(prompt);
                        }
                            }
                        }

                    });
                ui.label("Pad: Start toggles menu · D-pad/stick move · A select · B back.");
                let r = ui.button("Reset everything");
                if r.clicked() {
                    settings.reset_all();
                }
                nav_row(ui, &r, ids, "menu.reset_all".to_string(), focus, active, moved);
                self.nav_moved = false;
            });
        }

        // Pad menu navigation (needs the fresh id list from the draw).
        // Up/down move with press + hold-repeat; left/right adjust the
        // focused slider or flip the focused toggle; A activates; B closes.
        if !self.nav_ids.is_empty() {
            self.menu_focus %= self.nav_ids.len();
        }
        // Pad nav stays dead while rebinding (a select press binds
        // instead of activating the focused row).
        if self.menu_open && !rebinding && !self.nav_ids.is_empty() {
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
                        auto_cam: &mut self.auto_cam,
                        rebind: &mut self.rebind,
                        pad_rebind: &mut self.pad_rebind,
                        reset_requested: &mut self.reset_requested,
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
                    self.controls.connected()
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
                        if guard.as_ref().is_none_or(|s| s.mode != mesh.mode) {
                            *guard = Some(SceneGl::new(gl, &mesh));
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
                            scene.prog_main.draw(
                                gl,
                                &scene.models[BALL_MODEL],
                                &scene.textures[BALL_TEXTURE],
                                &model_matrix(Vec3::from(ball.pos), Mat3::from(ball.rot_mat)),
                            );
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
    /// TCP port the viewer listens on (the sim dials it).
    #[arg(default_value_t = DEFAULT_PORT)]
    port: u16,
    /// Start camera: birds|followN|director|autoN|free.
    #[arg(default_value = "birds")]
    cam: String,
    /// Initial arena mesh + containment (pre-connect placeholder); the
    /// viewer follows the sim's mode automatically once frames arrive.
    #[arg(short, long, default_value = "soccar", value_parser = parse_mode)]
    game_mode: GameMode,
    /// GPU-stall timing for profiling.
    #[arg(long)]
    gpushot: bool,
}

/// Clap parser over the shared [`parse_game_mode`] vocabulary.
fn parse_mode(s: &str) -> Result<GameMode, String> {
    parse_game_mode(s).ok_or_else(|| {
        format!("unknown game mode '{s}' (soccar|hoops|heatseeker|snowday|dropshot|thevoid|void)")
    })
}

fn main() {
    init_meshes();
    let cli = Cli::parse();
    let port = cli.port;
    // Start camera: `birds` default; `followN`, `director`, `autoN`,
    // `free`. Runtime: `1-8` follow, `9` director, `0` autocam,
    // `F` free camera, `C` cycle, `Space` ball-cam.
    // Driving (WASD + mouse, Backspace/F2/F4) needs this window focused.
    // The options menu starts open (cursor free); Escape toggles it.
    let init_mode = StartMode::parse(&cli.cam);
    let menu_open = true;
    // `gpushot` also accepted bare (pre-clap usage) anywhere in argv.
    let measure_gpu = cli.gpushot || std::env::args().any(|a| a == "gpushot");
    // Load settings (creates the file on first run).
    let settings = ViewerSettings::load();
    settings.save();
    let addr = format!("127.0.0.1:{port}");
    eprintln!("rocketsim-viewer: listening on {addr} (sim should dial this addr)");

    let mesh = Arc::new(ArenaMesh::build(cli.game_mode));
    eprintln!(
        "rocketsim-viewer: {} soup={} welded={} idx={}",
        game_mode_name(cli.game_mode),
        mesh.soup_verts,
        mesh.welded_verts(),
        mesh.index_count()
    );
    let client = FrameClient::connect(&addr);
    // Driving channel: the sim listens on DEFAULT_CONTROL_PORT. Dials
    // lazily; drops inputs until a sim appears. Never blocks or fails.
    let controls = ControlSender::connect(&format!("127.0.0.1:{DEFAULT_CONTROL_PORT}"));

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
            let mut app = ViewerApp::new(
                mesh.clone(),
                client,
                init_mode,
                settings,
                menu_open,
                measure_gpu,
                controls,
            );
            app.warm_bounds(cli.game_mode);
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

    fn ctx() -> (
        ViewerSettings,
        CamMode,
        Option<AutoCam>,
        Option<usize>,
        Option<usize>,
        bool,
    ) {
        (
            ViewerSettings::default(),
            CamMode::Man,
            None,
            None,
            None,
            false,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn act(
        id: &str,
        dx: i32,
        a: bool,
        settings: &mut ViewerSettings,
        cam_mode: &mut CamMode,
        auto_cam: &mut Option<AutoCam>,
        rebind: &mut Option<usize>,
        pad_rebind: &mut Option<usize>,
        reset_requested: &mut bool,
    ) {
        menu_pad_action(
            id.to_string(),
            dx,
            a,
            &mut MenuPadCtx {
                settings,
                cam_mode,
                auto_cam,
                rebind,
                pad_rebind,
                reset_requested,
            },
        );
    }

    #[test]
    fn pad_toggle_flips_and_dirties() {
        let (mut s, mut m, mut ac, mut r, mut pr, mut rr) = ctx();
        assert!(!s.spectate_only);
        act(
            "input.spectate",
            0,
            true,
            &mut s,
            &mut m,
            &mut ac,
            &mut r,
            &mut pr,
            &mut rr,
        );
        assert!(s.spectate_only);
        // Left/right flips too.
        act(
            "input.spectate",
            -1,
            false,
            &mut s,
            &mut m,
            &mut ac,
            &mut r,
            &mut pr,
            &mut rr,
        );
        assert!(!s.spectate_only);
    }

    #[test]
    fn pad_slider_steps_and_clamps() {
        let (mut s, mut m, mut ac, mut r, mut pr, mut rr) = ctx();
        act(
            "cam.fov", 1, false, &mut s, &mut m, &mut ac, &mut r, &mut pr, &mut rr,
        );
        assert_eq!(s.camera.fov_degrees, 111.0);
        // A alone never moves a slider.
        act(
            "cam.fov", 0, true, &mut s, &mut m, &mut ac, &mut r, &mut pr, &mut rr,
        );
        assert_eq!(s.camera.fov_degrees, 111.0);
        // Clamps at the widget range.
        s.camera.fov_degrees = 130.0;
        act(
            "cam.fov", 1, false, &mut s, &mut m, &mut ac, &mut r, &mut pr, &mut rr,
        );
        assert_eq!(s.camera.fov_degrees, 130.0);
        // Integer sliders convert without drift.
        s.msaa = 8;
        act(
            "disp.msaa",
            1,
            false,
            &mut s,
            &mut m,
            &mut ac,
            &mut r,
            &mut pr,
            &mut rr,
        );
        assert_eq!(s.msaa, 8);
        s.fps_limit_hz = 0;
        act(
            "disp.fps", 1, false, &mut s, &mut m, &mut ac, &mut r, &mut pr, &mut rr,
        );
        assert_eq!(s.fps_limit_hz, 10);
        // Pad deadzone steps and clamps.
        act(
            "bind.deadzone",
            1,
            false,
            &mut s,
            &mut m,
            &mut ac,
            &mut r,
            &mut pr,
            &mut rr,
        );
        assert!((s.bindings.pad.stick_deadzone - 0.15).abs() < 1e-6);
    }

    #[test]
    fn pad_buttons_fire_and_ignore_leftright() {
        let (mut s, mut m, mut ac, mut r, mut pr, mut rr) = ctx();
        act(
            "cam.reset_play",
            0,
            true,
            &mut s,
            &mut m,
            &mut ac,
            &mut r,
            &mut pr,
            &mut rr,
        );
        assert!(rr);
        act(
            "cam.director",
            0,
            true,
            &mut s,
            &mut m,
            &mut ac,
            &mut r,
            &mut pr,
            &mut rr,
        );
        assert_eq!(m, CamMode::Director);
        assert!(ac.is_none());
        // Left/right alone never fires a button.
        let (mut s, mut m, mut ac, mut r, mut pr, mut rr) = ctx();
        act(
            "cam.reset_play",
            1,
            false,
            &mut s,
            &mut m,
            &mut ac,
            &mut r,
            &mut pr,
            &mut rr,
        );
        assert!(!rr);
    }

    #[test]
    fn pad_preset_and_rebind_rows() {
        let (mut s, mut m, mut ac, mut r, mut pr, mut rr) = ctx();
        s.camera.car_cam.distance = 0.0;
        act(
            "cam.preset.0",
            0,
            true,
            &mut s,
            &mut m,
            &mut ac,
            &mut r,
            &mut pr,
            &mut rr,
        );
        assert_eq!(s.camera.car_cam.distance, 270.0);
        // Out-of-range preset index: no-op, no panic.
        act(
            "cam.preset.99",
            0,
            true,
            &mut s,
            &mut m,
            &mut ac,
            &mut r,
            &mut pr,
            &mut rr,
        );
        // Rebind rows arm capture (key clears pad and vice versa).
        act(
            "bind.key.0",
            0,
            true,
            &mut s,
            &mut m,
            &mut ac,
            &mut r,
            &mut pr,
            &mut rr,
        );
        assert_eq!(r, Some(0));
        assert_eq!(pr, None);
        act(
            "bind.pad.1",
            0,
            true,
            &mut s,
            &mut m,
            &mut ac,
            &mut r,
            &mut pr,
            &mut rr,
        );
        assert_eq!(pr, Some(1));
        assert_eq!(r, None);
        // Unknown id: no-op, no panic.
        act(
            "nope.nothing",
            1,
            true,
            &mut s,
            &mut m,
            &mut ac,
            &mut r,
            &mut pr,
            &mut rr,
        );
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
