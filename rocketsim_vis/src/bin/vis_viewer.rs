//! Detached viewer binary: live 3D scene streamed from the sim over TCP.
//!
//! Usage: `vis_viewer [PORT] [CAM] [flags...]` (defaults: 39201, birds).
//! CAM is `birds`|`followN`|`director`|`autoN`|`free`; flags: `gpushot`
//! (GPU-stall timing for profiling). The options menu starts open;
//! Escape (or pad Start) toggles it.

use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use glam::{Mat3, Vec3, Vec3A};
use glow::HasContext;
use rocketsim::{CarControls, GameMode};
use rocketsim_vis::{
    arena_gl::ArenaGl,
    camera::{
        ArenaBoundsCache, AutoCam, CameraConfig, CameraMan, CameraPreset, CarCamSmoothing,
        DirectorState, DirectorTarget, FreeCam,
    },
    frustum::frustum_planes,
    fx::CarFx,
    lines::{RibbonConfig, RibbonEmitter, Segment, ground_line},
    lines_gl::LinesGl,
    main_gl::{
        BALL_MODEL, BALL_TEXTURE, InstancedModelGl, MainInstProgram, MainProgram, ModelGl,
        car_model_name, car_texture_name, group_car_draws, model_matrix,
    },
    mesh::ArenaMesh,
    net_client::{ConnStatus, ControlSender, FrameClient},
    obj::ObjMesh,
    pads::{build_instances, load_pad_meshes, variant_radius},
    pads_gl::PadsGl,
    photo::init_meshes,
    proto::{ControlMsg, DEFAULT_CONTROL_PORT, DEFAULT_PORT},
    settings::ViewerSettings,
    spectate,
    textures::{PngImage, TextureGl},
    view,
};

/// Owned GL scene: arena + main pipeline + pads + models/textures.
struct SceneGl {
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
            ModelGl::new(gl, &ObjMesh::load(include_str!("../models/ball.obj"))),
        );
        let mut textures = HashMap::new();
        for (name, png) in [
            (
                BALL_TEXTURE,
                include_bytes!("../textures/ball.png").as_slice(),
            ),
            (
                "car_blue",
                include_bytes!("../textures/car_blue.png").as_slice(),
            ),
            (
                "car_orange",
                include_bytes!("../textures/car_orange.png").as_slice(),
            ),
        ] {
            textures.insert(name.to_string(), TextureGl::new(gl, &PngImage::decode(png)));
        }
        let mut car_models = HashMap::new();
        for (name, obj) in [
            ("car_octane", include_str!("../models/car_octane.obj")),
            ("car_dominus", include_str!("../models/car_dominus.obj")),
            ("car_breakout", include_str!("../models/car_breakout.obj")),
            ("car_merc", include_str!("../models/car_merc.obj")),
        ] {
            car_models.insert(
                name.to_string(),
                InstancedModelGl::new(gl, &ObjMesh::load(obj)),
            );
        }
        Self {
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
            last_frame: Instant::now(),
            fps_ema: 60.0,
            soup_verts: soup,
            welded_verts: welded,
            n_indices: indices,
        }
    }

    /// Builds the Soccar containment hull up front (other modes build
    /// lazily on first sight). Logs tri count + build time.
    fn warm_bounds(&mut self) {
        let t0 = Instant::now();
        let n = self
            .bounds
            .get(GameMode::Soccar)
            .map(|b| b.tri_count())
            .unwrap_or(0);
        eprintln!(
            "vis_viewer: soccar bounds={n} tris ({:.0} ms)",
            t0.elapsed().as_secs_f32() * 1000.0
        );
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
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
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            self.menu_open = !self.menu_open;
        }
        // Cursor: Free gets Confined + visible so look deltas work
        // while the pointer stays seen; Man/Director get Locked +
        // hidden; menu open or focus lost releases all. Sent on change.
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
        let (cam_pos, cam_dir, cam_fov, cam_line) = match &client_frame {
            Some(frame) => {
                let state = &frame.state;
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
                // NOTE: `D`/`A` steer the car, so they are not camera keys.
                // (`D` once opened director here and broke steering.)
                for (key, idx) in [
                    (egui::Key::Num1, 0),
                    (egui::Key::Num2, 1),
                    (egui::Key::Num3, 2),
                    (egui::Key::Num4, 3),
                    (egui::Key::Num5, 4),
                    (egui::Key::Num6, 5),
                    (egui::Key::Num7, 6),
                    (egui::Key::Num8, 7),
                ] {
                    if ctx.input(|inp| inp.key_pressed(key)) {
                        self.cam_mode = CamMode::Man;
                        self.auto_cam = None;
                        if let Some(man) = self.cam_man.as_mut() {
                            man.follow_car(state, idx);
                        }
                    }
                }
                if ctx.input(|i| i.key_pressed(egui::Key::Num9)) {
                    self.cam_mode = CamMode::Director;
                    self.auto_cam = None;
                }
                if ctx.input(|i| i.key_pressed(egui::Key::Num0)) {
                    self.cam_mode = CamMode::Man;
                    let idx = self.last_followed;
                    if let Some(man) = self.cam_man.as_mut() {
                        man.follow_car(state, idx);
                    }
                    self.auto_cam = Some(AutoCam::new(idx));
                }
                if ctx.input(|i| i.key_pressed(egui::Key::F)) {
                    self.cam_mode = if self.cam_mode == CamMode::Free {
                        CamMode::Man
                    } else {
                        CamMode::Free
                    };
                    self.auto_cam = None;
                }
                if ctx.input(|i| i.key_pressed(egui::Key::C)) {
                    self.cam_mode = CamMode::Man;
                    self.auto_cam = None;
                    if let Some(man) = self.cam_man.as_mut() {
                        man.cycle_target(state);
                    }
                }
                match self.cam_mode {
                    CamMode::Man => {
                        // Space toggles ball-cam here (free camera has no
                        // ball-cam; `A` steers and toggles nothing).
                        if ctx.input(|i| i.key_pressed(egui::Key::Space))
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
                        if !self.menu_open && !ctx.wants_pointer_input() {
                            let d = ctx.input(|i| i.pointer.delta());
                            self.freecam.look(-d.x * LOOK_SENS, -d.y * LOOK_SENS);
                            let wheel = ctx.input(|i| i.raw_scroll_delta.y);
                            self.freecam.dolly(wheel * DOLLY_SENS / 50.0);
                        }
                        let (f, r, u) = ctx.input(|i| {
                            let down = |k: egui::Key| i.key_down(k);
                            (
                                (down(egui::Key::W) as i8 - down(egui::Key::S) as i8) as f32,
                                (down(egui::Key::D) as i8 - down(egui::Key::A) as i8) as f32,
                                (down(egui::Key::Space) as i8 - (i.modifiers.shift as i8)) as f32,
                            )
                        });
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

        // Roof rule: below z 2100 the bowl draws enclosed; above it the
        // cutaway stays open. Covers car views, keeps overviews open.
        let enclosed = cam_pos.z < 2100.0;

        // Driving inputs come from this window (focused only). The driven
        // car is the directly watched car only: other views send neutral,
        // so inputs never jump between cars on a director cut. A frame is
        // sent every tick (seq keeps rising) so the sim sees a live channel.
        // Unfocused windows send neutral (no driving from a stale held key).
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
        // Unfocused: neutral controls, never a stale held key.
        let driving = spectated.is_some() && focused;
        let mut controls = CarControls::default();
        let mut buttons = 0u8;
        if driving {
            let (w, s, a, d, q, e, primary, secondary) = ctx.input(|i| {
                let down = |k: egui::Key| i.key_down(k);
                (
                    down(egui::Key::W),
                    down(egui::Key::S),
                    down(egui::Key::A),
                    down(egui::Key::D),
                    down(egui::Key::Q),
                    down(egui::Key::E),
                    i.pointer.primary_down(),
                    i.pointer.secondary_down(),
                )
            });
            let shift = ctx.input(|i| i.modifiers.shift);
            // Pointer input counts only outside UI (sliders keep their drag).
            let ui_busy = ctx.wants_pointer_input();
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
            if q {
                controls.roll -= 1.0;
            }
            if e {
                controls.roll += 1.0;
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
            if ctx.input(|i| i.key_pressed(egui::Key::Backspace)) || self.reset_requested {
                buttons |= 0b001;
            }
            if ctx.input(|i| i.key_pressed(egui::Key::F2)) {
                buttons |= 0b010;
            }
            if ctx.input(|i| i.key_pressed(egui::Key::F4)) {
                buttons |= 0b100;
            }
        }
        // Spectate-only beats everything: neutral + zeroed buttons, but the
        // frame below still sends every tick so the channel stays live.
        let (controls, buttons) =
            spectate::gate(self.settings.spectate_only, driving, controls, buttons);
        self.ctrl_seq += 1;
        self.controls.send(ControlMsg::new(
            self.driven_car,
            controls,
            buttons,
            self.ctrl_seq,
        ));
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
            egui::Window::new("settings")
                .scroll([false, true])
                .show(ctx, |ui| {
                ui.heading("Camera");
                ui.horizontal(|ui| {
                    if ui.button("Birds-eye").clicked() {
                        self.cam_mode = CamMode::Man;
                        self.auto_cam = None;
                    }
                    if ui.button("Director").clicked() {
                        self.cam_mode = CamMode::Director;
                        self.auto_cam = None;
                    }
                });
                ui.label(format!(
                    "mode: {cam_line} (keys: 1-8 follow, 9 director, 0 autocam, F free | C cycle, Space ball-cam | WASD drive)"
                ));
                ui.monospace(format!(
                    "drive: car {} (WASD+mouse in this window, F2/F4 dribble/launch)",
                    self.driven_car
                ));
                ui.monospace(format!(
                    "controls: {}",
                    if self.controls.connected() {
                        format!("live (seq {})", self.ctrl_seq)
                    } else {
                        "waiting for drive sim on 39202 — KBM is dropped until then".to_string()
                    }
                ));
                if self.cam_mode == CamMode::Free {
                    ui.monospace("FREE cam: WASD pans the camera, car coasts to neutral");
                }
                let cc = &mut settings.camera.car_cam;
                if ui
                    .add(
                        egui::Slider::new(&mut settings.camera.fov_degrees, 60.0..=130.0)
                            .text("FOV (h°)"),
                    )
                    .changed()
                    || ui
                        .add(egui::Slider::new(&mut cc.distance, 100.0..=600.0).text("distance"))
                        .changed()
                    || ui
                        .add(egui::Slider::new(&mut cc.height, 0.0..=300.0).text("height"))
                        .changed()
                    || ui
                        .add(egui::Slider::new(&mut cc.angle_deg, -30.0..=30.0).text("angle°"))
                        .changed()
                    || ui
                        .add(egui::Slider::new(&mut cc.stiffness, 0.0..=1.0).text("stiffness"))
                        .changed()
                    || ui
                        .add(
                            egui::Slider::new(&mut cc.transition_speed, 1.0..=2.0)
                                .text("transition"),
                        )
                        .changed()
                {
                    settings.mark_dirty();
                }
                if ui.button("Reset camera (RL defaults)").clicked() {
                    settings.reset_camera();
                }
                ui.horizontal_wrapped(|ui| {
                    ui.label("Presets:");
                    for preset in CameraPreset::ALL {
                        if ui.button(preset.name()).clicked() {
                            let (cc, fov) = (
                                &mut settings.camera.car_cam,
                                &mut settings.camera.fov_degrees,
                            );
                            preset.apply(cc, fov);
                            settings.mark_dirty();
                        }
                    }
                });
                egui::CollapsingHeader::new("Input")
                    .default_open(true)
                    .show(ui, |ui| {
                        if ui
                            .checkbox(&mut settings.spectate_only, "Spectate only (no inputs)")
                            .changed()
                        {
                            settings.mark_dirty();
                        }
                    });
                egui::CollapsingHeader::new("Overlays")
                    .default_open(false)
                    .show(ui, |ui| {
                        let mut touched = false;
                        touched |= ui
                            .checkbox(&mut settings.show_trail, "Ball trail")
                            .changed();
                        touched |= ui
                            .checkbox(&mut settings.show_boost, "Boost flames")
                            .changed();
                        touched |= ui
                            .checkbox(&mut settings.show_ground_line, "Ground line")
                            .changed();
                        touched |= ui.checkbox(&mut settings.show_pads, "Boost pads").changed();
                        if touched {
                            settings.mark_dirty();
                        }
                    });
                egui::CollapsingHeader::new("Display")
                    .default_open(true)
                    .show(ui, |ui| {
                        if ui
                            .add(
                                egui::Slider::new(&mut settings.fps_limit_hz, 0..=240)
                                    .text("FPS cap (0=off)"),
                            )
                            .changed()
                            || ui
                                .add(
                                    egui::Slider::new(&mut settings.ui_scale, 0.5..=2.0)
                                        .text("UI scale"),
                                )
                                .changed()
                        {
                            settings.mark_dirty();
                        }
                        ui.horizontal(|ui| {
                            if ui.checkbox(&mut settings.vsync, "VSync").changed() {
                                settings.mark_dirty();
                            }
                            if ui
                                .add(egui::Slider::new(&mut settings.msaa, 0..=8).text("MSAA"))
                                .changed()
                            {
                                settings.mark_dirty();
                            }
                        });
                        ui.label("(VSync/MSAA apply on relaunch)");
                    });
                if ui.button("Reset everything").clicked() {
                    settings.reset_all();
                }
            });
        }

        egui::Window::new("rocketsim vis_viewer").show(ctx, |ui| {
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

        egui::CentralPanel::default().show(ctx, |ui| {
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
                        let scene = guard.get_or_insert_with(|| SceneGl::new(gl, &mesh));
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
            }
        });
        self.cpu_ms = cpu_t0.elapsed().as_secs_f64() * 1000.0;
        ctx.request_repaint_after(Duration::from_millis(16));
    }
}

fn main() {
    init_meshes();
    let args: Vec<String> = std::env::args().collect();
    let port: u16 = args
        .get(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(DEFAULT_PORT);
    // Start camera: `birds` default; `followN`, `director`, `autoN`,
    // `free`. Runtime: `1-8` follow, `9` director, `0` autocam,
    // `F` free camera, `C` cycle, `Space` ball-cam.
    // Driving (WASD + mouse, Backspace/F2/F4) needs this window focused.
    // The options menu starts open (cursor free); Escape toggles it.
    let init_mode = StartMode::parse(args.get(2).map(|s| s.as_str()).unwrap_or(""));
    let menu_open = true;
    let measure_gpu = args.iter().any(|a| a == "gpushot");
    // Load settings (creates the file on first run).
    let settings = ViewerSettings::load();
    settings.save();
    let addr = format!("127.0.0.1:{port}");
    eprintln!("vis_viewer: listening on {addr} (sim should dial this addr)");

    let mesh = Arc::new(ArenaMesh::build(GameMode::Soccar));
    eprintln!(
        "vis_viewer: soccar soup={} welded={} idx={}",
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
        vsync: settings.vsync,
        multisampling: settings.msaa,
        ..Default::default()
    };
    if let Err(e) = eframe::run_native(
        "rocketsim vis_viewer",
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
            app.warm_bounds();
            Ok(Box::new(app) as _)
        }),
    ) {
        eprintln!("vis_viewer: eframe::run_native failed: {e:?}");
        std::process::exit(1);
    }
}
