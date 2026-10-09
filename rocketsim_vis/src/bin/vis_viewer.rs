//! Detached viewer binary (v2 rebuild, stage 5b: old-look live scene).
//!
//! Fixed-pose beauty shot: team-grid arena + textured ball/cars streamed
//! from the sim over TCP (latest frame, no interpolation — matches the old
//! in-process look). 60fps cap via explicit 16ms limiter.
//!
//! Usage: `vis_viewer [PORT]` (default 39201).

use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use glam::{Mat3, Vec3, Vec3A};
use rocketsim::GameMode;
use rocketsim_vis::{
    arena_gl::ArenaGl,
    frustum::frustum_planes,
    lines::{RibbonConfig, RibbonEmitter, Segment, ground_line},
    lines_gl::LinesGl,
    main_gl::{
        BALL_MODEL, BALL_TEXTURE, MainProgram, ModelGl, car_model_name, car_texture_name,
        model_matrix,
    },
    mesh::ArenaMesh,
    net_client::{ConnStatus, FrameClient},
    obj::ObjMesh,
    pads::{build_instances, load_pad_meshes, variant_radius},
    pads_gl::PadsGl,
    photo::init_meshes,
    proto::DEFAULT_PORT,
    textures::{PngImage, TextureGl},
    view,
};

/// Owned GL scene: arena + main pipeline + pads + all models/textures.
/// Built once on the GL thread; reuses across frames.
struct SceneGl {
    arena: ArenaGl,
    prog_main: MainProgram,
    pads: PadsGl,
    pad_radii: [f32; 4],
    lines: LinesGl,
    models: HashMap<String, ModelGl>,
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
        for (name, obj) in [
            (BALL_MODEL, include_str!("../models/ball.obj")),
            ("car_octane", include_str!("../models/car_octane.obj")),
            ("car_dominus", include_str!("../models/car_dominus.obj")),
            ("car_breakout", include_str!("../models/car_breakout.obj")),
            ("car_merc", include_str!("../models/car_merc.obj")),
        ] {
            models.insert(name.to_string(), ModelGl::new(gl, &ObjMesh::load(obj)));
        }
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
        Self {
            arena,
            prog_main,
            pads,
            pad_radii,
            lines,
            models,
            textures,
        }
    }

    // NOTE: no destroy() — the scene lives with the process (stage 8 adds
    // the mode-switch path that needs teardown).
}

struct ViewerApp {
    frames: u64,
    mesh: Arc<ArenaMesh>,
    scene: Arc<Mutex<Option<SceneGl>>>,
    client: FrameClient,
    last_draw_ms: Arc<Mutex<f64>>,
    /// (pads drawn, pads frustum-culled) from the GL callback.
    pads_info: Arc<Mutex<(usize, usize)>>,
    /// (line segments drawn, truncated) from the GL callback.
    lines_info: Arc<Mutex<(usize, usize)>>,
    /// Trail state (CPU-side, updated per viewer frame from sim ticks).
    ball_ribbon: RibbonEmitter,
    boost_ribbons: Vec<RibbonEmitter>,
    /// Last sim tick consumed (replays missed ticks into the ribbons).
    last_tick: Option<u64>,
    /// Line segments for the GL callback (built on the UI thread).
    pending_segs: Arc<Mutex<Vec<Segment>>>,
    last_frame: Instant,
    fps_ema: f32,
    soup_verts: usize,
    welded_verts: usize,
    n_indices: usize,
}

/// 60fps frame budget.
const FRAME_BUDGET: Duration = Duration::from_millis(16);

impl ViewerApp {
    fn new(mesh: Arc<ArenaMesh>, client: FrameClient) -> Self {
        let (soup, welded, indices) = (mesh.soup_verts, mesh.welded_verts(), mesh.index_count());
        Self {
            frames: 0,
            mesh,
            scene: Arc::new(Mutex::new(None)),
            client,
            last_draw_ms: Arc::new(Mutex::new(0.0)),
            pads_info: Arc::new(Mutex::new((0, 0))),
            lines_info: Arc::new(Mutex::new((0, 0))),
            ball_ribbon: RibbonEmitter::new(RibbonConfig::ball_trail()),
            boost_ribbons: Vec::new(),
            last_tick: None,
            pending_segs: Arc::new(Mutex::new(Vec::new())),
            last_frame: Instant::now(),
            fps_ema: 60.0,
            soup_verts: soup,
            welded_verts: welded,
            n_indices: indices,
        }
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let now = Instant::now();
        let since_last = now.duration_since(self.last_frame);
        if since_last < FRAME_BUDGET {
            std::thread::sleep(FRAME_BUDGET - since_last);
        }
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
        let (soup, welded, indices, fps) = (
            self.soup_verts,
            self.welded_verts,
            self.n_indices,
            self.fps_ema,
        );
        let (status, received) = (self.client.status(), self.client.received());
        let tick = client_frame.as_ref().map(|f| f.state.tick_count);

        // Advance trails in SIM time: replay ticks missed since the last
        // viewer frame (clamped — a reset jump must not spiral emitters).
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
            let mut segs = self.ball_ribbon.segments(ball_pos);
            let floor_z = rocketsim::consts::arena::get_aabb(frame.state.game_mode())
                .min
                .z;
            segs.push(ground_line(ball_pos, floor_z));
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
                segs.extend(self.boost_ribbons[i].segments(emit_pos));
            }
            *self.pending_segs.lock().expect("viewer segs poisoned") = segs;
        }

        egui::Window::new("rocketsim vis_viewer").show(ctx, |ui| {
            ui.label("stage 7 — lines and trails (fixed pose)");
            ui.monospace(format!("soccar soup={soup} welded={welded} idx={indices}"));
            match &status {
                ConnStatus::Waiting => ui.monospace("sim: waiting".to_string()),
                ConnStatus::Connected { addr } => ui.monospace(format!(
                    "sim: {addr} tick={} frames={received}",
                    tick.unwrap_or(0)
                )),
            };
            ui.monospace(format!("frames drawn: {frames}"));
            ui.monospace(format!("fps: {fps:.0} (cap 60)"));
            let ms = last_draw_ms.lock().map(|g| *g).unwrap_or(-1.0);
            ui.monospace(format!("scene draw: {ms:.3} ms"));
            let (drawn, culled) = pads_info.lock().map(|g| *g).unwrap_or((0, 0));
            ui.monospace(format!("pads: {drawn} drawn, {culled} culled"));
            let (nlines, ltrunc) = lines_info.lock().map(|g| *g).unwrap_or((0, 0));
            ui.monospace(format!("lines: {nlines} ({ltrunc} truncated)"));
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
                        let (view_mat, proj_mat) = view::arena_pose_view_proj(aspect);
                        let t0 = Instant::now();
                        let mut guard = scene.lock().expect("viewer scene poisoned");
                        let scene = guard.get_or_insert_with(|| SceneGl::new(gl, &mesh));
                        scene.arena.draw(
                            gl,
                            (x_px, y_px, w_px, h_px),
                            &glam::Mat4::IDENTITY,
                            &view_mat,
                            &proj_mat,
                        );
                        scene.prog_main.set_view_proj(gl, &view_mat, &proj_mat);
                        if let Some(frame) = &client_frame {
                            // Ball (old vis: textured, pos + rot).
                            let ball = &frame.state.ball.phys;
                            scene.prog_main.draw(
                                gl,
                                &scene.models[BALL_MODEL],
                                &scene.textures[BALL_TEXTURE],
                                &model_matrix(Vec3::from(ball.pos), Mat3::from(ball.rot_mat)),
                            );
                            // Cars (old vis: per-body model, team texture, skip demoed).
                            for (info, state) in &frame.state.cars {
                                if state.is_demoed {
                                    continue;
                                }
                                let model = &scene.models[car_model_name(&info.config)];
                                let texture = &scene.textures[car_texture_name(info.team)];
                                scene.prog_main.draw(
                                    gl,
                                    model,
                                    texture,
                                    &model_matrix(
                                        Vec3::from(state.phys.pos),
                                        Mat3::from(state.phys.rot_mat),
                                    ),
                                );
                            }
                        }
                        scene.prog_main.unbind(gl);
                        // Pads: frustum-filtered instances, 4 instanced draws.
                        let planes = frustum_planes(proj_mat * view_mat);
                        let empty: [Vec<glam::Mat4>; 4] = Default::default();
                        let (instances, culled) = match &client_frame {
                            Some(frame) => {
                                build_instances(&frame.state.boost_pads, &planes, &scene.pad_radii)
                            }
                            None => (empty, 0),
                        };
                        scene.pads.upload(gl, &instances);
                        scene.pads.draw(gl, &view_mat, &proj_mat);
                        if let Ok(mut info) = pads_info.lock() {
                            *info = (instances.iter().map(|v| v.len()).sum(), culled);
                        }
                        // Lines last (transparent): CPU-expanded tight quads.
                        let segs = std::mem::take(
                            &mut *pending_segs.lock().expect("viewer segs poisoned"),
                        );
                        // Eye for expansion = fixed-pose camera position
                        // (view inverse translation); proj11 + height
                        // drive min-width.
                        let eye = Vec3A::from(view::arena_pose().0);
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
                        drop(guard);
                        if let Ok(mut g) = last_draw_ms.lock() {
                            *g = t0.elapsed().as_secs_f64() * 1000.0;
                        }
                    })),
                };
                ui.painter().add(cb);
            }
        });
        ctx.request_repaint_after(Duration::from_millis(16));
    }
}

fn main() {
    init_meshes();
    let port: u16 = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(DEFAULT_PORT);
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

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1280.0, 720.0]),
        renderer: eframe::Renderer::Glow,
        // Load-bearing: eframe defaults to ZERO depth bits (no depth buffer),
        // which silently turns every depth call into a no-op and reduces all
        // draws to last-wins painter's order. The 3D view needs real depth.
        depth_buffer: 24,
        ..Default::default()
    };
    if let Err(e) = eframe::run_native(
        "rocketsim vis_viewer",
        options,
        Box::new(move |_cc| Ok(Box::new(ViewerApp::new(mesh.clone(), client)) as _)),
    ) {
        eprintln!("vis_viewer: eframe::run_native failed: {e:?}");
        std::process::exit(1);
    }
}
