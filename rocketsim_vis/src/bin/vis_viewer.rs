//! Detached viewer binary (v2 rebuild, stage 4: arena mesh).
//!
//! Fixed-pose beauty shot of the welded Soccar arena, unlit flat color.
//! 60fps cap via explicit 16ms limiter (backend timer alone did not pace).

use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use rocketsim::GameMode;
use rocketsim_vis::{arena_gl::ArenaGl, mesh::ArenaMesh, photo::init_meshes, view};

struct ViewerApp {
    frames: u64,
    /// CPU-side welded mesh (built once at startup, uploaded on the GL thread).
    mesh: Arc<ArenaMesh>,
    arena: Arc<Mutex<Option<ArenaGl>>>,
    last_draw_ms: Arc<Mutex<f64>>,
    last_frame: Instant,
    fps_ema: f32,
    /// Welded counts for the overlay (measured once at startup).
    soup_verts: usize,
    welded_verts: usize,
    n_indices: usize,
}

/// 60fps frame budget.
const FRAME_BUDGET: Duration = Duration::from_millis(16);

impl ViewerApp {
    fn new(mesh: Arc<ArenaMesh>) -> Self {
        let (soup, welded, indices) = (mesh.soup_verts, mesh.welded_verts(), mesh.index_count());
        Self {
            frames: 0,
            mesh,
            arena: Arc::new(Mutex::new(None)),
            last_draw_ms: Arc::new(Mutex::new(0.0)),
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
        let (arena, mesh, last_draw_ms) = (
            self.arena.clone(),
            self.mesh.clone(),
            self.last_draw_ms.clone(),
        );
        let (soup, welded, indices, fps) = (
            self.soup_verts,
            self.welded_verts,
            self.n_indices,
            self.fps_ema,
        );

        egui::Window::new("rocketsim vis_viewer").show(ctx, |ui| {
            ui.label("stage 4 — arena mesh (flat, fixed pose)");
            ui.monospace(format!("soccar soup={soup} welded={welded} idx={indices}"));
            ui.monospace(format!("frames drawn: {frames}"));
            ui.monospace(format!("fps: {fps:.0} (cap 60)"));
            let ms = last_draw_ms.lock().map(|g| *g).unwrap_or(-1.0);
            ui.monospace(format!("arena draw: {ms:.3} ms"));
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
                        let mvp = view::arena_pose_mvp(aspect);
                        let t0 = Instant::now();
                        let mut guard = arena.lock().expect("viewer arena poisoned");
                        let arena = guard.get_or_insert_with(|| ArenaGl::new(gl, &mesh));
                        arena.draw(gl, (x_px, y_px, w_px, h_px), &mvp);
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
    let mesh = Arc::new(ArenaMesh::build(GameMode::Soccar));
    eprintln!(
        "vis_viewer: soccar soup={} welded={} idx={}",
        mesh.soup_verts,
        mesh.welded_verts(),
        mesh.index_count()
    );

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
        Box::new(move |_cc| Ok(Box::new(ViewerApp::new(mesh.clone())) as _)),
    ) {
        eprintln!("vis_viewer: eframe::run_native failed: {e:?}");
        std::process::exit(1);
    }
}
