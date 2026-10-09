//! Detached viewer binary (v2 rebuild, stage 3: first triangle).
//!
//! One eframe/glow window, one paint callback, one triangle on black plus a
//! stats overlay. Frame pacing is `request_repaint_after(16ms)` — the 60fps
//! cap is a day-one rule, not a retrofit (v1 spun uncapped and ate a core).

use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use rocketsim_vis::triangle::TriangleGl;

struct ViewerApp {
    frames: u64,
    /// Process-lifetime triangle (created once on the GL thread; the context
    /// owns it until process exit, like all later static GL objects).
    tri: Arc<Mutex<Option<TriangleGl>>>,
    /// Last triangle draw time in ms (written by the GL callback).
    last_draw_ms: Arc<Mutex<f64>>,
    /// Explicit frame limiter state (`request_repaint_after` alone did not
    /// pace the loop on this backend — measured 310fps with it).
    last_frame: Instant,
    fps_ema: f32,
}

/// 60fps frame budget. Both the timer request and the explicit sleep below
/// enforce it; the sleep is the one that actually held on every backend.
const FRAME_BUDGET: Duration = Duration::from_millis(16);

impl ViewerApp {
    fn new() -> Self {
        Self {
            frames: 0,
            tri: Arc::new(Mutex::new(None)),
            last_draw_ms: Arc::new(Mutex::new(0.0)),
            last_frame: Instant::now(),
            fps_ema: 60.0,
        }
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Explicit limiter: sleep the remainder of the 16ms budget.
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
        let tri = self.tri.clone();
        let last_draw_ms = self.last_draw_ms.clone();

        egui::Window::new("rocketsim vis_viewer").show(ctx, |ui| {
            ui.label("stage 3 — first triangle (no scene yet)");
            ui.monospace(format!("frames drawn: {frames}"));
            ui.monospace(format!("fps: {:.0} (cap 60)", self.fps_ema));
            let ms = last_draw_ms.lock().map(|g| *g).unwrap_or(-1.0);
            ui.monospace(format!("triangle draw: {ms:.3} ms"));
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
                        // GL origin is bottom-left; egui points are top-left.
                        let x_px = (info.viewport.min.x * ppp).round() as i32;
                        let y_px = (info.screen_size_px[1] as f32 - info.viewport.max.y * ppp)
                            .round() as i32;
                        let t0 = Instant::now();
                        let mut guard = tri.lock().expect("viewer triangle poisoned");
                        let tri = guard.get_or_insert_with(|| TriangleGl::new(gl));
                        tri.draw(gl, (x_px, y_px, w_px, h_px));
                        drop(guard);
                        if let Ok(mut g) = last_draw_ms.lock() {
                            *g = t0.elapsed().as_secs_f64() * 1000.0;
                        }
                    })),
                };
                ui.painter().add(cb);
            }
        });
        // 60fps cap: repaint on a timer, never a hot loop.
        ctx.request_repaint_after(Duration::from_millis(16));
    }
}

fn main() {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1280.0, 720.0]),
        renderer: eframe::Renderer::Glow,
        ..Default::default()
    };
    if let Err(e) = eframe::run_native(
        "rocketsim vis_viewer",
        options,
        Box::new(|_cc| Ok(Box::new(ViewerApp::new()) as _)),
    ) {
        eprintln!("vis_viewer: eframe::run_native failed: {e:?}");
        std::process::exit(1);
    }
}
