//! Detached viewer binary (v2 rebuild, stage 1: empty window + overlay).
//!
//! Establishes the eframe/glow window, CLI shape, and screenshot path with
//! zero 3D risk: no scene, no net, just a window with one egui panel.
//! Later stages add: TCP frames (2), triangle (3), arena (4), cars (5)…

struct ViewerApp {
    frames: u64,
}

impl ViewerApp {
    fn new() -> Self {
        Self { frames: 0 }
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.frames += 1;
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("rocketsim vis_viewer");
            ui.label("stage 1 — empty window + overlay (no scene yet)");
            ui.separator();
            ui.monospace(format!("frames drawn: {}", self.frames));
            // The viewer does not step the sim yet (stage 2 wires TCP frames).
        });
        // No ctx.request_repaint(): repaint on demand only, so an idle viewer
        // costs ~0 CPU (the v1 viewer spun uncapped and ate a core).
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
