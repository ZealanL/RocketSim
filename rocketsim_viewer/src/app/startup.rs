use rocketsim::GameMode;

use super::*;

/// Command line: `rocketsim-viewer [PORT] [CAM] [-g MODE] [--gpushot]`.
/// PORT/CAM stay positional (pre-clap usage keeps working).
#[derive(clap::Parser)]
#[command(name = "rocketsim-viewer", version)]
pub(super) struct Cli {
    /// TCP port the viewer listens on (the sim dials this).
    #[arg(default_value_t = DEFAULT_PORT)]
    port: u16,
    /// Address the viewer binds (default: 127.0.0.1).
    #[arg(long, default_value = "127.0.0.1", value_parser = clap::value_parser!(std::net::IpAddr))]
    bind: std::net::IpAddr,
    /// Start camera: birds|followN|director|autoN|free.
    /// Overrides the camera saved in settings.toml.
    #[arg(value_parser = parse_camera)]
    cam: Option<StartMode>,
    /// Initial placeholder game mode. Stream metadata replaces it on connect.
    #[arg(short = 'g', long = "game-mode", default_value = "soccar", value_parser = parse_mode)]
    game_mode: GameMode,

    /// GPU-stall timing for profiling.
    #[arg(long)]
    gpushot: bool,
    /// Write debug-render timing and upload counters as JSON. Enables GPU timer queries.
    #[arg(long)]
    debug_stats: Option<std::path::PathBuf>,
    /// Start with the settings menu closed.
    #[arg(long)]
    no_menu: bool,
    /// Hide the ball mesh without changing physics or camera tracking.
    #[arg(long)]
    hide_ball: bool,
}

pub(super) fn dropshot_tile_states(
    state: &rocketsim::ArenaState,
) -> Option<&rocketsim::TileStates> {
    if state.game_mode() == GameMode::Dropshot {
        state.tile_states.as_ref()
    } else {
        None
    }
}

pub(super) fn parse_mode(s: &str) -> Result<GameMode, String> {
    parse_game_mode(s).ok_or_else(|| {
        format!("unknown game mode '{s}' (soccar|hoops|heatseeker|snowday|dropshot|thevoid|void)")
    })
}

pub(super) fn parse_camera(s: &str) -> Result<StartMode, String> {
    StartMode::try_parse(s)
        .ok_or_else(|| format!("invalid camera '{s}' (birds|followN|director|autoN|free)"))
}

/// Choose the start camera and the initial menu state.
///
/// An explicit CLI camera wins. An absent or invalid saved value falls back
/// to birds. `--no-menu` closes the menu even when the saved state is open.
fn initial_state(
    settings: &ViewerSettings,
    cli_cam: Option<StartMode>,
    no_menu: bool,
) -> (StartMode, bool) {
    let camera = cli_cam.unwrap_or_else(|| {
        StartMode::try_parse(settings.ui.start_camera.trim()).unwrap_or(StartMode::Birds)
    });
    (camera, !no_menu && settings.ui.menu_open)
}

pub(super) fn metadata_changed(
    previous: Option<&Arc<rocketsim_vis::proto::SceneMetadata>>,
    current: &Arc<rocketsim_vis::proto::SceneMetadata>,
) -> bool {
    previous.is_none_or(|previous| !Arc::ptr_eq(previous, current))
}

pub(crate) fn run() {
    let mut args: Vec<String> = std::env::args().collect();
    let measure_gpu_compat = args.iter().any(|a| a == "gpushot");
    args.retain(|a| a != "gpushot");
    let cli = Cli::parse_from(args);
    let port = cli.port;
    // The `CAM` option overrides the saved start camera. Runtime camera and
    // driving inputs use the remappable action map. The menu state is saved.
    let settings = ViewerSettings::load();
    let (init_mode, menu_open) = initial_state(&settings, cli.cam, cli.no_menu);
    // `gpushot` also accepted bare (pre-clap usage) anywhere in argv.
    let measure_gpu = cli.gpushot || measure_gpu_compat;
    let addr = std::net::SocketAddr::new(cli.bind, port).to_string();
    eprintln!("rocketsim-viewer: listening on {addr} (sim should dial this addr)");

    let mesh = Arc::new(ArenaMesh::placeholder(cli.game_mode));
    let listener = ViewerListener::try_bind(&addr).unwrap_or_else(|e| {
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
            let mut app = ViewerApp::new(
                mesh.clone(),
                listener,
                init_mode,
                settings,
                menu_open,
                measure_gpu,
            );
            app.hide_ball = cli.hide_ball;
            if let Some(path) = cli.debug_stats {
                app.debug_metrics = Some(debug_metrics::DebugMetrics::new(path));
            }
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
    fn cli_options_bind_and_camera_cases_parse_together() {
        let cli = Cli::try_parse_from([
            "rocketsim-viewer",
            "--hide-ball",
            "--no-menu",
            "--gpushot",
            "-g",
            "hoops",
            "--bind",
            "::1",
            "1234",
            "follow7",
        ])
        .unwrap();
        assert!(cli.hide_ball && cli.no_menu && cli.gpushot);
        assert_eq!(cli.game_mode, GameMode::Hoops);
        assert_eq!(
            std::net::SocketAddr::new(cli.bind, cli.port).to_string(),
            "[::1]:1234"
        );
        assert!(matches!(cli.cam, Some(StartMode::Follow(7))));

        assert_eq!(
            initial_state(&ViewerSettings::default(), cli.cam, cli.no_menu),
            (StartMode::Follow(7), false)
        );

        for (camera, valid) in [
            ("follow7", true),
            ("auto0", true),
            ("follow8", true),
            ("follow1000000000", true),
            ("follow999999999999999999999999999999", false),
            ("unknown", false),
        ] {
            assert_eq!(parse_camera(camera).is_ok(), valid, "{camera}");
        }
    }

    #[test]
    fn initial_state_prefers_cli_then_saved_settings() {
        let mut settings = ViewerSettings::default();
        assert_eq!(
            initial_state(&settings, None, false),
            (StartMode::Birds, false)
        );

        settings.ui.start_camera = "follow3".into();
        settings.ui.menu_open = true;
        assert_eq!(
            initial_state(&settings, None, false),
            (StartMode::Follow(3), true)
        );
        assert_eq!(
            initial_state(&settings, Some(StartMode::Free), false),
            (StartMode::Free, true)
        );
        assert_eq!(
            initial_state(&settings, None, true),
            (StartMode::Follow(3), false)
        );

        settings.ui.start_camera = "bad".into();
        assert_eq!(initial_state(&settings, None, false).0, StartMode::Birds);
    }
}
