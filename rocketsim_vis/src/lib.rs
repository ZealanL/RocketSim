//! Send RocketSim state to a separate viewer process.
//!
//! The viewer displays frames and sends optional controls. The simulation
//! applies controls and action requests. The viewer does not run the physics.
//!
//! Use [`ArenaVisExt`] for normal simulation applications. Use
//! [`transport::viewer::ViewerListener`] to build a viewer. Use [`proto`] to
//! encode and decode messages. Use [`debug`] to draw retained scene data.
//!
//! The sender keeps the latest physics frame. A slow viewer can miss frames.
//! Retained groups and resources use reliable full and delta updates. They stay
//! available while the viewer is disconnected or the simulation is paused.
//!
//! World positions and dimensions use Unreal units. There are 50 Unreal units
//! per meter. `+Z` points up. This protocol has no authentication or encryption.
//!
//! # Attach a viewer
//!
//! Install `rocketsim-viewer` separately. Put its executable on `PATH`, or set
//! `ROCKETSIM_VIEWER` to its path. Supply collision meshes before you create a
//! game mode that needs them.
//!
//! ```no_run
//! use rocketsim::{Arena, GameMode};
//! use rocketsim_vis::ArenaVisExt;
//!
//! let mut arena = Arena::new(GameMode::TheVoid);
//! arena.try_set_vis_enabled(true)?;
//! arena.step_tick();
//! arena.try_set_vis_enabled(false)?;
//! # Ok::<(), std::io::Error>(())
//! ```
//!
//! Each arena accepts one viewer address. Use a separate process and port for
//! each arena. Use numeric `IP:port` addresses. Remote connections are not
//! encrypted or authenticated.
//!
//! # Apply controls
//!
//! Read controls before each simulation tick. Check `car_index` before use.
//! Use neutral controls when no fresh input is available. Read action requests
//! with [`ArenaVisExt::take_vis_actions`]. The application performs each action.
//!
//! # Draw retained scene data
//!
//! Get a [`debug::DebugRenderer`] from [`ArenaVisExt::debug_renderer`]. Use
//! named groups for persistent drawings. Use `Lifetime::Frame` for a drawing
//! that lasts through one displayed update. See [`debug`] for examples.
//!
//! # Run at a fixed rate
//!
//! [`Arena::step_tick`](rocketsim::Arena::step_tick) advances the simulation by `1/120` second. It does not
//! wait for real time. Use [`pace::TickPacer`] for interactive applications.
//! Omit the wait for batch simulation.
//!
//! [`Arena`]: rocketsim::Arena
//! [`Lifetime`]: debug::Lifetime

#![warn(missing_docs)]

// Compile transport tracing only into unit-test executables.
macro_rules! test_trace {
    ($($arg:tt)*) => {
        #[cfg(test)]
        crate::test_trace(format_args!($($arg)*));
        #[cfg(not(test))]
        if false {
            let _ = format_args!($($arg)*);
        }
    };
}

#[cfg(test)]
fn test_trace(message: std::fmt::Arguments<'_>) {
    use std::{io::Write, sync::OnceLock, time::Instant};

    static ENABLED: OnceLock<bool> = OnceLock::new();
    if !*ENABLED
        .get_or_init(|| std::env::var_os("ROCKETSIM_TEST_TRACE").is_some_and(|value| value == "1"))
    {
        return;
    }
    static START: OnceLock<Instant> = OnceLock::new();
    let elapsed = START.get_or_init(Instant::now).elapsed();
    let thread = std::thread::current();
    // Write directly to stderr so libtest cannot hide progress from a blocked test.
    let _ = writeln!(
        std::io::stderr().lock(),
        "[vis-test {:.3}s pid={} {:?} {}] {message}",
        elapsed.as_secs_f64(),
        std::process::id(),
        thread.id(),
        thread.name().unwrap_or("worker"),
    );
}

pub mod arena;
pub mod debug;
pub mod launcher;
pub mod mutators;

pub mod pace;
pub mod proto;
pub mod transport;

mod sync;

pub use arena::ArenaVisExt;
pub use launcher::{ViewerHandle, ensure_viewer_running, try_ensure_viewer_running};
use rocketsim::GameMode;
pub use transport::simulation::NetVis;

/// Default numeric address for a viewer started by [`ArenaVisExt`].
///
/// The viewer listens on `127.0.0.1:39201`. Use
/// [`ArenaVisExt::try_set_net_vis`] to select another address.
pub const DEFAULT_VIEWER_ADDR: &str = "127.0.0.1:39201";

/// Return the lowercase viewer name for a game mode.
///
/// ```
/// use rocketsim::GameMode;
/// use rocketsim_vis::game_mode_name;
///
/// assert_eq!(game_mode_name(GameMode::TheVoid), "thevoid");
/// ```
pub fn game_mode_name(mode: GameMode) -> &'static str {
    match mode {
        GameMode::Soccar => "soccar",
        GameMode::Hoops => "hoops",
        GameMode::Heatseeker => "heatseeker",
        GameMode::Snowday => "snowday",
        GameMode::Dropshot => "dropshot",
        GameMode::TheVoid => "thevoid",
    }
}

/// Parse a game-mode name without case sensitivity.
///
/// Accept `soccar`, `hoops`, `heatseeker`, `snowday`, `dropshot`, and `thevoid`.
/// Accept `void` as a second name for `TheVoid`. Do not trim whitespace.
/// Return `None` for an unknown name.
///
/// ```
/// use rocketsim::GameMode;
/// use rocketsim_vis::parse_game_mode;
///
/// assert_eq!(parse_game_mode("Hoops"), Some(GameMode::Hoops));
/// assert_eq!(parse_game_mode("void"), Some(GameMode::TheVoid));
/// assert_eq!(parse_game_mode("unknown"), None);
/// ```
pub fn parse_game_mode(text: &str) -> Option<GameMode> {
    match text.to_lowercase().as_str() {
        "soccar" => Some(GameMode::Soccar),
        "hoops" => Some(GameMode::Hoops),
        "heatseeker" => Some(GameMode::Heatseeker),
        "snowday" => Some(GameMode::Snowday),
        "dropshot" => Some(GameMode::Dropshot),
        "thevoid" | "void" => Some(GameMode::TheVoid),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn game_mode_names_parse_without_case_sensitivity() {
        for mode in [
            GameMode::Soccar,
            GameMode::Hoops,
            GameMode::Heatseeker,
            GameMode::Snowday,
            GameMode::Dropshot,
            GameMode::TheVoid,
        ] {
            assert_eq!(parse_game_mode(game_mode_name(mode)), Some(mode));
        }
        assert_eq!(parse_game_mode("void"), Some(GameMode::TheVoid));
        assert_eq!(parse_game_mode(" unknown"), None);
    }
}
