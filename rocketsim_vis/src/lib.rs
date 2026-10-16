//! Detached-viewer network interface for RocketSim (no window, no renderer).
//!
//! The sim streams frames over TCP to the `rocketsim-viewer` process (see the
//! `rocketsim_viewer` crate for the viewer itself); the viewer streams driving
//! inputs back. Sim crates depend on this crate, not on `rocketsim_viewer`,
//! so they never compile the GUI stack.
//!
//! Quick start (launches a viewer if none listens):
//!
//! ```no_run
//! use rocketsim::{Arena, CarBodyConfig, GameMode, Team, init_from_default};
//! use rocketsim_vis::ArenaVisExt;
//!
//! init_from_default(true).unwrap();
//! let mut arena = Arena::new(GameMode::Soccar);
//! arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
//! arena.reset_to_random_kickoff(None);
//! arena.set_vis_enabled(true);
//!
//! for _ in 0..240 {
//!     arena.step_tick();
//!     std::thread::sleep(std::time::Duration::from_millis(8));
//! }
//! arena.set_vis_enabled(false);
//! ```

pub mod mutators;
pub mod net_client;
pub mod net_vis;
pub mod pace;
pub mod photo;
pub mod proto;

pub use net_vis::{NetVis, ensure_viewer_running};
use rocketsim::{Arena, GameMode};

/// Default viewer address for [`ArenaVisExt::set_vis_enabled`]
/// (matches [`proto::DEFAULT_PORT`]).
pub const DEFAULT_VIEWER_ADDR: &str = "127.0.0.1:39201";

/// Canonical lowercase name for a [`GameMode`], inverse of
/// [`parse_game_mode`] (used in log lines).
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
/// Parses an arena game mode name (case-insensitive): `soccar`, `hoops`,
/// `heatseeker`, `snowday`, `dropshot`, `thevoid`, or `void`. Shared by the
/// viewer binary and the sim examples so `-g hoops` means the same
/// everywhere; `None` for anything else.
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

/// Extension trait that attaches/detaches the detached viewer to an arena.
///
/// [`set_vis_enabled`](ArenaVisExt::set_vis_enabled) launches a viewer if
/// none listens and streams every tick; disabling detaches and closes a
/// viewer this call opened (never one the user started).
pub trait ArenaVisExt {
    /// Enable (`true`) or disable (`false`) the 3D visualizer on
    /// [`DEFAULT_VIEWER_ADDR`].
    fn set_vis_enabled(&mut self, vis_enabled: bool) {
        if vis_enabled {
            ensure_viewer_running(DEFAULT_VIEWER_ADDR);
            self.set_net_vis(Some(DEFAULT_VIEWER_ADDR));
        } else {
            self.set_net_vis(None);
            net_vis::stop_spawned_viewer();
        }
    }

    /// Enable streaming to the viewer at `addr` (e.g. `"127.0.0.1:39201"`),
    /// or disable it with `None`. No auto-launch (start the viewer yourself).
    fn set_net_vis(&mut self, addr: Option<&str>);

    /// Convenience: `set_net_vis(Some(addr))`.
    fn set_net_vis_enabled(&mut self, addr: &str) {
        self.set_net_vis(Some(addr));
    }

    /// Convenience: `set_net_vis(None)`.
    fn set_net_vis_disabled(&mut self) {
        self.set_net_vis(None);
    }
}

impl ArenaVisExt for Arena {
    fn set_net_vis(&mut self, addr: Option<&str>) {
        match (addr, self.is_vis_enabled()) {
            (Some(addr), false) => {
                self.vis = Some(Box::new(NetVis::connect(addr)));
            }
            (None, true) => {
                self.vis = None;
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_addr_matches_port() {
        let port: u16 = DEFAULT_VIEWER_ADDR
            .rsplit(':')
            .next()
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(port, proto::DEFAULT_PORT);
    }

    #[test]
    fn old_api_toggles_vis() {
        // Best-effort spawn (no viewer binary beside the test harness):
        // either way the arena attach/detach must flip cleanly, never panic.
        crate::photo::init_meshes();
        let mut arena = crate::photo::build_arena();
        assert!(!arena.is_vis_enabled());
        arena.set_vis_enabled(true);
        assert!(arena.is_vis_enabled());
        arena.set_vis_enabled(false);
        assert!(!arena.is_vis_enabled());
    }

    #[test]
    fn game_mode_names_parse() {
        assert_eq!(parse_game_mode("soccar"), Some(GameMode::Soccar));
        assert_eq!(parse_game_mode("Hoops"), Some(GameMode::Hoops));
        assert_eq!(parse_game_mode("HEATSEEKER"), Some(GameMode::Heatseeker));
        assert_eq!(parse_game_mode("snowday"), Some(GameMode::Snowday));
        assert_eq!(parse_game_mode("dropshot"), Some(GameMode::Dropshot));
        assert_eq!(parse_game_mode("thevoid"), Some(GameMode::TheVoid));
        assert_eq!(parse_game_mode("void"), Some(GameMode::TheVoid));
        assert_eq!(parse_game_mode("soccer"), None);
        assert_eq!(parse_game_mode(""), None);
    }
}
