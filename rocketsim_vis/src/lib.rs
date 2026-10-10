//! Detached 3D visualizer for RocketSim.
//!
//! The sim streams frames over TCP to the `vis_viewer` process; see
//! `tools/README.md` for the screenshot harness. Network types
//! (`ArenaVisExt`, `NetVis`, `proto`, `bindings`, ...) live in the
//! lightweight [`rocketsim_vis_net`] crate and are re-exported here, so
//! existing `rocketsim_vis::` imports keep compiling unchanged.
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

pub mod arena_gl;
pub mod camera;
pub mod frustum;
pub mod fx;
pub mod gl_util;
pub mod hud;
pub mod lines;
pub mod lines_gl;
pub mod main_gl;
pub mod mesh;
pub mod obj;
pub mod pads;
pub mod pads_gl;
pub mod settings;
pub mod textures;
pub mod triangle;
pub mod view;

// Network interface (re-exported so `rocketsim_vis::proto`, ... keep
// resolving; new code can depend on `rocketsim_vis_net` directly).
pub use rocketsim_vis_net::{
    ArenaVisExt, DEFAULT_VIEWER_ADDR, NetVis, bindings, ensure_viewer_running, mutators,
    net_client, net_vis, pace, photo, proto, spectate,
};

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
}
