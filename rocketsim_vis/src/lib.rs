//! Detached 3D visualizer for RocketSim (v2 rebuild).
//!
//! The sim streams frames over TCP to the `vis_viewer` process; see
//! `tools/README.md` for the screenshot harness and stage plan.
//!
//! Quick start (old base API — launches a viewer if none listens):
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
pub mod gl_util;
pub mod lines;
pub mod lines_gl;
pub mod main_gl;
pub mod mesh;
pub mod net_client;
pub mod net_vis;
pub mod obj;
pub mod pads;
pub mod pads_gl;
pub mod photo;
pub mod proto;
pub mod settings;
pub mod textures;
pub mod triangle;
pub mod view;

pub use net_vis::{NetVis, ensure_viewer_running};
use rocketsim::Arena;

/// Default viewer address for the old [`ArenaVisExt::set_vis_enabled`] API
/// (matches [`proto::DEFAULT_PORT`]).
pub const DEFAULT_VIEWER_ADDR: &str = "127.0.0.1:39201";

/// Extension trait that attaches/detaches the detached viewer to an arena.
///
/// [`set_vis_enabled`](ArenaVisExt::set_vis_enabled) is the old base API:
/// enabling launches a viewer if none listens and streams every tick;
/// disabling detaches and closes a viewer this call opened (never one the
/// user started themselves). [`set_net_vis`](ArenaVisExt::set_net_vis)
/// skips the auto-launch and streams to an explicit address.
pub trait ArenaVisExt {
    /// Old base API: enable (`true`) or disable (`false`) the 3D visualizer.
    /// Same signature as the classic in-process vis; here it drives the
    /// detached viewer on [`DEFAULT_VIEWER_ADDR`].
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
}
