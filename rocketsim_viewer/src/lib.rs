//! Detached 3D visualizer for RocketSim.
//!
//! The sim streams frames over TCP to the `rocketsim-viewer` process. Network
//! types (`ArenaVisExt`, `NetVis`, `proto`, ...) live in the lightweight
//! `rocketsim_vis` crate.
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

mod app;
mod debug_metrics;
mod session;
mod ui;

pub mod camera;
pub mod effects;
pub mod geometry;
pub mod input;
pub mod render;
pub mod settings;

/// Start the viewer with command-line options.
pub fn run() {
    app::run();
}
