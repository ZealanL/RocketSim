//! 3D visualizer for RocketSim games.
//!
//! Call [`ArenaVisExt::set_vis_enabled`] on a [`rocketsim::Arena`], then keep
//! calling [`rocketsim::Arena::step_tick`]. A window opens on a background
//! thread and follows the game (cars, ball, boost pads, ball trail).
//!
//! # Quick start
//!
//! ```no_run
//! use rocketsim::{Arena, ArenaConfig, CarBodyConfig, GameMode, Team, init_from_default};
//! use rocketsim_vis::ArenaVisExt;
//!
//! init_from_default(true).unwrap();
//! let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
//! arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
//! arena.reset_to_random_kickoff(None);
//! arena.set_vis_enabled(true);
//!
//! for _ in 0..240 {
//!     arena.step_tick();
//!     std::thread::sleep(std::time::Duration::from_millis(8));
//! }
//! ```
//!
//! # Controls
//!
//! * `C` cycles the camera: birds-eye, then each car in order.
//! * `Space` toggles ball-cam while following a car.
//!
//! See `examples/vis.rs` for a driveable interactive demo and
//! `examples/minimal.rs` for the smallest windowed sim.
//!
//! # Modules
//!
//! * [`camera`] — birds-eye / car-cam logic and tuning ([`camera::CameraConfig`]).
//! * [`backend`] — low-level render primitives ([`backend::Color`],
//!   [`backend::Elem2D`], [`backend::VisRenderState`]). Most users never touch
//!   this directly.
//! * [`vis_asset_loader`] — bundles the built-in models and textures.
//!
//! [`VisInst`] is the [`rocketsim::Vis`] implementation driven by the arena.
//! The renderer runs on its own thread; the sim only writes the latest
//! [`backend::VisRenderState`] each tick.

pub mod backend;
pub mod camera;
mod ribbon_emitter;
pub mod vis_asset_loader;
mod vis_inst;

use rocketsim::Arena;
pub use vis_inst::*;

/// Extension trait that attaches/detaches the visualizer to an arena.
///
/// Enabling visualization spawns the renderer window on a background thread.
/// Disabling it drops the window. Calling it with the current state is a no-op.
///
/// ```no_run
/// use rocketsim::{Arena, GameMode};
/// use rocketsim_vis::ArenaVisExt;
/// # rocketsim::init_from_default(true).unwrap();
/// # let mut arena = Arena::new(GameMode::Soccar);
/// arena.set_vis_enabled(true);
/// // ... step ticks ...
/// arena.set_vis_enabled(false);
/// ```
pub trait ArenaVisExt {
    /// Enable (`true`) or disable (`false`) the 3D visualizer for this arena.
    fn set_vis_enabled(&mut self, vis_enabled: bool);
}

impl ArenaVisExt for Arena {
    fn set_vis_enabled(&mut self, vis_enabled: bool) {
        match (vis_enabled, self.is_vis_enabled()) {
            (true, false) => {
                let game_mode = self.game_mode();
                self.vis = Some(Box::new(VisInst::new(game_mode)));
            }
            (false, true) => {
                self.vis = None;
            }
            _ => {}
        }
    }
}
