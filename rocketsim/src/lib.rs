//! Simulate Rocket League games at maximum efficiency.
//!
//! RocketSim steps a full game (` [`Arena`]`) at 120 Hz using Unreal units
//! (1 uu = 1 cm in-game, see [`consts::TICK_RATE`]).
//!
//! # Quick start
//!
//! Collision meshes must be loaded once before creating an [`Arena`].
//! [`init_from_default`] loads them from `./collision_meshes/`:
//!
//! ```no_run
//! use rocketsim::{Arena, CarBodyConfig, GameMode, Team, init_from_default};
//!
//! init_from_default(true).unwrap();
//! let mut arena = Arena::new(GameMode::Soccar);
//! let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
//! arena.reset_to_random_kickoff(None);
//!
//! for _ in 0..120 {
//!     arena.step_tick();
//! }
//! ```
//!
//! # Units and coordinates
//!
//! * Positions, velocities, and dimensions are in **Unreal units (uu)** unless
//!   an item says otherwise (`BT` = Bullet physics meters, `BT_TO_UU` = 50).
//! * `+X` is the car's forward axis, `+Z` is up.
//! * Blue defends `-Y`, Orange defends `+Y` (see [`Team::get_y_dir`]).
//!
//! # Feature overview
//!
//! * [`Arena`] / [`ArenaConfig`] — create the game, add cars, step ticks.
//! * [`CarControls`] / [`CarState`] / [`BallState`] — drive cars, read state.
//! * [`ArenaEvent`] — ball/world, car/ball, car/car, boost-pickup callbacks
//!   returned by [`Arena::step_tick`] / [`Arena::get_last_step_events`].
//! * [`MutatorConfig`] / [`GameMode`] / [`Team`] — game rules.
//! * [`Arena::cast_rays`] + [`RaycastQuery`] — batched (multiples of 4 are fastest) raycasts.

#![allow(
    clippy::suboptimal_flops,
    clippy::cast_precision_loss,
    clippy::cast_possible_wrap,
    clippy::cast_possible_truncation
)]

mod base;
mod bullet;
mod glam_inc;
mod logging;
pub mod shared;
mod sim;
///////////

pub use base::*;
pub use glam_inc::*;

pub use crate::sim::*;
