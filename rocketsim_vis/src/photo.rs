//! Create a fixed Soccar scene for screenshots and simulation tests.
//!
//! These helpers use a fixed seed, kickoff, car layout, and driving input.
//! They provide repeatable input for matching builds. They do not promise
//! identical results across different library versions or platforms.
//! Normal applications do not need this test scene.
//!
//! Supply collision meshes before running this example. No viewer is needed
//! unless you attach one yourself.
//!
//! ```no_run
//! use rocketsim_vis::photo;
//!
//! photo::init_meshes();
//! let mut arena = photo::build_arena();
//! photo::step_ticks(&mut arena, 120);
//! println!("Ball position: {}", arena.get_ball_state().phys.pos);
//! ```

use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};

/// Create the fixed Soccar arena with random seed `0` and kickoff seed `0`.
///
/// Car `0` is a Blue Octane. Car `1` is an Orange Dominus. This function
/// does not attach a viewer or advance simulation time.
///
/// # Panics
///
/// Panic if the Soccar collision meshes have not been loaded. Call
/// [`init_meshes`] or [`rocketsim::init`] before this function.
pub fn build_arena() -> Arena {
    let mut arena = Arena::new_with_config(ArenaConfig {
        rng_seed: Some(0),
        ..ArenaConfig::new(GameMode::Soccar)
    });
    arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.add_car(Team::Orange, CarBodyConfig::DOMINUS);
    arena.reset_to_random_kickoff(Some(0));
    arena
}

/// Load collision meshes for workspace examples and tests.
///
/// If `ROCKETSIM_MESHES_DIR` is set, use that directory without a fallback.
/// Otherwise, check `./collision_meshes/`, `../collision_meshes/`, and
/// `../../collision_meshes/` in that order. Use the first directory with a
/// `soccar` subdirectory. If none exists, call
/// [`rocketsim::init_from_default`]. Paths are relative to the process's
/// working directory.
///
/// # Panics
///
/// Panic if initialization returns an error. Applications that must handle
/// mesh errors should call [`rocketsim::init`] directly.
pub fn init_meshes() {
    if let Some(dir) = std::env::var_os("ROCKETSIM_MESHES_DIR") {
        rocketsim::init(dir, true).unwrap();
        return;
    }
    for cand in [
        "./collision_meshes/",
        "../collision_meshes/",
        "../../collision_meshes/",
    ] {
        if std::path::Path::new(cand).join("soccar").exists() {
            rocketsim::init(cand, true).unwrap();
            return;
        }
    }
    rocketsim::init_from_default(true).unwrap();
}
/// Apply full throttle to car `0` and run exactly `ticks` physics ticks.
///
/// Leave controls for other cars unchanged. Do not sleep or reset the arena.
/// Advance simulated time by `ticks / 120` seconds. An attached visualizer
/// still receives the resulting updates.
///
/// # Panics
///
/// Panic if `ticks` is nonzero and the arena has no car `0`.
pub fn step_ticks(arena: &mut Arena, ticks: usize) {
    let drive = CarControls {
        throttle: 1.0,
        ..CarControls::default()
    };
    for _ in 0..ticks {
        arena.set_car_controls(0, drive);
        arena.step_tick();
    }
}
