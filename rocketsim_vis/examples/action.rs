//! Scripted action scene: deterministic motion for dynamic gates.
//!
//! Fixed seed + fixed script (no input): blue drives with infinite boost,
//! the ball is launched at tick 60, and everything streams to the viewer.
//! Same ticks ⇒ same states (sleeps never touch the sim), so screenshots
//! of trails (stage 7) and effects (stage 10) are reproducible.
//!
//! Run with: `cargo run -p rocketsim_vis --example action`

use glam::Vec3A;
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};
use rocketsim_vis::{ArenaVisExt, photo::init_meshes};

/// Tick at which the script launches the ball (fast +Y with pop).
pub const LAUNCH_TICK: u64 = 60;
/// Total streamed ticks (~10s at 8ms pacing).
pub const ACTION_TICKS: u64 = 1200;

fn main() {
    init_meshes();
    let mut arena = Arena::new_with_config(ArenaConfig {
        rng_seed: Some(0),
        ..ArenaConfig::new(GameMode::Soccar)
    });
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff(Some(0));
    arena.set_vis_enabled(true);

    let drive = CarControls {
        throttle: 1.0,
        boost: true,
        ..CarControls::default()
    };
    for _ in 0..ACTION_TICKS {
        if arena.tick_count() == LAUNCH_TICK {
            let mut ball = *arena.get_ball_state();
            ball.phys.vel = Vec3A::new(0.0, 2500.0, 900.0);
            arena.set_ball_state(ball);
        }
        // Demo boost: pin to full (the old interactive demo did the same).
        let mut car = *arena.get_car_state(car_idx);
        car.boost = 100.0;
        arena.set_car_state(car_idx, car);

        arena.set_car_controls(car_idx, drive);
        arena.step_tick();
        std::thread::sleep(std::time::Duration::from_millis(8));
    }
}
