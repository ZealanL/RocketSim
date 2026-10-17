//! Scripted action scene: deterministic motion for dynamic gates.
//!
//! Fixed seed + fixed script (no input): blue drives with infinite boost,
//! the ball launches at tick 60. Same ticks give same states, so trail and
//! effect screenshots are reproducible.
//!
//! Run with: `cargo run -p rocketsim_vis --example action`

use glam::Vec3A;
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};
use rocketsim_vis::{ArenaVisExt, photo::init_meshes};

/// Tick at which the script launches the ball (fast +Y with pop).
pub const LAUNCH_TICK: u64 = 60;
/// Tick of the single-tick jump press (jump dust edge).
pub const JUMP_TICK: u64 = 100;
/// Tick of the second jump press while airborne (double-jump ring edge).
pub const DOUBLE_JUMP_TICK: u64 = 115;
/// Tick at which the ball is set under the descending car (flip-reset
/// edge: wheel rays strike the ball dome, no world contact).
pub const RESET_TICK: u64 = 250;
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

    let mut drive = CarControls {
        throttle: 1.0,
        boost: true,
        ..CarControls::default()
    };
    for _ in 0..ACTION_TICKS {
        let tick = arena.tick_count();
        if tick == LAUNCH_TICK {
            arena.edit_ball_state(|ball| ball.phys.vel = Vec3A::new(0.0, 2500.0, 900.0));
        }
        // Single-tick jump presses (edges, not holds).
        drive.jump = tick == JUMP_TICK || tick == DOUBLE_JUMP_TICK;
        if tick == RESET_TICK {
            // Ball under the front axle: all four wheel rays strike the
            // dome (centered only ever gives the backs). Offset follows the
            // live heading. No world contact, airborne.
            let car = *arena.get_car_state(car_idx);
            let fwd2d = car.phys.get_forward_dir().truncate().normalize_or_zero();
            let pos = car.phys.pos + Vec3A::new(fwd2d.x * 51.0, fwd2d.y * 51.0, -105.0);
            arena.edit_ball_state(|ball| {
                ball.phys.pos = pos;
                ball.phys.vel = Vec3A::ZERO;
            });
        }
        // Demo boost: pin to full so flames show.
        arena.set_car_boost(car_idx, 100.0);

        arena.set_car_controls(car_idx, drive);
        arena.step_tick();
        // Tick markers on stderr for tick-aligned screenshot captures.
        if tick.is_multiple_of(10) {
            eprintln!("action tick={tick}");
        }
        std::thread::sleep(std::time::Duration::from_millis(8));
    }
}
