//! Deterministic scene used for screenshot review and determinism gates.
//!
//! Fixed seed, fixed cars, fixed controls, no input: stepping this example N
//! times must always print the same checksum. Later stages attach the NetVis
//! streamer here; for stage 0 it is headless so the harness (tools/vis-shot.sh)
//! and determinism can be proven before any renderer exists.
//!
//! Run with: `cargo run -p rocketsim_vis --example photo`

use rocketsim::{
    Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team, init_from_default,
};

/// Builds the fixed scene: Soccar, seed 0, blue OCTANE + orange DOMINUS.
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

/// Steps `ticks` times with fixed controls (blue drives forward, orange idles).
/// Sleeps ~2ms per tick so a future streaming viewer can keep up; sleep does
/// not affect sim determinism.
pub fn run_ticks(arena: &mut Arena, ticks: usize) {
    let drive = CarControls {
        throttle: 1.0,
        ..CarControls::default()
    };
    for _ in 0..ticks {
        arena.set_car_controls(0, drive);
        arena.step_tick();
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
}

fn main() {
    init_from_default(true).unwrap();
    let mut arena = build_arena();
    run_ticks(&mut arena, 600);
    let ball = arena.get_ball_state().phys.pos;
    let car = arena.get_car_state(0).phys.pos;
    println!(
        "photo tick={} ball=({:.1},{:.1},{:.1}) car0=({:.1},{:.1},{:.1})",
        arena.tick_count(),
        ball.x,
        ball.y,
        ball.z,
        car.x,
        car.y,
        car.z
    );
}
