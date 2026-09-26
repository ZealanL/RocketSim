//! Minimal windowed sim: one car driving forward, no input handling.
//!
//! Shows the smallest setup needed for [`rocketsim_vis`]: init meshes,
//! create the arena, enable the visualizer, then step ticks.
//!
//! Run with: `cargo run -p rocketsim_vis --example minimal`

use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team, init_from_default};
use rocketsim_vis::ArenaVisExt;

fn main() {
    init_from_default(true).unwrap();

    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff(None);
    arena.set_vis_enabled(true);

    let controls = CarControls {
        throttle: 1.0,
        boost: true,
        ..CarControls::default()
    };

    // ~5 seconds at 120 Hz; the window stays alive while `arena` lives.
    for _ in 0..600 {
        arena.set_car_controls(car_idx, controls);
        arena.step_tick();
        std::thread::sleep(std::time::Duration::from_millis(8));
    }
}
