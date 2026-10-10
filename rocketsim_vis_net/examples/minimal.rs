//! Minimal windowed sim: one car driving forward, no input.
//!
//! Shows the smallest setup: init meshes, create the arena, enable the
//! visualizer, then step ticks.
//!
//! Run with: `cargo run -p rocketsim_vis_net --example minimal`

use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};
use rocketsim_vis_net::{ArenaVisExt, photo::init_meshes};

fn main() {
    init_meshes();

    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff(None);

    // Detached-viewer API: launches a viewer if none listens, streams every tick.
    arena.set_vis_enabled(true);

    let controls = CarControls {
        throttle: 1.0,
        boost: true,
        ..CarControls::default()
    };

    // ~5 seconds at 120 Hz; the viewer keeps the last frame when we exit.
    for _ in 0..600 {
        // Pin boost to full so flames show.
        let mut car = *arena.get_car_state(car_idx);
        car.boost = 100.0;
        arena.set_car_state(car_idx, car);

        arena.set_car_controls(car_idx, controls);
        arena.step_tick();
        std::thread::sleep(std::time::Duration::from_millis(8));
    }
}
