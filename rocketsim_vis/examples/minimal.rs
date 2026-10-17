//! Minimal windowed sim: one car driving forward, no input.
//!
//! Shows the smallest setup: init meshes, create the arena, enable the
//! visualizer, then step ticks.
//!
//! Run with: `cargo run -p rocketsim_vis --example minimal -- [-g hoops]`

use clap::Parser;
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};
use rocketsim_vis::{ArenaVisExt, parse_game_mode, photo::init_meshes};

/// Minimal streaming sim: `minimal [-g MODE]` (defaults: soccar).
#[derive(clap::Parser)]
#[command(name = "minimal", version)]
struct Cli {
    /// Arena game mode (the viewer follows automatically).
    #[arg(short, long, default_value = "soccar", value_parser = parse_mode)]
    game_mode: GameMode,
}

/// Clap parser over the shared [`parse_game_mode`] vocabulary.
fn parse_mode(s: &str) -> Result<GameMode, String> {
    parse_game_mode(s).ok_or_else(|| {
        format!("unknown game mode '{s}' (soccar|hoops|heatseeker|snowday|dropshot|thevoid|void)")
    })
}

fn main() {
    let cli = Cli::parse();
    init_meshes();
    let mut arena = Arena::new_with_config(ArenaConfig::new(cli.game_mode));
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff(None);

    // Detached-viewer API: launches a viewer if none listens, streams every tick.
    arena.set_vis_enabled(true);

    let controls = CarControls {
        throttle: 1.0,
        boost: true,
        ..CarControls::default()
    };

    // Run for about five seconds. An auto-started viewer closes when the arena drops.
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
