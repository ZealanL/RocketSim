//! Drive a simulation from the detached viewer.
//!
//! Frames and controls use the same connection. Missing or stale controls
//! produce neutral inputs. Pending reset, dribble, and launch actions are
//! consumed separately from the latest analog inputs.
//!
//! Run `cargo run -p rocketsim_vis --example drive -- [-g hoops]`.
//! Use `--viewer IP:PORT` to connect to a viewer started separately.

use std::net::SocketAddr;

use clap::Parser;
use glam::Vec3A;
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};
use rocketsim_vis::{
    ArenaVisExt, mutators::Mutators, pace::TickPacer, parse_game_mode, photo::init_meshes,
    proto::MUT_KNOWN_MASK,
};

#[derive(Parser)]
#[command(name = "drive", version)]
struct Cli {
    /// Arena game mode.
    #[arg(short, long, default_value = "soccar", value_parser = parse_mode)]
    game_mode: GameMode,
    /// Connect to an existing viewer at this numeric socket address.
    #[arg(long)]
    viewer: Option<SocketAddr>,
}

fn parse_mode(text: &str) -> Result<GameMode, String> {
    parse_game_mode(text).ok_or_else(|| format!("unknown game mode '{text}'"))
}

fn main() -> std::io::Result<()> {
    let cli = Cli::parse();
    init_meshes();
    let mut arena = Arena::new_with_config(ArenaConfig {
        rng_seed: Some(0),
        ..ArenaConfig::new(cli.game_mode)
    });
    let default_car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff(None);
    if let Some(addr) = cli.viewer {
        arena.try_set_net_vis(Some(&addr.to_string()))?;
    } else {
        arena.try_set_vis_enabled(true)?;
    }
    println!("Follow a car in the viewer to drive it. Close the settings menu first.");
    println!("Reset, dribble, and launch use Backspace, F2, and F4 by default.");

    let mut pacer = TickPacer::at_120hz();
    let mut previous_target = default_car;
    // Keep the last mutator selection on link loss. Driving inputs still become neutral.
    let mut mutators = 0;
    loop {
        for _ in 0..pacer.wait() {
            let message = arena.latest_vis_controls();
            let target = message
                .map(|message| message.car_index as usize)
                .filter(|&index| index < arena.num_cars())
                .unwrap_or(default_car);
            if previous_target != target {
                arena.set_car_controls(previous_target, CarControls::default());
                previous_target = target;
            }
            if let Some(message) = message {
                mutators = message.mutators & MUT_KNOWN_MASK;
            }
            let controls = message.map_or_else(CarControls::default, |message| message.controls);
            let actions = arena.take_vis_actions();
            if actions & 1 != 0 {
                arena.reset_to_random_kickoff(None);
            }
            if actions & 2 != 0 {
                let car = arena.get_car_state(target);
                let mut ball = *arena.get_ball_state();
                ball.phys.pos = car.phys.pos + Vec3A::Z * 150.0;
                ball.phys.vel = car.phys.vel;
                arena.set_ball_state(ball);
            }
            if actions & 4 != 0 {
                let mut ball = *arena.get_ball_state();
                ball.phys.vel += Vec3A::Z * 1000.0;
                arena.set_ball_state(ball);
            }
            let mut car = *arena.get_car_state(target);
            Mutators::from_mask(mutators).apply_pre_tick(&mut car);
            arena.set_car_state(target, car);
            arena.set_car_controls(target, controls);
            arena.step_tick();
        }
    }
}
