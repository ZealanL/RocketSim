//! Drive a simulation from the detached viewer.
//!
//! Frames and controls use the same connection. Missing or stale controls
//! produce neutral inputs. Pending reset, dribble, and launch actions are
//! consumed separately from the latest analog inputs.
//!
//! Run `cargo run -p rocketsim_vis --example drive -- [-g hoops]`.
//! Use `--viewer IP:PORT` to connect to a viewer started separately.

#[path = "support/drive.rs"]
mod drive_controls;
#[path = "support/photo.rs"]
mod photo_support;
mod support;

use std::net::SocketAddr;

use clap::Parser;
use drive_controls::Driver;
use photo_support::init_meshes;
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, GameMode, Team};
use rocketsim_vis::{ArenaVisExt, pace::TickPacer, parse_game_mode};

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
    let _viewer = if cli.viewer.is_none() {
        support::prepare_viewer()?
    } else {
        None
    };
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
    let mut driver = Driver::new(default_car);
    loop {
        for _ in 0..pacer.wait() {
            driver.tick(&mut arena);
        }
    }
}
