//! Windowed sim with two named cars and world-anchored name tags.
//!
//! Run a viewer separately (or let another example start one), then:
//!   cargo run -p rocketsim_vis --example name_tags -- --addr 127.0.0.1:39201
//!
//! The spectated car has no tag. Use the viewer's birds camera to see both
//! tags at once. Press Ctrl+C in this terminal to stop.

#[path = "support/photo.rs"]
mod photo_support;

use clap::Parser;
use photo_support::init_meshes;
use rocketsim::{Arena, CarBodyConfig, CarCore, CarState, GameMode, Team};
use rocketsim_vis::ArenaVisExt;

/// Streaming sim with two name-tagged cars.
#[derive(clap::Parser)]
#[command(name = "name_tags", version)]
struct Cli {
    /// Numeric viewer address. This example never launches a viewer.
    #[arg(long, default_value = "127.0.0.1:39201")]
    addr: String,
}

fn main() -> std::io::Result<()> {
    let cli = Cli::parse();
    init_meshes();

    let mut arena = Arena::new(GameMode::Soccar);
    let blue = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let orange = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    arena.try_set_net_vis(Some(&cli.addr))?;
    arena.set_vis_car_names(vec![(blue, "Alice".into()), (orange, "Bob".into())])?;
    arena.reset_to_random_kickoff(None);

    // Place the two cars a fixed distance apart so both tags stay in view.
    // Blue is nearer the birds camera than orange, which shows that the tag
    // keeps the same screen gap above each car at different distances.
    for (car, y) in [(blue, -1500.0), (orange, 1500.0)] {
        let mut phys = CarState::DEFAULT.phys;
        phys.pos = glam::Vec3A::new(0.0, y, phys.pos.z);
        arena.set_car_state(car, CarCore::grounded(phys, 50.0));
        arena.refresh_car_contacts(car);
    }

    loop {
        arena.step_tick();
        std::thread::sleep(std::time::Duration::from_micros(8_333));
    }
}
