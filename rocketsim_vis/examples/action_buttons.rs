//! Simulation-defined custom actions with on-screen viewer buttons.
//!
//! Run a viewer separately, then:
//!   cargo run -p rocketsim_vis --example action_buttons -- --addr 127.0.0.1:39201
//!
//! Click the buttons in the viewer's Actions window. Presses arrive through
//! the same event stream as keybind presses. Press Ctrl+C to stop.

#[path = "support/photo.rs"]
mod photo_support;

use clap::Parser;
use photo_support::init_meshes;
use rocketsim::{Arena, GameMode};
use rocketsim_vis::{ArenaVisExt, proto::CustomActionDef};

/// Streaming sim with keybind, button, and button-only actions.
#[derive(clap::Parser)]
#[command(name = "action_buttons", version)]
struct Cli {
    /// Numeric viewer address. This example never launches a viewer.
    #[arg(long, default_value = "127.0.0.1:39201")]
    addr: String,
    /// Extra button-only actions to declare. Use a large value to see the
    /// Actions window wrap its buttons into rows.
    #[arg(long, default_value_t = 0)]
    extra_buttons: usize,
}

fn main() -> std::io::Result<()> {
    let cli = Cli::parse();
    init_meshes();

    let mut actions = vec![
        CustomActionDef::new("show_route").with_default_keys(["F5"]),
        CustomActionDef::new("ping")
            .with_default_keys(["V"])
            .with_button("Send ping"),
        CustomActionDef::new("toggle_slow_mo").with_button(""),
    ];
    for index in 0..cli.extra_buttons {
        actions.push(
            CustomActionDef::new(format!("extra_{index}")).with_button(format!("Button {index}")),
        );
    }

    let mut arena = Arena::new(GameMode::TheVoid);
    arena.try_set_net_vis(Some(&cli.addr))?;
    arena.set_vis_custom_actions(actions)?;

    println!("Declared actions. Click buttons in the viewer's Actions window.");
    loop {
        arena.step_tick();
        for name in arena.take_vis_custom_actions() {
            println!("custom action: {name}");
        }
        std::thread::sleep(std::time::Duration::from_micros(8_333));
    }
}
