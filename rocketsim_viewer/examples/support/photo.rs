#![allow(dead_code)]

use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};

pub fn init_meshes() {
    if let Some(dir) = std::env::var_os("ROCKETSIM_MESHES_DIR") {
        rocketsim::init(dir, true).unwrap();
        return;
    }
    for candidate in [
        "./collision_meshes/",
        "../collision_meshes/",
        "../../collision_meshes/",
    ] {
        if std::path::Path::new(candidate).join("soccar").exists() {
            rocketsim::init(candidate, true).unwrap();
            return;
        }
    }
    rocketsim::init_from_default(true).unwrap();
}

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

pub fn step_ticks(arena: &mut Arena, ticks: usize) {
    let controls = CarControls {
        throttle: 1.0,
        ..CarControls::default()
    };
    for _ in 0..ticks {
        arena.set_car_controls(0, controls);
        arena.step_tick();
    }
}
