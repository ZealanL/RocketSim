//! Custom rules: mutators, memory mode, RNG seed, and custom pads.
//!
//! Run with:
//! ```sh
//! cargo run -p rocketsim --example custom_mutators
//! ```

use rocketsim::{
    Arena, ArenaConfig, ArenaMemWeightMode, BoostPadConfig, CarBodyConfig, DemoMode, GameMode,
    MutatorConfig, Team, init_from_default,
};

fn main() {
    init_from_default(true).unwrap();

    // Start from Soccar defaults, then tweak: heavy ball hits, no demos.
    let mut mutators = MutatorConfig::new(GameMode::Soccar);
    mutators.ball_hit_extra_force_scale = 1.5;
    mutators.demo_mode = DemoMode::Disabled;
    mutators.car_spawn_boost_amount = 100.0;

    let config = ArenaConfig::new(GameMode::Soccar)
        .with_mutators(mutators)
        .with_mem_weight_mode(ArenaMemWeightMode::Balanced)
        .with_rng_seed(42);

    let mut arena = Arena::new_with_config(config);
    println!("gravity: {}", arena.mutator_config().gravity);
    println!("ball radius: {:.2} uu", arena.mutator_config().ball_radius);

    arena.add_car(Team::Blue, CarBodyConfig::DOMINUS);
    arena.reset_to_random_kickoff(None);
    arena.step_tick();

    // Custom pad layout: replace the default 34 Soccar pads with two big pads.
    let custom_pads = vec![
        BoostPadConfig {
            pos: glam::Vec3A::new(0.0, -4240.0, 70.0),
            is_big: true,
        },
        BoostPadConfig {
            pos: glam::Vec3A::new(0.0, 4240.0, 70.0),
            is_big: true,
        },
    ];
    let mut custom_arena = Arena::new_with_config(
        ArenaConfig::new(GameMode::Soccar).with_custom_boost_pads(custom_pads),
    );
    custom_arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    println!("custom pads: {}", custom_arena.num_boost_pads());
}
