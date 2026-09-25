//! Game-mode tour: Soccar/Hoops/Dropshot scoring, Heatseeker state, TheVoid.
//!
//! Run with:
//! ```sh
//! cargo run -p rocketsim --example game_modes
//! ```

use rocketsim::{Arena, CarBodyConfig, GameMode, Team, consts, init_from_default};

fn show_mode(mode: GameMode) {
    // TheVoid needs no meshes, but every other mode requires init first.
    let mut arena = Arena::new(mode);
    println!(
        "{mode:?}: name={} soccar_arena={} ball_r={:.2} aabb_max={}",
        mode.name(),
        mode.has_soccar_arena(),
        consts::ball::get_radius(mode),
        consts::arena::get_aabb(mode).max
    );

    if mode == GameMode::Dropshot {
        arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        arena.reset_to_random_kickoff(Some(0));
        println!("  tiles: {} (70 blue + 70 orange)", arena.num_tiles());
        println!(
            "  blue tiles damaged: {:?}",
            arena.get_tile_states().get_team_states(Team::Blue)[0]
        );
    } else if mode != GameMode::TheVoid {
        arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        arena.reset_to_random_kickoff(Some(0));
        println!(
            "  pads: {}, scored: {}",
            arena.num_boost_pads(),
            arena.is_ball_scored()
        );
    } else {
        // No floor/goals/pads — ball just falls.
        println!(
            "  pads: {}, tiles: {}",
            arena.num_boost_pads(),
            arena.num_tiles()
        );
    }
}

fn main() {
    init_from_default(true).unwrap();

    for mode in [
        GameMode::Soccar,
        GameMode::Hoops,
        GameMode::Heatseeker,
        GameMode::Snowday,
        GameMode::Dropshot,
    ] {
        show_mode(mode);
    }

    // Heatseeker serves the ball toward a goal once touched; the serve state
    // lives in `hs_info` (0 = no target yet).
    let mut hs = Arena::new(GameMode::Heatseeker);
    hs.add_car(Team::Blue, CarBodyConfig::OCTANE);
    hs.reset_to_random_kickoff(Some(0));
    let ball = hs.get_ball_state();
    println!(
        "heatseeker target dir: {} (0 = pre-serve)",
        ball.hs_info.y_target_dir
    );

    // TheVoid works even without arena geometry loaded.
    show_mode(GameMode::TheVoid);
}
