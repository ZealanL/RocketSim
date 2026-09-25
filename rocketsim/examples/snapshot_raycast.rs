//! Snapshots, teleports, boost-pad state, and batched raycasts.
//!
//! Run with:
//! ```sh
//! cargo run -p rocketsim --example snapshot_raycast
//! ```

use glam::Vec3A;
use rocketsim::{
    Arena, CarBodyConfig, GameMode, RaycastQuery, Team,
    consts::{BT_TO_UU, UU_TO_BT},
    init_from_default,
};

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new(GameMode::Soccar);
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff(Some(7));

    // 1. Snapshot everything (cars, ball, pads) for replays/debugging.
    let snapshot = arena.get_arena_state();
    println!(
        "cars: {}, pads: {}",
        snapshot.num_cars(),
        snapshot.num_boost_pads()
    );

    // 2. Teleport the ball above the car and let it drop for a second.
    let mut ball_state = *arena.get_ball_state();
    let car_state = arena.get_car_state(car_idx);
    ball_state.phys.pos = car_state.phys.pos + Vec3A::new(0.0, 0.0, 500.0);
    ball_state.phys.vel = Vec3A::ZERO;
    arena.set_ball_state(ball_state);

    for _ in 0..120 {
        arena.step_tick();
    }
    println!("ball fell to: {}", arena.get_ball_state().pos);

    // 3. Boost pads: layout is RLBot/RLGym ordered (Y, then X).
    println!("pads: {}", arena.num_boost_pads());
    let first_config = *arena.get_boost_pad_config(0);
    let first_state = arena.get_boost_pad_state(0);
    println!(
        "pad 0 at {} (big: {}), active: {}",
        first_config.pos,
        first_config.is_big,
        first_state.is_active()
    );

    // 4. Raycasts are SIMD-batched in 4s — send multiples of 4 when possible.
    // NOTE: `cast_rays` currently forwards coordinates to the Bullet world
    // unconverted, so convert uu -> BT here (and BT -> uu on hits).
    // Each ray must also fit in one broadphase cell (370 uu default).
    let ball_uu = arena.get_ball_state().pos + Vec3A::new(500.0, 0.0, 0.0);
    let down = RaycastQuery {
        from: ball_uu * UU_TO_BT,
        to: (ball_uu - Vec3A::new(0.0, 0.0, 300.0)) * UU_TO_BT,
        hit_dynamic: false,
    };
    let queries = [down; 4];
    let results = arena.cast_rays(&queries);
    for (i, result) in results.iter().enumerate() {
        match &result.hit_info {
            Some(hit) => println!(
                "ray {i}: hit at {} (t={:.3})",
                hit.hit_point * BT_TO_UU,
                hit.hit_fraction
            ),
            None => println!("ray {i}: miss"),
        }
    }

    // 5. Restore: teleport back to the snapshot's ball state.
    arena.set_ball_state(snapshot.ball);
    println!("restored to: {}", arena.get_ball_state().pos);
}
