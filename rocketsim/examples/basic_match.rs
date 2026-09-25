//! Minimal 1v1 match: init, kickoff, drive, step, read events.
//!
//! Run with:
//! ```sh
//! cargo run -p rocketsim --example basic_match
//! ```

use rocketsim::{Arena, ArenaEvent, CarBodyConfig, CarControls, GameMode, Team, init_from_default};

fn main() {
    // Load `./collision_meshes/` once per process (`true` = quiet logging).
    init_from_default(true).unwrap();

    let mut arena = Arena::new(GameMode::Soccar);
    let blue = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let orange = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);

    // Deterministic kickoff (seeded); pass `None` for a random one.
    arena.reset_to_random_kickoff(Some(0));

    // Hold some inputs: blue drives forward with boost, orange steers.
    arena.set_car_controls(
        blue,
        CarControls::default().with_throttle(1.0).with_boost(true),
    );
    arena.set_car_controls(
        orange,
        CarControls::default().with_throttle(1.0).with_steer(0.2),
    );

    for _ in 0..120 {
        let events = arena.step_tick();
        for event in events {
            match event {
                ArenaEvent::CarHitBall(hit) => {
                    println!(
                        "car {} hit ball (+{:?} uu/s)",
                        hit.car_idx, hit.extra_hit_vel
                    );
                }
                ArenaEvent::CarPickupBoost(pickup) => {
                    println!(
                        "car {} picked up pad {}",
                        pickup.car_idx, pickup.boost_pad_idx
                    );
                }
                ArenaEvent::CarHitCar(bump) => {
                    println!(
                        "car {} bumped car {} (demo: {})",
                        bump.bumper_car_idx, bump.victim_car_idx, bump.is_demo
                    );
                }
                _ => {}
            }
        }

        if arena.is_ball_scored() {
            println!("goal at tick {}", arena.tick_count());
            break;
        }
    }

    let ball = arena.get_ball_state();
    let car = arena.get_car_state(blue);
    println!("ball pos: {}, vel: {}", ball.pos, ball.vel);
    println!(
        "blue boost: {:.1}, on ground: {}",
        car.boost, car.is_on_ground
    );
    println!("ticks: {}", arena.tick_count());
}
