use std::time::Instant;

use glam::Vec3A;
use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, BallState, CarBodyConfig, CarControls, CarState, GameMode,
    Team, consts, init_from_default,
};

const NUM_CARS: u8 = 6;
const NUM_EPISODE_TICKS: usize = 10_000;
const NUM_EPISODE: usize = 100;
const TOTAL_TICKS: usize = NUM_EPISODE * NUM_EPISODE_TICKS;

fn rand() -> f32 {
    fastrand::f32()
}

fn rand_axis_val() -> f32 {
    rand() * 2.0 - 1.0
}

fn rand_chance(thresh: f32) -> bool {
    rand() < thresh
}

fn calc_bot_controls(car_state: &CarState, ball_state: &BallState) -> CarControls {
    let ball_delta = ball_state.phys.pos - car_state.phys.pos;

    let min_reach_time = ball_delta.length() / consts::car::MAX_SPEED;
    let extrap_ball_pos =
        ball_state.phys.pos + (ball_state.phys.vel * Vec3A::new(1.0, 1.0, 0.0) * min_reach_time);

    let extrap_ball_delta = extrap_ball_pos - car_state.phys.pos;
    let ball_forward_align = extrap_ball_delta.dot(car_state.phys.rot_mat.x_axis);
    let ball_right_align = extrap_ball_delta.dot(car_state.phys.rot_mat.y_axis);

    let mut controls = CarControls {
        throttle: 1.0,
        steer: (ball_right_align * 80.0).clamp(-1.0, 1.0),
        ..Default::default()
    };

    {
        // Driving control
        if (ball_forward_align < 0.0) && // Ball is somewhat behind
            (car_state.phys.ang_vel.z.abs() >= 1.0) && // We are turning
            (car_state.phys.pos.z < 300.0) && // We are somewhat grounded
            rand_chance(0.8)
        {
            controls.handbrake = true;
        }

        if ball_forward_align < -0.4 {
            // Brake proportionally
            controls.throttle = ball_forward_align;
        }

        if ball_forward_align < 0.3 {
            // Aligned with ball somewhat, boost
            controls.boost = true;
        }
    }

    {
        // Jump/air control
        controls.yaw = rand_axis_val();
        controls.pitch = rand_axis_val();
        controls.roll = rand_axis_val() * rand();

        if car_state.is_on_ground {
            controls.jump = rand_chance(0.04);
        } else {
            if car_state.is_jumping {
                // Keep holding jump sometimes
                controls.jump = rand_chance(0.5);
            } else {
                // Flip chance when in air
                controls.jump = rand_chance(0.1);
            }

            if !car_state.is_jumping && car_state.has_flip_or_jump() && controls.jump {
                if rand_chance(0.5) {
                    // Align direction towards ball
                    let align_frac = rand().sqrt();
                    controls.pitch *= 1.0 - align_frac;
                    controls.yaw *= 1.0 - align_frac;
                    controls.pitch += -ball_forward_align * align_frac;
                    controls.yaw += ball_right_align * align_frac;
                } else if rand_chance(0.2) {
                    // Double-jump
                    controls.yaw = 0.0;
                    controls.pitch = 0.0;
                    controls.roll = 0.0;
                }
            }
        }
    }

    {
        // Add some randomization to everything
        controls.throttle += rand_axis_val() * rand().powi(3);
        controls.steer += rand_axis_val() * rand().powi(3);
        controls.yaw += rand_axis_val() * rand();
        controls.pitch += rand_axis_val() * rand();
        controls.roll += rand_axis_val() * rand();

        if rand_chance(0.2) {
            controls.jump = !controls.jump;
        }

        if rand_chance(0.2) {
            controls.boost = !controls.boost;
        }

        if rand_chance(0.2) {
            controls.handbrake = !controls.handbrake;
        }
    }

    controls.clamp()
}

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new_with_config(
        GameMode::Soccar,
        ArenaConfig {
            rng_seed: Some(0),
            ..Default::default()
        },
    );

    fastrand::seed(0);

    let mut ids = Vec::new();
    for i in 0..NUM_CARS {
        let id = arena.add_car(Team::try_from(i % 2).unwrap(), CarBodyConfig::OCTANE);
        ids.push(id);
    }

    let mut total_ball_touches = 0;
    let start = Instant::now();
    for _ in 0..NUM_EPISODE {
        {
            const VEL_ADD_MAG: f32 = 1000.0;

            // Set up new episode
            // Reset to kickoff
            arena.reset_to_random_kickoff();

            // Accelerate the ball randomly
            let mut ball_state = *arena.get_ball_state();
            ball_state.phys.vel +=
                Vec3A::new(rand_axis_val(), rand_axis_val(), rand_axis_val()) * VEL_ADD_MAG;
            arena.set_ball_state(ball_state);
        }

        for _ in 0..NUM_EPISODE_TICKS {
            let ball_state = *arena.get_ball_state();
            for &idx in &ids {
                const UPDATE_CHANCE: f32 = 0.05; // (120 * 0.05) = Avg of 6 per sec

                let car_state = arena.get_car_state(idx);

                if rand_chance(UPDATE_CHANCE) {
                    arena.set_car_controls(idx, calc_bot_controls(car_state, &ball_state));
                }
            }

            arena.step(1);
            for event in arena.get_step_events() {
                if let ArenaEvent::CarHitBall(_car_hit_ball_event) = event {
                    total_ball_touches += 1;
                }
            }
        }
    }

    let elapsed = Instant::now().duration_since(start).as_secs_f32();
    let tps = (TOTAL_TICKS as f32) / elapsed;
    println!(
        "Elapsed: {elapsed}\
        \nTPS: {tps:.0}\
        \nBall hits: {total_ball_touches}"
    );
}
