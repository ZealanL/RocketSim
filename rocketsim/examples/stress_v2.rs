use std::time::Instant;

use clap::Parser;
use fastrand::Rng;
use glam::Vec3A;
use rocketsim_rs::{
    consts,
    math::Vec3,
    sim::{Arena, ArenaConfig, BallState, CarConfig, CarControls, CarState, GameMode, Team},
};
use stress_common::{
    Args, BotBallState, BotCarState, BotControls, GameModeArg, NUM_EPISODE, NUM_EPISODE_TICKS,
    UPDATE_CHANCE, VEL_ADD_MAG, calc_bot_controls, print_results, rand_axis_val, rand_chance,
};

mod stress_common;

impl From<GameModeArg> for GameMode {
    fn from(value: GameModeArg) -> Self {
        match value {
            GameModeArg::Soccar => Self::Soccar,
            GameModeArg::Hoops => Self::Hoops,
            GameModeArg::Heatseeker => Self::Heatseeker,
            GameModeArg::Snowday => Self::Snowday,
            GameModeArg::Dropshot => Self::Dropshot,
            GameModeArg::TheVoid => Self::TheVoid,
        }
    }
}

fn vec3_to_glam(v: Vec3) -> Vec3A {
    Vec3A::new(v.x, v.y, v.z)
}

fn glam_to_vec3(v: Vec3A) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

fn bot_car_state(car_state: &CarState) -> BotCarState {
    BotCarState {
        pos: vec3_to_glam(car_state.pos),
        ang_vel: vec3_to_glam(car_state.ang_vel),
        forward: vec3_to_glam(car_state.rot_mat.forward),
        right: vec3_to_glam(car_state.rot_mat.right),
        is_on_ground: car_state.is_on_ground,
        is_jumping: car_state.is_jumping,
        has_flip_or_jump: car_state.has_flip_or_jump(),
    }
}

fn bot_ball_state(ball_state: &BallState) -> BotBallState {
    BotBallState {
        pos: vec3_to_glam(ball_state.pos),
        vel: vec3_to_glam(ball_state.vel),
    }
}

fn car_controls(controls: BotControls) -> CarControls {
    CarControls {
        throttle: controls.throttle,
        steer: controls.steer,
        pitch: controls.pitch,
        yaw: controls.yaw,
        roll: controls.roll,
        jump: controls.jump,
        boost: controls.boost,
        handbrake: controls.handbrake,
    }
}

fn main() {
    let cli = Args::parse();

    rocketsim_rs::init(
        Some(concat!(env!("CARGO_MANIFEST_DIR"), "/../collision_meshes")),
        true,
    );
    let mut arena = Arena::new(cli.game_mode.into(), ArenaConfig::default(), 120);

    let mut rng = Rng::with_seed(0);

    let mut ids = Vec::with_capacity(cli.num_cars as usize);
    for i in 0..cli.num_cars {
        let team = if i % 2 == 0 { Team::Blue } else { Team::Orange };
        let id = arena.pin_mut().add_car(team, CarConfig::octane());
        ids.push(id);
    }

    let mut total_ball_touches = 0;
    let mut last_ball_hit_ticks = vec![0; cli.num_cars as usize];
    let start = Instant::now();
    for _ in 0..NUM_EPISODE {
        {
            // Set up new episode
            // Reset to kickoff
            arena.pin_mut().reset_to_random_kickoff(None);

            // Accelerate the ball randomly
            let mut ball_state = arena.pin_mut().get_ball();
            let ball_vel = vec3_to_glam(ball_state.vel)
                + Vec3A::new(
                    rand_axis_val(&mut rng),
                    rand_axis_val(&mut rng),
                    rand_axis_val(&mut rng),
                ) * VEL_ADD_MAG;
            ball_state.vel = glam_to_vec3(ball_vel);
            arena.pin_mut().set_ball(ball_state);

            last_ball_hit_ticks.fill(0);
        }

        for _ in 0..NUM_EPISODE_TICKS {
            let ball_state = bot_ball_state(&arena.pin_mut().get_ball());
            for &id in &ids {
                let car_state = arena.pin_mut().get_car(id);

                if rand_chance(&mut rng, UPDATE_CHANCE) {
                    let controls = calc_bot_controls(
                        &mut rng,
                        bot_car_state(&car_state),
                        ball_state,
                        consts::CAR_MAX_SPEED,
                    );
                    arena
                        .pin_mut()
                        .set_car_controls(id, car_controls(controls))
                        .unwrap();
                }
            }

            arena.pin_mut().step(1);
            for (car_idx, &id) in ids.iter().enumerate() {
                let car_state = arena.pin_mut().get_car(id);
                if car_state.ball_hit_info.is_valid
                    && car_state.ball_hit_info.tick_count_when_hit != last_ball_hit_ticks[car_idx]
                {
                    last_ball_hit_ticks[car_idx] = car_state.ball_hit_info.tick_count_when_hit;
                    total_ball_touches += 1;
                }
            }
        }
    }

    print_results(
        Instant::now().duration_since(start).as_secs_f32(),
        total_ball_touches,
    );
}
