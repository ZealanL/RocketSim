use std::time::Instant;

use clap::Parser;
use fastrand::Rng;
use glam::Vec3A;
use rocketsim_rs::{
    consts,
    math::Vec3,
    sim::{
        Arena, ArenaConfig, ArenaMemWeightMode, BallState, CarConfig, CarControls, CarState,
        GameMode, Team,
    },
};
use stress_common::{
    Args, BotBallState, BotCarState, BotControls, GameModeArg, MemWeightModeArg, NUM_EPISODE,
    NUM_EPISODE_TICKS, UPDATE_CHANCE, VEL_ADD_MAG, calc_bot_controls, print_results, rand_axis_val,
    rand_chance,
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

fn arena_mem_weight_mode(value: MemWeightModeArg) -> ArenaMemWeightMode {
    match value {
        MemWeightModeArg::Light => ArenaMemWeightMode::Light,
        MemWeightModeArg::Heavy => ArenaMemWeightMode::Heavy,
        MemWeightModeArg::Balanced => {
            panic!("stress_v2 does not support the balanced memory mode")
        }
    }
}

fn vec3(value: Vec3) -> Vec3A {
    Vec3A::new(value.x, value.y, value.z)
}

fn bot_car_state(car_state: &CarState) -> BotCarState {
    BotCarState {
        pos: vec3(car_state.pos),
        ang_vel: vec3(car_state.ang_vel),
        forward: vec3(car_state.rot_mat.forward),
        right: vec3(car_state.rot_mat.right),
        is_on_ground: car_state.is_on_ground,
        is_jumping: car_state.is_jumping,
        has_flip_or_jump: car_state.has_flip_or_jump(),
    }
}

fn bot_ball_state(ball_state: &BallState) -> BotBallState {
    BotBallState {
        pos: vec3(ball_state.pos),
        vel: vec3(ball_state.vel),
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

fn car_hit_ball_on_tick(car_state: &CarState, tick: u64) -> bool {
    car_state.ball_hit_info.is_valid && car_state.ball_hit_info.tick_count_when_hit == tick
}

fn main() {
    let cli = Args::parse();

    rocketsim_rs::init(None, true);

    let arena_config = ArenaConfig {
        mem_weight_mode: arena_mem_weight_mode(cli.mem_weight_mode),
        no_ball_rot: false,
        ..Default::default()
    };
    let mut arenas: Vec<_> = (0..cli.num_arenas)
        .map(|arena_idx| {
            let mut arena = Arena::new(cli.game_mode.into(), arena_config, 120);
            let ids: Vec<_> = (0..cli.num_cars)
                .map(|car_idx| {
                    arena
                        .pin_mut()
                        .add_car(Team::try_from(car_idx % 2).unwrap(), CarConfig::octane())
                })
                .collect();
            (arena, ids, Rng::with_seed(arena_idx as u64), 0_u64)
        })
        .collect();

    let mut total_ball_touches = 0;
    let start = Instant::now();
    for _ in 0..NUM_EPISODE {
        for (arena, _, rng, _) in &mut arenas {
            arena.pin_mut().reset_to_random_kickoff(None);

            let mut ball_state = arena.pin_mut().get_ball();
            ball_state.vel.x += rand_axis_val(rng) * VEL_ADD_MAG;
            ball_state.vel.y += rand_axis_val(rng) * VEL_ADD_MAG;
            ball_state.vel.z += rand_axis_val(rng) * VEL_ADD_MAG;
            arena.pin_mut().set_ball(ball_state);
        }

        for tick_idx in 0..NUM_EPISODE_TICKS {
            for (arena, ids, rng, tick_count) in &mut arenas {
                let ball_state = bot_ball_state(&arena.pin_mut().get_ball());
                for &id in ids.iter() {
                    let car_state = arena.pin_mut().get_car(id);
                    if tick_idx != 0 && car_hit_ball_on_tick(&car_state, *tick_count - 1) {
                        total_ball_touches += 1;
                    }

                    if rand_chance(rng, UPDATE_CHANCE) {
                        let controls = calc_bot_controls(
                            rng,
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
                *tick_count += 1;
            }
        }

        for (arena, ids, _, tick_count) in &mut arenas {
            for &id in ids.iter() {
                let car_state = arena.pin_mut().get_car(id);
                if car_hit_ball_on_tick(&car_state, *tick_count - 1) {
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
