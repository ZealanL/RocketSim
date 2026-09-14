use std::{fmt::Write as _, fs, path::Path, time::Instant};

use clap::Parser;
use fastrand::Rng;
use glam::Vec3A;
use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, ArenaMemWeightMode, BallState, CarBodyConfig, CarControls,
    CarState, GameMode, Team, consts, init_from_default,
};
use stress_common::{
    Args, BotBallState, BotCarState, BotControls, GameModeArg, MemWeightModeArg, NUM_EPISODE_TICKS,
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

impl From<MemWeightModeArg> for ArenaMemWeightMode {
    fn from(value: MemWeightModeArg) -> Self {
        match value {
            MemWeightModeArg::Light => Self::Light,
            MemWeightModeArg::Balanced => Self::Balanced,
            MemWeightModeArg::Heavy => Self::Heavy,
        }
    }
}

fn bot_car_state(car_state: &CarState) -> BotCarState {
    BotCarState {
        pos: car_state.phys.pos,
        ang_vel: car_state.phys.ang_vel,
        forward: car_state.phys.rot_mat.x_axis,
        right: car_state.phys.rot_mat.y_axis,
        is_on_ground: car_state.is_on_ground,
        is_jumping: car_state.is_jumping,
        has_flip_or_jump: car_state.has_flip_or_jump(),
    }
}

fn bot_ball_state(ball_state: &BallState) -> BotBallState {
    BotBallState {
        pos: ball_state.phys.pos,
        vel: ball_state.phys.vel,
    }
}

fn write_state_snapshot(path: &Path, arenas: &[(Arena, Vec<usize>, Rng)]) {
    let mut text = String::new();
    writeln!(text, "RocketSim V3 stress snapshot").unwrap();
    writeln!(text, "arenas={}", arenas.len()).unwrap();
    for (arena_idx, (arena, ids, _)) in arenas.iter().enumerate() {
        writeln!(text, "arena={arena_idx} ball={:?}", arena.get_ball_state()).unwrap();
        for (car_idx, &id) in ids.iter().enumerate() {
            writeln!(text, "arena={arena_idx} car={car_idx} state={:?}", arena.get_car_state(id)).unwrap();
        }
    }
    fs::write(path, text).expect("write state snapshot");
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

    init_from_default(true).unwrap();

    let arena_config = ArenaConfig::new(cli.game_mode.into())
        .with_rng_seed(0)
        .with_mem_weight_mode(cli.mem_weight_mode.into());
    let mut arenas: Vec<_> = (0..cli.num_arenas)
        .map(|arena_idx| {
            let mut arena_config = arena_config.clone();
            arena_config.rng_seed = Some(arena_idx as u64);
            let mut arena = Arena::new_with_config(arena_config);
            let ids: Vec<_> = (0..cli.num_cars)
                .map(|car_idx| {
                    arena.add_car(Team::try_from(car_idx % 2).unwrap(), CarBodyConfig::OCTANE)
                })
                .collect();
            (arena, ids, Rng::with_seed(arena_idx as u64))
        })
        .collect();

    let mut total_ball_touches = 0;
    let start = Instant::now();
    for _ in 0..cli.episodes {
        for (arena, _, rng) in &mut arenas {
            arena.reset_to_random_kickoff(None);

            let mut ball_state = *arena.get_ball_state();
            ball_state.phys.vel +=
                Vec3A::new(rand_axis_val(rng), rand_axis_val(rng), rand_axis_val(rng))
                    * VEL_ADD_MAG;
            arena.set_ball_state(ball_state);
        }

        for _ in 0..NUM_EPISODE_TICKS {
            for (arena, ids, rng) in &mut arenas {
                let ball_state = bot_ball_state(arena.get_ball_state());
                for &idx in ids.iter() {
                    let car_state = arena.get_car_state(idx);

                    if rand_chance(rng, UPDATE_CHANCE) {
                        let controls = calc_bot_controls(
                            rng,
                            bot_car_state(car_state),
                            ball_state,
                            consts::car::MAX_SPEED,
                        );
                        arena.set_car_controls(idx, car_controls(controls));
                    }
                }

                arena.step_tick();
                for event in arena.get_last_step_events() {
                    if let ArenaEvent::CarHitBall(_car_hit_ball_event) = event {
                        total_ball_touches += 1;
                    }
                }
            }
        }
    }

    print_results(
        Instant::now().duration_since(start).as_secs_f32(),
        total_ball_touches,
        cli.episodes,
        cli.num_arenas,
    );
    if let Some(path) = cli.state_output.as_deref() {
        write_state_snapshot(path, &arenas);
    }
    let profile = rocketsim::profiling::report();
    if !profile.is_empty() {
        print!("{profile}");
    }
}
