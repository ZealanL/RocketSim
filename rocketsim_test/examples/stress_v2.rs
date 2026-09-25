use std::{path::Path, time::Instant};

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
use rocketsim_test::rlpr::{Recording, recording_has_boost_state, recording_has_handbrake_state};
use stress_common::{
    Args, BALL_ONLY_NUM_EPISODE, BALL_ONLY_TOTAL_TICKS, BotBallState, BotCarState, BotControls,
    GameModeArg, MemWeightModeArg, NUM_EPISODE, NUM_EPISODE_TICKS, UPDATE_CHANCE, VEL_ADD_MAG,
    calc_bot_controls, print_results, rand_axis_val, rand_chance,
};

#[allow(dead_code)]
#[path = "rlpr_metric/common.rs"]
mod common;
#[allow(dead_code)]
#[path = "rlpr_replay/mod.rs"]
mod rlpr_replay;
mod stress_common;
#[allow(dead_code)]
#[path = "rlpr_metric/v2.rs"]
mod v2;

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

impl rlpr_replay::ReplayBenchmarkBackend for v2::V2Backend {
    type Control = CarControls;

    fn control_from_record(
        controls: rocketsim_test::rlpr::cpp_records::ControlsRecord,
    ) -> Self::Control {
        v2::V2Backend::benchmark_control(controls)
    }

    fn restore_replay_state(
        &mut self,
        ticks: &[rocketsim_test::rlpr::tick_record::TickRecord],
        state_index: usize,
        run_start: usize,
        recording_version: u32,
    ) {
        let has_boost_state = recording_has_boost_state(recording_version);
        let has_handbrake_state = recording_has_handbrake_state(recording_version);
        <Self as common::ReplayBackend>::set_state(self, &ticks[state_index]);
        if has_handbrake_state {
            common::restore_recorded_handbrake(self, ticks, state_index, true);
        } else {
            common::restore_handbrake_seed(self, ticks, run_start, state_index);
        }
        common::restore_recorded_boost_state(self, ticks, state_index, has_boost_state);
        <Self as common::ReplayBackend>::refresh_sticky_gates(self);
    }

    fn step_simulation(&mut self, controls: &[Self::Control]) {
        self.benchmark_step(controls);
    }
}

fn run_ball_only_benchmark(cli: &Args) {
    let arena_config = ArenaConfig {
        mem_weight_mode: arena_mem_weight_mode(cli.mem_weight_mode),
        no_ball_rot: false,
        ..Default::default()
    };
    let mut arenas: Vec<_> = (0..cli.num_arenas)
        .map(|arena_idx| {
            let arena = Arena::new(cli.game_mode.into(), arena_config, 120);
            (arena, Rng::with_seed(arena_idx as u64))
        })
        .collect();

    let start = Instant::now();
    for _ in 0..BALL_ONLY_NUM_EPISODE {
        for (arena, rng) in &mut arenas {
            arena.pin_mut().reset_to_random_kickoff(None);

            let mut ball_state = arena.pin_mut().get_ball();
            ball_state.vel.x += rand_axis_val(rng) * VEL_ADD_MAG;
            ball_state.vel.y += rand_axis_val(rng) * VEL_ADD_MAG;
            ball_state.vel.z += rand_axis_val(rng) * VEL_ADD_MAG;
            arena.pin_mut().set_ball(ball_state);
        }

        for _ in 0..NUM_EPISODE_TICKS {
            for (arena, _) in &mut arenas {
                arena.pin_mut().step(1);
            }
        }
    }

    let elapsed = start.elapsed().as_secs_f32();
    let tps = BALL_ONLY_TOTAL_TICKS as f32 * cli.num_arenas as f32 / elapsed;
    println!("Ball-only elapsed: {elapsed:.6}");
    println!("Ball-only TPS: {tps:.0}");
}

fn run_rlpr_replay(path: &Path, cli: &Args) -> Result<(), Box<dyn std::error::Error>> {
    if cli.num_cars as usize != rlpr_replay::THREE_V_THREE_CARS {
        return Err("--num-cars must be 6 in RLPR replay mode".into());
    }
    if !matches!(cli.game_mode, GameModeArg::Soccar) {
        return Err("RLPR replay mode requires --game-mode soccar".into());
    }
    if matches!(cli.mem_weight_mode, MemWeightModeArg::Balanced) {
        return Err("stress_v2 does not support the balanced memory mode".into());
    }

    let recording = Recording::from_file(path)?;
    let plan = rlpr_replay::ReplayPlan::from_recording(&recording)
        .map_err(|error| format!("invalid RLPR replay: {error}"))?;
    let mem_weight_mode = arena_mem_weight_mode(cli.mem_weight_mode);
    let mut backends: Vec<_> = (0..cli.num_arenas)
        .map(|_| v2::V2Backend::benchmark(mem_weight_mode))
        .collect();
    let stats = rlpr_replay::run_replay(&mut backends, &recording, &plan)
        .map_err(|error| format!("RLPR replay failed: {error}"))?;

    println!(
        "Replay: {} (RLPR v{}, 3v3)",
        recording.name, recording.version
    );
    println!("File: {}", path.display());
    println!("Recording ticks: {}", recording.ticks.len());
    println!(
        "State restores: {} total ({} timed)",
        plan.state_reset_count(),
        stats.timed_state_restores
    );
    println!(
        "Simulation ticks: {} per arena ({} total)",
        stats.simulation_ticks,
        stats.total_simulation_ticks()
    );
    println!("Elapsed: {:.6}", stats.elapsed.as_secs_f64());
    println!("TPS: {:.0}", stats.ticks_per_second());
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Args::parse();

    rocketsim_rs::init(None, true);

    if let Some(path) = cli.rlpr_file.as_deref() {
        return run_rlpr_replay(path, &cli);
    }

    if cli.num_cars == 0 {
        run_ball_only_benchmark(&cli);
        return Ok(());
    }

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
    Ok(())
}
