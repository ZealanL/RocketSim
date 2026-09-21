use std::{path::Path, time::Instant};

use clap::Parser;
use fastrand::Rng;
use glam::Vec3A;
use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, ArenaMemWeightMode, BallState, CarBodyConfig, CarControls,
    CarState, GameMode, Team, consts, init_from_default,
};
use rocketsim_test::rlpr::{Recording, recording_has_boost_state, recording_has_handbrake_state};
use stress_common::{
    Args, BotBallState, BotCarState, BotControls, GameModeArg, MemWeightModeArg, NUM_EPISODE,
    NUM_EPISODE_TICKS, UPDATE_CHANCE, VEL_ADD_MAG, calc_bot_controls, print_results, rand_axis_val,
    rand_chance,
};

#[allow(dead_code)]
#[path = "rlpr_metric/common.rs"]
mod common;
#[allow(dead_code)]
#[path = "rlpr_replay/mod.rs"]
mod rlpr_replay;
mod stress_common;
#[allow(dead_code)]
#[path = "rlpr_metric/v3.rs"]
mod v3;

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

impl rlpr_replay::ReplayBenchmarkBackend for v3::V3Backend {
    type Control = CarControls;

    fn control_from_record(
        controls: rocketsim_test::rlpr::cpp_records::ControlsRecord,
    ) -> Self::Control {
        controls.into()
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

fn run_rlpr_replay(path: &Path, cli: &Args) -> Result<(), Box<dyn std::error::Error>> {
    if cli.num_cars as usize != rlpr_replay::THREE_V_THREE_CARS {
        return Err("--num-cars must be 6 in RLPR replay mode".into());
    }
    if !matches!(cli.game_mode, GameModeArg::Soccar) {
        return Err("RLPR replay mode requires --game-mode soccar".into());
    }

    let recording = Recording::from_file(path)?;
    let plan = rlpr_replay::ReplayPlan::from_recording(&recording)
        .map_err(|error| format!("invalid RLPR replay: {error}"))?;
    let mem_weight_mode: ArenaMemWeightMode = cli.mem_weight_mode.into();
    let mut backends: Vec<_> = (0..cli.num_arenas)
        .map(|_| v3::V3Backend::with_mem_weight_mode(0.5, mem_weight_mode))
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

    init_from_default(true).unwrap();

    if let Some(path) = cli.rlpr_file.as_deref() {
        return run_rlpr_replay(path, &cli);
    }

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
    for _ in 0..NUM_EPISODE {
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
    );
    Ok(())
}
