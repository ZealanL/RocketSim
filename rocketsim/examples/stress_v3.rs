use std::time::Instant;

use clap::Parser;
use fastrand::Rng;
use glam::Vec3A;
use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, BallState, CarBodyConfig, CarControls, CarState, GameMode,
    Team, consts, init_from_default,
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

fn main() {
    let cli = Args::parse();

    init_from_default(true).unwrap();
    let mut arena = Arena::new_with_config(ArenaConfig {
        rng_seed: Some(0),
        ..ArenaConfig::new(cli.game_mode.into())
    });

    let mut rng = Rng::with_seed(0);

    let mut ids = Vec::with_capacity(cli.num_cars as usize);
    for i in 0..cli.num_cars {
        let id = arena.add_car(Team::try_from(i % 2).unwrap(), CarBodyConfig::OCTANE);
        ids.push(id);
    }

    let mut total_ball_touches = 0;
    let start = Instant::now();
    for _ in 0..NUM_EPISODE {
        {
            // Set up new episode
            // Reset to kickoff
            arena.reset_to_random_kickoff(None);

            // Accelerate the ball randomly
            let mut ball_state = *arena.get_ball_state();
            ball_state.phys.vel += Vec3A::new(
                rand_axis_val(&mut rng),
                rand_axis_val(&mut rng),
                rand_axis_val(&mut rng),
            ) * VEL_ADD_MAG;
            arena.set_ball_state(ball_state);
        }

        for _ in 0..NUM_EPISODE_TICKS {
            let ball_state = bot_ball_state(arena.get_ball_state());
            for &idx in &ids {
                let car_state = arena.get_car_state(idx);

                if rand_chance(&mut rng, UPDATE_CHANCE) {
                    let controls = calc_bot_controls(
                        &mut rng,
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

    print_results(
        Instant::now().duration_since(start).as_secs_f32(),
        total_ball_touches,
    );
}
