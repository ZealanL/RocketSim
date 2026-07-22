use clap::{Parser, ValueEnum};
use fastrand::Rng;
use glam::Vec3A;

pub const NUM_CARS: u8 = 6;
pub const NUM_EPISODE_TICKS: usize = 10_000;
pub const NUM_EPISODE: usize = 100;
pub const TOTAL_TICKS: usize = NUM_EPISODE * NUM_EPISODE_TICKS;
pub const VEL_ADD_MAG: f32 = 1000.0;
pub const UPDATE_CHANCE: f32 = 0.05; // (120 * 0.05) = Avg of 6 per sec

#[derive(Clone, Copy, Debug, ValueEnum)]
#[clap(rename_all = "kebab-case")]
pub enum GameModeArg {
    Soccar,
    Hoops,
    Heatseeker,
    Snowday,
    Dropshot,
    #[clap(name = "void")]
    TheVoid,
}

#[derive(Clone, Copy, Debug, ValueEnum)]
#[clap(rename_all = "kebab-case")]
pub enum MemWeightModeArg {
    Light,
    Balanced,
    Heavy,
}

#[derive(Parser)]
pub struct Args {
    #[arg(short, long, default_value_t = NUM_CARS)]
    pub num_cars: u8,
    #[arg(short, long, value_enum, default_value_t = GameModeArg::Soccar)]
    pub game_mode: GameModeArg,
    #[arg(long, value_enum, default_value_t = MemWeightModeArg::Heavy)]
    pub mem_weight_mode: MemWeightModeArg,
    #[arg(long, default_value_t = 1)]
    pub num_arenas: usize,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BotControls {
    pub throttle: f32,
    pub steer: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub jump: bool,
    pub boost: bool,
    pub handbrake: bool,
}

impl BotControls {
    #[must_use]
    pub fn clamp(mut self) -> Self {
        self.throttle = self.throttle.clamp(-1.0, 1.0);
        self.steer = self.steer.clamp(-1.0, 1.0);
        self.pitch = self.pitch.clamp(-1.0, 1.0);
        self.yaw = self.yaw.clamp(-1.0, 1.0);
        self.roll = self.roll.clamp(-1.0, 1.0);
        self
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BotCarState {
    pub pos: Vec3A,
    pub ang_vel: Vec3A,
    pub forward: Vec3A,
    pub right: Vec3A,
    pub is_on_ground: bool,
    pub is_jumping: bool,
    pub has_flip_or_jump: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct BotBallState {
    pub pos: Vec3A,
    pub vel: Vec3A,
}

pub fn rand_axis_val(rng: &mut Rng) -> f32 {
    rng.f32() * 2.0 - 1.0
}

pub fn rand_chance(rng: &mut Rng, thresh: f32) -> bool {
    rng.f32() < thresh
}

pub fn calc_bot_controls(
    rng: &mut Rng,
    car_state: BotCarState,
    ball_state: BotBallState,
    car_max_speed: f32,
) -> BotControls {
    let ball_delta = ball_state.pos - car_state.pos;

    let min_reach_time = ball_delta.length() / car_max_speed;
    let extrap_ball_pos =
        ball_state.pos + (ball_state.vel * Vec3A::new(1.0, 1.0, 0.0) * min_reach_time);

    let extrap_ball_delta = extrap_ball_pos - car_state.pos;
    let ball_forward_align = extrap_ball_delta.dot(car_state.forward);
    let ball_right_align = extrap_ball_delta.dot(car_state.right);

    let mut controls = BotControls {
        throttle: 1.0,
        steer: (ball_right_align * 80.0).clamp(-1.0, 1.0),
        ..Default::default()
    };

    {
        // Driving control
        if (ball_forward_align < 0.0) && // Ball is somewhat behind
            (car_state.ang_vel.z.abs() >= 1.0) && // We are turning
            (car_state.pos.z < 300.0) && // We are somewhat grounded
            rand_chance(rng, 0.8)
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
        controls.yaw = rand_axis_val(rng);
        controls.pitch = rand_axis_val(rng);
        controls.roll = rand_axis_val(rng) * rng.f32();

        if car_state.is_on_ground {
            controls.jump = rand_chance(rng, 0.04);
        } else {
            if car_state.is_jumping {
                // Keep holding jump sometimes
                controls.jump = rand_chance(rng, 0.5);
            } else {
                // Flip chance when in air
                controls.jump = rand_chance(rng, 0.1);
            }

            if !car_state.is_jumping && car_state.has_flip_or_jump && controls.jump {
                if rand_chance(rng, 0.5) {
                    // Align direction towards ball
                    let align_frac = rng.f32().sqrt();
                    controls.pitch *= 1.0 - align_frac;
                    controls.yaw *= 1.0 - align_frac;
                    controls.pitch += -ball_forward_align * align_frac;
                    controls.yaw += ball_right_align * align_frac;
                } else if rand_chance(rng, 0.2) {
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
        controls.throttle += rand_axis_val(rng) * rng.f32().powi(3);
        controls.steer += rand_axis_val(rng) * rng.f32().powi(3);
        controls.yaw += rand_axis_val(rng) * rng.f32();
        controls.pitch += rand_axis_val(rng) * rng.f32();
        controls.roll += rand_axis_val(rng) * rng.f32();

        if rand_chance(rng, 0.2) {
            controls.jump = !controls.jump;
        }

        if rand_chance(rng, 0.2) {
            controls.boost = !controls.boost;
        }

        if rand_chance(rng, 0.2) {
            controls.handbrake = !controls.handbrake;
        }
    }

    controls.clamp()
}

pub fn print_results(elapsed: f32, total_ball_touches: usize) {
    let tps = (TOTAL_TICKS as f32) / elapsed;
    println!(
        "Elapsed: {elapsed}\
        \nTPS: {tps:.0}\
        \nBall hits: {total_ball_touches}"
    );
}
