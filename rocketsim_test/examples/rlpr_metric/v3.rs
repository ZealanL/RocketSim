//! V3 replay backend for one-car RLPR recordings.
//!
//! Use one Octane car in Soccar.
//! Reset restores car and ball state from the start tick.
//! Step applies recorded controls for one tick.

use rocketsim::{Arena, CarBodyConfig, CarControls, CarState, GameMode, PhysState, Team};
use rocketsim_test::rlpr::{cpp_records::ControlsRecord, tick_record::TickRecord};

use super::common::{BodySnapshot, ReplayBackend, Snapshot};

/// Sim holder for one blue Octane car in Soccar.
pub struct V3Backend {
    arena: Arena,
    car_id: usize,
}

/// Init RocketSim collision meshes for this example tool.
pub fn init() {
    rocketsim::init(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../collision_meshes"),
        true,
    )
    .expect("init RocketSim collision meshes");
}

impl V3Backend {
    /// Make a backend with one blue Octane car.
    ///
    /// Call [`init`] once before use.
    pub fn new() -> Self {
        let mut arena = Arena::new(GameMode::Soccar);
        let car_id = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        Self { arena, car_id }
    }
}

impl Default for V3Backend {
    fn default() -> Self {
        Self::new()
    }
}

impl ReplayBackend for V3Backend {
    fn reset(&mut self, start: &TickRecord) {
        self.arena = Arena::new(GameMode::Soccar);
        self.car_id = self.arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        self.set_state(start);
    }

    fn set_state(&mut self, state_tick: &TickRecord) {
        let [car] = state_tick.car_records.as_slice() else {
            panic!("state needs one car");
        };
        let recorded: CarState = (*car).into();
        let mut state = *self.arena.get_car_state(self.car_id);
        state.phys = recorded.phys;
        state.is_on_ground = recorded.is_on_ground;
        state.wheels_with_contact = recorded.wheels_with_contact;
        state.is_jumping = recorded.is_jumping;
        state.is_flipping = recorded.is_flipping;
        state.jump_ticks = recorded.jump_ticks;
        state.flip_time = recorded.flip_time;
        state.has_jumped = recorded.has_jumped;
        state.prev_controls = car.prev_controls.into();
        state.controls = car.prev_controls.into();
        state.flip_rel_torque = recorded.flip_rel_torque;
        state.boost = recorded.boost;
        state.is_demoed = false;
        state.demo_respawn_timer = 0.0;

        if car.has_flip {
            state.has_double_jumped = false;
            state.has_flipped = false;
        } else if car.is_flipping {
            state.has_double_jumped = false;
            state.has_flipped = true;
        } else if car.double_jumped_or_flipped && !state.has_flipped {
            state.has_double_jumped = true;
        }

        self.arena.set_car_state(self.car_id, state);

        let recorded_ball: PhysState = state_tick.ball_record.into();
        let mut ball = *self.arena.get_ball_state();
        ball.phys = recorded_ball;
        self.arena.set_ball_state(ball);
    }

    fn step(&mut self, controls: &ControlsRecord) {
        let controls: CarControls = (*controls).into();
        self.arena.set_car_controls(self.car_id, controls);
        self.arena.step_tick();
    }

    fn snapshot(&mut self) -> Snapshot {
        let car = *self.arena.get_car_state(self.car_id);
        let ball = *self.arena.get_ball_state();
        Snapshot {
            car: BodySnapshot {
                pos: car.phys.pos,
                vel: car.phys.vel,
                ang_vel: car.phys.ang_vel,
                forward: car.phys.get_forward_dir(),
                up: car.phys.get_up_dir(),
            },
            ball: BodySnapshot {
                pos: ball.phys.pos,
                vel: ball.phys.vel,
                ang_vel: ball.phys.ang_vel,
                forward: ball.phys.get_forward_dir(),
                up: ball.phys.get_up_dir(),
            },
        }
    }
}
