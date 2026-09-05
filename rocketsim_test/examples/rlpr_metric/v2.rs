//! v2 replay backend for one-car RLPR recordings.
//!
//! Reset builds a new Soccar arena at 120 Hz.
//! Step applies recorded controls for one tick.
//! Snapshot reads car and ball state.
//!
//! Call [`V2Backend::init`] one time before [`V2Backend::new`].
//!
//! Fields with no RLPR source get neutral values:
//! tick counters, air timers, boost timers, supersonic flags,
//! handbrake value, auto flip state, contact state, heatseeker
//! and dropshot ball info. `ball_hit_info` is cleared on reset.
//! `has_double_jumped` and `has_flipped` use the same inference
//! as the v3 comparison path.

use glam::Vec3A;
use rocketsim_rs::{
    math::{RotMat, Vec3},
    sim::{Arena, ArenaConfig, BallState, CarConfig, CarControls, CarState, GameMode, Team},
};
use rocketsim_test::rlpr::{
    cpp_records::{CarRecord, ControlsRecord, Mat3Record, VecRecord},
    tick_record::TickRecord,
};

use super::common::{BodySnapshot, ReplayBackend, Snapshot};

/// v2 sim holder with one blue Octane car.
pub struct V2Backend {
    arena: rocketsim_rs::cxx::UniquePtr<Arena>,
    car_id: u32,
}

/// Load collision meshes. Call one time before use.
pub fn init() {
    rocketsim_rs::init(
        Some(concat!(env!("CARGO_MANIFEST_DIR"), "/../collision_meshes")),
        true,
    );
}

impl V2Backend {
    /// Make a new backend. Call [`init`] first.
    pub fn new() -> Self {
        let (arena, car_id) = fresh_arena();
        Self { arena, car_id }
    }
}

impl Default for V2Backend {
    fn default() -> Self {
        Self::new()
    }
}

impl ReplayBackend for V2Backend {
    fn reset(&mut self, start: &TickRecord) {
        let (arena, car_id) = fresh_arena();
        self.arena = arena;
        self.car_id = car_id;
        self.set_state(start);
    }

    fn set_state(&mut self, state_tick: &TickRecord) {
        let [car] = state_tick.car_records.as_slice() else {
            panic!("state needs one car");
        };
        let mut state = self.arena.pin_mut().get_car(self.car_id);
        apply_car_record(&mut state, car);
        self.arena
            .pin_mut()
            .set_car(self.car_id, state)
            .expect("v2 car id is valid");

        self.arena
            .pin_mut()
            .set_ball(ball_state_for_tick(state_tick));
        self.arena
            .pin_mut()
            .set_car_controls(self.car_id, v2_controls(&car.prev_controls))
            .expect("v2 car id is valid");
    }

    fn step(&mut self, controls: &ControlsRecord) {
        self.arena
            .pin_mut()
            .set_car_controls(self.car_id, v2_controls(controls))
            .expect("v2 car id is valid");
        self.arena.pin_mut().step(1);
    }

    fn snapshot(&mut self) -> Snapshot {
        let car = self.arena.pin_mut().get_car(self.car_id);
        let ball = self.arena.pin_mut().get_ball();
        Snapshot {
            car: BodySnapshot {
                pos: vec_to_glam(car.pos),
                vel: vec_to_glam(car.vel),
                ang_vel: vec_to_glam(car.ang_vel),
                forward: vec_to_glam(car.rot_mat.forward),
                up: vec_to_glam(car.rot_mat.up),
            },
            ball: BodySnapshot {
                pos: vec_to_glam(ball.pos),
                vel: vec_to_glam(ball.vel),
                ang_vel: vec_to_glam(ball.ang_vel),
                forward: vec_to_glam(ball.rot_mat.forward),
                up: vec_to_glam(ball.rot_mat.up),
            },
        }
    }
}

/// Make a Soccar arena at 120 Hz with one blue Octane.
fn fresh_arena() -> (rocketsim_rs::cxx::UniquePtr<Arena>, u32) {
    let config = ArenaConfig {
        no_ball_rot: false,
        ..Default::default()
    };
    let mut arena = Arena::new(GameMode::Soccar, config, 120);
    let car_id = arena.pin_mut().add_car(Team::Blue, CarConfig::octane());
    (arena, car_id)
}

/// Copy one recorded car into a v2 car state.
fn apply_car_record(state: &mut CarState, car: &CarRecord) {
    let controls = v2_controls(&car.prev_controls);
    state.pos = record_vec(car.phys.pos);
    state.rot_mat = v2_rot_mat(&car.phys.rot);
    state.vel = record_vec(car.phys.lin_vel);
    state.ang_vel = record_vec(car.phys.ang_vel);
    state.tick_count_since_update = 0;
    state.is_on_ground = car.is_on_ground;
    state.wheels_with_contact = car.wheels.map(|wheel| wheel.has_contact);
    state.has_jumped = car.has_jumped;
    state.flip_rel_torque = record_vec(car.flip_rel_torque);
    state.jump_time = car.jump_time;
    state.flip_time = car.flip_time;
    state.is_flipping = car.is_flipping;
    state.is_jumping = car.is_jumping;
    state.boost = car.boost_amount * 100.0;
    state.is_demoed = false;
    state.demo_respawn_timer = 0.0;
    state.ball_hit_info = Default::default();
    state.last_controls = controls;

    if car.has_flip {
        state.has_double_jumped = false;
        state.has_flipped = false;
    } else if car.is_flipping {
        state.has_double_jumped = false;
        state.has_flipped = true;
    } else if car.double_jumped_or_flipped && !state.has_flipped {
        state.has_double_jumped = true;
    }
}

/// Make a v2 ball state from a tick ball record.
fn ball_state_for_tick(tick: &TickRecord) -> BallState {
    BallState {
        pos: record_vec(tick.ball_record.pos),
        rot_mat: v2_rot_mat(&tick.ball_record.rot),
        vel: record_vec(tick.ball_record.lin_vel),
        ang_vel: record_vec(tick.ball_record.ang_vel),
        tick_count_since_update: 0,
        hs_info: rocketsim_rs::sim::HeatseekerInfo {
            y_target_dir: 0.0,
            cur_target_speed: 0.0,
            time_since_hit: 0.0,
        },
        ds_info: rocketsim_rs::sim::DropshotInfo {
            charge_level: 0,
            accumulated_hit_force: 0.0,
            y_target_dir: 0.0,
            has_damaged: false,
            last_damage_tick: 0,
        },
    }
}

/// Map RLPR controls to v2 controls.
fn v2_controls(controls: &ControlsRecord) -> CarControls {
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

/// Map RLPR rotation columns to v2 forward, right, up.
fn v2_rot_mat(rot: &Mat3Record) -> RotMat {
    RotMat {
        forward: record_vec(rot.column(0)),
        right: record_vec(rot.column(1)),
        up: record_vec(rot.column(2)),
    }
}

/// Map one RLPR vector to a v2 vector.
fn record_vec(vec: VecRecord) -> Vec3 {
    Vec3::new(vec.x, vec.y, vec.z)
}

/// Map one v2 vector to glam.
fn vec_to_glam(vec: Vec3) -> Vec3A {
    Vec3A::new(vec.x, vec.y, vec.z)
}
