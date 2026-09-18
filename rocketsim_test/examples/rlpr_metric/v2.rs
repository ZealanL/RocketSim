//! v2 replay backend for RLPR recordings.
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

use super::common::{BodySnapshot, ReplayBackend, SimContactEvents, Snapshot};

// Bump events observed during the last stepped tick: (bumper, victim).
// Collected by v2_bump_callback into a thread-local because cxx
// callbacks cannot capture state.
thread_local! {
    static V2_BUMP_EVENTS: std::cell::RefCell<Vec<(u32, u32)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

/// v2 car-bump callback. Records ids only; never touches the arena.
fn v2_bump_callback(
    _arena: std::pin::Pin<&mut Arena>,
    bumper: u32,
    victim: u32,
    _is_demo: bool,
    _user_data: usize,
) {
    V2_BUMP_EVENTS.with(|events| events.borrow_mut().push((bumper, victim)));
}

/// v2 sim holder with one Octane per recorded car (Blue first, then Orange).
pub struct V2Backend {
    arena: rocketsim_rs::cxx::UniquePtr<Arena>,
    car_ids: Vec<u32>,
    dodge_deadzone: f32,
    /// Last observed ball-hit tick per car, for hit change detection.
    last_ball_hit: Vec<u64>,
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
        Self::with_dodge_deadzone(0.5)
    }

    /// Make a new backend with a custom dodge deadzone.
    pub fn with_dodge_deadzone(dodge_deadzone: f32) -> Self {
        let (arena, car_ids) = fresh_arena(1, dodge_deadzone);
        let last_ball_hit = vec![0; car_ids.len()];
        Self {
            arena,
            car_ids,
            dodge_deadzone,
            last_ball_hit,
        }
    }

    /// Rebuild the arena when the car count changes.
    fn ensure_cars(&mut self, num_cars: usize) {
        if self.car_ids.len() != num_cars {
            let (arena, car_ids) = fresh_arena(num_cars, self.dodge_deadzone);
            self.arena = arena;
            self.car_ids = car_ids;
            self.last_ball_hit = vec![0; num_cars];
        }
    }
}

impl Default for V2Backend {
    fn default() -> Self {
        Self::new()
    }
}

impl ReplayBackend for V2Backend {
    fn reset(&mut self, start: &TickRecord) {
        self.ensure_cars(start.car_records.len());
        self.set_state(start);
    }

    fn set_state(&mut self, state_tick: &TickRecord) {
        self.ensure_cars(state_tick.car_records.len());
        for (slot, car) in state_tick.car_records.iter().enumerate() {
            let Some(&car_id) = self.car_ids.get(slot) else {
                panic!("state has more cars than the arena");
            };
            let mut state = self.arena.pin_mut().get_car(car_id);
            apply_car_record(&mut state, car);
            self.arena
                .pin_mut()
                .set_car(car_id, state)
                .expect("v2 car id is valid");
            self.arena
                .pin_mut()
                .set_car_controls(car_id, v2_controls(&car.prev_controls))
                .expect("v2 car id is valid");
        }

        self.arena
            .pin_mut()
            .set_ball(ball_state_for_tick(state_tick));
    }

    fn step(&mut self, controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
        for (slot, controls) in controls.iter().enumerate() {
            if let Some(&car_id) = self.car_ids.get(slot) {
                self.arena
                    .pin_mut()
                    .set_car_controls(car_id, v2_controls(controls))
                    .expect("v2 car id is valid");
            }
        }
        self.arena.pin_mut().step(1);
        // Drain bump events collected by v2_bump_callback during the step.
        let mut observed = vec![SimContactEvents::default(); self.car_ids.len()];
        V2_BUMP_EVENTS.with(|events| {
            for (bumper, victim) in events.borrow_mut().drain(..) {
                for arena_car in [bumper, victim] {
                    if let Some(slot) = self.car_ids.iter().position(|&id| id == arena_car) {
                        observed[slot].car_car = true;
                    }
                }
            }
        });
        // Car/ball and chassis/world contacts come from sim state:
        // ball_hit_info is per-touch (change-detected against latching),
        // world_contact reflects the current tick.
        for (slot, &car_id) in self.car_ids.iter().enumerate() {
            let state = self.arena.pin_mut().get_car(car_id);
            let hit = state.ball_hit_info;
            if hit.is_valid && hit.tick_count_when_hit != self.last_ball_hit[slot] {
                observed[slot].car_ball = true;
            }
            self.last_ball_hit[slot] = hit.tick_count_when_hit;
            if state.world_contact.has_contact {
                observed[slot].chassis_world = true;
            }
        }
        observed
    }

    fn snapshot(&mut self, car_idx: usize) -> Snapshot {
        let Some(&car_id) = self.car_ids.get(car_idx) else {
            panic!("snapshot needs car {car_idx}");
        };
        let car = self.arena.pin_mut().get_car(car_id);
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

/// Make a Soccar arena at 120 Hz with one Octane per recorded car.
fn fresh_arena(
    num_cars: usize,
    dodge_deadzone: f32,
) -> (rocketsim_rs::cxx::UniquePtr<Arena>, Vec<u32>) {
    let config = ArenaConfig {
        no_ball_rot: false,
        ..Default::default()
    };
    let mut arena = Arena::new(GameMode::Soccar, config, 120);
    arena.pin_mut().set_car_bump_callback(v2_bump_callback, 0);
    let car_ids = (0..num_cars.max(1))
        .map(|slot| {
            let team = if slot.is_multiple_of(2) {
                Team::Blue
            } else {
                Team::Orange
            };
            let mut car_config = *CarConfig::octane();
            car_config.dodge_deadzone = dodge_deadzone;
            arena.pin_mut().add_car(team, &car_config)
        })
        .collect();
    (arena, car_ids)
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
