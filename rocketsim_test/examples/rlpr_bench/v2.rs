//! v2 (C++ RocketSim via `rocketsim-rs`) replay backend for `rlpr_bench`.
//!
//! Mirrors `V3Backend`: same reconcile plan, same stale-car parking, same
//! v10 state reconstruction, through the v2 API. Build with `--features v2`
//! and select with `--engine v2`.

use std::sync::Once;

use glam::Vec3A;
use rocketsim_rs::{
    math::{RotMat, Vec3},
    sim::{Arena, ArenaConfig, CarConfig, CarControls, GameMode, Team},
};
use rocketsim_test::rlpr::cpp_records::{ControlsRecord, Mat3Record, VecRecord};

use super::{
    CarAction, CarState, Ctx, Reconcile, ReplayBackend, Snapshot, BodySnapshot, mesh_dir,
};

static INIT: Once = Once::new();

/// Sim holder for N Octane cars in Soccar on the C++ engine.
pub struct V2Backend {
    arena: rocketsim_rs::cxx::UniquePtr<Arena>,
    car_ids: Vec<u32>,
    /// Steps completed on the current arena; a ball hit during the last step
    /// reports `tick_count_when_hit == tick - 1`.
    tick: u64,
    /// Set by `step` when a car-ball hit was emitted this tick.
    last_hit: bool,
}

impl V2Backend {
    /// Make a backend with no cars. `init` runs once per process.
    pub fn new() -> Self {
        INIT.call_once(|| {
            rocketsim_rs::init(Some(&mesh_dir()), true);
        });
        Self {
            arena: Arena::new(GameMode::Soccar, arena_config(), 120),
            car_ids: Vec::new(),
            tick: 0,
            last_hit: false,
        }
    }

    fn ensure_cars(&mut self, count: usize) {
        if self.car_ids.len() == count {
            return;
        }
        self.arena = Arena::new(GameMode::Soccar, arena_config(), 120);
        self.car_ids = (0..count)
            .map(|i| {
                self.arena.pin_mut().add_car(
                    if i % 2 == 0 { Team::Blue } else { Team::Orange },
                    CarConfig::octane(),
                )
            })
            .collect();
        self.tick = 0;
        self.last_hit = false;
    }

    /// Restore one car from its record, mirroring `V3Backend::set_car`'s
    /// v10 reconstruction. The v2 jump timer is in seconds (`jump_time`),
    /// the flip torque is stored already scaled, and the extra-hit
    /// cooldown lives in the preserved `ball_hit_info`.
    fn set_car(&mut self, ctx: &Ctx, car_idx: usize, tick_idx: usize, stale: bool) {
        let record = &ctx.ticks[tick_idx].car_records[car_idx];
        let car_id = self.car_ids[car_idx];
        let controls = ctx
            .ticks
            .get(tick_idx + 1)
            .and_then(|t| t.car_records.get(car_idx))
            .map(|c| v2_controls(&c.prev_controls))
            .unwrap_or_else(|| v2_controls(&record.prev_controls));
        let mut state = self.arena.pin_mut().get_car(car_id);
        state.pos = record_vec(record.phys.pos);
        state.rot_mat = v2_rot_mat(&record.phys.rot);
        state.vel = record_vec(record.phys.lin_vel);
        state.ang_vel = record_vec(record.phys.ang_vel);
        state.tick_count_since_update = 0;
        state.is_on_ground = record.is_on_ground;
        state.wheels_with_contact = record.wheels.map(|wheel| wheel.has_contact);
        state.is_jumping = record.is_jumping;
        // The v10 recorder exposes the current jump input phase; an active
        // state with a released upcoming input is already post-impulse.
        if state.is_jumping && !controls.jump {
            state.is_jumping = false;
        }
        // A reported flip takes precedence over the jump-hold bit in the
        // same delayed input window.
        if record.is_flipping {
            state.is_jumping = false;
        }
        state.is_flipping = record.is_flipping;
        // The v10 writer serializes the current jump hold phase as a
        // zero-time sample on the first active tick.
        state.jump_time = if record.jump_time == 0.0
            && record.is_jumping
            && record.prev_controls.jump
        {
            rocketsim::consts::TICK_TIME
        } else {
            record.jump_time
        };
        // A single in-air `is_jumping` sample with a zero timer that follows
        // a non-jumping record is a phantom press: retire it so it does not
        // receive a hold impulse.
        if record.is_jumping
            && !record.is_on_ground
            && record.jump_time == 0.0
            && controls.jump
            && ctx
                .ticks
                .get(tick_idx.wrapping_sub(1))
                .and_then(|p| p.car_records.get(car_idx))
                .is_some_and(|p| !p.is_jumping)
        {
            state.jump_time = rocketsim_rs::consts::JUMP_MAX_TIME;
        }
        state.flip_time = record.flip_time;
        state.has_jumped = record.has_jumped;
        state.air_time_since_jump = ctx.air_time_since_jump[tick_idx][car_idx];
        state.last_controls = v2_controls(&record.prev_controls);
        // The v10 writer stores the already-scaled dodge torque, which is
        // the representation v2 consumes directly.
        state.flip_rel_torque = record_vec(record.flip_rel_torque);
        state.boost = record.boost_amount * 100.0;
        // RLPR does not serialize the extra-hit cooldown; preserve the
        // recorded contact normal so the live cooldown state survives.
        state.world_contact.has_contact = record.phys.has_world_contact;
        state.world_contact.contact_normal = record_vec(record.phys.world_contact_normal);
        // A car whose phys record repeats verbatim while its physics frame
        // advances was not simulated that tick (demolished or paused
        // placeholder): park it as demoed so it cannot interact.
        state.is_demoed = stale;
        state.demo_respawn_timer = if stale { 1e9 } else { 0.0 };

        if record.has_flip {
            state.has_double_jumped = false;
            state.has_flipped = false;
        } else if record.is_flipping {
            state.has_double_jumped = false;
            state.has_flipped = record.prev_controls.jump || record.flip_time > 0.0;
        } else if record.double_jumped_or_flipped && !state.has_flipped {
            // v10 exposes this bit as the available double-jump/flip phase.
            state.has_double_jumped = false;
        }

        self.arena
            .pin_mut()
            .set_car(car_id, state)
            .expect("v2 car id is valid");
        self.arena
            .pin_mut()
            .set_car_controls(car_id, controls)
            .expect("v2 car id is valid");
    }

    fn set_ball_phys(&mut self, ctx: &Ctx, tick_idx: usize) {
        let record = &ctx.ticks[tick_idx].ball_record;
        let mut ball = self.arena.pin_mut().get_ball();
        ball.pos = record_vec(record.pos);
        ball.rot_mat = v2_rot_mat(&record.rot);
        ball.vel = record_vec(record.lin_vel);
        ball.ang_vel = record_vec(record.ang_vel);
        self.arena.pin_mut().set_ball(ball);
    }

    fn park_ball(&mut self) {
        let mut ball = self.arena.pin_mut().get_ball();
        ball.pos = Vec3::new(0.0, 0.0, 30000.0);
        ball.vel = Vec3::new(0.0, 0.0, 0.0);
        ball.ang_vel = Vec3::new(0.0, 0.0, 0.0);
        self.arena.pin_mut().set_ball(ball);
    }
}

fn arena_config() -> ArenaConfig {
    ArenaConfig {
        no_ball_rot: false,
        ..Default::default()
    }
}

impl ReplayBackend for V2Backend {
    fn reset(&mut self, ctx: &Ctx, start: usize) {
        self.arena = Arena::new(GameMode::Soccar, arena_config(), 120);
        self.car_ids = (0..ctx.ticks[start].car_records.len())
            .map(|i| {
                self.arena.pin_mut().add_car(
                    if i % 2 == 0 { Team::Blue } else { Team::Orange },
                    CarConfig::octane(),
                )
            })
            .collect();
        self.tick = 0;
        self.last_hit = false;
        self.set_state(ctx, start);
    }

    fn set_state(&mut self, ctx: &Ctx, tick_idx: usize) {
        self.ensure_cars(ctx.ticks[tick_idx].car_records.len());
        for car_idx in 0..ctx.ticks[tick_idx].car_records.len() {
            self.set_car(ctx, car_idx, tick_idx, false);
        }
        self.set_ball_phys(ctx, tick_idx);
    }

    fn reconcile_pre(&mut self, ctx: &Ctx, target: usize, plan: &Reconcile) {
        self.ensure_cars(ctx.ticks[target].car_records.len());
        for (j, action) in plan.car.iter().enumerate() {
            match action {
                CarAction::Live => {}
                CarAction::Park | CarAction::Restore => {
                    let car_id = self.car_ids[j];
                    let mut state = self.arena.pin_mut().get_car(car_id);
                    state.is_demoed = true;
                    state.demo_respawn_timer = 1e9;
                    self.arena
                        .pin_mut()
                        .set_car(car_id, state)
                        .expect("v2 car id is valid");
                }
            }
        }
        if plan.ball_teleport || plan.ball_park {
            self.park_ball();
        }
    }

    fn reconcile_post(&mut self, ctx: &Ctx, target: usize, plan: &Reconcile) {
        for (j, action) in plan.car.iter().enumerate() {
            if *action == CarAction::Restore {
                self.set_car(ctx, j, target, false);
            }
        }
        if plan.ball_teleport {
            self.set_ball_phys(ctx, target);
        }
    }

    fn step(&mut self, controls: &[ControlsRecord]) {
        for (control, &car_id) in controls.iter().zip(&self.car_ids) {
            self.arena
                .pin_mut()
                .set_car_controls(car_id, v2_controls(control))
                .expect("v2 car id is valid");
        }
        self.arena.pin_mut().step(1);
        self.tick += 1;
        // `tick_count_when_hit` is stamped with the pre-increment arena tick
        // count, so a hit during this step reads `self.tick - 1`.
        self.last_hit = self.car_ids.iter().any(|&car_id| {
            let car = self.arena.pin_mut().get_car(car_id);
            car.ball_hit_info.is_valid
                && car.ball_hit_info.tick_count_when_hit + 1 == self.tick
        });
    }

    fn snapshot(&mut self) -> Snapshot {
        let cars = self
            .car_ids
            .iter()
            .map(|&car_id| {
                let car = self.arena.pin_mut().get_car(car_id);
                BodySnapshot {
                    pos: vec_to_glam(car.pos),
                    vel: vec_to_glam(car.vel),
                    ang_vel: vec_to_glam(car.ang_vel),
                    forward: vec_to_glam(car.rot_mat.forward),
                    up: vec_to_glam(car.rot_mat.up),
                }
            })
            .collect();
        let ball = self.arena.pin_mut().get_ball();
        Snapshot {
            cars,
            ball: BodySnapshot {
                pos: vec_to_glam(ball.pos),
                vel: vec_to_glam(ball.vel),
                ang_vel: vec_to_glam(ball.ang_vel),
                forward: vec_to_glam(ball.rot_mat.forward),
                up: vec_to_glam(ball.rot_mat.up),
            },
        }
    }

    fn car_state(&mut self, car_idx: usize) -> CarState {
        let s = self.arena.pin_mut().get_car(self.car_ids[car_idx]);
        CarState {
            is_flipping: s.is_flipping,
            flip_time: s.flip_time,
            jump_ticks: (s.jump_time * 120.0).round() as u32,
            has_jumped: s.has_jumped,
            has_flipped: s.has_flipped,
            has_double_jumped: s.has_double_jumped,
            is_jumping: s.is_jumping,
            is_demoed: s.is_demoed,
            ..Default::default()
        }
    }

    fn fox_set_state(&mut self, ctx: &Ctx, state_idx: usize, car_stale: &[bool], ball_park: bool) {
        self.ensure_cars(ctx.ticks[state_idx].car_records.len());
        for j in 0..ctx.ticks[state_idx].car_records.len() {
            self.set_car(ctx, j, state_idx, car_stale.get(j).copied().unwrap_or(false));
        }
        self.fox_set_ball(ctx, state_idx);
        if ball_park {
            self.park_ball();
        }
    }

    fn fox_set_cars(&mut self, ctx: &Ctx, state_idx: usize, car_stale: &[bool]) {
        self.ensure_cars(ctx.ticks[state_idx].car_records.len());
        for j in 0..ctx.ticks[state_idx].car_records.len() {
            self.set_car(ctx, j, state_idx, car_stale.get(j).copied().unwrap_or(false));
        }
    }

    fn fox_set_ball(&mut self, ctx: &Ctx, state_idx: usize) {
        self.set_ball_phys(ctx, state_idx);
    }

    fn ball_was_hit(&self) -> bool {
        self.last_hit
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
