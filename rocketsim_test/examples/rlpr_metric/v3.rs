//! V3 replay backend for RLPR recordings.
//!
//! Uses one Octane car per recorded car in Soccar (Blue first, then Orange).
//! Reset restores every car and ball state from the start tick.
//! Step applies each recorded car's controls for one tick.

use rocketsim::{
    Arena, ArenaEvent, CarBodyConfig, CarControls, CarState, GameMode, PhysState, Team,
};
use rocketsim_test::rlpr::{cpp_records::ControlsRecord, tick_record::TickRecord};

use super::common::{BodySnapshot, ReplayBackend, SimContactEvents, Snapshot};

/// Sim holder: one Octane per recorded car in Soccar.
pub struct V3Backend {
    arena: Arena,
    car_ids: Vec<usize>,
    dodge_deadzone: f32,
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
    /// Make a backend with no cars. [`reset`] sizes the arena.
    ///
    /// Call [`init`] once before use.
    pub fn new() -> Self {
        Self::with_dodge_deadzone(0.5)
    }

    /// Make a backend with a custom dodge deadzone.
    pub fn with_dodge_deadzone(dodge_deadzone: f32) -> Self {
        Self {
            arena: Arena::new(GameMode::Soccar),
            car_ids: Vec::new(),
            dodge_deadzone,
        }
    }

    /// Octane config with this backend's dodge deadzone.
    fn car_config(&self) -> CarBodyConfig {
        let mut config = CarBodyConfig::OCTANE;
        config.dodge_deadzone = self.dodge_deadzone;
        config
    }

    /// Team per slot: Blue first, then alternating.
    fn team_for_slot(slot: usize) -> Team {
        if slot.is_multiple_of(2) {
            Team::Blue
        } else {
            Team::Orange
        }
    }

    /// Recording slot for one arena car id.
    fn slot_for_arena_car(&self, car_id: usize) -> Option<usize> {
        self.car_ids.iter().position(|&id| id == car_id)
    }

    /// Rebuild the arena when the car count changes.
    fn ensure_cars(&mut self, num_cars: usize) {
        if self.car_ids.len() != num_cars {
            self.arena = Arena::new(GameMode::Soccar);
            let config = self.car_config();
            self.car_ids = (0..num_cars)
                .map(|slot| self.arena.add_car(Self::team_for_slot(slot), config))
                .collect();
        }
    }
}

impl Default for V3Backend {
    fn default() -> Self {
        Self::new()
    }
}

impl ReplayBackend for V3Backend {
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
            let recorded: CarState = (*car).into();
            let mut state = *self.arena.get_car_state(car_id);
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

            self.arena.set_car_state(car_id, state);
        }

        let recorded_ball: PhysState = state_tick.ball_record.into();
        let mut ball = *self.arena.get_ball_state();
        ball.phys = recorded_ball;
        self.arena.set_ball_state(ball);
    }

    fn step(&mut self, controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
        for (slot, controls) in controls.iter().enumerate() {
            if let Some(&car_id) = self.car_ids.get(slot) {
                let controls: CarControls = (*controls).into();
                self.arena.set_car_controls(car_id, controls);
            }
        }
        self.arena.step_tick();
        let mut observed = vec![SimContactEvents::default(); self.car_ids.len()];
        for event in self.arena.get_last_step_events() {
            match event {
                ArenaEvent::BallHitWorld(_) => {
                    for slot in observed.iter_mut() {
                        slot.ball_world = true;
                    }
                }
                ArenaEvent::CarHitBall(hit) => {
                    if let Some(slot) = self.slot_for_arena_car(hit.car_idx) {
                        observed[slot].car_ball = true;
                    }
                }
                ArenaEvent::CarHitCar(hit) => {
                    for arena_car in [hit.bumper_car_idx, hit.victim_car_idx] {
                        if let Some(slot) = self.slot_for_arena_car(arena_car) {
                            observed[slot].car_car = true;
                        }
                    }
                }
                ArenaEvent::CarHitWorld(hit) => {
                    if let Some(slot) = self.slot_for_arena_car(hit.car_idx) {
                        observed[slot].chassis_world = true;
                    }
                }
                _ => {}
            }
        }
        observed
    }

    fn snapshot(&mut self, car_idx: usize) -> Snapshot {
        let Some(&car_id) = self.car_ids.get(car_idx) else {
            panic!("snapshot needs car {car_idx}");
        };
        let car = *self.arena.get_car_state(car_id);
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
