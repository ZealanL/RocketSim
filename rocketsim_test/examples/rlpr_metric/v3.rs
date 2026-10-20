//! V3 replay backend for RLPR recordings.
//!
//! Uses one car per recorded car in Soccar (Blue first, then Orange),
//! with the body preset from the recording header.
//! Reset restores every car and ball state from the start tick.
//! Step applies each recorded car's controls for one tick.

use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, ArenaMemWeightMode, CarBodyConfig, CarControls, GameMode,
    HITBOX_OFFSETS, HITBOX_SIZES, PhysState, Team,
};
use rocketsim_test::rlpr::{
    cpp_records::{ControlsRecord, RecordingInfo},
    tick_record::TickRecord,
};

use super::common::{
    BODY_PRESET_NAMES, BodySnapshot, HitboxBounds, NUM_BODY_PRESETS, ReplayBackend,
    SimContactEvents, Snapshot, body_preset_from_info,
};

/// Sim holder: one car per recorded car in Soccar.
///
/// The body presets come from [`V3Backend::set_bodies`] or
/// [`V3Backend::set_body_from_info`]; they stay Octane until the caller
/// selects a roster. Presets are per car slot, so a mixed roster replays
/// with each car on its own body.
pub struct V3Backend {
    arena: Arena,
    car_ids: Vec<usize>,
    /// Body preset index per car slot, in [`BODY_PRESET_NAMES`] order.
    bodies: Vec<usize>,
    /// Presets the current arena was built with, to detect roster changes.
    arena_body: Option<Vec<CarBodyConfig>>,
    dodge_deadzone: f32,
    mem_weight_mode: ArenaMemWeightMode,
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
        Self::with_mem_weight_mode(dodge_deadzone, ArenaMemWeightMode::default())
    }

    /// Make a backend with an explicit arena memory mode.
    pub fn with_mem_weight_mode(dodge_deadzone: f32, mem_weight_mode: ArenaMemWeightMode) -> Self {
        Self {
            arena: Arena::new_with_config(
                ArenaConfig::new(GameMode::Soccar).with_mem_weight_mode(mem_weight_mode),
            ),
            car_ids: Vec::new(),
            bodies: Vec::new(),
            arena_body: None,
            dodge_deadzone,
            mem_weight_mode,
        }
    }

    fn new_arena(&self) -> Arena {
        Arena::new_with_config(
            ArenaConfig::new(GameMode::Soccar).with_mem_weight_mode(self.mem_weight_mode),
        )
    }

    /// Body config for one car slot, with this backend's dodge deadzone.
    fn car_config(&self, slot: usize) -> CarBodyConfig {
        let mut config = BODY_PRESETS[self.preset_for_slot(slot)];
        config.dodge_deadzone = self.dodge_deadzone;
        config
    }

    /// Preset index for one car slot. Missing slots fall back to Octane so a
    /// short or empty roster still builds a playable arena.
    fn preset_for_slot(&self, slot: usize) -> usize {
        self.bodies.get(slot).copied().unwrap_or(OCTANE_PRESET)
    }

    /// Select one body preset for every car slot from a recording header.
    ///
    /// Call once per recording before `reset`, or use [`Self::set_bodies`] for
    /// a mixed roster. The next `reset`/`set_state` rebuilds the arena when
    /// the roster differs, even when the car count is unchanged. Unknown
    /// headers return an error and leave the previous roster in place: the
    /// metric must fail rather than score a guessed body. The header samples
    /// the first recorded car only, so it cannot see a mixed roster.
    #[allow(dead_code)]
    pub fn set_body_from_info(&mut self, info: &RecordingInfo) -> Result<&'static str, String> {
        let index = body_preset_index(info)?;
        self.bodies = vec![index; info.num_cars as usize];
        Ok(BODY_PRESET_NAMES[index])
    }

    /// Select a body preset per car slot, in recording order.
    ///
    /// This is the mixed-roster path: RLPR headers carry one hitbox for the
    /// whole file, so a replay with more than one car body has to come from
    /// per-car detection. A short roster pads with Octane.
    pub fn set_bodies(&mut self, bodies: &[usize]) {
        self.bodies = bodies.to_vec();
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
    fn slot_for_arena_car(&self, arena_car: usize) -> Option<usize> {
        self.car_ids.iter().position(|&stored| stored == arena_car)
    }

    /// Arena car id for one recording slot.
    ///
    /// Panics as `<caller> needs car <index>` on a missing slot.
    #[track_caller]
    fn car_id(&self, car_idx: usize, caller: &str) -> usize {
        match self.car_ids.get(car_idx) {
            Some(&id) => id,
            None => panic!("{caller} needs car {car_idx}"),
        }
    }

    /// Rebuild the arena when the car count or the body roster changes.
    ///
    /// The roster check matters when consecutive recordings hold the same car
    /// count with different bodies: the ids would still line up, but the
    /// hitbox and wheels would stay wrong without a rebuild.
    fn ensure_cars(&mut self, num_cars: usize) {
        let configs: Vec<CarBodyConfig> = (0..num_cars).map(|slot| self.car_config(slot)).collect();
        if self.car_ids.len() != num_cars || self.arena_body.as_ref() != Some(&configs) {
            self.arena = self.new_arena();
            self.car_ids = configs
                .iter()
                .enumerate()
                .map(|(slot, &config)| self.arena.add_car(Self::team_for_slot(slot), config))
                .collect();
            self.arena_body = Some(configs);
        }
    }

    /// Restore the handbrake integrator before a replayed tick.
    pub fn set_handbrake_value(&mut self, car_idx: usize, value: f32) {
        let car_id = self.car_id(car_idx, "set_handbrake_value");
        self.arena.edit_car_state_full(car_id, |state| {
            state.handbrake_val = value.clamp(0.0, 1.0);
        });
    }

    /// Restore jump-hold continuity before a replayed tick.
    pub fn set_jump_hold_broken(&mut self, car_idx: usize, broken: bool) {
        let car_id = self.car_id(car_idx, "set_jump_hold_broken");
        self.arena.edit_car_state_full(car_id, |state| {
            state.jump_hold_broken = broken;
        });
    }

    /// Refresh prior-tick wheel gates without advancing dynamics.
    pub fn refresh_sticky_gates(&mut self) {
        for &car_id in &self.car_ids {
            self.arena.refresh_car_sticky_gate(car_id);
        }
    }

    /// Motion-only reset: overwrite exactly car/ball `pos`/`vel`/`ang_vel`.
    ///
    /// Edit three fields in place. Preserve orientation and logical state.
    /// Editors clear transient accumulators and the car
    /// impulse cache. Do not explicitly clear wheel or manifold caches, refresh
    /// sticky gates, or rebuild the arena. The segment seed sizes the arena.
    /// Panics on a missing slot. Harness only.
    pub fn reset_motion(&mut self, state_tick: &TickRecord) {
        for (slot, car) in state_tick.car_records.iter().enumerate() {
            let car_id = self.car_id(slot, "reset_motion");
            let recorded: PhysState = car.phys.into();
            self.arena.edit_car_state_full(car_id, |live| {
                live.phys.pos = recorded.pos;
                live.phys.vel = recorded.vel;
                live.phys.ang_vel = recorded.ang_vel;
            });
        }
        let recorded_ball: PhysState = state_tick.ball_record.into();
        self.arena.edit_ball_state(|ball| {
            ball.phys.pos = recorded_ball.pos;
            ball.phys.vel = recorded_ball.vel;
            ball.phys.ang_vel = recorded_ball.ang_vel;
        });
    }

    /// Step the arena without collecting replay metric events.
    #[allow(dead_code)]
    pub fn benchmark_step(&mut self, controls: &[CarControls]) {
        for (slot, &controls) in controls.iter().enumerate() {
            if let Some(&car_id) = self.car_ids.get(slot) {
                self.arena.set_car_controls(car_id, controls);
            }
        }
        let _ = self.arena.step_tick();
    }
}

/// Body presets in [`HITBOX_SIZES`]/[`HITBOX_OFFSETS`] order.
const BODY_PRESETS: [CarBodyConfig; 7] = [
    CarBodyConfig::OCTANE,
    CarBodyConfig::DOMINUS,
    CarBodyConfig::PLANK,
    CarBodyConfig::BREAKOUT,
    CarBodyConfig::HYBRID,
    CarBodyConfig::MERC,
    CarBodyConfig::PSYCLOPS,
];

/// Preset index for Octane, the default body of a fresh backend.
pub const OCTANE_PRESET: usize = 0;

/// v3 preset bounds in [`common::BODY_PRESET_NAMES`] order.
const BODY_BOUNDS: [HitboxBounds; NUM_BODY_PRESETS] = [
    HitboxBounds {
        size: HITBOX_SIZES[0],
        offset: HITBOX_OFFSETS[0],
    },
    HitboxBounds {
        size: HITBOX_SIZES[1],
        offset: HITBOX_OFFSETS[1],
    },
    HitboxBounds {
        size: HITBOX_SIZES[2],
        offset: HITBOX_OFFSETS[2],
    },
    HitboxBounds {
        size: HITBOX_SIZES[3],
        offset: HITBOX_OFFSETS[3],
    },
    HitboxBounds {
        size: HITBOX_SIZES[4],
        offset: HITBOX_OFFSETS[4],
    },
    HitboxBounds {
        size: HITBOX_SIZES[5],
        offset: HITBOX_OFFSETS[5],
    },
    HitboxBounds {
        size: HITBOX_SIZES[6],
        offset: HITBOX_OFFSETS[6],
    },
];

/// Match recording header hitbox bounds to one known body preset.
///
/// Unknown bounds are an error: the metric must fail rather than score a
/// guessed body. See [`common::body_preset_from_info`].
pub fn body_preset_index(info: &RecordingInfo) -> Result<usize, String> {
    body_preset_from_info(info, &BODY_BOUNDS)
}

/// Ticks a body needs to come to rest on flat ground.
const SETTLE_TICKS: u32 = 600;

/// Settled chassis height of every preset, in uu, in [`BODY_PRESET_NAMES`] order.
///
/// Drops one car per preset on flat ground and reads the height it settles
/// at. The engine is the reference here on purpose: this is suspension
/// tuning, not a geometric constant, so hardcoding the numbers would rot
/// whenever the suspension is retuned. It is measured, not derived.
///
/// Call [`init`] first. Used by [`common::bodies_from_ticks`] to fingerprint
/// each recorded car.
pub fn preset_settled_heights() -> [f32; NUM_BODY_PRESETS] {
    std::array::from_fn(|index| {
        let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
        let car_id = arena.add_car(Team::Blue, BODY_PRESETS[index]);
        for _ in 0..SETTLE_TICKS {
            arena.step_tick();
        }
        arena.get_car_state(car_id).phys.pos.z
    })
}

impl Default for V3Backend {
    fn default() -> Self {
        Self::new()
    }
}

impl ReplayBackend for V3Backend {
    fn set_handbrake_value(&mut self, car_idx: usize, value: f32) {
        V3Backend::set_handbrake_value(self, car_idx, value);
    }

    fn set_jump_hold_broken(&mut self, car_idx: usize, broken: bool) {
        V3Backend::set_jump_hold_broken(self, car_idx, broken);
    }

    fn refresh_sticky_gates(&mut self) {
        V3Backend::refresh_sticky_gates(self);
    }

    fn set_boost_state(&mut self, car_idx: usize, armed: bool, time: f32) {
        let car_id = self.car_id(car_idx, "set_boost_state");
        self.arena.edit_car_state_full(car_id, |state| {
            state.is_boosting = armed;
            state.boosting_time = time;
        });
    }

    fn supports_cooldown_restore(&self) -> bool {
        true
    }

    fn suppress_next_extra_hit(&mut self, car_idx: usize, suppress: bool) {
        let car_id = self.car_id(car_idx, "suppress_next_extra_hit");
        // Block exactly the next executed step: the upcoming step runs at
        // the current tick count, and the gate re-opens the step after.
        // Fresh arenas (count 0) cannot represent "block step 0, allow
        // step 1" in u64, so they over-suppress by one step; rebuilds are
        // the only path there and are rare.
        let last_extra_hit_tick = if suppress {
            let current = self.arena.tick_count();
            Some(if current == 0 { 0 } else { current - 1 })
        } else {
            None
        };
        self.arena.edit_car_state_full(car_id, |state| {
            state.last_extra_hit_tick = last_extra_hit_tick;
        });
    }

    fn supports_bump_restore(&self) -> bool {
        true
    }

    fn set_bump_cooldown(&mut self, car_idx: usize, victim_idx: usize, seconds: f32) {
        let car_id = self.car_id(car_idx, "set_bump_cooldown");
        let victim_id = self.car_id(victim_idx, "set_bump_cooldown");
        self.arena.edit_car_state_full(car_id, |state| {
            state.bump_cooldown_timer = seconds.max(0.0);
            state.bump_cooldown_victim = (seconds > 0.0).then_some(victim_id);
        });
    }

    fn reset(&mut self, start: &TickRecord) {
        self.ensure_cars(start.car_records.len());
        self.set_state(start);
    }

    fn reset_motion(&mut self, state_tick: &TickRecord) {
        V3Backend::reset_motion(self, state_tick);
    }

    fn set_state(&mut self, state_tick: &TickRecord) {
        self.ensure_cars(state_tick.car_records.len());
        for (slot, car) in state_tick.car_records.iter().enumerate() {
            let Some(&car_id) = self.car_ids.get(slot) else {
                panic!("state has more cars than the arena");
            };
            self.arena.edit_car_state_full(car_id, |state| {
                state
                    .set_contact_flags(car.is_on_ground, car.wheels.map(|wheel| wheel.has_contact));
                state.set_prev_controls(car.prev_controls.into());
                state.phys = car.phys.into();
                state.is_jumping = car.is_jumping;
                state.is_flipping = car.is_flipping;
                state.jump_ticks = (car.jump_time * rocketsim::consts::TICK_RATE).round() as u32;
                state.flip_time = car.flip_time;
                state.has_jumped = car.has_jumped;
                state.controls = car.prev_controls.into();
                state.flip_rel_torque = car.flip_rel_torque.into();
                state.boost = car.boost_amount * 100.0;
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
            });
        }

        let recorded_ball: PhysState = state_tick.ball_record.into();
        self.arena.edit_ball_state(|ball| {
            ball.phys = recorded_ball;
        });
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
        let car_id = self.car_id(car_idx, "snapshot");
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

#[cfg(test)]
mod tests {
    use glam::Vec3A;
    use rocketsim::{CarState, consts::BT_TO_UU};
    use rocketsim_test::rlpr::cpp_records::VecRecord;

    use super::{super::common::SETTLED_HEIGHT_TOL_UU, *};

    fn info_for(min: [f32; 3], max: [f32; 3]) -> RecordingInfo {
        RecordingInfo {
            num_cars: 4,
            hitbox_rel_min_bt: VecRecord::new(min[0], min[1], min[2]),
            hitbox_rel_max_bt: VecRecord::new(max[0], max[1], max[2]),
        }
    }

    /// Exact Daizen header bounds (decompressed bytes 17-40).
    fn daizen_info() -> RecordingInfo {
        info_for(
            [-1.133026, -0.871704, -0.077060],
            [1.493369, 0.871704, 0.560828],
        )
    }

    /// Shared Wisp/PartyCannon header bounds (decompressed bytes 17-40).
    fn octane_info() -> RecordingInfo {
        info_for(
            [-0.927561, -0.866994, 0.028509],
            [1.482587, 0.866994, 0.801690],
        )
    }

    #[test]
    fn daizen_header_maps_to_plank() {
        let index = body_preset_index(&daizen_info()).expect("Daizen header is a known preset");
        assert_eq!(BODY_PRESET_NAMES[index], "Plank");
        assert_eq!(
            BODY_PRESETS[index].hitbox_size,
            CarBodyConfig::PLANK.hitbox_size
        );
        assert_eq!(
            BODY_PRESETS[index].hitbox_pos_offset,
            CarBodyConfig::PLANK.hitbox_pos_offset
        );
    }

    #[test]
    fn octane_header_stays_octane() {
        let index = body_preset_index(&octane_info()).expect("Wisp header is a known preset");
        assert_eq!(BODY_PRESET_NAMES[index], "Octane");
        assert_eq!(
            BODY_PRESETS[index].hitbox_size,
            CarBodyConfig::OCTANE.hitbox_size
        );
        assert_eq!(
            BODY_PRESETS[index].hitbox_pos_offset,
            CarBodyConfig::OCTANE.hitbox_pos_offset
        );
    }

    #[test]
    fn closest_presets_stay_distinct() {
        // Psyclops is Octane + 0.134 uu on every size axis: with a 0.01 uu
        // tolerance it must match Psyclops, never Octane.
        let half = HITBOX_SIZES[6] * 0.5;
        let min = (HITBOX_OFFSETS[6] - half) / BT_TO_UU;
        let max = (HITBOX_OFFSETS[6] + half) / BT_TO_UU;
        let info = info_for(min.to_array(), max.to_array());
        let index = body_preset_index(&info).expect("exact preset bounds match");
        assert_eq!(BODY_PRESET_NAMES[index], "Psyclops");
    }

    #[test]
    fn unknown_header_is_an_error() {
        let info = info_for([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]);
        assert!(body_preset_index(&info).is_err());
    }

    #[test]
    fn same_count_body_switch_rebuilds_arena() {
        init();
        let mut backend = V3Backend::new();
        backend.set_body_from_info(&octane_info()).unwrap();
        backend.ensure_cars(4);
        // Same body, same count: no rebuild, so a planted state survives.
        let sentinel = Vec3A::new(1234.0, 567.0, 89.0);
        let mut planted = *backend.arena.get_car_state(backend.car_ids[0]);
        planted.phys.pos = sentinel;
        backend
            .arena
            .set_car_state_full(backend.car_ids[0], planted);
        backend.ensure_cars(4);
        assert_eq!(
            backend.arena.get_car_state(backend.car_ids[0]).phys.pos,
            sentinel
        );
        // Same count, new body: rebuild, so the planted state is gone.
        assert_eq!(backend.set_body_from_info(&daizen_info()).unwrap(), "Plank");
        backend.ensure_cars(4);
        assert_eq!(backend.car_ids.len(), 4);
        assert_ne!(
            backend.arena.get_car_state(backend.car_ids[0]).phys.pos,
            sentinel
        );
    }

    #[test]
    fn unknown_header_keeps_previous_roster() {
        init();
        let mut backend = V3Backend::new();
        backend.set_bodies(&[2, 0]);
        let bad = info_for([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]);
        assert!(backend.set_body_from_info(&bad).is_err());
        assert_eq!(backend.bodies, vec![2, 0]);
    }

    #[test]
    fn mixed_roster_builds_per_car_bodies() {
        init();
        let mut backend = V3Backend::new();
        // Plank against Octane, as in the bundled london_vs_nexto capture.
        backend.set_bodies(&[2, 0]);
        backend.ensure_cars(2);
        assert_eq!(backend.arena_body.as_ref().unwrap().len(), 2);
        assert_eq!(
            backend.arena_body.as_ref().unwrap()[0].hitbox_size,
            CarBodyConfig::PLANK.hitbox_size
        );
        assert_eq!(
            backend.arena_body.as_ref().unwrap()[1].hitbox_size,
            CarBodyConfig::OCTANE.hitbox_size
        );
    }

    #[test]
    fn short_roster_pads_with_octane() {
        init();
        let mut backend = V3Backend::new();
        backend.set_bodies(&[2]);
        backend.ensure_cars(3);
        let configs = backend.arena_body.as_ref().unwrap();
        assert_eq!(configs[0].hitbox_size, CarBodyConfig::PLANK.hitbox_size);
        assert_eq!(configs[1].hitbox_size, CarBodyConfig::OCTANE.hitbox_size);
        assert_eq!(configs[2].hitbox_size, CarBodyConfig::OCTANE.hitbox_size);
    }

    #[test]
    fn roster_change_rebuilds_without_count_change() {
        init();
        let mut backend = V3Backend::new();
        backend.set_bodies(&[0, 0]);
        backend.ensure_cars(2);
        let before = backend.arena_body.clone();
        backend.set_bodies(&[2, 0]);
        backend.ensure_cars(2);
        assert_ne!(backend.arena_body, before);
    }

    #[test]
    fn suppress_next_extra_hit_blocks_exactly_one_step() {
        use super::super::common::ReplayBackend;

        init();
        let mut backend = V3Backend::new();
        backend.ensure_cars(1);
        let car_id = backend.car_ids[0];
        // Fresh backend: gating open.
        assert_eq!(
            backend.arena.get_car_state(car_id).last_extra_hit_tick,
            None
        );
        // Suppress with the arena at count 0: over-suppresses by one step
        // (documented u64 edge), still blocks the immediate next step.
        assert_eq!(backend.arena.tick_count(), 0);
        backend.suppress_next_extra_hit(0, true);
        assert_eq!(
            backend.arena.get_car_state(car_id).last_extra_hit_tick,
            Some(0)
        );
        // Clearing re-opens the gate.
        backend.suppress_next_extra_hit(0, false);
        assert_eq!(
            backend.arena.get_car_state(car_id).last_extra_hit_tick,
            None
        );
        // At count C >= 1 the block covers exactly the next step: last is
        // set to C - 1, so last + 1 < C fails once, then passes.
        backend.arena.step_tick();
        assert_eq!(backend.arena.tick_count(), 1);
        backend.suppress_next_extra_hit(0, true);
        assert_eq!(
            backend.arena.get_car_state(car_id).last_extra_hit_tick,
            Some(0)
        );
    }

    #[test]
    fn preset_settled_heights_separate_the_wheels() {
        init();
        let heights = preset_settled_heights();
        for (index, height) in heights.iter().enumerate() {
            assert!(
                height.is_finite() && *height > 0.0,
                "{} settled at {height}",
                BODY_PRESET_NAMES[index]
            );
        }
        // Octane and Hybrid share their wheels and are the one genuine tie.
        assert!((heights[0] - heights[4]).abs() < SETTLED_HEIGHT_TOL_UU);
        // Every other preset must be separable by the match tolerance.
        for a in 0..NUM_BODY_PRESETS {
            for b in (a + 1)..NUM_BODY_PRESETS {
                let gap = (heights[a] - heights[b]).abs();
                let tied = (a == 0 && b == 4) || (a == 4 && b == 0);
                assert!(
                    tied || gap > SETTLED_HEIGHT_TOL_UU,
                    "{} and {} are only {gap} uu apart",
                    BODY_PRESET_NAMES[a],
                    BODY_PRESET_NAMES[b]
                );
            }
        }
    }

    fn motion_vec(x: f32, y: f32, z: f32) -> VecRecord {
        VecRecord::new(x, y, z)
    }

    fn motion_ident_rot() -> rocketsim_test::rlpr::cpp_records::Mat3Record {
        rocketsim_test::rlpr::cpp_records::Mat3Record {
            rows: [
                motion_vec(1.0, 0.0, 0.0),
                motion_vec(0.0, 1.0, 0.0),
                motion_vec(0.0, 0.0, 1.0),
            ],
        }
    }

    fn motion_phys(
        pos: (f32, f32, f32),
        vel: (f32, f32, f32),
        ang: (f32, f32, f32),
    ) -> rocketsim_test::rlpr::cpp_records::PhysRecord {
        let mut phys: rocketsim_test::rlpr::cpp_records::PhysRecord = unsafe { std::mem::zeroed() };
        phys.rot = motion_ident_rot();
        phys.pos = motion_vec(pos.0, pos.1, pos.2);
        phys.lin_vel = motion_vec(vel.0, vel.1, vel.2);
        phys.ang_vel = motion_vec(ang.0, ang.1, ang.2);
        phys
    }

    fn motion_car(
        pos: (f32, f32, f32),
        vel: (f32, f32, f32),
        ang: (f32, f32, f32),
    ) -> rocketsim_test::rlpr::cpp_records::CarRecord {
        rocketsim_test::rlpr::cpp_records::CarRecord {
            phys: motion_phys(pos, vel, ang),
            is_on_ground: true,
            is_jumping: false,
            is_flipping: false,
            jump_time: 0.0,
            flip_time: 0.0,
            has_jumped: false,
            double_jumped_or_flipped: false,
            has_flip: false,
            flip_rel_torque: motion_vec(0.0, 0.0, 0.0),
            boost_amount: 0.33,
            is_touching_ball: false,
            prev_controls: rocketsim_test::rlpr::cpp_records::ControlsRecord {
                throttle: 0.0,
                steer: 0.0,
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
                jump: false,
                boost: false,
                handbrake: false,
            },
            wheels: [rocketsim_test::rlpr::cpp_records::WheelRecord {
                susp_length: 0.0,
                susp_rel_vel: 0.0,
                has_contact: false,
                contact_normal: motion_vec(0.0, 0.0, 1.0),
                steer_amount: 0.0,
                engine_force: 0.0,
                brake: 0.0,
                lat_friction: 0.0,
                long_friction: 0.0,
                extra_pushback: 0.0,
            }; 4],
            is_touching_car: false,
            _touch_pad: [0; 3],
            is_boosting: false,
            _boost_pad: [0; 3],
            boosting_time: 0.0,
            handbrake_val: 0.0,
            last_ball_touch_frame: rocketsim_test::rlpr::TOUCH_FRAME_UNKNOWN,
        }
    }

    fn motion_tick(
        car0: (f32, f32, f32),
        car1: (f32, f32, f32),
        ball: (f32, f32, f32),
    ) -> TickRecord {
        TickRecord {
            car_records: vec![
                motion_car(car0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
                motion_car(car1, (10.0, 0.0, 0.0), (0.1, 0.0, 0.0)),
            ],
            ball_record: motion_phys(ball, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        }
    }

    #[test]
    fn reset_motion_updates_only_pos_vel_ang_vel() {
        init();
        let mut backend = V3Backend::new();
        backend.set_bodies(&[0, 0]);
        let seed = motion_tick((0.0, 0.0, 100.0), (100.0, 0.0, 100.0), (0.0, 0.0, 500.0));
        backend.reset(&seed);
        // Plant distinct live history: orientation, timers, flags, controls,
        // boost latch, cooldowns, and ball mode state.
        let yaw90 = glam::Mat3A::from_rotation_z(std::f32::consts::FRAC_PI_2);
        for slot in 0..2 {
            let car_id = backend.car_ids[slot];
            let mut live = *backend.arena.get_car_state(car_id);
            live.phys.rot_mat = yaw90;
            live.handbrake_val = 0.7;
            live.is_boosting = true;
            live.boosting_time = 1.25;
            live.bump_cooldown_timer = 0.4;
            live.bump_cooldown_victim = Some(backend.car_ids[1 - slot]);
            live.last_extra_hit_tick = Some(42);
            live.jump_hold_broken = true;
            live.is_jumping = true;
            live.boost = 50.0;
            live.controls.throttle = 1.0;
            live.jump_ticks = 7;
            live.flip_time = 2.5;
            backend.arena.set_car_state_full(car_id, live);
        }
        let mut ball = *backend.arena.get_ball_state();
        ball.phys.rot_mat = yaw90;
        ball.tick_count_since_kickoff = 123;
        backend.arena.set_ball_state(ball);
        let ball_before = *backend.arena.get_ball_state();
        let ground_before: Vec<bool> = backend
            .car_ids
            .iter()
            .map(|&car_id| backend.arena.get_car_state(car_id).is_on_ground())
            .collect();

        // Target carries new motion vectors but identity orientation: the
        // recorded orientation must be ignored.
        let mut target = motion_tick(
            (1000.0, 2000.0, 300.0),
            (1100.0, 2000.0, 300.0),
            (4000.0, 5000.0, 600.0),
        );
        target.car_records[0].phys.lin_vel = motion_vec(100.0, 200.0, 300.0);
        target.car_records[0].phys.ang_vel = motion_vec(1.0, 2.0, 3.0);
        target.car_records[1].phys.lin_vel = motion_vec(110.0, 210.0, 310.0);
        target.car_records[1].phys.ang_vel = motion_vec(1.5, 2.5, 3.5);
        target.ball_record.lin_vel = motion_vec(400.0, 500.0, 600.0);
        target.ball_record.ang_vel = motion_vec(4.0, 5.0, 6.0);
        backend.reset_motion(&target);

        for slot in 0..2 {
            let car_id = backend.car_ids[slot];
            let live = *backend.arena.get_car_state(car_id);
            let want: CarState = target.car_records[slot].into();
            // Exactly three vector fields follow the recording.
            assert_eq!(live.phys.pos, want.phys.pos, "car {slot} pos");
            assert_eq!(live.phys.vel, want.phys.vel, "car {slot} vel");
            assert_eq!(live.phys.ang_vel, want.phys.ang_vel, "car {slot} ang_vel");
            // Orientation preserved.
            assert_eq!(live.phys.rot_mat, yaw90, "car {slot} rot");
            // Representative history preserved.
            assert_eq!(live.handbrake_val, 0.7, "car {slot} handbrake");
            assert!(live.is_boosting, "car {slot} boost latch");
            assert_eq!(live.boosting_time, 1.25, "car {slot} boost time");
            assert_eq!(live.bump_cooldown_timer, 0.4, "car {slot} bump cooldown");
            assert_eq!(
                live.bump_cooldown_victim,
                Some(backend.car_ids[1 - slot]),
                "car {slot} bump victim"
            );
            assert_eq!(live.last_extra_hit_tick, Some(42), "car {slot} extra gate");
            assert!(live.jump_hold_broken, "car {slot} jump latch");
            assert_eq!(
                live.is_on_ground(),
                ground_before[slot],
                "car {slot} ground cache"
            );
            assert!(live.is_jumping, "car {slot} jump flag");
            assert_eq!(live.boost, 50.0, "car {slot} boost");
            assert_eq!(live.controls.throttle, 1.0, "car {slot} controls");
            assert_eq!(live.jump_ticks, 7, "car {slot} jump ticks");
            assert_eq!(live.flip_time, 2.5, "car {slot} flip time");
        }
        let ball = *backend.arena.get_ball_state();
        let want_ball: PhysState = target.ball_record.into();
        assert_eq!(ball.phys.pos, want_ball.pos);
        assert_eq!(ball.phys.vel, want_ball.vel);
        assert_eq!(ball.phys.ang_vel, want_ball.ang_vel);
        assert_eq!(ball.phys.rot_mat, yaw90);
        assert_eq!(
            ball.tick_count_since_kickoff,
            ball_before.tick_count_since_kickoff
        );
        assert_eq!(ball.hs_info.y_target_dir, ball_before.hs_info.y_target_dir);
        assert_eq!(ball.ds_info.charge_level, ball_before.ds_info.charge_level);
    }

    #[test]
    fn reset_motion_matches_full_state_setter_reference() {
        init();
        let mut optimized = V3Backend::new();
        optimized.set_bodies(&[0, 0]);
        let seed = motion_tick((0.0, 0.0, 100.0), (100.0, 0.0, 100.0), (0.0, 0.0, 500.0));
        optimized.reset(&seed);
        optimized.refresh_sticky_gates();

        // Give the live state contact and input history that a motion reset
        // must retain. The reference backend starts from an exact arena clone.
        for (slot, &car_id) in optimized.car_ids.iter().enumerate() {
            let mut controls = CarControls::DEFAULT;
            controls.jump = slot == 0;
            controls.throttle = if slot == 0 { 0.75 } else { -0.25 };
            optimized.arena.edit_car_state_full(car_id, |state| {
                state.set_prev_controls(controls);
            });
        }
        let mut reference = V3Backend {
            arena: optimized.arena.clone(),
            car_ids: optimized.car_ids.clone(),
            bodies: optimized.bodies.clone(),
            arena_body: optimized.arena_body.clone(),
            dodge_deadzone: optimized.dodge_deadzone,
            mem_weight_mode: optimized.mem_weight_mode,
        };

        let target = motion_tick(
            (1200.0, 2100.0, 350.0),
            (1450.0, 2200.0, 400.0),
            (4100.0, 5100.0, 650.0),
        );

        // This is the prior manual set_car_state_full implementation. It
        // deliberately copies the live state and changes only three vectors.
        for (slot, car) in target.car_records.iter().enumerate() {
            let car_id = reference.car_ids[slot];
            let recorded: CarState = (*car).into();
            let mut live = *reference.arena.get_car_state(car_id);
            live.phys.pos = recorded.phys.pos;
            live.phys.vel = recorded.phys.vel;
            live.phys.ang_vel = recorded.phys.ang_vel;
            reference.arena.set_car_state_full(car_id, live);
        }
        let recorded_ball: PhysState = target.ball_record.into();
        let mut ball = *reference.arena.get_ball_state();
        ball.phys.pos = recorded_ball.pos;
        ball.phys.vel = recorded_ball.vel;
        ball.phys.ang_vel = recorded_ball.ang_vel;
        reference.arena.set_ball_state(ball);

        optimized.reset_motion(&target);
        let assert_same_state = |actual: &V3Backend, expected: &V3Backend, phase: &str| {
            for slot in 0..actual.car_ids.len() {
                assert_eq!(
                    format!("{:?}", actual.arena.get_car_state(actual.car_ids[slot])),
                    format!("{:?}", expected.arena.get_car_state(expected.car_ids[slot])),
                    "car {slot} differs {phase}"
                );
            }
            assert_eq!(
                format!("{:?}", actual.arena.get_ball_state()),
                format!("{:?}", expected.arena.get_ball_state()),
                "ball differs {phase}"
            );
        };
        assert_same_state(&optimized, &reference, "after reset");

        for step in 0..10 {
            let controls: Vec<CarControls> = (0..2)
                .map(|slot| CarControls {
                    throttle: if slot == 0 { 0.6 } else { -0.3 },
                    steer: if step % 2 == 0 { 0.2 } else { -0.2 },
                    jump: step == 3 && slot == 0,
                    ..CarControls::DEFAULT
                })
                .collect();
            for (slot, &car_id) in optimized.car_ids.iter().enumerate() {
                optimized.arena.set_car_controls(car_id, controls[slot]);
            }
            for (slot, &car_id) in reference.car_ids.iter().enumerate() {
                reference.arena.set_car_controls(car_id, controls[slot]);
            }
            optimized.arena.step_tick();
            reference.arena.step_tick();
            assert_same_state(&optimized, &reference, "after step");
        }
    }

    #[test]
    fn reset_motion_needs_sized_arena() {
        init();
        let mut backend = V3Backend::new();
        let tick = motion_tick((0.0, 0.0, 100.0), (100.0, 0.0, 100.0), (0.0, 0.0, 500.0));
        // No `reset` yet, so no cars exist: explicit panic, not a rebuild.
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            backend.reset_motion(&tick);
        }));
        assert!(result.is_err());
    }
}
