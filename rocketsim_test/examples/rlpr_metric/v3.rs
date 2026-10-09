//! V3 replay backend for RLPR recordings.
//!
//! Uses one car per recorded car in Soccar (Blue first, then Orange),
//! with the body preset from the recording header.
//! Reset restores every car and ball state from the start tick.
//! Step applies each recorded car's controls for one tick.

use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, ArenaMemWeightMode, CarBodyConfig, CarControls, CarState,
    GameMode, HITBOX_OFFSETS, HITBOX_SIZES, PhysState, Team,
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
        Self::with_mem_weight_mode(dodge_deadzone, ArenaMemWeightMode::Heavy)
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
        let mut state = *self.arena.get_car_state(car_id);
        state.handbrake_val = value.clamp(0.0, 1.0);
        self.arena.set_car_state(car_id, state);
    }

    /// Restore jump-hold continuity before a replayed tick.
    pub fn set_jump_hold_broken(&mut self, car_idx: usize, broken: bool) {
        let car_id = self.car_id(car_idx, "set_jump_hold_broken");
        let mut state = *self.arena.get_car_state(car_id);
        state.jump_hold_broken = broken;
        self.arena.set_car_state(car_id, state);
    }

    /// Refresh prior-tick wheel gates without advancing dynamics.
    pub fn refresh_sticky_gates(&mut self) {
        for &car_id in &self.car_ids {
            self.arena.refresh_car_sticky_gate(car_id);
        }
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
        let mut state = *self.arena.get_car_state(car_id);
        state.is_boosting = armed;
        state.boosting_time = time;
        self.arena.set_car_state(car_id, state);
    }

    fn supports_cooldown_restore(&self) -> bool {
        true
    }

    fn suppress_next_extra_hit(&mut self, car_idx: usize, suppress: bool) {
        let car_id = self.car_id(car_idx, "suppress_next_extra_hit");
        let mut state = *self.arena.get_car_state(car_id);
        if suppress {
            // Block exactly the next executed step: the upcoming step runs at
            // the current tick count, and the gate re-opens the step after.
            // Fresh arenas (count 0) cannot represent "block step 0, allow
            // step 1" in u64, so they over-suppress by one step; rebuilds are
            // the only path there and are rare.
            let current = self.arena.tick_count();
            state.last_extra_hit_tick = Some(if current == 0 { 0 } else { current - 1 });
        } else {
            state.last_extra_hit_tick = None;
        }
        self.arena.set_car_state(car_id, state);
    }

    fn supports_bump_restore(&self) -> bool {
        true
    }

    fn set_bump_cooldown(&mut self, car_idx: usize, seconds: f32) {
        let car_id = self.car_id(car_idx, "set_bump_cooldown");
        let mut state = *self.arena.get_car_state(car_id);
        state.bump_cooldown_timer = seconds.max(0.0);
        self.arena.set_car_state(car_id, state);
    }

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
    use rocketsim::consts::BT_TO_UU;
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
        backend.arena.set_car_state(backend.car_ids[0], planted);
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
}
