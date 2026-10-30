//! v2 replay backend for RLPR recordings.
//!
//! Reset builds a new Soccar arena at 120 Hz.
//! Step applies recorded controls for one tick.
//! Snapshot reads car and ball state.
//!
//! Call [`V2Backend::init`] one time before [`V2Backend::new`].
//!
//! Fields with no RLPR source get neutral values:
//! tick counters, air timers, boost pad timers, supersonic flags,
//! contact state, heatseeker and dropshot ball info. `ball_hit_info`
//! is cleared on reset. `has_double_jumped` and `has_flipped` use the
//! same record-derived inference as the v3 comparison path. Boost latch,
//! handbrake, and bump cooldown restore from the recording like v3;
//! ball extra-hit gating has no v2 engine field and replays live.

use glam::Vec3A;
use rocketsim_rs::{
    math::{RotMat, Vec3},
    sim::{
        Arena, ArenaConfig, ArenaMemWeightMode, BallState, CarConfig, CarControls, CarState,
        GameMode, Team,
    },
};
use rocketsim_test::rlpr::{
    cpp_records::{CarRecord, ControlsRecord, Mat3Record, RecordingInfo, VecRecord},
    tick_record::TickRecord,
};

use super::common::{
    BODY_PRESET_NAMES, BodySnapshot, HitboxBounds, NUM_BODY_PRESETS, ReplayBackend,
    SimContactEvents, Snapshot, body_preset_from_info,
};

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

/// v2 sim holder with one recorded-body car per recorded car
/// (Blue first, then Orange).
///
/// The body presets come from [`V2Backend::set_bodies`] or
/// [`V2Backend::set_body_from_info`]; they stay Octane until the caller
/// selects a roster. Presets are per car slot, so a mixed roster replays
/// with each car on its own body.
pub struct V2Backend {
    arena: rocketsim_rs::cxx::UniquePtr<Arena>,
    /// Arena car id per recording slot. `None` means the slot is unavailable
    /// in the current partial arena.
    car_ids: Vec<Option<u32>>,
    /// Availability mask the current arena was built with.
    arena_mask: Option<Vec<bool>>,
    /// Body preset index per car slot, in [`BODY_PRESET_NAMES`] order.
    bodies: Vec<usize>,
    /// Presets the current arena was built with, to detect roster changes.
    arena_body: Option<Vec<usize>>,
    dodge_deadzone: f32,
    mem_weight_mode: ArenaMemWeightMode,
    capture_events: bool,
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
        Self::with_options(dodge_deadzone, ArenaMemWeightMode::Heavy, true)
    }

    /// Make a backend for the throughput benchmark.
    ///
    /// The benchmark does not install the metric event callback.
    #[allow(dead_code)]
    pub fn benchmark(mem_weight_mode: ArenaMemWeightMode) -> Self {
        Self::with_options(0.5, mem_weight_mode, false)
    }

    fn with_options(
        dodge_deadzone: f32,
        mem_weight_mode: ArenaMemWeightMode,
        capture_events: bool,
    ) -> Self {
        let (arena, car_ids) = fresh_arena(
            &[true],
            &[OCTANE_PRESET],
            dodge_deadzone,
            mem_weight_mode,
            capture_events,
        );
        let last_ball_hit = vec![0; car_ids.len()];
        Self {
            arena,
            car_ids,
            arena_mask: None,
            bodies: vec![OCTANE_PRESET],
            arena_body: None,
            dodge_deadzone,
            mem_weight_mode,
            capture_events,
            last_ball_hit,
        }
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
        let index = body_preset_from_info(info, &v2_preset_bounds())?;
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

    /// Preset index for one car slot. Missing slots fall back to Octane so a
    /// short or empty roster still builds a playable arena.
    fn preset_for_slot(&self, slot: usize) -> usize {
        self.bodies.get(slot).copied().unwrap_or(OCTANE_PRESET)
    }

    /// Arena car id for one recording slot, or `None` when unavailable.
    fn car_id_opt(&self, car_idx: usize) -> Option<u32> {
        self.car_ids.get(car_idx).copied().flatten()
    }

    /// Arena car id for one available recording slot.
    ///
    /// Panics as `<caller> needs car <index>` on a missing or unavailable
    /// slot. Callers that may see an unavailable slot use [`Self::car_id_opt`].
    #[track_caller]
    fn car_id(&self, car_idx: usize, caller: &str) -> u32 {
        match self.car_id_opt(car_idx) {
            Some(id) => id,
            None => panic!("{caller} needs car {car_idx}"),
        }
    }

    /// Recording slot for one arena car id.
    fn slot_for_arena_car(&self, arena_car: u32) -> Option<usize> {
        self.car_ids
            .iter()
            .position(|stored| *stored == Some(arena_car))
    }

    /// Team per frozen recording slot: Blue first, then alternating.
    fn team_for_slot(slot: usize) -> Team {
        if slot.is_multiple_of(2) {
            Team::Blue
        } else {
            Team::Orange
        }
    }

    /// Availability mask for a tick, one bit per frozen car slot.
    fn tick_mask(tick: &TickRecord) -> Vec<bool> {
        (0..tick.car_records.len())
            .map(|slot| tick.car_available(slot))
            .collect()
    }

    /// Rebuild the arena when the availability mask, car count, or body
    /// roster changes.
    ///
    /// Only available slots get an arena car. The team comes from the frozen
    /// recording slot, not the compact arena index. The roster check matters
    /// when consecutive recordings hold the same car count with different
    /// bodies: the ids would still line up, but the hitbox and wheels would
    /// stay wrong without a rebuild.
    fn ensure_cars(&mut self, mask: &[bool]) {
        let roster: Vec<usize> = (0..mask.len())
            .map(|slot| self.preset_for_slot(slot))
            .collect();
        let mask_vec = mask.to_vec();
        if self.car_ids.len() != mask.len()
            || self.arena_body.as_ref() != Some(&roster)
            || self.arena_mask.as_ref() != Some(&mask_vec)
        {
            let (arena, car_ids) = fresh_arena(
                mask,
                &roster,
                self.dodge_deadzone,
                self.mem_weight_mode,
                self.capture_events,
            );
            self.arena = arena;
            self.car_ids = car_ids;
            self.arena_body = Some(roster);
            self.arena_mask = Some(mask_vec);
            self.last_ball_hit = vec![0; mask.len()];
        }
    }

    /// Test helper: build an arena with `num_cars` available slots.
    #[cfg(test)]
    fn ensure_all_cars(&mut self, num_cars: usize) {
        self.ensure_cars(&vec![true; num_cars]);
    }

    /// Step the arena without callbacks, event scans, or metric allocations.
    #[allow(dead_code)]
    pub fn benchmark_step(&mut self, controls: &[CarControls]) {
        for (slot, controls) in controls.iter().enumerate() {
            let Some(car_id) = self.car_id_opt(slot) else {
                continue;
            };
            self.arena
                .pin_mut()
                .set_car_controls(car_id, *controls)
                .expect("v2 car id is valid");
        }
        self.arena.pin_mut().step(1);
    }

    /// Convert one recording control before the timed loop.
    #[allow(dead_code)]
    pub fn benchmark_control(controls: ControlsRecord) -> CarControls {
        v2_controls(&controls)
    }

    /// Motion-only reset: overwrite exactly car/ball `pos`/`vel`/`ang_vel`.
    ///
    /// Copy live state, edit three fields, write back. Preserve orientation
    /// and logical state. Do not explicitly clear backend caches, overwrite
    /// controls, clear `ball_hit_info`, or rebuild via `ensure_cars`. The
    /// segment seed sizes the arena. Panics on a missing slot. Harness only.
    pub fn reset_motion(&mut self, state_tick: &TickRecord) {
        for (slot, car) in state_tick.car_records.iter().enumerate() {
            if !car.is_available() {
                continue;
            }
            let car_id = self.car_id(slot, "reset_motion");
            let mut live = self.arena.pin_mut().get_car(car_id);
            live.pos = record_vec(car.phys.pos);
            live.vel = record_vec(car.phys.lin_vel);
            live.ang_vel = record_vec(car.phys.ang_vel);
            self.arena
                .pin_mut()
                .set_car(car_id, live)
                .expect("v2 car id is valid");
        }
        if !state_tick.ball_available() {
            return;
        }
        let mut ball = self.arena.pin_mut().get_ball();
        ball.pos = record_vec(state_tick.ball_record.pos);
        ball.vel = record_vec(state_tick.ball_record.lin_vel);
        ball.ang_vel = record_vec(state_tick.ball_record.ang_vel);
        self.arena.pin_mut().set_ball(ball);
    }
}

impl Default for V2Backend {
    fn default() -> Self {
        Self::new()
    }
}

impl ReplayBackend for V2Backend {
    fn reset(&mut self, start: &TickRecord) {
        assert!(
            start.has_physics() && start.ball_available(),
            "v2 reset needs a physics row with an available ball"
        );
        self.ensure_cars(&Self::tick_mask(start));
        self.set_state(start);
    }

    fn reset_motion(&mut self, state_tick: &TickRecord) {
        V2Backend::reset_motion(self, state_tick);
    }

    fn set_state(&mut self, state_tick: &TickRecord) {
        assert!(
            state_tick.has_physics() && state_tick.ball_available(),
            "v2 set_state needs a physics row with an available ball"
        );
        self.ensure_cars(&Self::tick_mask(state_tick));
        for (slot, car) in state_tick.car_records.iter().enumerate() {
            if !car.is_available() {
                continue;
            }
            let car_id = self.car_id(slot, "set_state");
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

    fn set_boost_state(&mut self, car_idx: usize, armed: bool, time: f32) {
        let Some(car_id) = self.car_id_opt(car_idx) else {
            return;
        };
        let mut state = self.arena.pin_mut().get_car(car_id);
        state.is_boosting = armed;
        state.boosting_time = if armed { time } else { 0.0 };
        self.arena
            .pin_mut()
            .set_car(car_id, state)
            .expect("v2 car id is valid");
    }

    fn set_handbrake_value(&mut self, car_idx: usize, value: f32) {
        let Some(car_id) = self.car_id_opt(car_idx) else {
            return;
        };
        let mut state = self.arena.pin_mut().get_car(car_id);
        state.handbrake_val = value.clamp(0.0, 1.0);
        self.arena
            .pin_mut()
            .set_car(car_id, state)
            .expect("v2 car id is valid");
    }

    fn supports_bump_restore(&self) -> bool {
        true
    }

    fn set_bump_cooldown(&mut self, car_idx: usize, victim_idx: usize, seconds: f32) {
        // v2's C++ contact cooldown is per-attacker only, so the victim
        // scope is intentionally ignored here; v2 keeps its legacy
        // broad suppression. The v3 gate metric is scored on the v3 backend.
        let _ = victim_idx;
        let Some(car_id) = self.car_id_opt(car_idx) else {
            return;
        };
        let mut state = self.arena.pin_mut().get_car(car_id);
        state.car_contact.cooldown_timer = seconds.max(0.0);
        self.arena
            .pin_mut()
            .set_car(car_id, state)
            .expect("v2 car id is valid");
    }

    fn step(&mut self, controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
        for (slot, controls) in controls.iter().enumerate() {
            let Some(car_id) = self.car_id_opt(slot) else {
                continue;
            };
            self.arena
                .pin_mut()
                .set_car_controls(car_id, v2_controls(controls))
                .expect("v2 car id is valid");
        }
        self.arena.pin_mut().step(1);
        // Drain bump events collected by v2_bump_callback during the step.
        let mut observed = vec![SimContactEvents::default(); self.car_ids.len()];
        V2_BUMP_EVENTS.with(|events| {
            for (bumper, victim) in events.borrow_mut().drain(..) {
                for arena_car in [bumper, victim] {
                    if let Some(slot) = self.slot_for_arena_car(arena_car) {
                        observed[slot].car_car = true;
                    }
                }
            }
        });
        // Car/ball and chassis/world contacts come from sim state:
        // ball_hit_info is per-touch (change-detected against latching),
        // world_contact reflects the current tick.
        for (slot, car_id) in self.car_ids.iter().enumerate() {
            let Some(car_id) = *car_id else {
                continue;
            };
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
        let car_id = self.car_id(car_idx, "snapshot");
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

/// Preset index for Octane, the default body of a fresh v2 backend.
const OCTANE_PRESET: usize = 0;

/// v2 body presets in [`BODY_PRESET_NAMES`] order.
///
/// The v2 crate returns each preset as a `&'static CarConfig`, so the table
/// is built at runtime from the crate's own getters rather than duplicated
/// as constants here.
fn v2_preset(index: usize) -> &'static CarConfig {
    match index {
        0 => CarConfig::octane(),
        1 => CarConfig::dominus(),
        2 => CarConfig::plank(),
        3 => CarConfig::breakout(),
        4 => CarConfig::hybrid(),
        5 => CarConfig::merc(),
        6 => CarConfig::psyclops(),
        _ => panic!("v2 body preset index out of range: {index}"),
    }
}

/// Hitbox bounds per v2 preset, in uu, in [`BODY_PRESET_NAMES`] order.
#[allow(dead_code)]
fn v2_preset_bounds() -> [HitboxBounds; NUM_BODY_PRESETS] {
    std::array::from_fn(|index| {
        let config = v2_preset(index);
        HitboxBounds {
            size: vec_to_glam(config.hitbox_size),
            offset: vec_to_glam(config.hitbox_pos_offset),
        }
    })
}

/// Make a Soccar arena at 120 Hz with one body per recorded car.
///
/// `roster` holds one preset index per car slot; a short roster pads with
/// Octane.
fn fresh_arena(
    mask: &[bool],
    roster: &[usize],
    dodge_deadzone: f32,
    mem_weight_mode: ArenaMemWeightMode,
    capture_events: bool,
) -> (rocketsim_rs::cxx::UniquePtr<Arena>, Vec<Option<u32>>) {
    let config = ArenaConfig {
        mem_weight_mode,
        no_ball_rot: false,
        ..Default::default()
    };
    let mut arena = Arena::new(GameMode::Soccar, config, 120);
    if capture_events {
        arena.pin_mut().set_car_bump_callback(v2_bump_callback, 0);
    }
    let car_ids = mask
        .iter()
        .enumerate()
        .map(|(slot, &available)| {
            available.then(|| {
                let mut car_config = *v2_preset(roster.get(slot).copied().unwrap_or(OCTANE_PRESET));
                car_config.dodge_deadzone = dodge_deadzone;
                arena
                    .pin_mut()
                    .add_car(V2Backend::team_for_slot(slot), &car_config)
            })
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

    // `has_double_jumped` and `has_flipped` use the same record-derived
    // inference as the v3 comparison path: recorded flags win, but a spent
    // flag the record does not explain is left alone rather than cleared.
    // Live gate evolution otherwise carries across resets (see the v3
    // path): stale foreign-trajectory gates track RL better than defaults.
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

#[cfg(test)]
mod tests {
    use rocketsim::{HITBOX_SIZES, consts::BT_TO_UU};
    use rocketsim_test::rlpr::cpp_records::VecRecord;

    use super::*;

    fn info_for(min: [f32; 3], max: [f32; 3]) -> RecordingInfo {
        RecordingInfo {
            num_cars: 2,
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

    /// Header bounds for one v2 preset, in BT.
    fn bounds_to_bt(config: &CarConfig) -> ([f32; 3], [f32; 3]) {
        let size = vec_to_glam(config.hitbox_size);
        let offset = vec_to_glam(config.hitbox_pos_offset);
        let min = (offset - size * 0.5) / BT_TO_UU;
        let max = (offset + size * 0.5) / BT_TO_UU;
        (min.to_array(), max.to_array())
    }

    #[test]
    fn daizen_header_maps_to_plank() {
        init();
        let mut backend = V2Backend::new();
        assert_eq!(backend.set_body_from_info(&daizen_info()).unwrap(), "Plank");
        // The test header declares two cars, so the roster fills both slots.
        assert_eq!(backend.bodies, vec![2, 2]);
    }

    #[test]
    fn octane_header_stays_octane() {
        init();
        let mut backend = V2Backend::new();
        assert_eq!(
            backend.set_body_from_info(&octane_info()).unwrap(),
            "Octane"
        );
        assert_eq!(backend.bodies, vec![0, 0]);
    }

    #[test]
    fn every_preset_matches_its_own_bounds() {
        init();
        // The v2 preset table must be self-consistent: each preset's own
        // hitbox bounds round-trip through the header matcher back to it.
        for index in 0..NUM_BODY_PRESETS {
            let (min, max) = bounds_to_bt(v2_preset(index));
            let info = info_for(min, max);
            assert_eq!(
                body_preset_from_info(&info, &v2_preset_bounds()).unwrap(),
                index,
                "{} must match its own bounds",
                BODY_PRESET_NAMES[index]
            );
        }
    }

    #[test]
    fn closest_presets_stay_distinct() {
        init();
        // Psyclops is Octane + 0.134 uu on every size axis: with a 0.01 uu
        // tolerance it must match Psyclops, never Octane.
        let (min, max) = bounds_to_bt(CarConfig::psyclops());
        let info = info_for(min, max);
        assert_eq!(
            body_preset_from_info(&info, &v2_preset_bounds()).unwrap(),
            6
        );
    }

    #[test]
    fn unknown_header_is_an_error() {
        let info = info_for([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]);
        assert!(body_preset_from_info(&info, &v2_preset_bounds()).is_err());
    }

    #[test]
    fn unknown_header_keeps_previous_body() {
        init();
        let mut backend = V2Backend::new();
        backend.set_body_from_info(&daizen_info()).unwrap();
        assert_eq!(backend.bodies, vec![2, 2]);
        let bad = info_for([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]);
        assert!(backend.set_body_from_info(&bad).is_err());
        assert_eq!(backend.bodies, vec![2, 2]);
    }

    #[test]
    fn same_count_body_switch_rebuilds_arena() {
        init();
        let mut backend = V2Backend::new();
        backend.set_body_from_info(&octane_info()).unwrap();
        backend.ensure_all_cars(2);
        // The v2 arena hands out the same ids on every rebuild, so a planted
        // state is what proves whether the arena was replaced.
        let sentinel = Vec3::new(1234.0, 567.0, 89.0);
        let first_id = backend.car_ids[0].expect("slot 0 available");
        let mut planted = backend.arena.pin_mut().get_car(first_id);
        planted.pos = sentinel;
        backend
            .arena
            .pin_mut()
            .set_car(first_id, planted)
            .expect("car id is valid");
        // Same body, same count: no rebuild, so the planted state survives.
        backend.ensure_all_cars(2);
        assert_eq!(backend.arena.pin_mut().get_car(first_id).pos, sentinel);
        // Same count, new body: rebuild, so the planted state is gone.
        assert_eq!(backend.set_body_from_info(&daizen_info()).unwrap(), "Plank");
        backend.ensure_all_cars(2);
        assert_eq!(backend.car_ids.len(), 2);
        assert_ne!(backend.arena.pin_mut().get_car(first_id).pos, sentinel);
        assert_eq!(backend.arena_body, Some(vec![2, 2]));
        // Motion-only preservation, folded here to avoid an extra parallel
        // C++ arena: two cars plus ball, orientation, controls, timers,
        // cooldowns, boost latch, hit info.
        use rocketsim_rs::math::{RotMat, Vec3 as V2Vec3};
        let seed = motion_tick((0.0, 0.0, 100.0), (100.0, 0.0, 100.0), (0.0, 0.0, 500.0));
        backend.reset(&seed);
        let yaw90 = RotMat {
            forward: V2Vec3::new(0.0, 1.0, 0.0),
            right: V2Vec3::new(-1.0, 0.0, 0.0),
            up: V2Vec3::new(0.0, 0.0, 1.0),
        };
        for slot in 0..2 {
            let car_id = backend.car_ids[slot].expect("slot available");
            let mut live = backend.arena.pin_mut().get_car(car_id);
            live.rot_mat = yaw90;
            live.last_controls.throttle = 1.0;
            live.is_boosting = true;
            live.boosting_time = 1.25;
            live.handbrake_val = 0.7;
            live.boost = 50.0;
            live.car_contact.cooldown_timer = 0.4;
            live.car_contact.other_car_id = backend.car_ids[1 - slot].expect("slot available");
            live.ball_hit_info.is_valid = true;
            live.ball_hit_info.tick_count_when_hit = 42;
            live.is_on_ground = false;
            live.jump_time = 0.5;
            live.flip_time = 2.5;
            backend
                .arena
                .pin_mut()
                .set_car(car_id, live)
                .expect("v2 car id is valid");
        }
        let mut ball = backend.arena.pin_mut().get_ball();
        ball.rot_mat = yaw90;
        ball.tick_count_since_update = 123;
        backend.arena.pin_mut().set_ball(ball);
        let ball_before = backend.arena.pin_mut().get_ball();
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
            let car_id = backend.car_ids[slot].expect("slot available");
            let live = backend.arena.pin_mut().get_car(car_id);
            for (got, want) in [
                (live.pos, record_vec(target.car_records[slot].phys.pos)),
                (live.vel, record_vec(target.car_records[slot].phys.lin_vel)),
                (
                    live.ang_vel,
                    record_vec(target.car_records[slot].phys.ang_vel),
                ),
            ] {
                assert!(
                    (got.x - want.x).abs() < 1e-3
                        && (got.y - want.y).abs() < 1e-3
                        && (got.z - want.z).abs() < 1e-3,
                    "motion vector differs: {got:?} vs {want:?}"
                );
            }
            assert_eq!(live.rot_mat.forward, yaw90.forward);
            assert_eq!(live.rot_mat.up, yaw90.up);
            assert_eq!(live.last_controls.throttle, 1.0);
            assert!(live.is_boosting);
            assert_eq!(live.boosting_time, 1.25);
            assert_eq!(live.handbrake_val, 0.7);
            assert_eq!(live.boost, 50.0);
            assert_eq!(live.car_contact.cooldown_timer, 0.4);
            assert_eq!(
                live.car_contact.other_car_id,
                backend.car_ids[1 - slot].expect("slot available")
            );
            assert!(live.ball_hit_info.is_valid);
            assert_eq!(live.ball_hit_info.tick_count_when_hit, 42);
            assert!(!live.is_on_ground);
        }
        let after = backend.arena.pin_mut().get_ball();
        for (got, want) in [
            (after.pos, record_vec(target.ball_record.pos)),
            (after.vel, record_vec(target.ball_record.lin_vel)),
            (after.ang_vel, record_vec(target.ball_record.ang_vel)),
        ] {
            assert!(
                (got.x - want.x).abs() < 1e-3
                    && (got.y - want.y).abs() < 1e-3
                    && (got.z - want.z).abs() < 1e-3,
                "ball motion differs: {got:?} vs {want:?}"
            );
        }
        assert_eq!(after.rot_mat.forward, yaw90.forward);
        assert_eq!(
            after.tick_count_since_update,
            ball_before.tick_count_since_update
        );
    }

    #[test]
    fn unknown_header_keeps_previous_roster() {
        init();
        let mut backend = V2Backend::new();
        backend.set_bodies(&[2, 0]);
        let bad = info_for([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]);
        assert!(backend.set_body_from_info(&bad).is_err());
        assert_eq!(backend.bodies, vec![2, 0]);
    }

    #[test]
    fn mixed_roster_builds_per_car_bodies() {
        init();
        let mut backend = V2Backend::new();
        // Plank against Octane, as in the bundled london_vs_nexto capture.
        backend.set_bodies(&[2, 0]);
        backend.ensure_all_cars(2);
        assert_eq!(backend.arena_body, Some(vec![2, 0]));
        assert_eq!(
            vec_to_glam(v2_preset(2).hitbox_size),
            HITBOX_SIZES[2],
            "car 0 must be Plank"
        );
        assert_eq!(
            vec_to_glam(v2_preset(0).hitbox_size),
            HITBOX_SIZES[0],
            "car 1 must be Octane"
        );
    }

    #[test]
    fn short_roster_pads_with_octane() {
        init();
        let mut backend = V2Backend::new();
        backend.set_bodies(&[2]);
        backend.ensure_all_cars(3);
        assert_eq!(backend.arena_body, Some(vec![2, 0, 0]));
    }

    #[test]
    fn roster_change_rebuilds_without_count_change() {
        init();
        let mut backend = V2Backend::new();
        backend.set_bodies(&[0, 0]);
        backend.ensure_all_cars(2);
        let before = backend.arena_body.clone();
        backend.set_bodies(&[2, 0]);
        backend.ensure_all_cars(2);
        assert_ne!(backend.arena_body, before);
    }

    fn motion_vec(x: f32, y: f32, z: f32) -> VecRecord {
        VecRecord::new(x, y, z)
    }

    fn motion_ident_rot() -> Mat3Record {
        Mat3Record {
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
            last_demolish_frame: rocketsim_test::rlpr::DEMOLISH_FRAME_UNKNOWN,
            physics_available: 1,
            _availability_pad: [0; 3],
        }
    }

    fn motion_tick(
        car0: (f32, f32, f32),
        car1: (f32, f32, f32),
        ball: (f32, f32, f32),
    ) -> TickRecord {
        TickRecord {
            level_tick: 0,
            row_flags: rocketsim_test::rlpr::ROW_FLAGS_BALL_AVAILABLE,
            reserved0: 0,
            car_records: vec![
                motion_car(car0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
                motion_car(car1, (10.0, 0.0, 0.0), (0.1, 0.0, 0.0)),
            ],
            ball_record: motion_phys(ball, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        }
    }

    /// Build a tick with an explicit per-slot availability mask.
    fn mask_tick(mask: &[bool]) -> TickRecord {
        let car_records = mask
            .iter()
            .enumerate()
            .map(|(slot, &available)| {
                let mut car = motion_car(
                    (100.0 * slot as f32, 0.0, 100.0),
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                );
                car.physics_available = u8::from(available);
                car
            })
            .collect();
        TickRecord {
            level_tick: 0,
            row_flags: rocketsim_test::rlpr::ROW_FLAGS_BALL_AVAILABLE,
            reserved0: 0,
            car_records,
            ball_record: motion_phys((0.0, 0.0, 200.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        }
    }

    fn assert_teams_follow_frozen_slots(backend: &mut V2Backend) {
        for (slot, id) in backend.car_ids.clone().iter().enumerate() {
            let Some(id) = id else {
                continue;
            };
            assert_eq!(
                backend.arena.pin_mut().get_car_info(*id).team,
                V2Backend::team_for_slot(slot),
                "slot {slot} team"
            );
        }
    }

    #[test]
    fn partial_arena_maps_active_ids_and_frozen_teams() {
        init();
        let mut backend = V2Backend::new();
        backend.set_bodies(&[0; 6]);
        for mask in [
            [true, false, false, true, false, false],
            [false, true, true, false, true, true],
            [true; 6],
        ] {
            backend.ensure_cars(&mask);
            assert_eq!(backend.car_ids.len(), 6);
            for (slot, available) in mask.iter().enumerate() {
                assert_eq!(
                    backend.car_ids[slot].is_some(),
                    *available,
                    "slot {slot} availability"
                );
            }
            assert_teams_follow_frozen_slots(&mut backend);
        }
    }

    #[test]
    fn repeated_mask_same_count_different_active_slots_rebuilds() {
        init();
        let mut backend = V2Backend::new();
        backend.set_bodies(&[0; 4]);
        backend.ensure_cars(&[true, true, false, false]);
        let first_id = backend.car_ids[0].expect("slot 0 available");
        let sentinel = Vec3::new(1234.0, 567.0, 89.0);
        let mut planted = backend.arena.pin_mut().get_car(first_id);
        planted.pos = sentinel;
        backend
            .arena
            .pin_mut()
            .set_car(first_id, planted)
            .expect("car id is valid");

        // Same active count, different active slots: the arena must rebuild.
        backend.ensure_cars(&[false, false, true, true]);
        assert!(backend.car_ids[0].is_none());
        assert!(backend.car_ids[1].is_none());
        assert!(backend.car_ids[2].is_some());
        assert!(backend.car_ids[3].is_some());
        let new_id = backend.car_ids[2].expect("slot 2 available");
        assert_ne!(backend.arena.pin_mut().get_car(new_id).pos, sentinel);
        assert_teams_follow_frozen_slots(&mut backend);
    }

    #[test]
    fn snapshot_uses_frozen_slot() {
        init();
        let mut backend = V2Backend::new();
        backend.set_bodies(&[0; 4]);
        let tick = mask_tick(&[false, true, false, true]);
        backend.reset(&tick);
        for slot in [1usize, 3] {
            let snap = backend.snapshot(slot);
            let want = vec_to_glam(record_vec(tick.car_records[slot].phys.pos));
            assert!((snap.car.pos - want).length() < 1.0e-3, "slot {slot} pos");
        }
    }

    #[test]
    #[should_panic(expected = "snapshot needs car 1")]
    fn snapshot_unavailable_slot_panics() {
        init();
        let mut backend = V2Backend::new();
        backend.set_bodies(&[0; 3]);
        backend.ensure_cars(&[true, false, true]);
        backend.snapshot(1);
    }

    #[test]
    fn full_mask_after_partial_restores_six_slots() {
        init();
        let mut backend = V2Backend::new();
        backend.set_bodies(&[0; 6]);
        backend.ensure_cars(&[true, false, true, false, true, false]);
        assert_eq!(backend.car_ids.iter().filter(|id| id.is_some()).count(), 3);
        backend.ensure_cars(&[true; 6]);
        assert_eq!(backend.car_ids.len(), 6);
        assert!(backend.car_ids.iter().all(|id| id.is_some()));
        assert_teams_follow_frozen_slots(&mut backend);
    }

    fn controls_for(throttle: f32) -> ControlsRecord {
        ControlsRecord {
            throttle,
            steer: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
            jump: false,
            boost: false,
            handbrake: false,
        }
    }

    #[test]
    fn bump_callback_maps_only_active_frozen_slots() {
        init();
        // Frozen slots 1 and 3 are active. The compact arena ids are 0 and 1,
        // so a compact-index mapping would mark slots 0 and 1 instead.
        let mask = [false, true, false, true, false, false];
        let mut backend = V2Backend::new();
        backend.set_bodies(&[0; 6]);
        backend.ensure_cars(&mask);
        assert_eq!(
            backend.car_ids.iter().flatten().count(),
            2,
            "only the two active frozen slots get a car"
        );
        // Compact order is slot 1 then slot 3. A compact-index mapping would
        // mark events[0] and events[1], so the slot assertions below would
        // fail. The mapping must use the frozen slot, not the compact index.

        // Drain any stale queue from another test. This is a drain, not a
        // synthetic event.
        V2_BUMP_EVENTS.with(|events| events.borrow_mut().clear());

        let mut tick = mask_tick(&mask);
        // Head-on contact at slots 1 and 3, below demo speed.
        tick.car_records[1].phys.pos = motion_vec(0.0, 0.0, 100.0);
        tick.car_records[1].phys.lin_vel = motion_vec(800.0, 0.0, 0.0);
        tick.car_records[3].phys.pos = motion_vec(120.0, 0.0, 100.0);
        tick.car_records[3].phys.lin_vel = motion_vec(-800.0, 0.0, 0.0);
        // Keep the ball far away so only car-car contacts can fire.
        tick.ball_record.pos = motion_vec(0.0, 5000.0, 100.0);
        tick.ball_record.lin_vel = motion_vec(0.0, 0.0, 0.0);
        backend.reset(&tick);

        let controls = [
            controls_for(0.0),
            controls_for(1.0),
            controls_for(0.0),
            controls_for(1.0),
            controls_for(0.0),
            controls_for(0.0),
        ];

        let mut observed = None;
        for _ in 0..30 {
            let events = backend.step(&controls);
            if events[1].car_car || events[3].car_car {
                observed = Some(events);
                break;
            }
        }
        let events = observed.expect("the C++ bump callback must fire within 30 steps");

        assert!(events[1].car_car, "frozen slot 1 must be marked");
        assert!(events[3].car_car, "frozen slot 3 must be marked");
        for slot in [0usize, 2, 4, 5] {
            assert!(
                !events[slot].car_car,
                "inactive frozen slot {slot} must not be marked"
            );
        }
        assert!(!events[1].car_ball && !events[3].car_ball);
    }
}
