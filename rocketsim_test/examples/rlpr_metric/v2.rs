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
    car_ids: Vec<u32>,
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
            1,
            &[OCTANE_PRESET],
            dodge_deadzone,
            mem_weight_mode,
            capture_events,
        );
        let last_ball_hit = vec![0; car_ids.len()];
        Self {
            arena,
            car_ids,
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

    /// Rebuild the arena when the car count or the body roster changes.
    ///
    /// The roster check matters when consecutive recordings hold the same car
    /// count with different bodies: the ids would still line up, but the
    /// hitbox and wheels would stay wrong without a rebuild.
    fn ensure_cars(&mut self, num_cars: usize) {
        let roster: Vec<usize> = (0..num_cars)
            .map(|slot| self.preset_for_slot(slot))
            .collect();
        if self.car_ids.len() != num_cars || self.arena_body.as_ref() != Some(&roster) {
            let (arena, car_ids) = fresh_arena(
                num_cars,
                &roster,
                self.dodge_deadzone,
                self.mem_weight_mode,
                self.capture_events,
            );
            self.arena = arena;
            self.car_ids = car_ids;
            self.arena_body = Some(roster);
            self.last_ball_hit = vec![0; num_cars];
        }
    }

    /// Step the arena without callbacks, event scans, or metric allocations.
    #[allow(dead_code)]
    pub fn benchmark_step(&mut self, controls: &[CarControls]) {
        for (slot, controls) in controls.iter().enumerate() {
            if let Some(&car_id) = self.car_ids.get(slot) {
                self.arena
                    .pin_mut()
                    .set_car_controls(car_id, *controls)
                    .expect("v2 car id is valid");
            }
        }
        self.arena.pin_mut().step(1);
    }

    /// Convert one recording control before the timed loop.
    #[allow(dead_code)]
    pub fn benchmark_control(controls: ControlsRecord) -> CarControls {
        v2_controls(&controls)
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
    num_cars: usize,
    roster: &[usize],
    dodge_deadzone: f32,
    mem_weight_mode: ArenaMemWeightMode,
    capture_events: bool,
) -> (rocketsim_rs::cxx::UniquePtr<Arena>, Vec<u32>) {
    let config = ArenaConfig {
        mem_weight_mode,
        no_ball_rot: false,
        ..Default::default()
    };
    let mut arena = Arena::new(GameMode::Soccar, config, 120);
    if capture_events {
        arena.pin_mut().set_car_bump_callback(v2_bump_callback, 0);
    }
    let car_ids = (0..num_cars.max(1))
        .map(|slot| {
            let team = if slot.is_multiple_of(2) {
                Team::Blue
            } else {
                Team::Orange
            };
            let mut car_config = *v2_preset(roster.get(slot).copied().unwrap_or(OCTANE_PRESET));
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
        backend.ensure_cars(2);
        // The v2 arena hands out the same ids on every rebuild, so a planted
        // state is what proves whether the arena was replaced.
        let sentinel = Vec3::new(1234.0, 567.0, 89.0);
        let first_id = backend.car_ids[0];
        let mut planted = backend.arena.pin_mut().get_car(first_id);
        planted.pos = sentinel;
        backend
            .arena
            .pin_mut()
            .set_car(first_id, planted)
            .expect("car id is valid");
        // Same body, same count: no rebuild, so the planted state survives.
        backend.ensure_cars(2);
        assert_eq!(backend.arena.pin_mut().get_car(first_id).pos, sentinel);
        // Same count, new body: rebuild, so the planted state is gone.
        assert_eq!(backend.set_body_from_info(&daizen_info()).unwrap(), "Plank");
        backend.ensure_cars(2);
        assert_eq!(backend.car_ids.len(), 2);
        assert_ne!(backend.arena.pin_mut().get_car(first_id).pos, sentinel);
        assert_eq!(backend.arena_body, Some(vec![2, 2]));
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
        backend.ensure_cars(2);
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
        backend.ensure_cars(3);
        assert_eq!(backend.arena_body, Some(vec![2, 0, 0]));
    }

    #[test]
    fn roster_change_rebuilds_without_count_change() {
        init();
        let mut backend = V2Backend::new();
        backend.set_bodies(&[0, 0]);
        backend.ensure_cars(2);
        let before = backend.arena_body.clone();
        backend.set_bodies(&[2, 0]);
        backend.ensure_cars(2);
        assert_ne!(backend.arena_body, before);
    }
}
