//! V3 replay backend for RLPR recordings.
//!
//! Uses one car per recorded car in Soccar (Blue first, then Orange),
//! with the body preset from the recording header.
//! Reset restores every car and ball state from the start tick.
//! Step applies each recorded car's controls for one tick.

use glam::Vec3A;
use rocketsim::{
    Arena, ArenaConfig, ArenaEvent, ArenaMemWeightMode, CarBodyConfig, CarControls, CarState,
    GameMode, HITBOX_OFFSETS, HITBOX_SIZES, PhysState, Team, consts::BT_TO_UU,
};
use rocketsim_test::rlpr::{
    cpp_records::{ControlsRecord, RecordingInfo},
    tick_record::TickRecord,
};

use super::common::{BodySnapshot, ReplayBackend, SimContactEvents, Snapshot};

/// Sim holder: one car per recorded car in Soccar.
///
/// The body preset comes from [`V3Backend::set_body_from_info`]; it stays
/// Octane until the caller selects a header.
pub struct V3Backend {
    arena: Arena,
    car_ids: Vec<usize>,
    body: CarBodyConfig,
    body_name: &'static str,
    arena_body: Option<CarBodyConfig>,
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
            body: CarBodyConfig::OCTANE,
            body_name: "Octane",
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

    /// Header-selected body config with this backend's dodge deadzone.
    fn car_config(&self) -> CarBodyConfig {
        let mut config = self.body;
        config.dodge_deadzone = self.dodge_deadzone;
        config
    }

    /// Select the sim body from one recording header.
    ///
    /// Call once per recording before `reset`. The next `reset`/`set_state`
    /// rebuilds the arena when the preset differs, even when the car count
    /// is unchanged. Unknown headers return an error and leave the previous
    /// preset in place: the metric must fail rather than score a guessed
    /// body. The header samples the first recorded car only, so a
    /// mixed-body roster is undetectable here.
    pub fn set_body_from_info(&mut self, info: &RecordingInfo) -> Result<&'static str, String> {
        let (preset, name) = body_from_info(info)?;
        self.body = preset;
        self.body_name = name;
        Ok(name)
    }

    /// Preset chosen by the last [`V3Backend::set_body_from_info`] call.
    pub fn body_name(&self) -> &'static str {
        self.body_name
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

    /// Rebuild the arena when the car count or the header-selected body changes.
    ///
    /// The body check matters when consecutive recordings hold the same car
    /// count with different presets: the ids would still line up, but the
    /// hitbox and wheels would stay wrong without a rebuild.
    fn ensure_cars(&mut self, num_cars: usize) {
        let config = self.car_config();
        if self.car_ids.len() != num_cars || self.arena_body != Some(config) {
            self.arena = self.new_arena();
            self.car_ids = (0..num_cars)
                .map(|slot| self.arena.add_car(Self::team_for_slot(slot), config))
                .collect();
            self.arena_body = Some(config);
        }
    }

    /// Restore the handbrake integrator before a replayed tick.
    pub fn set_handbrake_value(&mut self, car_idx: usize, value: f32) {
        let car_id = self.car_id(car_idx, "set_handbrake_value");
        let mut state = *self.arena.get_car_state(car_id);
        state.handbrake_val = value.clamp(0.0, 1.0);
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

/// Preset names in [`BODY_PRESETS`] order.
const BODY_PRESET_NAMES: [&str; 7] = [
    "Octane", "Dominus", "Plank", "Breakout", "Hybrid", "Merc", "Psyclops",
];

/// Largest header-vs-preset mismatch that still counts as a match, in uu.
///
/// The header stores f32 bounds in BT; scaling by [`BT_TO_UU`] leaves about
/// 4e-4 uu of rounding on the recorded captures. The closest presets
/// (Octane and Psyclops sizes) differ by 0.134 uu, so 0.01 separates every
/// known preset with wide margin on both sides.
const BODY_MATCH_TOL_UU: f32 = 0.01;

/// Match recording header hitbox bounds to one known body preset.
///
/// Scales the `RecordingInfo` min/max from BT to uu, then compares full size
/// and center offset against every known preset. Unknown bounds are an
/// error: the metric must fail rather than score a guessed body. The header
/// samples the first recorded car only, so a mixed-body roster or a custom
/// body is undetectable here and must not be guessed.
pub fn body_from_info(info: &RecordingInfo) -> Result<(CarBodyConfig, &'static str), String> {
    let min: Vec3A = info.hitbox_rel_min_bt.into();
    let max: Vec3A = info.hitbox_rel_max_bt.into();
    let size_uu = (max - min) * BT_TO_UU;
    let offset_uu = (max + min) * 0.5 * BT_TO_UU;
    for (index, name) in BODY_PRESET_NAMES.iter().enumerate() {
        let size_err = (size_uu - HITBOX_SIZES[index]).abs().max_element();
        let offset_err = (offset_uu - HITBOX_OFFSETS[index]).abs().max_element();
        if size_err <= BODY_MATCH_TOL_UU && offset_err <= BODY_MATCH_TOL_UU {
            return Ok((BODY_PRESETS[index], name));
        }
    }
    Err(format!(
        "unknown car body (size {size_uu:?} uu, offset {offset_uu:?} uu): no preset matches within {BODY_MATCH_TOL_UU} uu"
    ))
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
    use super::*;
    use rocketsim_test::rlpr::cpp_records::VecRecord;

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
        let (config, name) =
            body_from_info(&daizen_info()).expect("Daizen header is a known preset");
        assert_eq!(name, "Plank");
        assert_eq!(config.hitbox_size, CarBodyConfig::PLANK.hitbox_size);
        assert_eq!(
            config.hitbox_pos_offset,
            CarBodyConfig::PLANK.hitbox_pos_offset
        );
    }

    #[test]
    fn octane_header_stays_octane() {
        let (config, name) = body_from_info(&octane_info()).expect("Wisp header is a known preset");
        assert_eq!(name, "Octane");
        assert_eq!(config.hitbox_size, CarBodyConfig::OCTANE.hitbox_size);
        assert_eq!(
            config.hitbox_pos_offset,
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
        let (_, name) = body_from_info(&info).expect("exact preset bounds match");
        assert_eq!(name, "Psyclops");
    }

    #[test]
    fn unknown_header_is_an_error() {
        let info = info_for([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]);
        assert!(body_from_info(&info).is_err());
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
        assert_eq!(backend.body_name(), "Plank");
        backend.ensure_cars(4);
        assert_eq!(backend.car_ids.len(), 4);
        assert_ne!(
            backend.arena.get_car_state(backend.car_ids[0]).phys.pos,
            sentinel
        );
    }

    #[test]
    fn unknown_header_keeps_previous_body() {
        init();
        let mut backend = V3Backend::new();
        backend.set_body_from_info(&daizen_info()).unwrap();
        let bad = info_for([-1.0, -1.0, -1.0], [1.0, 1.0, 1.0]);
        assert!(backend.set_body_from_info(&bad).is_err());
        assert_eq!(backend.body_name(), "Plank");
    }
}
