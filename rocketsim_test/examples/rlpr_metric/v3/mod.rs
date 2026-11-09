//! V3 replay backend for RLPR recordings.
//!
//! Each available recording slot has one arena car in Soccar. Body presets
//! and teams follow the selected roster. The header is a body fallback.
//! Reset restores recorded car and ball state and supported prefix history.
//! V12 adds supersonic state, body internals, auto-flip and airborne clocks,
//! derived pad cooldowns, and retained contacts between dynamic bodies.
//! The v12 wheel world-contact group is captured but not applied: the v11
//! core wheel contact and `is_on_ground` stay authoritative. The v12 ball
//! activation state, deactivation timer, and sleeping thresholds restore the
//! recorded native sleep state. Complete captured-build sleep parity is not
//! proven. Do not add the upstream active-island wake rule without native
//! evidence. World contact history and warmstart impulses are not restored.
//! Step applies each recorded car's controls for one tick.

use glam::{Mat3A, Vec3A};
use rocketsim::{
    ActivationState, Arena, ArenaConfig, ArenaEvent, ArenaMemWeightMode, CarBodyConfig,
    CarControls, CarState, GameMode, HITBOX_OFFSETS, HITBOX_SIZES, PhysState, Team,
};
use rocketsim_test::rlpr::{
    cpp_records::{ControlsRecord, RecordingInfo},
    row_ext::CarExt,
    tick_record::TickRecord,
};

mod replay_car_history;
mod replay_car_state;
mod replay_collision_state;
mod replay_pad_state;

use replay_car_history::{CarHistoryTable, apply_car_replay_history};
use replay_car_state::{AutoFlipApply, ClockApply, apply_car_replay_state};
use replay_pad_state::{PadReplay, PadRestoreReport};

use super::{
    common,
    common::{
        BODY_PRESET_NAMES, BodySnapshot, HitboxBounds, NUM_BODY_PRESETS, ReplayBackend,
        SimContactEvents, Snapshot, body_preset_from_info,
    },
};

/// Sim holder: one car per recorded car in Soccar.
///
/// The body presets come from [`V3Backend::set_bodies`] or
/// [`V3Backend::set_body_from_info`]; they stay Octane until the caller
/// selects a roster. Presets are per car slot, so a mixed roster replays
/// with each car on its own body.
pub struct V3Backend {
    arena: Arena,
    /// Arena car id per recording slot. `None` means the slot is unavailable
    /// in the current partial arena.
    car_ids: Vec<Option<usize>>,
    /// Availability mask the current arena was built with.
    arena_mask: Option<Vec<bool>>,
    /// Body preset index per car slot, in [`BODY_PRESET_NAMES`] order.
    bodies: Vec<usize>,
    /// Team per car slot, in frozen recording order.
    teams: Vec<Team>,
    /// Presets the current arena was built with, to detect roster changes.
    arena_body: Option<Vec<CarBodyConfig>>,
    /// Teams the current arena was built with, to detect team changes.
    arena_teams: Option<Vec<Team>>,
    dodge_deadzone: f32,
    mem_weight_mode: ArenaMemWeightMode,
    car_history: Vec<CarHistoryTable>,
    pad_history: PadReplay,
    restore_counts: RestoreCounts,
}

#[derive(Default)]
struct RestoreCounts {
    car_rows: usize,
    autoflip_inactive: usize,
    autoflip_timed: usize,
    autoflip_unknown: usize,
    air_time_known: usize,
    since_jump_known: usize,
    pad_resets: usize,
    pads: PadRestoreReport,
    collision_resets: usize,
    collision_restored: usize,
    collision_skipped_manifolds: usize,
    collision_manifolds: usize,
    collision_points: usize,
    collision_cleared: usize,
    collision_reoriented: usize,
    collision_refused: usize,
    /// At least one reset restored captured ball sleep fields.
    /// Native sleep is honored. Captured-build wake parity is not proven.
    ball_sleep_restored: bool,
    /// Car reset writes that carried the v12 wheel world-contact group.
    /// The group is reported but not applied.
    wheel_world_ignored: usize,
    /// Extra-hit suppressions at tick count 0. The next step is blocked, but
    /// the step after it is blocked too. Exact one-step suppression is not
    /// representable at tick count 0 with the production gate.
    extra_hit_fresh_arena_overblock: usize,
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
            arena_mask: None,
            bodies: Vec::new(),
            teams: Vec::new(),
            arena_body: None,
            arena_teams: None,
            dodge_deadzone,
            mem_weight_mode,
            car_history: Vec::new(),
            pad_history: PadReplay::default(),
            restore_counts: RestoreCounts::default(),
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

    /// Select the team for every car slot, in recording order.
    ///
    /// Missing slots use the old alternating fallback. A team change rebuilds
    /// the arena even when the car count and bodies stay the same.
    pub fn set_teams(&mut self, teams: &[Team]) {
        self.teams = teams.to_vec();
    }

    /// Team per slot. Missing metadata keeps the old alternating behavior.
    fn team_for_slot(&self, slot: usize) -> Team {
        self.teams.get(slot).copied().unwrap_or_else(|| {
            if slot.is_multiple_of(2) {
                Team::Blue
            } else {
                Team::Orange
            }
        })
    }

    /// Recording slot for one arena car id.
    fn slot_for_arena_car(&self, arena_car: usize) -> Option<usize> {
        self.car_ids
            .iter()
            .position(|&stored| stored == Some(arena_car))
    }

    /// Arena car id for one available recording slot.
    ///
    /// Panics as `<caller> needs car <index>` on a missing or unavailable
    /// slot. Callers that may see an unavailable slot use [`Self::car_id_opt`].
    #[track_caller]
    fn car_id(&self, car_idx: usize, caller: &str) -> usize {
        match self.car_id_opt(car_idx) {
            Some(id) => id,
            None => panic!("{caller} needs car {car_idx}"),
        }
    }

    /// Arena car id for one recording slot, or `None` when unavailable.
    fn car_id_opt(&self, car_idx: usize) -> Option<usize> {
        self.car_ids.get(car_idx).copied().flatten()
    }

    /// Availability mask for a tick.
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
        let num_cars = mask.len();
        let configs: Vec<CarBodyConfig> = (0..num_cars).map(|slot| self.car_config(slot)).collect();
        let teams: Vec<Team> = (0..num_cars).map(|slot| self.team_for_slot(slot)).collect();
        let mask_vec = mask.to_vec();
        if self.car_ids.len() != num_cars
            || self.arena_body.as_ref() != Some(&configs)
            || self.arena_teams.as_ref() != Some(&teams)
            || self.arena_mask.as_ref() != Some(&mask_vec)
        {
            self.arena = self.new_arena();
            self.car_ids = mask
                .iter()
                .enumerate()
                .map(|(slot, &available)| {
                    available.then(|| self.arena.add_car(teams[slot], configs[slot]))
                })
                .collect();
            self.arena_body = Some(configs);
            self.arena_teams = Some(teams);
            self.arena_mask = Some(mask_vec);
        }
    }

    /// Test helper: build an arena with `num_cars` available slots.
    #[cfg(test)]
    fn ensure_all_cars(&mut self, num_cars: usize) {
        self.ensure_cars(&vec![true; num_cars]);
    }

    /// Restore the handbrake integrator before a replayed tick.
    pub fn set_handbrake_value(&mut self, car_idx: usize, value: f32) {
        let Some(car_id) = self.car_id_opt(car_idx) else {
            return;
        };
        self.arena.edit_car_state_full(car_id, |state| {
            state.handbrake_val = value.clamp(0.0, 1.0);
        });
    }

    /// Restore jump-hold continuity before a replayed tick.
    pub fn set_jump_hold_broken(&mut self, car_idx: usize, broken: bool) {
        let Some(car_id) = self.car_id_opt(car_idx) else {
            return;
        };
        self.arena.edit_car_state_full(car_id, |state| {
            state.jump_hold_broken = broken;
        });
    }

    /// Refresh prior-tick wheel gates without advancing dynamics.
    pub fn refresh_sticky_gates(&mut self) {
        for &car_id in self.car_ids.iter().flatten() {
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
            if !car.is_available() {
                continue;
            }
            // An available slot without an arena car is a caller bug: panic
            // instead of silently scoring nothing.
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
            ball.wake();
        });
    }

    /// Report writes made at full resets, not native checkpoint coverage.
    pub fn print_restore_counts(&self) {
        let counts = &self.restore_counts;
        if counts.car_rows == 0
            && !self.pad_history.has_mapping()
            && counts.collision_restored == 0
            && !counts.ball_sleep_restored
            && counts.wheel_world_ignored == 0
            && counts.extra_hit_fresh_arena_overblock == 0
        {
            return;
        }
        println!(
            "v3 restored car rows {}: auto-flip inactive {}, timed {}, unknown {}; airborne clock known {}, since-jump clock known {}",
            counts.car_rows,
            counts.autoflip_inactive,
            counts.autoflip_timed,
            counts.autoflip_unknown,
            counts.air_time_known,
            counts.since_jump_known,
        );
        println!(
            "v3 derived pads: {}; resets {}; active writes {}, cooldown writes {}, unchanged {}",
            self.pad_history.coverage(),
            counts.pad_resets,
            counts.pads.active,
            counts.pads.cooldown,
            counts.pads.unknown,
        );
        println!(
            "v3 partial collision restores {}/{}: manifolds {}, points {}, cleared {}, reoriented {}, unmapped manifolds {}, refused {}; warmstart impulses and world contacts not restored",
            counts.collision_restored,
            counts.collision_resets,
            counts.collision_manifolds,
            counts.collision_points,
            counts.collision_cleared,
            counts.collision_reoriented,
            counts.collision_skipped_manifolds,
            counts.collision_refused,
        );
        if counts.ball_sleep_restored {
            println!(
                "v3 ball sleep restored from captured activation state, deactivation timer, and sleeping thresholds; complete captured-build wake parity not proven"
            );
        }
        if counts.wheel_world_ignored > 0 {
            println!(
                "v3 wheel world contact captured but not replayed: {} car reset writes (core wheel has_contact and is_on_ground authoritative)",
                counts.wheel_world_ignored,
            );
        }
        if counts.extra_hit_fresh_arena_overblock > 0 {
            println!(
                "v3 extra-hit suppression at a fresh arena over-blocks one step: {} cases (deferred; exact one-step gate is not representable at tick count 0)",
                counts.extra_hit_fresh_arena_overblock,
            );
        }
    }

    /// Step the arena without collecting replay metric events.
    #[allow(dead_code)]
    pub fn benchmark_step(&mut self, controls: &[CarControls]) {
        for (slot, &controls) in controls.iter().enumerate() {
            let Some(car_id) = self.car_id_opt(slot) else {
                continue;
            };
            self.arena.set_car_controls(car_id, controls);
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

/// Apply the v12 car fields that are proven and safe to replay.
///
/// Supersonic: copy the flag. Copy the raw native timer only while the flag
/// is true. RocketSim requires a zero timer when the flag is false. The
/// native timer freezes after expiry; that frozen value is not copied.
///
/// Wheel world contact: captured but not applied. The v11 core wheel
/// `has_contact` and native `is_on_ground` are authoritative. The
/// `has_contact_world_geometry` flag is a strict subset and is distinct
/// unsupported data, not a substitute. This function does not touch the
/// contact cache.
///
/// Prefix-derived auto-flip and airborne clocks are applied separately.
/// Inverse inertia is applied later by [`ReplayBackend::restore_body_state`].
/// The update tag is not a verified execution order and is not applied.
fn apply_v12_car_state(state: &mut CarState, ext: &CarExt) {
    if let (Some(supersonic), Some(timer)) = (ext.b_supersonic, ext.time_below_supersonic_s) {
        state.is_supersonic = supersonic;
        state.supersonic_grace_timer = if supersonic { timer } else { 0.0 };
    }
}

/// Convert one RLPR matrix from row order to the `Mat3A` column layout.
fn tensor_from_rows(rows: [[f32; 3]; 3]) -> Mat3A {
    Mat3A::from_rows(
        Vec3A::from_array(rows[0]),
        Vec3A::from_array(rows[1]),
        Vec3A::from_array(rows[2]),
    )
}

/// Map the v12 native activation value to the core state.
///
/// RLPR v12 writes 1, 2, and 3 only. Other values are rejected during CLI
/// validation. They are never mapped to a default state.
fn v12_activation_state(value: u32) -> Option<ActivationState> {
    match value {
        1 => Some(ActivationState::Active),
        2 => Some(ActivationState::Sleeping),
        3 => Some(ActivationState::WantsDeactivation),
        _ => None,
    }
}

impl Default for V3Backend {
    fn default() -> Self {
        Self::new()
    }
}

impl ReplayBackend for V3Backend {
    fn prepare_replay(&mut self, ticks: &[TickRecord]) {
        self.restore_counts = RestoreCounts::default();
        self.car_history = if ticks.iter().any(|tick| tick.replay_state.is_some()) {
            (0..ticks.first().map_or(0, common::tick_car_count))
                .map(|slot| CarHistoryTable::from_ticks(ticks, slot))
                .collect()
        } else {
            Vec::new()
        };
        for (slot, history) in self.car_history.iter().enumerate() {
            if let Some(reason) = history.invalid_reason() {
                eprintln!(
                    "WARNING: car slot {slot} has invalid replay history: {reason}; clocks are partial"
                );
            }
        }
        self.pad_history = PadReplay::build(ticks, &self.arena.get_all_boost_pad_configs());
    }

    fn restore_auxiliary_state(&mut self, ticks: &[TickRecord], row: usize) {
        let counts = &mut self.restore_counts;
        if self.pad_history.has_mapping() {
            let report = self.pad_history.restore(&mut self.arena, ticks, row);
            counts.pad_resets += 1;
            counts.pads.merge(&report);
        }
        let tick = &ticks[row];
        if tick
            .replay_state
            .as_ref()
            .is_none_or(|ext| ext.collision_snapshot.is_none())
        {
            return;
        }
        counts.collision_resets += 1;
        let available: Vec<bool> = self.car_ids.iter().map(Option::is_some).collect();
        let extract =
            match replay_collision_state::extract_for_row_with_availability(tick, &available) {
                Ok(extract) => extract,
                Err(error) => {
                    if counts.collision_refused == 0 {
                        eprintln!("WARNING: collision restore refused at row {row}: {error}");
                    }
                    counts.collision_refused += 1;
                    return;
                }
            };
        match rocketsim_test::replay::collision::apply(
            &mut self.arena,
            &self.car_ids,
            &extract.state,
        ) {
            Ok(report) => {
                counts.collision_restored += 1;
                counts.collision_skipped_manifolds += extract.summary.skipped_manifolds;
                counts.collision_manifolds += report.manifolds_installed;
                counts.collision_points += report.points_installed;
                counts.collision_cleared += report.manifolds_cleared;
                counts.collision_reoriented += report.manifolds_reoriented;
            }
            Err(error) => {
                if counts.collision_refused == 0 {
                    eprintln!("WARNING: collision restore refused at row {row}: {error}");
                }
                counts.collision_refused += 1;
            }
        }
    }

    fn set_handbrake_value(&mut self, car_idx: usize, value: f32) {
        V3Backend::set_handbrake_value(self, car_idx, value);
    }

    fn set_jump_hold_broken(&mut self, car_idx: usize, broken: bool) {
        V3Backend::set_jump_hold_broken(self, car_idx, broken);
    }

    fn refresh_sticky_gates(&mut self) {
        V3Backend::refresh_sticky_gates(self);
    }

    fn restore_body_state(&mut self, state_tick: &TickRecord) {
        let Some(ext) = &state_tick.replay_state else {
            return;
        };
        for (slot, car) in state_tick.car_records.iter().enumerate() {
            if !car.is_available() {
                continue;
            }
            let Some(rows) = ext
                .cars
                .get(slot)
                .and_then(|car_ext| car_ext.inv_inertia_world)
            else {
                continue;
            };
            let Some(car_id) = self.car_id_opt(slot) else {
                continue;
            };
            let body_id = self
                .arena
                .car_body_id(car_id)
                .expect("mapped car has a body");
            let mut body = self
                .arena
                .body_state(body_id)
                .expect("body belongs to this arena");
            body.inv_inertia_tensor_world = tensor_from_rows(rows);
            self.arena
                .set_body_state(body_id, body)
                .expect("v12 car body state was validated before evaluation");
        }

        let Some(ball_ext) = &ext.ball else {
            return;
        };
        if ball_ext.inv_inertia_world.is_none()
            && ball_ext.interp_basis.is_none()
            && ball_ext.linear_sleeping_threshold.is_none()
            && ball_ext.angular_sleeping_threshold.is_none()
        {
            return;
        }

        // Start from the synchronized body so absent optional fields keep
        // their normal getter values.
        let body_id = self.arena.ball_body_id();
        let mut body = self
            .arena
            .body_state(body_id)
            .expect("ball body belongs to this arena");
        if let Some(rows) = ball_ext.inv_inertia_world {
            body.inv_inertia_tensor_world = tensor_from_rows(rows);
        }
        if let Some(rows) = ball_ext.interp_basis {
            body.interp_world_basis = tensor_from_rows(rows);
        }

        if let Some(value) = ball_ext.linear_sleeping_threshold {
            body.linear_sleeping_threshold = value;
        }
        if let Some(value) = ball_ext.angular_sleeping_threshold {
            body.angular_sleeping_threshold = value;
        }
        self.arena
            .set_body_state(body_id, body)
            .expect("v12 ball body state was validated before evaluation");
        if ball_ext.linear_sleeping_threshold.is_some()
            || ball_ext.angular_sleeping_threshold.is_some()
        {
            self.restore_counts.ball_sleep_restored = true;
        }
    }

    fn set_boost_state(&mut self, car_idx: usize, armed: bool, time: f32) {
        let Some(car_id) = self.car_id_opt(car_idx) else {
            return;
        };
        self.arena.edit_car_state_full(car_id, |state| {
            state.is_boosting = armed;
            state.boosting_time = time;
        });
    }

    fn supports_cooldown_restore(&self) -> bool {
        true
    }

    fn suppress_next_extra_hit(&mut self, car_idx: usize, suppress: bool) {
        let Some(car_id) = self.car_id_opt(car_idx) else {
            return;
        };
        // Deferred: exact one-step suppression at tick count 0 is not
        // representable. The production gate is `last + 1 < tick_count`.
        // At tick count 0, every `Some(last)` blocks both step 0 and step 1,
        // and `None` blocks neither. This harness has no production API to
        // advance the counter without a step. Keep the conservative
        // over-block and count it in the restore report.
        let last_extra_hit_tick = if suppress {
            let current = self.arena.tick_count();
            if current == 0 {
                self.restore_counts.extra_hit_fresh_arena_overblock += 1;
            }
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
        let Some(car_id) = self.car_id_opt(car_idx) else {
            return;
        };
        let Some(victim_id) = self.car_id_opt(victim_idx) else {
            return;
        };
        self.arena.edit_car_state_full(car_id, |state| {
            state.bump_cooldown_timer = seconds.max(0.0);
            state.bump_cooldown_victim = (seconds > 0.0).then_some(victim_id);
        });
    }

    fn reset(&mut self, start: &TickRecord) {
        self.ensure_cars(&Self::tick_mask(start));
        self.set_state(start);
    }

    fn reset_motion(&mut self, state_tick: &TickRecord) {
        V3Backend::reset_motion(self, state_tick);
    }

    fn set_state(&mut self, state_tick: &TickRecord) {
        self.ensure_cars(&Self::tick_mask(state_tick));
        for (slot, car) in state_tick.car_records.iter().enumerate() {
            if !car.is_available() {
                continue;
            }
            // An available slot without an arena car is a caller bug: panic
            // instead of silently scoring nothing.
            let car_id = self.car_id(slot, "set_state");
            let history = self.car_history.get(slot);
            let counts = &mut self.restore_counts;
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

                if let Some(ext) = state_tick
                    .replay_state
                    .as_ref()
                    .and_then(|replay| replay.cars.get(slot))
                {
                    apply_v12_car_state(state, ext);
                    if ext.wheel_contacts.is_some() {
                        counts.wheel_world_ignored += 1;
                    }
                    let report = if let Some(history) = history {
                        apply_car_replay_history(state, ext, history, car.phys.physics_frame)
                    } else {
                        apply_car_replay_state(state, ext)
                    };
                    counts.car_rows += 1;
                    match report.autoflip {
                        AutoFlipApply::Inactive => counts.autoflip_inactive += 1,
                        AutoFlipApply::Timed { .. } => counts.autoflip_timed += 1,
                        AutoFlipApply::Unknown => counts.autoflip_unknown += 1,
                        AutoFlipApply::Absent => {}
                    }
                    counts.air_time_known +=
                        usize::from(matches!(report.air_time, ClockApply::Exact(_)));
                    counts.since_jump_known +=
                        usize::from(matches!(report.air_time_since_jump, ClockApply::Exact(_)));
                }
            });
        }

        let recorded_ball: PhysState = state_tick.ball_record.into();
        let ball_ext = state_tick
            .replay_state
            .as_ref()
            .and_then(|ext| ext.ball.as_ref());
        self.arena.edit_ball_state(|ball| {
            ball.phys = recorded_ball;
            ball.wake();
            if let Some(ext) = ball_ext {
                if let Some(value) = ext.activation_state {
                    ball.activation_state = v12_activation_state(value)
                        .expect("v12 activation state was validated before evaluation");
                }
                if let Some(value) = ext.deactivation_time_s {
                    ball.deactivation_time = value;
                }
            }
        });
        if ball_ext
            .is_some_and(|ext| ext.activation_state.is_some() || ext.deactivation_time_s.is_some())
        {
            self.restore_counts.ball_sleep_restored = true;
        }
    }

    fn step(&mut self, controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
        for (slot, controls) in controls.iter().enumerate() {
            let Some(car_id) = self.car_id_opt(slot) else {
                continue;
            };
            let controls: CarControls = (*controls).into();
            self.arena.set_car_controls(car_id, controls);
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
mod tests;
