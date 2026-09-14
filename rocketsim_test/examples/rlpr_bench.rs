//! Multi-car segmented replay metric over RLPR recordings.
//!
//! Generalizes `rlpr_metric` to recordings with any constant car count and
//! adds the `car_car` category. Reset at each segment start, run open-loop,
//! score ticks after warmup. Prints one table per recording plus an
//! aggregate table over all input files.
//!
//! Scoring rules match `rlpr_metric/common.rs`: a tick passes when the
//! normalized error (each component divided by its tolerance, combined as a
//! norm over every car plus the ball) is below 1. Contact categories overlap;
//! support is the number of scored RL ticks carrying that label.

use std::path::{Path, PathBuf};

use clap::Parser;
use glam::Vec3A;
use rocketsim::{
    consts, Arena, ArenaConfig, ArenaEvent, CarBodyConfig, CarControls, CarState, GameMode,
    PhysState, Team,
};
use rocketsim_test::rlpr::{
    cpp_records::{ControlsRecord, Mat3Record},
    tick_record::TickRecord,
    wheel_mode::detect_wheel_raycast_mode,
    Recording,
};

#[cfg(feature = "v2")]
#[path = "rlpr_bench/v2.rs"]
mod v2;

/// Arena config for a recording.
///
/// `v3-foxe-latest-change-today`: the wheel suspension raycast mode is
/// selected from the recorded wheel stream (`ArenaPlanes` for captures
/// produced by planes-only suspension engines, `World` otherwise).
/// Detect the corrected-v10 (Foxe) recorder from the serialized data:
/// its post-step angular cap keeps `ang_vel <= 5.5` for every car record,
/// it stores the already-scaled dodge torque, and it exposes zero-time
/// airborne `is_jumping` flickers. Legacy recordings violate the cap
/// routinely and store unit flip directions.
fn detect_v10_parity(recording: &Recording) -> bool {
    let mut has_marker = false;
    for tick in &recording.ticks {
        for r in &tick.car_records {
            if Vec3A::from(r.phys.ang_vel).length() > consts::car::MAX_ANG_SPEED + 1e-3 {
                return false;
            }
            if !has_marker {
                let t = Vec3A::from(r.flip_rel_torque);
                has_marker = (t.x.abs() > 1.5 || t.y.abs() > 1.5)
                    || (r.is_jumping && r.jump_time == 0.0 && !r.is_on_ground);
            }
        }
    }
    has_marker
}

fn arena_config(recording: &Recording) -> ArenaConfig {
    ArenaConfig::new(GameMode::Soccar).with_wheel_raycast_mode(detect_wheel_raycast_mode(
        recording,
        GameMode::Soccar,
        &CarBodyConfig::OCTANE,
    ))
}

fn mesh_dir() -> String {
    std::env::var("RLPR_MESH_DIR")
        .unwrap_or_else(|_| concat!(env!("CARGO_MANIFEST_DIR"), "/../collision_meshes").to_string())
}

/// Multi-car replay metric over RLPR files or directories of `*.rlpr`.
#[derive(Parser)]
struct Args {
    /// RLPR recording files and/or directories containing `*.rlpr` files.
    #[arg(required = true)]
    inputs: Vec<PathBuf>,

    /// Ticks per segment.
    #[arg(long, default_value_t = 120)]
    segment_ticks: usize,

    /// Warmup ticks per segment that advance the sim without scoring.
    #[arg(long, default_value_t = 2)]
    warmup_ticks: usize,

    /// Reset to the prior RL state before each scored tick.
    #[arg(long)]
    reset_each_tick: bool,

    /// Evaluate with `fox_eval`'s flow instead of the segmented metric:
    /// with `--reset-each-tick`, its one-step reset evaluator; without,
    /// its continuous FOX_LIVE replay.
    #[arg(long, alias = "fox_eval", visible_alias = "fox_eval")]
    fox_eval: bool,

    /// Label printed in the backend column.
    #[arg(long, default_value = "v3")]
    backend: String,

    /// Replay engine: `v3` (this workspace) or `v2` (C++ RocketSim via
    /// `rocketsim-rs`; requires building with `--features v2`).
    #[arg(long, default_value = "v3")]
    engine: String,
}

// Fixed Soccar geometry in Unreal units.
const SOCCAR_HALF_X: f32 = 4096.0;
const SOCCAR_HALF_Y: f32 = 5120.0;
const SOCCAR_CEIL_Z: f32 = 2048.0;
const SOCCAR_BALL_RADIUS: f32 = 91.25;
const SOCCAR_CAR_BOUND_RADIUS: f32 = 90.0;
// Distance band around a wall plane that counts as near contact.
const WORLD_PROX_MARGIN: f32 = 30.0;
// Minimum normal speed that counts as a velocity flip.
const VEL_FLIP_MIN: f32 = 50.0;
// Max car-ball center distance that counts as inferred contact.
const CAR_BALL_DIST: f32 = 220.0;
// Max car-car center distance that counts as inferred contact.
const CAR_CAR_DIST: f32 = 220.0;
// Min velocity jump on either body that confirms inferred contact.
const CONTACT_DELTA_VEL: f32 = 300.0;

// Strict per-component tolerances. See `normalized_error`.
const POS_TOL_UU: f32 = 10.0;
const VEL_TOL_UU_S: f32 = 3.0;
const ANG_VEL_TOL_RAD_S: f32 = 1.0;
const AXIS_TOL: f32 = 1.0;

/// Plain body state. Axes are unit vectors.
#[derive(Clone, Copy, Debug)]
struct BodySnapshot {
    pos: Vec3A,
    vel: Vec3A,
    ang_vel: Vec3A,
    forward: Vec3A,
    up: Vec3A,
}

/// All cars plus the ball.
#[derive(Clone, Debug)]
struct Snapshot {
    cars: Vec<BodySnapshot>,
    ball: BodySnapshot,
}

/// Per-car reconcile action for one open-loop step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CarAction {
    /// Drive from the recorded controls; include in the norm.
    Live,
    /// Record is a stale placeholder (frame advanced, phys bit-identical,
    /// not genuinely parked): park the sim car as demoed and exclude it.
    Park,
    /// Recorded state is a discontinuity (teleport, or a stale car coming
    /// back live): restore the sim car from the record and exclude it.
    Restore,
}

/// Per-tick reconcile plan, mirroring `fox_eval`'s stale/teleport rules.
#[derive(Clone, Debug)]
struct Reconcile {
    car: Vec<CarAction>,
    /// Ball record jumped more than one tick of travel can cover.
    ball_teleport: bool,
    /// Ball record is a stale placeholder while its frame advanced.
    ball_stale: bool,
    /// The *source* ball sample is already a parked mid-air placeholder;
    /// keep the sim ball out of play without blocking scoring.
    ball_park: bool,
    /// Every car record is stale: nothing physical was simulated.
    all_cars_stale: bool,
}

/// Per-recording context: the ticks plus forward-reconstructed per-car state
/// that RLPR does not serialize (ported from `fox_eval`).
struct Ctx<'a> {
    ticks: &'a [TickRecord],
    /// `air_time_since_jump[tick][car]` — dodge window timing.
    air_time_since_jump: Vec<Vec<f32>>,
    /// `flip_untrusted[tick][car]` — flip already in progress at capture
    /// start; its serialized `flip_time` cannot be reproduced.
    flip_untrusted: Vec<Vec<bool>>,
}

/// Reconstruct `air_time_since_jump`, which the RLPR record does not
/// serialize. Forward-tracked: counts while airborne with `has_jumped &&
/// !is_jumping`, resets otherwise. Copied from `fox_eval`.
fn reconstruct_air_time_since_jump(ticks: &[TickRecord], num_cars: usize) -> Vec<Vec<f32>> {
    const DT: f32 = consts::TICK_TIME;
    let n = ticks.len();
    let mut est = vec![vec![0.0f32; num_cars]; n];
    for j in 0..num_cars {
        let mut cur = 0.0f32;
        for i in 0..n {
            est[i][j] = cur;
            let Some(r) = ticks[i].car_records.get(j) else {
                continue;
            };
            // A single-tick airborne `is_jumping` flicker is an ignored jump
            // press (window expired, no impulse); don't let it zero the
            // window timer.
            let phantom_jump = r.is_jumping
                && !r.is_on_ground
                && r.jump_time == 0.0
                && i > 0
                && ticks[i - 1]
                    .car_records
                    .get(j)
                    .is_some_and(|p| !p.is_jumping)
                && ticks
                    .get(i + 1)
                    .and_then(|t| t.car_records.get(j))
                    .is_some_and(|p| !p.is_jumping);
            cur = if r.is_on_ground || (r.is_jumping && !phantom_jump) || !r.has_jumped {
                0.0
            } else {
                cur + DT
            };
        }
    }
    est
}

/// Per-car flag: a flip already in progress at the first observed tick has an
/// unverifiable internal `flip_time`. Those transitions are stepped but not
/// scored until the car is observed not flipping. Copied from `fox_eval`.
fn compute_flip_untrusted(ticks: &[TickRecord], num_cars: usize) -> Vec<Vec<bool>> {
    let n = ticks.len();
    let mut out = vec![vec![false; num_cars]; n];
    for j in 0..num_cars {
        let mut untrusted = ticks
            .first()
            .is_some_and(|t| t.car_records.get(j).is_some_and(|r| r.is_flipping));
        for i in 0..n {
            let Some(r) = ticks[i].car_records.get(j) else {
                continue;
            };
            if !r.is_flipping {
                untrusted = false;
            }
            out[i][j] = untrusted && r.is_flipping;
        }
    }
    out
}

/// Backend adapter. Holds the sim. Resets only at segment starts.
trait ReplayBackend {
    fn reset(&mut self, ctx: &Ctx, start: usize);
    fn set_state(&mut self, ctx: &Ctx, tick: usize);
    /// Before the step: park stale/restoring cars as demoed and move a
    /// stale or teleporting ball out of play.
    fn reconcile_pre(&mut self, ctx: &Ctx, target: usize, plan: &Reconcile);
    /// After the step: restore teleported/resumed cars and a teleported
    /// ball to the target record so following ticks stay on track.
    fn reconcile_post(&mut self, ctx: &Ctx, target: usize, plan: &Reconcile);
    fn step(&mut self, controls: &[ControlsRecord]);
    fn snapshot(&mut self) -> Snapshot;
    fn car_state(&mut self, car_idx: usize) -> CarState;

    /// `fox_eval`-style full restore: every car from `state_idx` (controls
    /// from `state_idx + 1`), stale cars parked as demoed, ball restored
    /// and optionally moved out of play.
    fn fox_set_state(&mut self, ctx: &Ctx, state_idx: usize, car_stale: &[bool], ball_park: bool);
    /// `fox_eval`-style per-tick car restore only (live mode); stale
    /// placeholder cars are parked as demoed.
    fn fox_set_cars(&mut self, ctx: &Ctx, state_idx: usize, car_stale: &[bool]);
    /// Restore only the ball phys from `state_idx` (live mode).
    fn fox_set_ball(&mut self, ctx: &Ctx, state_idx: usize);
    /// Whether the last step emitted a car-ball hit event (live mode's
    /// `preserve_ball_state`).
    fn ball_was_hit(&self) -> bool;
}

/// Classify one transition `from` -> `to` into a reconcile plan.
fn reconcile_plan(from: &TickRecord, to: &TickRecord, was_parked: &[bool]) -> Reconcile {
    let mut car = vec![CarAction::Live; to.car_records.len()];
    let mut all_stale = true;
    for (j, (ca, cb)) in from.car_records.iter().zip(&to.car_records).enumerate() {
        if from.car_records.len() != to.car_records.len() {
            break;
        }
        let frame_advanced = cb.phys.physics_frame != ca.phys.physics_frame;
        let phys_identical = ca.phys.pos == cb.phys.pos
            && ca.phys.rot == cb.phys.rot
            && ca.phys.lin_vel == cb.phys.lin_vel
            && ca.phys.ang_vel == cb.phys.ang_vel;
        let parked =
            ca.is_on_ground && Vec3A::from(ca.phys.lin_vel).length() < 1.0;
        let stale = frame_advanced && phys_identical && !parked;
        let moved = (Vec3A::from(cb.phys.pos) - Vec3A::from(ca.phys.pos)).length();
        let teleport = moved > Vec3A::from(ca.phys.lin_vel).length() / 120.0 + 100.0;
        if stale {
            car[j] = CarAction::Park;
        } else {
            all_stale = false;
            if teleport || was_parked.get(j).copied().unwrap_or(false) {
                car[j] = CarAction::Restore;
            }
        }
    }
    let ball_moved =
        (Vec3A::from(to.ball_record.pos) - Vec3A::from(from.ball_record.pos)).length();
    let ball_teleport = ball_moved > 100.0;
    let ball_stale = to.ball_record.physics_frame != from.ball_record.physics_frame
        && from.ball_record.pos == to.ball_record.pos
        && from.ball_record.rot == to.ball_record.rot
        && from.ball_record.lin_vel == to.ball_record.lin_vel
        && from.ball_record.ang_vel == to.ball_record.ang_vel
        && !(Vec3A::from(from.ball_record.pos).z < 120.0
            && Vec3A::from(from.ball_record.lin_vel).length() < 1.0);
    let ball_park = ball_stale
        || (Vec3A::from(from.ball_record.pos).z >= 120.0
            && Vec3A::from(from.ball_record.lin_vel).length() < 1.0
            && Vec3A::from(from.ball_record.ang_vel).length() < 1.0);
    Reconcile {
        car,
        ball_teleport,
        ball_stale,
        ball_park,
        all_cars_stale: all_stale && !to.car_records.is_empty(),
    }
}

fn body_from_phys(pos: Vec3A, vel: Vec3A, ang_vel: Vec3A, rot: &Mat3Record) -> BodySnapshot {
    BodySnapshot {
        pos,
        vel,
        ang_vel,
        forward: Vec3A::from(rot.column(0)),
        up: Vec3A::from(rot.column(2)),
    }
}

/// Read ground truth from a tick. `None` without any car.
fn snapshot_from_tick(tick: &TickRecord) -> Option<Snapshot> {
    if tick.car_records.is_empty() {
        return None;
    }
    let cars = tick
        .car_records
        .iter()
        .map(|car| {
            body_from_phys(
                car.phys.pos.into(),
                car.phys.lin_vel.into(),
                car.phys.ang_vel.into(),
                &car.phys.rot,
            )
        })
        .collect();
    let ball = body_from_phys(
        tick.ball_record.pos.into(),
        tick.ball_record.lin_vel.into(),
        tick.ball_record.ang_vel.into(),
        &tick.ball_record.rot,
    );
    Some(Snapshot { cars, ball })
}

/// Segment length config.
#[derive(Clone, Copy, Debug)]
struct SegmentConfig {
    segment_ticks: usize,
    warmup_ticks: usize,
}

impl SegmentConfig {
    fn is_valid(&self) -> bool {
        self.segment_ticks > 0 && self.warmup_ticks < self.segment_ticks
    }

    fn scored_len(&self, len: usize) -> usize {
        len.saturating_sub(self.warmup_ticks)
    }
}

/// Non-overlapping run of recording ticks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Segment {
    start: usize,
    len: usize,
}

impl Segment {
    fn end(&self) -> usize {
        self.start + self.len
    }
}

/// Physics frames advance by one for every car and the ball.
fn frame_is_contiguous(from: &TickRecord, to: &TickRecord) -> bool {
    if from.car_records.len() != to.car_records.len() {
        return false;
    }
    to.ball_record.physics_frame == from.ball_record.physics_frame + 1
        && from
            .car_records
            .iter()
            .zip(&to.car_records)
            .all(|(a, b)| b.phys.physics_frame == a.phys.physics_frame + 1)
}

/// No body moved between ticks (pause or replay stall).
fn tick_is_frozen(from: &TickRecord, to: &TickRecord) -> bool {
    if from.car_records.len() != to.car_records.len() {
        return false;
    }
    from.ball_record.pos == to.ball_record.pos
        && from.ball_record.lin_vel == to.ball_record.lin_vel
        && from.ball_record.ang_vel == to.ball_record.ang_vel
        && from
            .car_records
            .iter()
            .zip(&to.car_records)
            .all(|(a, b)| {
                a.phys.pos == b.phys.pos
                    && a.phys.lin_vel == b.phys.lin_vel
                    && a.phys.ang_vel == b.phys.ang_vel
            })
}

/// Split ticks into non-overlapping segments.
///
/// Break runs at car-count changes, frame gaps, and frozen transitions.
/// Chunk each run into groups of `segment_ticks`. Drop groups with no scored
/// ticks. Reset only at segment starts.
fn split_segments(ticks: &[TickRecord], config: SegmentConfig) -> Vec<Segment> {
    if !config.is_valid() {
        return Vec::new();
    }
    let mut segments = Vec::new();
    let mut run_start: Option<usize> = None;

    let mut flush_run = |end: usize, run_start: &mut Option<usize>| {
        if let Some(start) = run_start.take() {
            push_chunks(&mut segments, start, end, config);
        }
    };

    for (index, tick) in ticks.iter().enumerate() {
        if tick.car_records.is_empty() {
            flush_run(index, &mut run_start);
            continue;
        }
        match run_start {
            None => run_start = Some(index),
            Some(start) => {
                let prev = &ticks[index - 1];
                if prev.car_records.len() != tick.car_records.len()
                    || prev.car_records.len() != ticks[start].car_records.len()
                    || !frame_is_contiguous(prev, tick)
                    || tick_is_frozen(prev, tick)
                {
                    flush_run(index, &mut run_start);
                    run_start = Some(index);
                }
            }
        }
    }
    flush_run(ticks.len(), &mut run_start);
    segments
}

/// Chunk one clean run. Drop chunks with no scored ticks.
fn push_chunks(segments: &mut Vec<Segment>, start: usize, end: usize, config: SegmentConfig) {
    let mut offset = start;
    while offset < end {
        let len = config.segment_ticks.min(end - offset);
        if config.scored_len(len) > 0 {
            segments.push(Segment { start: offset, len });
        }
        offset += len;
    }
}

/// Overlapping contact categories plus the `Total` aggregate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum ContactCategory {
    CarBall,
    CarCar,
    BallWorld,
    ChassisWorld,
    WheelWorld,
    NoContact,
    Total,
}

impl ContactCategory {
    /// All categories in CLI column order.
    const ALL: [ContactCategory; 7] = [
        ContactCategory::CarBall,
        ContactCategory::CarCar,
        ContactCategory::BallWorld,
        ContactCategory::ChassisWorld,
        ContactCategory::WheelWorld,
        ContactCategory::NoContact,
        ContactCategory::Total,
    ];

    fn as_str(&self) -> &'static str {
        match self {
            ContactCategory::CarBall => "car_ball",
            ContactCategory::CarCar => "car_car",
            ContactCategory::BallWorld => "ball_world",
            ContactCategory::ChassisWorld => "chassis_world",
            ContactCategory::WheelWorld => "wheel_world",
            ContactCategory::NoContact => "no_contact",
            ContactCategory::Total => "total",
        }
    }
}

/// Per-tick contact labels. Only `NoContact` is exclusive.
#[derive(Clone, Copy, Debug, Default)]
struct ContactLabels {
    car_ball: bool,
    car_car: bool,
    ball_world: bool,
    chassis_world: bool,
    wheel_world: bool,
}

impl ContactLabels {
    fn is_quiet(&self) -> bool {
        !(self.car_ball || self.car_car || self.ball_world || self.chassis_world || self.wheel_world)
    }

    fn contains(&self, category: ContactCategory) -> bool {
        match category {
            ContactCategory::CarBall => self.car_ball,
            ContactCategory::CarCar => self.car_car,
            ContactCategory::BallWorld => self.ball_world,
            ContactCategory::ChassisWorld => self.chassis_world,
            ContactCategory::WheelWorld => self.wheel_world,
            ContactCategory::NoContact => self.is_quiet(),
            ContactCategory::Total => true,
        }
    }
}

/// Label one target tick from RL flags plus conservative inference.
///
/// Side-wall hits can miss `has_world_contact`, so near-wall ticks with a
/// flipped normal velocity also count. Missed car-ball and car-car touches
/// count when centers are close and either body velocity jumps.
fn classify_tick(tick: &TickRecord, prev: Option<&TickRecord>) -> ContactLabels {
    let cars = &tick.car_records;
    if cars.is_empty() {
        return ContactLabels::default();
    }
    let car_ball =
        cars.iter().any(|car| car.is_touching_ball) || infer_car_ball(tick, prev);
    let car_car = infer_car_car(tick, prev);
    let wheel_world = cars
        .iter()
        .any(|car| car.wheels.iter().any(|wheel| wheel.has_contact));
    let ball_world = tick.ball_record.has_world_contact
        || infer_wall_hit(
            prev.map(|p| p.ball_record.lin_vel.into()),
            tick.ball_record.pos.into(),
            tick.ball_record.lin_vel.into(),
            SOCCAR_BALL_RADIUS,
        );
    let chassis_world = cars.iter().any(|car| car.phys.has_world_contact)
        || infer_chassis_wall_hit(tick, prev);
    ContactLabels {
        car_ball,
        car_car,
        ball_world,
        chassis_world,
        wheel_world,
    }
}

/// Per-car velocity jump between prev and tick, aligned by index.
fn car_vel_jump(tick: &TickRecord, prev: Option<&TickRecord>, car_idx: usize) -> f32 {
    let Some(prev) = prev else { return 0.0 };
    if prev.car_records.len() != tick.car_records.len() {
        return 0.0;
    }
    let (Some(car), Some(prev_car)) = (
        tick.car_records.get(car_idx),
        prev.car_records.get(car_idx),
    ) else {
        return 0.0;
    };
    let cur: Vec3A = car.phys.lin_vel.into();
    let old: Vec3A = prev_car.phys.lin_vel.into();
    (cur - old).length()
}

/// Missed car-ball touch: close centers plus a velocity jump on either body.
fn infer_car_ball(tick: &TickRecord, prev: Option<&TickRecord>) -> bool {
    let Some(prev) = prev else { return false };
    let ball_pos: Vec3A = tick.ball_record.pos.into();
    let ball_vel: Vec3A = tick.ball_record.lin_vel.into();
    let prev_ball_vel: Vec3A = prev.ball_record.lin_vel.into();
    let ball_jump = (ball_vel - prev_ball_vel).length() >= CONTACT_DELTA_VEL;
    tick.car_records.iter().enumerate().any(|(i, car)| {
        let car_pos: Vec3A = car.phys.pos.into();
        (car_pos - ball_pos).length() < CAR_BALL_DIST
            && (ball_jump || car_vel_jump(tick, Some(prev), i) >= CONTACT_DELTA_VEL)
    })
}

/// Missed car-car touch: close centers plus a velocity jump on either car.
fn infer_car_car(tick: &TickRecord, prev: Option<&TickRecord>) -> bool {
    let cars = &tick.car_records;
    for i in 0..cars.len() {
        for j in (i + 1)..cars.len() {
            let a: Vec3A = cars[i].phys.pos.into();
            let b: Vec3A = cars[j].phys.pos.into();
            if (a - b).length() >= CAR_CAR_DIST {
                continue;
            }
            if car_vel_jump(tick, prev, i) >= CONTACT_DELTA_VEL
                || car_vel_jump(tick, prev, j) >= CONTACT_DELTA_VEL
            {
                return true;
            }
        }
    }
    false
}

/// Missed wall hit: wall proximity plus a flipped normal velocity.
fn infer_wall_hit(prev_vel: Option<Vec3A>, pos: Vec3A, vel: Vec3A, radius: f32) -> bool {
    let Some(prev_vel) = prev_vel else {
        return false;
    };
    for axis in 0..3 {
        let (limit, is_floor_ceil) = match axis {
            0 => (SOCCAR_HALF_X, false),
            1 => (SOCCAR_HALF_Y, false),
            _ => (SOCCAR_CEIL_Z, true),
        };
        if !near_plane(pos[axis], limit, radius, is_floor_ceil) {
            continue;
        }
        if flipped(prev_vel[axis], vel[axis]) {
            return true;
        }
    }
    false
}

/// Missed chassis wall hit on any car.
fn infer_chassis_wall_hit(tick: &TickRecord, prev: Option<&TickRecord>) -> bool {
    tick.car_records.iter().enumerate().any(|(i, car)| {
        let prev_vel: Option<Vec3A> = prev.and_then(|p| {
            if p.car_records.len() == tick.car_records.len() {
                p.car_records.get(i).map(|c| c.phys.lin_vel.into())
            } else {
                None
            }
        });
        infer_wall_hit(
            prev_vel,
            car.phys.pos.into(),
            car.phys.lin_vel.into(),
            SOCCAR_CAR_BOUND_RADIUS,
        )
    })
}

/// Wall proximity on one axis. `limit` is the positive plane distance.
fn near_plane(pos: f32, limit: f32, radius: f32, is_vertical: bool) -> bool {
    if is_vertical {
        pos < radius + WORLD_PROX_MARGIN || (limit - pos) < radius + WORLD_PROX_MARGIN
    } else {
        (limit - pos.abs()) < radius + WORLD_PROX_MARGIN
    }
}

/// Normal velocity flipped sign with enough speed.
fn flipped(before: f32, after: f32) -> bool {
    before.abs() >= VEL_FLIP_MIN && after.abs() >= VEL_FLIP_MIN && before.signum() != after.signum()
}

/// Squared normalized error of one body.
fn body_err_sq(sim: &BodySnapshot, truth: &BodySnapshot) -> f32 {
    let pos = (sim.pos - truth.pos).length() / POS_TOL_UU;
    let vel = (sim.vel - truth.vel).length() / VEL_TOL_UU_S;
    let ang = (sim.ang_vel - truth.ang_vel).length() / ANG_VEL_TOL_RAD_S;
    let fwd = (sim.forward - truth.forward).length() / AXIS_TOL;
    let up = (sim.up - truth.up).length() / AXIS_TOL;
    pos * pos + vel * vel + ang * ang + fwd * fwd + up * up
}

/// Normalized physics error over every live car plus the ball.
/// `car_skip[i]` drops parked/restored cars; `ball_skip` drops the ball.
/// Passes when the norm is below 1.
fn normalized_error(
    sim: &Snapshot,
    truth: &Snapshot,
    car_skip: &[bool],
    ball_skip: bool,
) -> f32 {
    let mut sum = if ball_skip {
        0.0
    } else {
        body_err_sq(&sim.ball, &truth.ball)
    };
    for (i, (sim_car, truth_car)) in sim.cars.iter().zip(&truth.cars).enumerate() {
        if car_skip.get(i).copied().unwrap_or(false) {
            continue;
        }
        sum += body_err_sq(sim_car, truth_car);
    }
    sum.sqrt()
}

/// Aggregate stats for one category.
#[derive(Clone, Debug, Default)]
struct CategoryStats {
    support: usize,
    passed: usize,
    sum_norm: f64,
    max_norm: f32,
    first_fail_tick: Option<usize>,
}

impl CategoryStats {
    fn add(&mut self, tick_index: usize, norm_error: f32) {
        self.support += 1;
        self.sum_norm += norm_error as f64;
        self.max_norm = self.max_norm.max(norm_error);
        if norm_error < 1.0 {
            self.passed += 1;
        } else if self.first_fail_tick.is_none() {
            self.first_fail_tick = Some(tick_index);
        }
    }

    fn mean_norm(&self) -> f64 {
        if self.support == 0 {
            0.0
        } else {
            self.sum_norm / self.support as f64
        }
    }

    fn rate(&self) -> f64 {
        if self.support == 0 {
            0.0
        } else {
            100.0 * self.passed as f64 / self.support as f64
        }
    }
}

/// Per-category report. Contact categories overlap.
#[derive(Clone, Debug, Default)]
struct EvalReport {
    per_category: [CategoryStats; 7],
    /// First failing location per category as `recording@tick`, for the
    /// aggregate table across multiple recordings.
    first_fail_at: [Option<String>; 7],
}

impl EvalReport {
    fn add(&mut self, labels: ContactLabels, tick_index: usize, norm_error: f32) {
        for (i, category) in ContactCategory::ALL.iter().enumerate() {
            if labels.contains(*category) {
                self.per_category[i].add(tick_index, norm_error);
            }
        }
    }

    fn merge(&mut self, other: &EvalReport, recording_name: &str) {
        for i in 0..ContactCategory::ALL.len() {
            let dst = &mut self.per_category[i];
            let src = &other.per_category[i];
            dst.support += src.support;
            dst.passed += src.passed;
            dst.sum_norm += src.sum_norm;
            dst.max_norm = dst.max_norm.max(src.max_norm);
            if let (None, Some(tick)) = (dst.first_fail_tick, src.first_fail_tick) {
                dst.first_fail_tick = Some(tick);
                self.first_fail_at[i] = Some(format!("{recording_name}@{tick}"));
            }
        }
    }

    fn stats(&self, category: ContactCategory) -> &CategoryStats {
        &self.per_category[ContactCategory::ALL
            .iter()
            .position(|c| *c == category)
            .unwrap()]
    }

    fn first_fail_label(&self, category: ContactCategory) -> String {
        let i = ContactCategory::ALL
            .iter()
            .position(|c| *c == category)
            .unwrap();
        if let Some(at) = &self.first_fail_at[i] {
            return at.clone();
        }
        self.per_category[i]
            .first_fail_tick
            .map(|tick| tick.to_string())
            .unwrap_or_else(|| "-".to_string())
    }
}

/// Run each segment open-loop and aggregate errors.
/// Resets at each segment start, steps with each target tick's recorded
/// `prev_controls`, skips `warmup_ticks` ticks, labels scored ticks from
/// ground truth.
fn evaluate<B: ReplayBackend>(
    backend: &mut B,
    ctx: &Ctx,
    segments: &[Segment],
    warmup_ticks: usize,
    reset_each_tick: bool,
) -> EvalReport {
    let ticks = ctx.ticks;
    let trace_range = std::env::var("BENCH_TRACE")
        .ok()
        .and_then(|s| {
            let (a, b) = s.split_once(':')?;
            Some(a.parse::<usize>().ok()?..b.parse::<usize>().ok()?)
        })
        .unwrap_or(usize::MAX..usize::MAX);
    let mut report = EvalReport::default();
    // Cars currently parked as stale placeholders.
    let mut parked: Vec<bool> = Vec::new();
    for segment in segments {
        if segment.end() > ticks.len() {
            continue;
        }
        if !reset_each_tick {
            backend.reset(ctx, segment.start);
        }
        parked.clear();
        parked.resize(ticks[segment.start].car_records.len(), false);
        for offset in 1..segment.len {
            let target_index = segment.start + offset;
            let target = &ticks[target_index];
            let prev = &ticks[target_index - 1];
            if reset_each_tick {
                backend.set_state(ctx, target_index - 1);
            }
            if target.car_records.is_empty() {
                continue;
            }
            let plan = reconcile_plan(prev, target, &parked);
            backend.reconcile_pre(ctx, target_index, &plan);
            for (j, action) in plan.car.iter().enumerate() {
                if let Some(p) = parked.get_mut(j) {
                    *p = *action == CarAction::Park;
                }
            }
            let controls: Vec<ControlsRecord> = target
                .car_records
                .iter()
                .map(|car| car.prev_controls)
                .collect();
            backend.step(&controls);
            backend.reconcile_post(ctx, target_index, &plan);
            let Some(truth) = snapshot_from_tick(target) else {
                continue;
            };
            if !reset_each_tick && offset < warmup_ticks {
                continue;
            }
            // The v10 capture serializes the input held at record time while
            // the physics step consumes the input polled at tick start; when
            // a neighbouring record carries different controls, which sample
            // the game applied is ambiguous and the step does not test
            // physics. Same gate as `fox_eval`.
            let ambiguous_input = ticks
                .get(target_index + 1)
                .is_some_and(|next| {
                    target
                        .car_records
                        .iter()
                        .zip(&next.car_records)
                        .any(|(a, b)| a.prev_controls != b.prev_controls)
                })
                || prev
                    .car_records
                    .iter()
                    .zip(&target.car_records)
                    .any(|(a, b)| a.prev_controls != b.prev_controls);
            let untrusted_flip = ctx
                .flip_untrusted
                .get(target_index - 1)
                .is_some_and(|flags| flags.iter().any(|&u| u));
            // A tick with no trustworthy physical transition is not scored:
            // the ball teleported or repeated a placeholder sample, every
            // car record is a stale placeholder, the applied input is
            // ambiguous, or a pre-capture flip is in progress.
            if plan.ball_teleport
                || plan.ball_stale
                || plan.all_cars_stale
                || ambiguous_input
                || untrusted_flip
            {
                continue;
            }
            let car_skip: Vec<bool> = plan
                .car
                .iter()
                .map(|a| *a != CarAction::Live)
                .collect();
            let norm = normalized_error(&backend.snapshot(), &truth, &car_skip, false);
            if trace_range.contains(&target_index) {
                let sim = backend.snapshot();
                let mut parts = String::new();
                for (ci, (s, t)) in sim.cars.iter().zip(&truth.cars).enumerate() {
                    let e = body_err_sq(s, t);
                    parts.push_str(&format!(
                        " c{ci}={e:.4}(p{:.3} v{:.3} a{:.3})",
                        (s.pos - t.pos).length() / POS_TOL_UU,
                        (s.vel - t.vel).length() / VEL_TOL_UU_S,
                        (s.ang_vel - t.ang_vel).length() / ANG_VEL_TOL_RAD_S,
                    ));
                }
                let be = body_err_sq(&sim.ball, &truth.ball);
                println!(
                    "TRACE {target_index}: norm={norm:.4} ball={be:.4}{parts} plan={:?}",
                    plan.car
                );
                for (ci, rec) in target.car_records.iter().enumerate() {
                    let sim_car = &sim.cars[ci];
                    let pre = &prev.car_records[ci];
                    println!(
                        "  FROM c{ci}: flip={} flip_t={:.4} torque={:?} jumping={} jt={} ground={} has_flip={} djf={} has_jumped={} ang={:?} ctrls(jump={} pitch={:.2} yaw={:.2} roll={:.2})",
                        pre.is_flipping,
                        pre.flip_time,
                        glam::Vec3A::from(pre.flip_rel_torque),
                        pre.is_jumping,
                        pre.jump_time,
                        pre.is_on_ground,
                        pre.has_flip,
                        pre.double_jumped_or_flipped,
                        pre.has_jumped,
                        glam::Vec3A::from(pre.phys.ang_vel),
                        pre.prev_controls.jump,
                        pre.prev_controls.pitch,
                        pre.prev_controls.yaw,
                        pre.prev_controls.roll,
                    );
                    let full = backend.car_state(ci);
                    println!(
                        "  POST c{ci}: flip={} flip_t={:.4} jt={} hasj/fl/dj={}/{}/{} jumping={} demoed={}",
                        full.is_flipping,
                        full.flip_time,
                        full.jump_ticks,
                        full.has_jumped,
                        full.has_flipped,
                        full.has_double_jumped,
                        full.is_jumping,
                        full.is_demoed,
                    );
                    println!(
                        "  rec c{ci}: flip={} flip_t={:.4} torque={:?} jumping={} jt={} ground={} has_flip={} djf={} ang_vel={:?} sim_ang={:?} ctrls(jump={},pitch={:.2},yaw={:.2},roll={:.2})",
                        rec.is_flipping,
                        rec.flip_time,
                        glam::Vec3A::from(rec.flip_rel_torque),
                        rec.is_jumping,
                        rec.jump_time,
                        rec.is_on_ground,
                        rec.has_flip,
                        rec.double_jumped_or_flipped,
                        glam::Vec3A::from(rec.phys.ang_vel),
                        sim_car.ang_vel,
                        rec.prev_controls.jump,
                        rec.prev_controls.pitch,
                        rec.prev_controls.yaw,
                        rec.prev_controls.roll,
                    );
                }
            }
            report.add(classify_tick(target, Some(prev)), target_index, norm);
        }
    }
    report
}

/// `fox_eval`'s per-transition norm: each live car contributes its own
/// 5-component error plus the ball's 5-component error, summed in
/// quadrature over non-skipped cars.
fn fox_tick_norm(sim: &Snapshot, truth: &Snapshot, car_skip: &[bool]) -> f32 {
    let ball_sse = body_err_sq(&sim.ball, &truth.ball);
    let mut sum = 0.0;
    for (i, (s, t)) in sim.cars.iter().zip(&truth.cars).enumerate() {
        if car_skip.get(i).copied().unwrap_or(false) {
            continue;
        }
        sum += body_err_sq(s, t) + ball_sse;
    }
    sum.sqrt()
}

/// `fox_eval`'s default one-step reset evaluator: every transition
/// restores the recorded `from` state (stale cars parked, stale ball out
/// of play), steps once, and scores only transitions passing its
/// ambiguity/stale/frozen/teleport/contiguity gates. The `i == 0`
/// transition is stepped but not scored, matching `fox_eval`.
fn evaluate_fox_reset<B: ReplayBackend>(backend: &mut B, ctx: &Ctx) -> EvalReport {
    let ticks = ctx.ticks;
    let n = ticks.len();
    let mut report = EvalReport::default();
    for i in 0..n.saturating_sub(1) {
        let from = &ticks[i];
        let to = &ticks[i + 1];
        if from.car_records.is_empty() || from.car_records.len() != to.car_records.len() {
            continue;
        }
        let num_cars = to.car_records.len();
        let frame_delta = to.car_records[0]
            .phys
            .physics_frame
            .saturating_sub(from.car_records[0].phys.physics_frame);
        let ambiguous_input = (i + 2 < n
            && to
                .car_records
                .iter()
                .zip(&ticks[i + 2].car_records)
                .any(|(a, b)| a.prev_controls != b.prev_controls))
            || from
                .car_records
                .iter()
                .zip(&to.car_records)
                .any(|(a, b)| a.prev_controls != b.prev_controls);
        let untrusted_flip = ctx
            .flip_untrusted
            .get(i)
            .is_some_and(|flags| flags.iter().any(|&u| u));
        let mut car_stale = vec![false; num_cars];
        let mut car_teleport = vec![false; num_cars];
        for (j, (ca, cb)) in from.car_records.iter().zip(&to.car_records).enumerate() {
            let frame_advanced = cb.phys.physics_frame != ca.phys.physics_frame;
            let phys_identical = ca.phys.pos == cb.phys.pos
                && ca.phys.rot == cb.phys.rot
                && ca.phys.lin_vel == cb.phys.lin_vel
                && ca.phys.ang_vel == cb.phys.ang_vel;
            let parked = ca.is_on_ground && Vec3A::from(ca.phys.lin_vel).length() < 1.0;
            car_stale[j] = frame_advanced && phys_identical && !parked;
            let moved = (Vec3A::from(cb.phys.pos) - Vec3A::from(ca.phys.pos)).length();
            car_teleport[j] =
                moved > Vec3A::from(ca.phys.lin_vel).length() * consts::TICK_TIME + 100.0;
        }
        let ball_teleport = (Vec3A::from(to.ball_record.pos)
            - Vec3A::from(from.ball_record.pos))
        .length()
            > 100.0;
        let ball_stale = to.ball_record.physics_frame != from.ball_record.physics_frame
            && from.ball_record.pos == to.ball_record.pos
            && from.ball_record.rot == to.ball_record.rot
            && from.ball_record.lin_vel == to.ball_record.lin_vel
            && from.ball_record.ang_vel == to.ball_record.ang_vel
            && !(Vec3A::from(from.ball_record.pos).z < 120.0
                && Vec3A::from(from.ball_record.lin_vel).length() < 1.0);
        let ball_park = ball_stale
            || (Vec3A::from(from.ball_record.pos).z >= 120.0
                && Vec3A::from(from.ball_record.lin_vel).length() < 1.0
                && Vec3A::from(from.ball_record.ang_vel).length() < 1.0);
        let scorable = !ambiguous_input
            && !untrusted_flip
            && !tick_is_frozen(from, to)
            && !ball_teleport
            && !ball_stale
            && !car_stale.iter().all(|&s| s)
            && frame_delta == 1;
        let controls: Vec<ControlsRecord> =
            to.car_records.iter().map(|r| r.prev_controls).collect();
        backend.fox_set_state(ctx, i, &car_stale, ball_park);
        backend.step(&controls);
        if i == 0 {
            continue;
        }
        let Some(truth) = snapshot_from_tick(to) else {
            continue;
        };
        let car_skip: Vec<bool> = (0..num_cars)
            .map(|j| car_stale[j] || car_teleport[j])
            .collect();
        let tick_norm = fox_tick_norm(&backend.snapshot(), &truth, &car_skip);
        if scorable {
            report.add(classify_tick(to, Some(from)), i + 1, tick_norm);
        }
    }
    report
}

/// `fox_eval`'s `FOX_LIVE` continuous replay: one pass over the whole
/// recording retaining Bullet's private state. Cars are re-restored from
/// each source record; the ball keeps its simulated state across a tick
/// that produced a car-ball hit (preserving the unserialized cooldown),
/// and is re-restored otherwise. Frame gaps and frozen samples teleport
/// the state forward without scoring.
fn evaluate_fox_live<B: ReplayBackend>(backend: &mut B, ctx: &Ctx) -> EvalReport {
    let ticks = ctx.ticks;
    let n = ticks.len();
    let mut report = EvalReport::default();
    if n < 2 || ticks[0].car_records.is_empty() {
        return report;
    }
    let num_cars = ticks[0].car_records.len();
    backend.fox_set_state(ctx, 0, &vec![false; num_cars], false);
    backend.step(
        &ticks[1]
            .car_records
            .iter()
            .map(|r| r.prev_controls)
            .collect::<Vec<_>>(),
    );
    let mut preserve_ball_state = false;
    for i in 1..n.saturating_sub(1) {
        let from = &ticks[i];
        let to = &ticks[i + 1];
        if from.car_records.len() != num_cars || to.car_records.len() != num_cars {
            continue;
        }
        let frame_delta = to.car_records[0]
            .phys
            .physics_frame
            .saturating_sub(from.car_records[0].phys.physics_frame);
        if frame_delta != 1 || tick_is_frozen(from, to) {
            // Teleport to the next recorded state and resume on the
            // following contiguous transition.
            backend.fox_set_state(ctx, i + 1, &vec![false; num_cars], false);
            preserve_ball_state = false;
            continue;
        }
        // A car whose phys record repeats verbatim while its frame advances
        // was not simulated that tick (demolished or paused placeholder):
        // park it as demoed so it neither drifts nor interacts.
        let mut car_stale = vec![false; num_cars];
        for (j, (ca, cb)) in from.car_records.iter().zip(&to.car_records).enumerate() {
            let frame_advanced = cb.phys.physics_frame != ca.phys.physics_frame;
            let phys_identical = ca.phys.pos == cb.phys.pos
                && ca.phys.rot == cb.phys.rot
                && ca.phys.lin_vel == cb.phys.lin_vel
                && ca.phys.ang_vel == cb.phys.ang_vel;
            let parked = ca.is_on_ground && Vec3A::from(ca.phys.lin_vel).length() < 1.0;
            car_stale[j] = frame_advanced && phys_identical && !parked;
        }
        if !preserve_ball_state {
            backend.fox_set_ball(ctx, i);
        }
        backend.fox_set_cars(ctx, i, &car_stale);
        backend.step(
            &to.car_records
                .iter()
                .map(|r| r.prev_controls)
                .collect::<Vec<_>>(),
        );
        preserve_ball_state = backend.ball_was_hit();
        let Some(truth) = snapshot_from_tick(to) else {
            continue;
        };
        let tick_norm = fox_tick_norm(&backend.snapshot(), &truth, &[]);
        report.add(classify_tick(to, Some(from)), i + 1, tick_norm);
    }
    report
}

/// Sim holder for N blue Octane cars in Soccar.
struct V3Backend {
    config: ArenaConfig,
    arena: Arena,
    car_ids: Vec<usize>,
}

impl V3Backend {
    fn new(config: ArenaConfig) -> Self {
        Self {
            arena: Arena::new_with_config(config.clone()),
            config,
            car_ids: Vec::new(),
        }
    }

    fn ensure_cars(&mut self, count: usize) {
        if self.car_ids.len() == count {
            return;
        }
        self.arena = Arena::new_with_config(self.config.clone());
        self.car_ids = (0..count)
            .map(|_| self.arena.add_car(Team::Blue, CarBodyConfig::OCTANE))
            .collect();
    }

    /// Restore one car from its record, mirroring `fox_eval`'s v10
    /// reconstruction (jump timing, flip availability, un-scaled dodge
    /// torque, world-contact cooldown, reconstructed air time).
    ///
    /// `tick_idx` is the record to restore; the controls applied by the
    /// *next* step are taken from the following record. `stale` parks the
    /// restored car as demoed (`fox_eval`'s placeholder handling).
    fn set_car(&mut self, ctx: &Ctx, car_idx: usize, tick_idx: usize, stale: bool) {
        let record = &ctx.ticks[tick_idx].car_records[car_idx];
        let car_id = self.car_ids[car_idx];
        let controls: CarControls = ctx
            .ticks
            .get(tick_idx + 1)
            .and_then(|t| t.car_records.get(car_idx))
            .map(|c| c.prev_controls.into())
            .unwrap_or_else(|| record.prev_controls.into());
        let recorded: CarState = (*record).into();
        let mut state = *self.arena.get_car_state(car_id);
        state.phys = recorded.phys;
        state.is_on_ground = recorded.is_on_ground;
        state.wheels_with_contact = recorded.wheels_with_contact;
        state.is_jumping = recorded.is_jumping;
        // The v10 recorder exposes the current jump input phase; an active
        // state with a released upcoming input is already post-impulse.
        if recorded.is_jumping && !controls.jump {
            state.is_jumping = false;
        }
        // A reported flip takes precedence over the jump-hold bit in the
        // same delayed input window.
        if record.is_flipping {
            state.is_jumping = false;
        }
        state.is_flipping = recorded.is_flipping;
        // The v10 recorder serializes the current jump hold phase as a
        // zero-time sample on the first active tick.
        state.jump_ticks = if recorded.jump_ticks == 0
            && record.is_jumping
            && record.prev_controls.jump
        {
            1
        } else {
            recorded.jump_ticks
        };
        // A single in-air `is_jumping` sample with a zero timer that follows
        // a non-jumping record is a phantom press: retire it so it does not
        // receive a hold impulse.
        if recorded.is_jumping
            && !recorded.is_on_ground
            && record.jump_time == 0.0
            && controls.jump
            && ctx
                .ticks
                .get(tick_idx.wrapping_sub(1))
                .and_then(|p| p.car_records.get(car_idx))
                .is_some_and(|p| !p.is_jumping)
        {
            state.jump_ticks = consts::car::jump::MAX_TICKS;
        }
        state.flip_time = recorded.flip_time;
        state.has_jumped = recorded.has_jumped;
        state.air_time_since_jump = ctx.air_time_since_jump[tick_idx][car_idx];
        state.prev_controls = record.prev_controls.into();
        state.controls = controls;
        // The v10 writer stores the already-scaled dodge torque; the sim
        // consumes the unit flip direction.
        let mut flip_rel_torque = recorded.flip_rel_torque;
        if flip_rel_torque.x.abs() > 1.5 || flip_rel_torque.y.abs() > 1.5 {
            flip_rel_torque.x /= consts::car::flip::TORQUE.x;
            flip_rel_torque.y /= consts::car::flip::TORQUE.y;
            flip_rel_torque.z = 0.0;
        }
        state.flip_rel_torque = flip_rel_torque;
        state.boost = recorded.boost;
        // RLPR v2 does not serialize the extra-hit cooldown; preserve the
        // recorded contact normal so the live cooldown state survives.
        state.world_contact_normal = record
            .phys
            .has_world_contact
            .then(|| record.phys.world_contact_normal.into());
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
            // Keep the hidden availability clear until the delayed jump edge
            // is present; once the flip time is non-zero (or the jump edge
            // is recorded) preserve the consumed state for ongoing torque.
            state.has_flipped = record.prev_controls.jump || record.flip_time > 0.0;
        } else if record.double_jumped_or_flipped && !state.has_flipped {
            // v10 exposes this bit as the available double-jump/flip phase.
            state.has_double_jumped = false;
        }

        self.arena.set_car_state(car_id, state);
    }
}

impl ReplayBackend for V3Backend {
    fn reset(&mut self, ctx: &Ctx, start: usize) {
        self.arena = Arena::new_with_config(self.config.clone());
        self.car_ids = (0..ctx.ticks[start].car_records.len())
            .map(|i| {
                self.arena.add_car(
                    if i % 2 == 0 { Team::Blue } else { Team::Orange },
                    CarBodyConfig::OCTANE,
                )
            })
            .collect();
        self.set_state(ctx, start);
    }

    fn set_state(&mut self, ctx: &Ctx, tick_idx: usize) {
        self.ensure_cars(ctx.ticks[tick_idx].car_records.len());
        for car_idx in 0..ctx.ticks[tick_idx].car_records.len() {
            self.set_car(ctx, car_idx, tick_idx, false);
        }

        let recorded_ball: PhysState = ctx.ticks[tick_idx].ball_record.into();
        let mut ball = *self.arena.get_ball_state();
        ball.phys = recorded_ball;
        self.arena.set_ball_state(ball);
    }

    fn reconcile_pre(&mut self, ctx: &Ctx, target: usize, plan: &Reconcile) {
        self.ensure_cars(ctx.ticks[target].car_records.len());
        for (j, action) in plan.car.iter().enumerate() {
            match action {
                CarAction::Live => {}
                CarAction::Park | CarAction::Restore => {
                    // Park the sim car as demoed for this transition so it
                    // cannot interact; `Restore` re-syncs it post-step.
                    let car_id = self.car_ids[j];
                    let mut state = *self.arena.get_car_state(car_id);
                    state.is_demoed = true;
                    state.demo_respawn_timer = 1e9;
                    self.arena.set_car_state(car_id, state);
                }
            }
        }

        if plan.ball_teleport || plan.ball_park {
            // Move the ball out of play so it cannot phantom-touch a car
            // during the step; a teleported ball is re-synced post-step.
            let mut ball = *self.arena.get_ball_state();
            ball.phys.pos = Vec3A::new(0.0, 0.0, 30000.0);
            ball.phys.vel = Vec3A::ZERO;
            ball.phys.ang_vel = Vec3A::ZERO;
            self.arena.set_ball_state(ball);
        }
    }

    fn reconcile_post(&mut self, ctx: &Ctx, target: usize, plan: &Reconcile) {
        for (j, action) in plan.car.iter().enumerate() {
            if *action == CarAction::Restore {
                self.set_car(ctx, j, target, false);
            }
        }
        if plan.ball_teleport {
            let mut ball = *self.arena.get_ball_state();
            ball.phys = ctx.ticks[target].ball_record.into();
            self.arena.set_ball_state(ball);
        }
    }

    fn step(&mut self, controls: &[ControlsRecord]) {
        for (control, &car_id) in controls.iter().zip(&self.car_ids) {
            self.arena
                .set_car_controls(car_id, CarControls::from(*control));
        }
        self.arena.step_tick();
    }

    fn snapshot(&mut self) -> Snapshot {
        let cars = self
            .car_ids
            .iter()
            .map(|&car_id| {
                let car = *self.arena.get_car_state(car_id);
                BodySnapshot {
                    pos: car.phys.pos,
                    vel: car.phys.vel,
                    ang_vel: car.phys.ang_vel,
                    forward: car.phys.get_forward_dir(),
                    up: car.phys.get_up_dir(),
                }
            })
            .collect();
        let ball = *self.arena.get_ball_state();
        Snapshot {
            cars,
            ball: BodySnapshot {
                pos: ball.phys.pos,
                vel: ball.phys.vel,
                ang_vel: ball.phys.ang_vel,
                forward: ball.phys.get_forward_dir(),
                up: ball.phys.get_up_dir(),
            },
        }
    }

    fn car_state(&mut self, car_idx: usize) -> CarState {
        *self.arena.get_car_state(self.car_ids[car_idx])
    }

    fn fox_set_state(&mut self, ctx: &Ctx, state_idx: usize, car_stale: &[bool], ball_park: bool) {
        self.ensure_cars(ctx.ticks[state_idx].car_records.len());
        for j in 0..ctx.ticks[state_idx].car_records.len() {
            self.set_car(ctx, j, state_idx, car_stale.get(j).copied().unwrap_or(false));
        }
        self.fox_set_ball(ctx, state_idx);
        if ball_park {
            // The record is a parked placeholder; keep the sim ball out of
            // play so it cannot phantom-touch a car and consume the
            // extra-hit cooldown.
            let mut ball = *self.arena.get_ball_state();
            ball.phys.pos = Vec3A::new(0.0, 0.0, 30000.0);
            ball.phys.vel = Vec3A::ZERO;
            ball.phys.ang_vel = Vec3A::ZERO;
            self.arena.set_ball_state(ball);
        }
    }

    fn fox_set_cars(&mut self, ctx: &Ctx, state_idx: usize, car_stale: &[bool]) {
        self.ensure_cars(ctx.ticks[state_idx].car_records.len());
        for j in 0..ctx.ticks[state_idx].car_records.len() {
            self.set_car(ctx, j, state_idx, car_stale.get(j).copied().unwrap_or(false));
        }
    }

    fn fox_set_ball(&mut self, ctx: &Ctx, state_idx: usize) {
        let mut ball = *self.arena.get_ball_state();
        ball.phys = ctx.ticks[state_idx].ball_record.into();
        self.arena.set_ball_state(ball);
    }

    fn ball_was_hit(&self) -> bool {
        self.arena
            .get_last_step_events()
            .iter()
            .any(|e| matches!(e, ArenaEvent::CarHitBall(_)))
    }
}

/// Run the selected evaluator against one backend.
fn run_backend<B: ReplayBackend>(
    backend: &mut B,
    args: &Args,
    ctx: &Ctx,
    segments: &[Segment],
    config: SegmentConfig,
) -> EvalReport {
    if args.fox_eval {
        if args.reset_each_tick {
            evaluate_fox_reset(backend, ctx)
        } else {
            evaluate_fox_live(backend, ctx)
        }
    } else {
        evaluate(
            backend,
            ctx,
            segments,
            config.warmup_ticks,
            args.reset_each_tick,
        )
    }
}

fn metric_value(support: usize, value: f64) -> String {
    if support == 0 {
        "-".to_string()
    } else {
        format!("{value:.6}")
    }
}

fn print_header() {
    println!(
        "{:<7} {:<15} {:>9} {:>9} {:>9} {:>12} {:>12} {:>12}",
        "Backend", "Category", "Support", "Passed", "Pass %", "Mean norm", "Max norm", "First fail"
    );
    println!("{}", "-".repeat(102));
}

fn print_report(backend: &str, report: &EvalReport) {
    for category in ContactCategory::ALL {
        let stats = report.stats(category);
        let pass_pct = metric_value(stats.support, stats.rate());
        let mean_norm = metric_value(stats.support, stats.mean_norm());
        let max_norm = metric_value(stats.support, f64::from(stats.max_norm));
        println!(
            "{backend:<7} {:<15} {:>9} {:>9} {pass_pct:>9} {mean_norm:>12} {max_norm:>12} {:>12}",
            category.as_str(),
            stats.support,
            stats.passed,
            report.first_fail_label(category),
        );
    }
}

/// Expand inputs into a sorted, de-duplicated list of `.rlpr` files.
fn collect_recordings(inputs: &[PathBuf]) -> Result<Vec<PathBuf>, Box<dyn std::error::Error>> {
    let mut files = Vec::new();
    for input in inputs {
        if input.is_dir() {
            for entry in std::fs::read_dir(input)? {
                let path = entry?.path();
                if path.extension().is_some_and(|ext| ext == "rlpr") {
                    files.push(path);
                }
            }
        } else {
            files.push(input.clone());
        }
    }
    files.sort();
    files.dedup();
    Ok(files)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    if args.segment_ticks <= 1 {
        return Err("--segment-ticks must be greater than 1".into());
    }
    if args.warmup_ticks >= args.segment_ticks {
        return Err("--warmup-ticks must be less than --segment-ticks".into());
    }

    let files = collect_recordings(&args.inputs)?;
    if files.is_empty() {
        return Err("no .rlpr recordings found in the given inputs".into());
    }

    rocketsim::init(&mesh_dir(), true).expect("init RocketSim collision meshes");

    let config = SegmentConfig {
        segment_ticks: args.segment_ticks,
        warmup_ticks: args.warmup_ticks,
    };

    let mut total_report = EvalReport::default();
    let mut first_file = true;

    for file in &files {
        let recording = Recording::from_file(Path::new(file))?;
        let segments = if args.fox_eval {
            Vec::new()
        } else {
            let mut segments = split_segments(&recording.ticks, config);
            if let Ok(spec) = std::env::var("BENCH_FORCE_SEG") {
                if let Some((a, b)) = spec.split_once(':') {
                    if let (Ok(start), Ok(len)) = (a.parse::<usize>(), b.parse::<usize>()) {
                        segments = vec![Segment { start, len }];
                    }
                }
            }
            segments
        };
        if segments.is_empty() && !args.fox_eval {
            eprintln!("{}: no segments, skipped", file.display());
            continue;
        }

        println!(
            "Recording: {} (RLPR v{}, {} cars, {} ticks)",
            recording.name,
            recording.version,
            recording.info.num_cars,
            recording.ticks.len()
        );
        println!("File: {}", file.display());
        if args.fox_eval {
            println!(
                "Mode: fox_eval {}",
                if args.reset_each_tick {
                    "one-step reset"
                } else {
                    "live replay"
                }
            );
        } else if !args.reset_each_tick {
            println!(
                "Segments: {} x {} ticks ({} warmup ticks)",
                segments.len(),
                config.segment_ticks,
                config.warmup_ticks,
            );
        }
        if first_file {
            println!("Categories overlap. Support is the number of scored RL ticks.");
            first_file = false;
        }
        println!();
        print_header();

        let num_cars = recording.info.num_cars as usize;
        let ctx = Ctx {
            ticks: &recording.ticks,
            air_time_since_jump: reconstruct_air_time_since_jump(&recording.ticks, num_cars),
            flip_untrusted: compute_flip_untrusted(&recording.ticks, num_cars),
        };
        // v10 parity semantics are a process-global engine mode in this
        // backend: enable it only while evaluating corrected-recorder
        // captures. Backends without the call sites ignore the flag.
        let v10 = detect_v10_parity(&recording)
            && std::env::var_os("RLPR_BENCH_NO_V10").is_none();
        unsafe {
            if v10 {
                std::env::set_var("RS_V10_PARITY", "1");
            } else {
                std::env::remove_var("RS_V10_PARITY");
            }
        }
        println!("v10 parity: {v10}");

        let report = match args.engine.as_str() {
            #[cfg(feature = "v2")]
            "v2" => {
                let mut backend = v2::V2Backend::new();
                run_backend(&mut backend, &args, &ctx, &segments, config)
            }
            "v3" => {
                let mut backend = V3Backend::new(arena_config(&recording));
                run_backend(&mut backend, &args, &ctx, &segments, config)
            }
            other => {
                return Err(format!(
                    "unknown --engine {other:?} (expected v3{})",
                    if cfg!(feature = "v2") { ", v2" } else { "" }
                )
                .into());
            }
        };
        print_report(&args.backend, &report);
        println!();
        total_report.merge(&report, &recording.name);
    }

    if files.len() > 1 {
        println!("Total over {} recordings:", files.len());
        println!();
        print_header();
        print_report(&args.backend, &total_report);
    }

    Ok(())
}
