//! Backend-neutral multi-tick metric for RLPR recordings.
//!
//! Thin `v3`/`v2` adapters implement [`ReplayBackend`].
//! The CLI builds [`Segment`]s, runs [`evaluate`] once per car,
//! prints one [`EvalReport`] per car. Every recorded car shares the sim,
//! so car-car contacts are genuine engine observations.
//!
//! Each segment holds `segment_ticks` ticks.
//! The first `warmup_ticks` ticks only advance the sim.
//! The rest are scored. Reset happens only at segment starts.

use glam::Vec3A;
use rocketsim_test::rlpr::{cpp_records::ControlsRecord, tick_record::TickRecord};

// Max plausible per-tick travel in Unreal units. Fastest ball (~6000 UU/s)
// covers ~50 UU per 120 Hz tick; supersonic cars ~20 UU. Anything beyond
// this is a teleport: kickoff/goal reset, demo respawn, or respawn snap.
// Teleports always break runs (see split_segments) so no scored tick ever
// spans one, and every post-teleport run starts with a fresh state-set.
pub const CAR_TELEPORT_DIST: f32 = 500.0;
pub const BALL_TELEPORT_DIST: f32 = 500.0;

// Strict per-component tolerances. See [`normalized_error`].
pub const POS_TOL_UU: f32 = 10.0;
pub const VEL_TOL_UU_S: f32 = 3.0;
pub const ANG_VEL_TOL_RAD_S: f32 = 1.0;
pub const AXIS_TOL: f32 = 1.0;

/// Plain body state. Axes are unit vectors. Uses [`Vec3A`].
#[derive(Clone, Copy, Debug)]
pub struct BodySnapshot {
    pub pos: Vec3A,
    pub vel: Vec3A,
    pub ang_vel: Vec3A,
    pub forward: Vec3A,
    pub up: Vec3A,
}

/// Plain car plus ball state.
#[derive(Clone, Copy, Debug)]
pub struct Snapshot {
    pub car: BodySnapshot,
    pub ball: BodySnapshot,
}

/// Backend adapter. Holds the sim with one arena car per recorded car.
/// `step` reports the contacts the sim itself observed that tick, one
/// entry per arena car in recording order.
pub trait ReplayBackend {
    fn reset(&mut self, start: &TickRecord);
    fn set_state(&mut self, state: &TickRecord);

    /// Restore state that is not present in an RLPR car record.
    ///
    /// The v3 backend has a handbrake integrator. Other backends can keep
    /// their default when they do not expose this state.
    fn set_handbrake_value(&mut self, _car_idx: usize, _value: f32) {}

    /// Restore the boost armed bit plus time-since-arm from the recording.
    ///
    /// Only the v3 backend implements this; others keep live-bit evolution.
    /// Callers pass recorded state only when the format carries it; older
    /// versions must keep live-latch evolution instead of forcing false/zero.
    fn set_boost_state(&mut self, _car_idx: usize, _armed: bool, _time: f32) {}

    /// Refresh hidden prior-tick wheel state without advancing dynamics.
    fn refresh_sticky_gates(&mut self) {}

    fn step(&mut self, controls: &[ControlsRecord]) -> Vec<SimContactEvents>;
    fn snapshot(&mut self, car_idx: usize) -> Snapshot;
}

/// Contacts the sim observed during one stepped tick, for the scored car.
/// Ball labels are shared; car labels describe the scored car only.
#[derive(Clone, Copy, Debug, Default)]
pub struct SimContactEvents {
    pub car_ball: bool,
    pub car_car: bool,
    pub ball_world: bool,
    pub chassis_world: bool,
}

/// Read one body from parts.
fn body_from_parts(
    pos: Vec3A,
    vel: Vec3A,
    ang_vel: Vec3A,
    forward: Vec3A,
    up: Vec3A,
) -> BodySnapshot {
    BodySnapshot {
        pos,
        vel,
        ang_vel,
        forward,
        up,
    }
}

/// Read ground truth for one car from a tick. `None` without that car.
pub fn snapshot_from_tick(tick: &TickRecord, car_idx: usize) -> Option<Snapshot> {
    let car = tick.car_records.get(car_idx)?;
    let car_forward = Vec3A::from(car.phys.rot.column(0));
    let car_up = Vec3A::from(car.phys.rot.column(2));
    let ball_forward = Vec3A::from(tick.ball_record.rot.column(0));
    let ball_up = Vec3A::from(tick.ball_record.rot.column(2));
    Some(Snapshot {
        car: body_from_parts(
            car.phys.pos.into(),
            car.phys.lin_vel.into(),
            car.phys.ang_vel.into(),
            car_forward,
            car_up,
        ),
        ball: body_from_parts(
            tick.ball_record.pos.into(),
            tick.ball_record.lin_vel.into(),
            tick.ball_record.ang_vel.into(),
            ball_forward,
            ball_up,
        ),
    })
}

/// Segment length config.
#[derive(Clone, Copy, Debug)]
pub struct SegmentConfig {
    pub segment_ticks: usize,
    pub warmup_ticks: usize,
}

impl SegmentConfig {
    /// At least one scored tick per segment.
    pub fn is_valid(&self) -> bool {
        self.segment_ticks > 0 && self.warmup_ticks < self.segment_ticks
    }

    /// Scored ticks in a segment of `len` ticks.
    pub fn scored_len(&self, len: usize) -> usize {
        len.saturating_sub(self.warmup_ticks)
    }
}

/// Non-overlapping run of recording ticks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Segment {
    pub start: usize,
    pub len: usize,
}

impl Segment {
    /// End index (exclusive).
    pub fn end(&self) -> usize {
        self.start + self.len
    }
}

/// Number of cars in the tick.
pub fn tick_car_count(tick: &TickRecord) -> usize {
    tick.car_records.len()
}

/// Max cars scored per tick.
pub const MAX_SCORED_CARS: usize = 8;

/// Physics frames advance by one for every car and the ball.
pub fn frame_is_contiguous(from: &TickRecord, to: &TickRecord) -> bool {
    let n = from.car_records.len();
    if n == 0 || n > MAX_SCORED_CARS || to.car_records.len() != n {
        return false;
    }
    from.car_records
        .iter()
        .zip(to.car_records.iter())
        .all(|(a, b)| b.phys.physics_frame == a.phys.physics_frame + 1)
        && to.ball_record.physics_frame == from.ball_record.physics_frame + 1
}

/// No body moved between ticks (pause or replay stall).
pub fn tick_is_frozen(from: &TickRecord, to: &TickRecord) -> bool {
    let n = from.car_records.len();
    if n == 0 || to.car_records.len() != n {
        return false;
    }
    let cars_static = from
        .car_records
        .iter()
        .zip(to.car_records.iter())
        .all(|(a, b)| {
            a.phys.pos == b.phys.pos
                && a.phys.lin_vel == b.phys.lin_vel
                && a.phys.ang_vel == b.phys.ang_vel
        });
    cars_static
        && from.ball_record.pos == to.ball_record.pos
        && from.ball_record.lin_vel == to.ball_record.lin_vel
        && from.ball_record.ang_vel == to.ball_record.ang_vel
}

/// Kickoff countdown stasis, rule `kickoff-stasis-v1`. Metric only.
///
/// True when the game step is paused at kickoff while the recorder still
/// emits live controls and wheel contact flags. The ball sits unchanged
/// at the kickoff spot with a world-contact flag. Every car creeps with
/// epsilon motion and near-zero horizontal velocity. Exact `tick_is_frozen`
/// misses these rows because cars drift up to ~0.67 UU per tick. The
/// horizontal-velocity and ball-contact gates separate countdown spans
/// from genuine drive-off (ball contact false, horizontal speed 20+ UU/s)
/// and from slow play elsewhere (ball off center or cars driving).
/// Epsilon-only: exact-frozen pairs return false and stay on that path.
pub const KICKOFF_STASIS_RULE: &str = "kickoff-stasis-v1";
pub const KICKOFF_STASIS_MAX_CAR_MOVE: f32 = 1.0;
pub const KICKOFF_STASIS_MAX_CAR_SPEED: f32 = 100.0;
pub const KICKOFF_STASIS_MAX_CAR_HXY: f32 = 5.0;
pub const KICKOFF_STASIS_BALL_CENTER_TOL: f32 = 1.0;

/// True when one transition is kickoff countdown stasis. See above.
pub fn tick_is_kickoff_stasis(from: &TickRecord, to: &TickRecord) -> bool {
    if tick_is_frozen(from, to) {
        return false;
    }
    let n = from.car_records.len();
    if n == 0 || to.car_records.len() != n {
        return false;
    }
    if !frame_is_contiguous(from, to) || any_teleport(from, to) {
        return false;
    }
    let from_ball: Vec3A = from.ball_record.pos.into();
    let to_ball: Vec3A = to.ball_record.pos.into();
    if (to_ball - from_ball).length() != 0.0 {
        return false;
    }
    if to_ball.x.abs() > KICKOFF_STASIS_BALL_CENTER_TOL
        || to_ball.y.abs() > KICKOFF_STASIS_BALL_CENTER_TOL
    {
        return false;
    }
    if !to.ball_record.has_world_contact {
        return false;
    }
    from.car_records
        .iter()
        .zip(to.car_records.iter())
        .all(|(a, b)| {
            let pa: Vec3A = a.phys.pos.into();
            let pb: Vec3A = b.phys.pos.into();
            if (pb - pa).length() >= KICKOFF_STASIS_MAX_CAR_MOVE {
                return false;
            }
            let v: Vec3A = b.phys.lin_vel.into();
            if v.length() >= KICKOFF_STASIS_MAX_CAR_SPEED {
                return false;
            }
            if glam::Vec2::new(v.x, v.y).length() >= KICKOFF_STASIS_MAX_CAR_HXY {
                return false;
            }
            b.wheels.iter().any(|wheel| wheel.has_contact)
        })
}

/// Target tick indices with a kickoff-stasis incoming transition.
/// One entry per detected transition.
/// The aggregate-only CLI does not report per-file spans; unit tests cover this.
#[allow(dead_code)]
pub fn kickoff_stasis_targets(ticks: &[TickRecord]) -> Vec<usize> {
    let mut out = Vec::new();
    for i in 1..ticks.len() {
        if tick_is_kickoff_stasis(&ticks[i - 1], &ticks[i]) {
            out.push(i);
        }
    }
    out
}

/// Split ticks into non-overlapping segments.
///
/// Break runs at frame gaps, frozen transitions, ticks without cars,
/// ticks with too many cars, car-count changes, and teleports (any car
/// or the ball jumping further than one tick of travel allows: kickoff
/// and goal resets, demo respawns). A teleport arrival always starts a
/// new run, so the backend state-sets it before scoring resumes and no
/// scored tick ever spans the discontinuity.
/// Chunk each run into groups of `segment_ticks`.
/// Drop groups with no scored ticks. Reset only at segment starts.
pub fn split_segments(ticks: &[TickRecord], config: SegmentConfig) -> Vec<Segment> {
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
        let count = tick_car_count(tick);
        if count == 0 || count > MAX_SCORED_CARS {
            flush_run(index, &mut run_start);
            continue;
        }
        match run_start {
            None => {
                run_start = Some(index);
            }
            Some(_) => {
                let prev = &ticks[index - 1];
                if tick_car_count(prev) != count
                    || !frame_is_contiguous(prev, tick)
                    || tick_is_frozen(prev, tick)
                    || any_teleport(prev, tick)
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

/// Any car or the ball teleported between ticks (reset/respawn snap).
/// Counts are equal here; split_segments guarantees it before calling.
pub fn any_teleport(from: &TickRecord, to: &TickRecord) -> bool {
    if ball_teleported(from, to) {
        return true;
    }
    from.car_records
        .iter()
        .zip(to.car_records.iter())
        .any(|(a, b)| {
            let pa: Vec3A = a.phys.pos.into();
            let pb: Vec3A = b.phys.pos.into();
            (pa - pb).length() >= CAR_TELEPORT_DIST
        })
}

/// The ball jumped further than one tick of travel allows.
fn ball_teleported(from: &TickRecord, to: &TickRecord) -> bool {
    let pa: Vec3A = from.ball_record.pos.into();
    let pb: Vec3A = to.ball_record.pos.into();
    (pa - pb).length() >= BALL_TELEPORT_DIST
}

/// Find the start of the clean run that contains `segment_start`.
///
/// A segment can start in the middle of a run. Do not use the segment start
/// as the handbrake history start in that case. Stop at the same boundaries
/// used by [`split_segments`].
pub fn run_start(ticks: &[TickRecord], segment_start: usize) -> usize {
    let mut start = segment_start.min(ticks.len());
    if start == ticks.len() {
        return start;
    }
    while start > 0 {
        let from = &ticks[start - 1];
        let to = &ticks[start];
        if tick_car_count(from) != tick_car_count(to)
            || !frame_is_contiguous(from, to)
            || tick_is_frozen(from, to)
            || any_teleport(from, to)
        {
            break;
        }
        start -= 1;
    }
    start
}

/// Reconstruct the handbrake integrator at one recorded state.
///
/// Versions v2-v7 do not store `handbrake_val`. Version v8 records it.
/// Use this fallback only when the format lacks direct state.
/// `prev_controls` at tick `i` is the control used to produce tick `i`,
/// so include tick `state_index`.
/// Integrate from the true contiguous `run_start`. Alternating histories
/// can retain state older than 60 ticks, so do not truncate the window.
/// At a run boundary, assume the value before the run was zero.
pub fn reconstruct_handbrake(
    ticks: &[TickRecord],
    run_start: usize,
    state_index: usize,
    car_idx: usize,
) -> Option<f32> {
    if state_index >= ticks.len() || run_start > state_index {
        return None;
    }

    let first = run_start;
    let mut value = 0.0;
    for tick in &ticks[first..=state_index] {
        let car = tick.car_records.get(car_idx)?;
        let delta = if car.prev_controls.handbrake {
            rocketsim::consts::car::drive::POWERSLIDE_RISE_RATE
        } else {
            -rocketsim::consts::car::drive::POWERSLIDE_FALL_RATE
        } * rocketsim::consts::TICK_TIME;
        value = (value + delta).clamp(0.0, 1.0);
    }
    Some(value)
}

/// Chunk one clean run. Drop chunks with no scored ticks.
fn push_chunks(segments: &mut Vec<Segment>, start: usize, end: usize, config: SegmentConfig) {
    let mut offset = start;
    while offset < end {
        let len = (config.segment_ticks).min(end - offset);
        if config.scored_len(len) > 0 {
            segments.push(Segment { start: offset, len });
        }
        offset += len;
    }
}

/// Overlapping contact categories plus the `Total` aggregate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ContactCategory {
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
    pub const ALL: [ContactCategory; 7] = [
        ContactCategory::CarBall,
        ContactCategory::CarCar,
        ContactCategory::BallWorld,
        ContactCategory::ChassisWorld,
        ContactCategory::WheelWorld,
        ContactCategory::NoContact,
        ContactCategory::Total,
    ];

    /// Short CLI column name.
    pub fn as_str(&self) -> &'static str {
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
pub struct ContactLabels {
    pub car_ball: bool,
    pub car_car: bool,
    pub ball_world: bool,
    pub chassis_world: bool,
    pub wheel_world: bool,
}

impl ContactLabels {
    /// No label is set.
    pub fn is_quiet(&self) -> bool {
        !(self.car_ball
            || self.car_car
            || self.ball_world
            || self.chassis_world
            || self.wheel_world)
    }

    /// Membership. `NoContact` holds only when all labels are false.
    pub fn contains(&self, category: ContactCategory) -> bool {
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

/// Label one target tick for one car.
///
/// Each label is true when the RL recording flags it OR the sim observed
/// it while replaying the tick. Sim-observed contacts are exact engine
/// events, never trajectory guesses: the sim cannot miss a contact the
/// way flag-based inference can, and a contact only the sim sees is a
/// real divergence worth scoring.
pub fn classify_tick(tick: &TickRecord, car_idx: usize, sim: SimContactEvents) -> ContactLabels {
    let Some(car) = tick.car_records.get(car_idx) else {
        return ContactLabels::default();
    };
    let car_ball = car.is_touching_ball || sim.car_ball;
    let car_car = car.is_touching_car || sim.car_car;
    let wheel_world = car.wheels.iter().any(|wheel| wheel.has_contact);
    let ball_world = tick.ball_record.has_world_contact || sim.ball_world;
    let chassis_world = car.phys.has_world_contact || sim.chassis_world;
    ContactLabels {
        car_ball,
        car_car,
        ball_world,
        chassis_world,
        wheel_world,
    }
}

/// Per-term normalized errors. Each term is already divided by its tol.
/// Use it to find which body part dominates a `wheel_world` fail.
#[derive(Clone, Copy, Debug, Default)]
pub struct ComponentErrors {
    pub car_pos: f32,
    pub ball_pos: f32,
    pub car_vel: f32,
    pub ball_vel: f32,
    pub car_ang: f32,
    pub ball_ang: f32,
    pub car_fwd: f32,
    pub car_up: f32,
    pub ball_fwd: f32,
    pub ball_up: f32,
}

impl ComponentErrors {
    /// Combined norm. Same value as [`normalized_error`].
    pub fn norm(&self) -> f32 {
        (self.car_pos * self.car_pos
            + self.ball_pos * self.ball_pos
            + self.car_vel * self.car_vel
            + self.ball_vel * self.ball_vel
            + self.car_ang * self.car_ang
            + self.ball_ang * self.ball_ang
            + self.car_fwd * self.car_fwd
            + self.car_up * self.car_up
            + self.ball_fwd * self.ball_fwd
            + self.ball_up * self.ball_up)
            .sqrt()
    }
}

/// Per-term errors for one sim vs truth pair.
///
/// Each term is already divided by its tolerance:
/// car/ball pos by 10 UU, car/ball vel by 3 UU/s,
/// car/ball ang vel by 1 rad/s, axes by 1.
pub fn component_errors(sim: &Snapshot, truth: &Snapshot) -> ComponentErrors {
    ComponentErrors {
        car_pos: (sim.car.pos - truth.car.pos).length() / POS_TOL_UU,
        ball_pos: (sim.ball.pos - truth.ball.pos).length() / POS_TOL_UU,
        car_vel: (sim.car.vel - truth.car.vel).length() / VEL_TOL_UU_S,
        ball_vel: (sim.ball.vel - truth.ball.vel).length() / VEL_TOL_UU_S,
        car_ang: (sim.car.ang_vel - truth.car.ang_vel).length() / ANG_VEL_TOL_RAD_S,
        ball_ang: (sim.ball.ang_vel - truth.ball.ang_vel).length() / ANG_VEL_TOL_RAD_S,
        car_fwd: (sim.car.forward - truth.car.forward).length() / AXIS_TOL,
        car_up: (sim.car.up - truth.car.up).length() / AXIS_TOL,
        ball_fwd: (sim.ball.forward - truth.ball.forward).length() / AXIS_TOL,
        ball_up: (sim.ball.up - truth.ball.up).length() / AXIS_TOL,
    }
}

/// Normalized physics error over car and ball.
///
/// Each component is divided by its tolerance, then combined as a norm:
/// car/ball position by 10 UU, car/ball velocity by 3 UU/s,
/// car/ball angular velocity by 1 rad/s, car/ball forward/up drift by 1.
/// Passes when norm < 1.
pub fn normalized_error(sim: &Snapshot, truth: &Snapshot) -> f32 {
    component_errors(sim, truth).norm()
}

/// Strict pass rule.
pub fn passes(norm_error: f32) -> bool {
    norm_error < 1.0
}

/// Pass rate in percent. 0 with no support.
pub fn pass_rate(passed: usize, support: usize) -> f64 {
    if support == 0 {
        0.0
    } else {
        100.0 * passed as f64 / support as f64
    }
}

/// Aggregate stats for one category.
#[derive(Clone, Debug, Default)]
pub struct CategoryStats {
    pub support: usize,
    pub passed: usize,
    sum_norm: f64,
    pub max_norm: f32,
    pub first_fail_tick: Option<usize>,
}

impl CategoryStats {
    /// Record one scored tick.
    pub fn add(&mut self, tick_index: usize, norm_error: f32) {
        self.support += 1;
        self.sum_norm += norm_error as f64;
        self.max_norm = self.max_norm.max(norm_error);
        if passes(norm_error) {
            self.passed += 1;
        } else if self.first_fail_tick.is_none() {
            self.first_fail_tick = Some(tick_index);
        }
    }

    /// Mean norm error. 0 with no support.
    pub fn mean_norm(&self) -> f64 {
        if self.support == 0 {
            0.0
        } else {
            self.sum_norm / self.support as f64
        }
    }

    /// Pass rate in percent.
    pub fn rate(&self) -> f64 {
        pass_rate(self.passed, self.support)
    }

    /// Fold another aggregate into this one. Support, passes, error mass,
    /// and max merge exactly; `first_fail_tick` is left untouched because
    /// tick indices are per-recording and meaningless once combined.
    pub fn merge(&mut self, other: &CategoryStats) {
        self.support += other.support;
        self.passed += other.passed;
        self.sum_norm += other.sum_norm;
        self.max_norm = self.max_norm.max(other.max_norm);
    }
}

/// Per-category report. Contact categories overlap.
#[derive(Clone, Debug, Default)]
pub struct EvalReport {
    pub total: CategoryStats,
    pub car_ball: CategoryStats,
    pub car_car: CategoryStats,
    pub ball_world: CategoryStats,
    pub chassis_world: CategoryStats,
    pub wheel_world: CategoryStats,
    pub no_contact: CategoryStats,
}

impl EvalReport {
    /// Read one category.
    pub fn for_category(&self, category: ContactCategory) -> &CategoryStats {
        match category {
            ContactCategory::CarBall => &self.car_ball,
            ContactCategory::CarCar => &self.car_car,
            ContactCategory::BallWorld => &self.ball_world,
            ContactCategory::ChassisWorld => &self.chassis_world,
            ContactCategory::WheelWorld => &self.wheel_world,
            ContactCategory::NoContact => &self.no_contact,
            ContactCategory::Total => &self.total,
        }
    }

    /// Update one category.
    fn for_category_mut(&mut self, category: ContactCategory) -> &mut CategoryStats {
        match category {
            ContactCategory::CarBall => &mut self.car_ball,
            ContactCategory::CarCar => &mut self.car_car,
            ContactCategory::BallWorld => &mut self.ball_world,
            ContactCategory::ChassisWorld => &mut self.chassis_world,
            ContactCategory::WheelWorld => &mut self.wheel_world,
            ContactCategory::NoContact => &mut self.no_contact,
            ContactCategory::Total => &mut self.total,
        }
    }

    /// Record one tick in each matching category.
    fn add(&mut self, labels: ContactLabels, tick_index: usize, norm_error: f32) {
        for category in ContactCategory::ALL {
            if labels.contains(category) {
                self.for_category_mut(category).add(tick_index, norm_error);
            }
        }
    }

    /// Fold another recording's report into this one for a cross-recording
    /// aggregate. Rates and means stay exact (support-weighted); max takes
    /// the worst. `first_fail_tick` is cleared: tick indices are
    /// per-recording, so a combined first-fail tick would be meaningless
    /// and prints as `-`.
    pub fn merge(&mut self, other: &EvalReport) {
        for category in ContactCategory::ALL {
            let stats = self.for_category_mut(category);
            stats.merge(other.for_category(category));
            stats.first_fail_tick = None;
        }
    }
}

/// Refresh hidden wheel state after a reset without advancing dynamics.
///
/// A full warmup tick also advances ball and manifold state. Use the wheel
/// raycast-only path so the scored body remains exactly at the recorded state.
pub fn settle_reset_state<B: ReplayBackend>(backend: &mut B, state: &TickRecord) {
    backend.set_state(state);
    backend.refresh_sticky_gates();
}

/// Restore reconstructed handbrake state at a segment state.
///
/// Use this fallback only for versions v2-v7. Version v8 records the value.
/// Harness only, not physics.
pub fn restore_handbrake_seed<B: ReplayBackend>(
    backend: &mut B,
    ticks: &[TickRecord],
    run_start: usize,
    state_index: usize,
) {
    let Some(state) = ticks.get(state_index) else {
        return;
    };
    for car_idx in 0..state.car_records.len() {
        if let Some(value) = reconstruct_handbrake(ticks, run_start, state_index, car_idx) {
            backend.set_handbrake_value(car_idx, value);
        }
    }
}

/// Restore recorded boost latch state after a reset.
///
/// Apply direct recorded state only when the format carries it.
/// Write raw time only when armed. Write zero when disarmed.
/// A stale disarmed time would inflate the next arm and expire the latch early.
/// Older versions keep live-latch evolution: this is a no-op for them.
/// The parser keeps the raw value. This guard is adaptation, not physics.
/// Harness only, not physics.
pub fn restore_recorded_boost_state<B: ReplayBackend>(
    backend: &mut B,
    ticks: &[TickRecord],
    state_index: usize,
    has_boost_state: bool,
) {
    if !has_boost_state {
        return;
    }
    let Some(state) = ticks.get(state_index) else {
        return;
    };
    for (car_idx, car) in state.car_records.iter().enumerate() {
        let time = if car.is_boosting {
            car.boosting_time
        } else {
            0.0
        };
        backend.set_boost_state(car_idx, car.is_boosting, time);
    }
}

/// Restore recorded handbrake integrator state after a reset.
///
/// Apply direct recorded state only when the format carries it.
/// Older versions keep the reconstruction fallback: this is a no-op for them.
/// The backend setter clamps the value. Harness only, not physics.
pub fn restore_recorded_handbrake<B: ReplayBackend>(
    backend: &mut B,
    ticks: &[TickRecord],
    state_index: usize,
    has_handbrake_state: bool,
) {
    if !has_handbrake_state {
        return;
    }
    let Some(state) = ticks.get(state_index) else {
        return;
    };
    for (car_idx, car) in state.car_records.iter().enumerate() {
        backend.set_handbrake_value(car_idx, car.handbrake_val);
    }
}

/// Report plus kickoff-stasis skip counts from one [`evaluate`] run.
/// Skipped counts hold scored transitions removed after warmup.
/// The sim still steps through them.
#[derive(Clone, Debug, Default)]
pub struct EvalOutcome {
    pub report: EvalReport,
    pub skipped_transitions: usize,
    pub skipped_car_ticks: usize,
}

/// Run each segment open-loop and aggregate every car-tick into one report.
/// Every arena car steps with its own recorded controls, so car-car
/// contacts are real sim observations. Resets at each segment start,
/// steps with target `prev_controls`, skips `warmup_ticks` ticks.
/// Support counts car-ticks: each scored tick contributes one sample per car.
/// With `use_sim_events`, sim-observed contacts also label ticks;
/// otherwise labels come from RL flags alone.
/// Kickoff-stasis transitions still step the sim but add no support,
/// pass, or error. Runs, chunks, and seeds are unchanged. Counts land
/// in [`EvalOutcome`].
/// `has_boost_state` must be true only when the recording version carries
/// recorded boost latch state; older versions keep live-latch evolution.
/// `has_handbrake_state` must be true only when the recording version
/// carries recorded handbrake state; older versions keep reconstruction.
#[allow(clippy::too_many_arguments)]
pub fn evaluate<B: ReplayBackend>(
    backend: &mut B,
    ticks: &[TickRecord],
    segments: &[Segment],
    warmup_ticks: usize,
    reset_each_tick: bool,
    reset_warmup: bool,
    use_sim_events: bool,
    has_boost_state: bool,
    has_handbrake_state: bool,
) -> EvalOutcome {
    let mut outcome = EvalOutcome::default();
    let report = &mut outcome.report;
    for segment in segments {
        if segment.end() > ticks.len() {
            continue;
        }
        let segment_run_start = run_start(ticks, segment.start);
        if !reset_each_tick {
            backend.reset(&ticks[segment.start]);
            restore_recorded_handbrake(backend, ticks, segment.start, has_handbrake_state);
            if !has_handbrake_state {
                restore_handbrake_seed(backend, ticks, segment_run_start, segment.start);
            }
            restore_recorded_boost_state(backend, ticks, segment.start, has_boost_state);
            // Seed the prior-tick wheel gate at the recorded pose.
            // Raycast only. Scored bodies stay at the recorded state.
            backend.refresh_sticky_gates();
        }
        for offset in 1..segment.len {
            let target_index = segment.start + offset;
            let target = &ticks[target_index];
            if reset_each_tick {
                let state_index = target_index - 1;
                if offset == 1 && reset_warmup {
                    settle_reset_state(backend, &ticks[state_index]);
                } else {
                    backend.set_state(&ticks[state_index]);
                }
                restore_recorded_boost_state(backend, ticks, state_index, has_boost_state);
                restore_recorded_handbrake(backend, ticks, state_index, has_handbrake_state);
                if !has_handbrake_state && offset == 1 {
                    restore_handbrake_seed(backend, ticks, segment_run_start, state_index);
                }
            }
            let controls: Vec<ControlsRecord> = target
                .car_records
                .iter()
                .map(|car| car.prev_controls)
                .collect();
            if controls.is_empty() {
                continue;
            }
            let sim_events = backend.step(&controls);
            if !reset_each_tick && offset < warmup_ticks {
                continue;
            }
            if target_index > 0 && tick_is_kickoff_stasis(&ticks[target_index - 1], target) {
                outcome.skipped_transitions += 1;
                outcome.skipped_car_ticks += controls.len();
                continue;
            }
            for car_idx in 0..controls.len() {
                let Some(truth) = snapshot_from_tick(target, car_idx) else {
                    continue;
                };
                let sim = if use_sim_events {
                    sim_events.get(car_idx).copied().unwrap_or_default()
                } else {
                    SimContactEvents::default()
                };
                let norm = normalized_error(&backend.snapshot(car_idx), &truth);
                report.add(classify_tick(target, car_idx, sim), target_index, norm);
            }
        }
    }
    outcome
}

#[cfg(test)]
mod tests {
    use rocketsim_test::rlpr::cpp_records::{
        CarRecord, Mat3Record, PhysRecord, VecRecord, WheelRecord,
    };

    use super::*;

    fn vec(x: f32, y: f32, z: f32) -> VecRecord {
        VecRecord::new(x, y, z)
    }

    fn ident_rot() -> Mat3Record {
        Mat3Record {
            rows: [vec(1.0, 0.0, 0.0), vec(0.0, 1.0, 0.0), vec(0.0, 0.0, 1.0)],
        }
    }

    fn blank_phys() -> PhysRecord {
        let mut phys: PhysRecord = unsafe { std::mem::zeroed() };
        phys.rot = ident_rot();
        phys
    }

    fn blank_car() -> CarRecord {
        CarRecord {
            phys: blank_phys(),
            is_on_ground: false,
            is_jumping: false,
            is_flipping: false,
            jump_time: 0.0,
            flip_time: 0.0,
            has_jumped: false,
            double_jumped_or_flipped: false,
            has_flip: false,
            flip_rel_torque: vec(0.0, 0.0, 0.0),
            boost_amount: 0.0,
            is_touching_ball: false,
            prev_controls: ControlsRecord {
                throttle: 0.0,
                steer: 0.0,
                pitch: 0.0,
                yaw: 0.0,
                roll: 0.0,
                jump: false,
                boost: false,
                handbrake: false,
            },
            wheels: [WheelRecord {
                susp_length: 0.0,
                susp_rel_vel: 0.0,
                has_contact: false,
                contact_normal: vec(0.0, 0.0, 1.0),
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
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn make_tick(
        frame: u32,
        car_pos: (f32, f32, f32),
        ball_pos: (f32, f32, f32),
        car_vel: (f32, f32, f32),
        ball_vel: (f32, f32, f32),
        car_ball: bool,
        ball_world: bool,
        chassis_world: bool,
        wheel_world: bool,
    ) -> TickRecord {
        let mut car = blank_car();
        car.phys.physics_frame = frame;
        car.phys.pos = vec(car_pos.0, car_pos.1, car_pos.2);
        car.phys.lin_vel = vec(car_vel.0, car_vel.1, car_vel.2);
        car.is_touching_ball = car_ball;
        car.phys.has_world_contact = chassis_world;
        car.wheels[0].has_contact = wheel_world;
        let mut ball = blank_phys();
        ball.physics_frame = frame;
        ball.pos = vec(ball_pos.0, ball_pos.1, ball_pos.2);
        ball.lin_vel = vec(ball_vel.0, ball_vel.1, ball_vel.2);
        ball.has_world_contact = ball_world;
        TickRecord {
            car_records: vec![car],
            ball_record: ball,
        }
    }

    fn quiet_tick(frame: u32, x: f32) -> TickRecord {
        make_tick(
            frame,
            (x, 0.0, 100.0),
            (0.0, 0.0, 500.0),
            (10.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
            false,
            false,
            false,
            false,
        )
    }

    fn config(segment_ticks: usize, warmup_ticks: usize) -> SegmentConfig {
        SegmentConfig {
            segment_ticks,
            warmup_ticks,
        }
    }

    #[test]
    fn splits_clean_run_into_non_overlapping_chunks() {
        let ticks: Vec<_> = (0..10).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        let segments = split_segments(&ticks, config(4, 1));
        assert_eq!(
            segments,
            vec![
                Segment { start: 0, len: 4 },
                Segment { start: 4, len: 4 },
                Segment { start: 8, len: 2 },
            ]
        );
    }

    #[test]
    fn drops_chunks_with_no_scored_ticks() {
        let ticks: Vec<_> = (0..5).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        let segments = split_segments(&ticks, config(4, 3));
        assert_eq!(segments, vec![Segment { start: 0, len: 4 }]);
    }

    #[test]
    fn rejects_invalid_config() {
        let ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32)).collect();
        assert!(split_segments(&ticks, config(4, 4)).is_empty());
        assert!(split_segments(&ticks, config(0, 0)).is_empty());
    }

    #[test]
    fn breaks_at_frame_gap_and_missing_car() {
        let mut ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        ticks.push(quiet_tick(10, 40.0));
        ticks.push(quiet_tick(11, 50.0));
        ticks.push(TickRecord {
            car_records: vec![],
            ball_record: ticks.last().unwrap().ball_record,
        });
        ticks.push(quiet_tick(12, 60.0));
        ticks.push(quiet_tick(13, 70.0));
        let segments = split_segments(&ticks, config(4, 1));
        for window in segments.windows(2) {
            assert!(window[0].end() <= window[1].start);
        }
        for segment in &segments {
            for i in segment.start..segment.end() {
                assert_eq!(ticks[i].car_records.len(), 1);
                if i > segment.start {
                    assert!(frame_is_contiguous(&ticks[i - 1], &ticks[i]));
                }
            }
        }
        // Missing-car tick is excluded; segments stay contiguous.
        assert!(segments.iter().all(|s| s.start != 6 || s.len == 1));
        assert!(!segments.iter().any(|s| (s.start..s.end()).contains(&6)));
    }

    #[test]
    fn breaks_at_frozen_transition() {
        let mut ticks: Vec<_> = (0..3).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        let mut frozen = ticks[2].clone();
        frozen.car_records[0].phys.physics_frame = 3;
        frozen.ball_record.physics_frame = 3;
        ticks.push(frozen);
        ticks.push(quiet_tick(4, 40.0));
        let segments = split_segments(&ticks, config(8, 1));
        for segment in &segments {
            let range = segment.start..segment.end();
            assert!(!(range.contains(&2) && range.contains(&3)));
        }
    }

    #[test]
    fn categories_overlap() {
        let tick = make_tick(
            0,
            (0.0, 0.0, 100.0),
            (50.0, 0.0, 100.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            true,
            false,
            false,
            true,
        );
        let labels = classify_tick(&tick, 0, SimContactEvents::default());
        assert!(labels.car_ball && labels.wheel_world);
        assert!(labels.contains(ContactCategory::CarBall));
        assert!(labels.contains(ContactCategory::WheelWorld));
        assert!(labels.contains(ContactCategory::Total));
        assert!(!labels.contains(ContactCategory::NoContact));
    }

    #[test]
    fn no_contact_only_when_all_labels_false() {
        let quiet = quiet_tick(0, 0.0);
        let labels = classify_tick(&quiet, 0, SimContactEvents::default());
        assert!(labels.is_quiet());
        assert!(labels.contains(ContactCategory::NoContact));
        let noisy = make_tick(
            0,
            (0.0, 0.0, 100.0),
            (0.0, 0.0, 500.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            true,
            false,
            false,
        );
        let labels = classify_tick(&noisy, 0, SimContactEvents::default());
        assert!(!labels.contains(ContactCategory::NoContact));
    }

    #[test]
    fn norm_error_matches_strict_thresholds() {
        let tick = quiet_tick(0, 0.0);
        let truth = snapshot_from_tick(&tick, 0).unwrap();
        assert_eq!(normalized_error(&truth, &truth), 0.0);
        assert!(passes(0.0));
        let mut moved = truth;
        moved.car.pos.x += POS_TOL_UU;
        assert!((normalized_error(&moved, &truth) - 1.0).abs() < 1e-5);
        assert!(!passes(normalized_error(&moved, &truth)));
        moved.car.pos.x -= POS_TOL_UU / 2.0;
        assert!(passes(normalized_error(&moved, &truth)));
        let mut ball_moved = truth;
        ball_moved.ball.forward = Vec3A::new(0.0, 1.0, 0.0);
        assert!(!passes(normalized_error(&ball_moved, &truth)));
    }

    #[test]
    fn aggregates_percentages_and_first_failure() {
        assert_eq!(pass_rate(1, 2), 50.0);
        assert_eq!(pass_rate(0, 0), 0.0);
        let mut stats = CategoryStats::default();
        stats.add(7, 0.5);
        stats.add(9, 2.0);
        assert_eq!(stats.support, 2);
        assert_eq!(stats.passed, 1);
        assert_eq!(stats.rate(), 50.0);
        assert!((stats.mean_norm() - 1.25).abs() < 1e-9);
        assert_eq!(stats.max_norm, 2.0);
        assert_eq!(stats.first_fail_tick, Some(9));
    }

    struct MirrorBackend {
        snaps: Vec<Vec<Snapshot>>,
        cursor: usize,
    }

    struct BoostProbe {
        boost_calls: Vec<(usize, bool, f32)>,
    }

    impl ReplayBackend for BoostProbe {
        fn reset(&mut self, _start: &TickRecord) {}
        fn set_state(&mut self, _state: &TickRecord) {}
        fn set_boost_state(&mut self, car_idx: usize, armed: bool, time: f32) {
            self.boost_calls.push((car_idx, armed, time));
        }
        fn step(&mut self, _controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
            vec![]
        }
        fn snapshot(&mut self, _car_idx: usize) -> Snapshot {
            let body = BodySnapshot {
                pos: Vec3A::ZERO,
                vel: Vec3A::ZERO,
                ang_vel: Vec3A::ZERO,
                forward: Vec3A::X,
                up: Vec3A::Z,
            };
            Snapshot {
                car: body,
                ball: body,
            }
        }
    }

    #[test]
    fn recorded_boost_state_restores_exact_values() {
        let mut tick = quiet_tick(1, 10.0);
        tick.car_records[0].is_boosting = true;
        tick.car_records[0].boosting_time = 0.05;
        let ticks = vec![quiet_tick(0, 0.0), tick];
        let mut probe = BoostProbe {
            boost_calls: vec![],
        };
        restore_recorded_boost_state(&mut probe, &ticks, 1, true);
        assert_eq!(probe.boost_calls, vec![(0, true, 0.05)]);
    }

    #[test]
    fn disarmed_large_time_restores_as_zero() {
        let mut tick = quiet_tick(1, 10.0);
        tick.car_records[0].is_boosting = false;
        tick.car_records[0].boosting_time = 5.0;
        let ticks = vec![quiet_tick(0, 0.0), tick];
        let mut probe = BoostProbe {
            boost_calls: vec![],
        };
        restore_recorded_boost_state(&mut probe, &ticks, 1, true);
        assert_eq!(probe.boost_calls, vec![(0, false, 0.0)]);
    }

    #[test]
    fn legacy_recordings_do_not_overwrite_the_latch() {
        let mut tick = quiet_tick(1, 10.0);
        tick.car_records[0].is_boosting = true;
        tick.car_records[0].boosting_time = 0.05;
        let ticks = vec![quiet_tick(0, 0.0), tick];
        let mut probe = BoostProbe {
            boost_calls: vec![],
        };
        restore_recorded_boost_state(&mut probe, &ticks, 1, false);
        assert!(probe.boost_calls.is_empty());
        restore_recorded_boost_state(&mut probe, &ticks, 99, true);
        assert!(probe.boost_calls.is_empty());
    }

    struct HandbrakeProbe {
        brake_calls: Vec<(usize, f32)>,
    }

    impl ReplayBackend for HandbrakeProbe {
        fn reset(&mut self, _start: &TickRecord) {}
        fn set_state(&mut self, _state: &TickRecord) {}
        fn set_handbrake_value(&mut self, car_idx: usize, value: f32) {
            self.brake_calls.push((car_idx, value));
        }
        fn step(&mut self, _controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
            vec![]
        }
        fn snapshot(&mut self, _car_idx: usize) -> Snapshot {
            let body = BodySnapshot {
                pos: Vec3A::ZERO,
                vel: Vec3A::ZERO,
                ang_vel: Vec3A::ZERO,
                forward: Vec3A::X,
                up: Vec3A::Z,
            };
            Snapshot {
                car: body,
                ball: body,
            }
        }
    }

    #[test]
    fn recorded_handbrake_restores_exact_value() {
        let mut tick = quiet_tick(1, 10.0);
        tick.car_records[0].handbrake_val = 0.875;
        let ticks = vec![quiet_tick(0, 0.0), tick];
        let mut probe = HandbrakeProbe {
            brake_calls: vec![],
        };
        restore_recorded_handbrake(&mut probe, &ticks, 1, true);
        assert_eq!(probe.brake_calls, vec![(0, 0.875)]);
    }

    #[test]
    fn legacy_handbrake_fallback_stays_live() {
        let mut tick = quiet_tick(1, 10.0);
        tick.car_records[0].handbrake_val = 0.875;
        let ticks = vec![quiet_tick(0, 0.0), tick];
        let mut probe = HandbrakeProbe {
            brake_calls: vec![],
        };
        restore_recorded_handbrake(&mut probe, &ticks, 1, false);
        assert!(probe.brake_calls.is_empty());
        restore_recorded_handbrake(&mut probe, &ticks, 99, true);
        assert!(probe.brake_calls.is_empty());
    }

    impl MirrorBackend {
        fn new(ticks: &[TickRecord]) -> Self {
            let num_cars = ticks
                .first()
                .map(|tick| tick.car_records.len())
                .unwrap_or(1);
            let snaps = (0..num_cars)
                .map(|car_idx| {
                    ticks
                        .iter()
                        .map(|tick| snapshot_from_tick(tick, car_idx).unwrap())
                        .collect()
                })
                .collect();
            Self { snaps, cursor: 0 }
        }
    }

    impl ReplayBackend for MirrorBackend {
        fn reset(&mut self, start: &TickRecord) {
            let want = snapshot_from_tick(start, 0).unwrap();
            self.cursor = self.snaps[0]
                .iter()
                .position(|snap| snap.car.pos == want.car.pos)
                .unwrap_or(0);
        }

        fn set_state(&mut self, state: &TickRecord) {
            self.reset(state);
        }

        fn step(&mut self, _controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
            self.cursor = (self.cursor + 1).min(self.snaps[0].len() - 1);
            vec![SimContactEvents::default(); self.snaps.len()]
        }

        fn snapshot(&mut self, car_idx: usize) -> Snapshot {
            self.snaps[car_idx][self.cursor]
        }
    }

    #[test]
    fn labels_sim_observed_contacts() {
        // Sim-observed contacts label the tick even when RL flags are clear.
        let tick = quiet_tick(1, 10.0);
        let sim = SimContactEvents {
            car_ball: false,
            car_car: true,
            ball_world: true,
            chassis_world: false,
        };
        let labels = classify_tick(&tick, 0, sim);
        assert!(labels.car_car);
        assert!(labels.ball_world);
        assert!(!labels.car_ball);
        assert!(!labels.contains(ContactCategory::NoContact));
        // No sim events and clear flags: quiet.
        let quiet = classify_tick(&tick, 0, SimContactEvents::default());
        assert!(quiet.is_quiet());
        // RL flags still label without sim events.
        let mut touch = quiet_tick(1, 10.0);
        touch.car_records[0].is_touching_ball = true;
        let labels = classify_tick(&touch, 0, SimContactEvents::default());
        assert!(labels.car_ball);
    }

    #[test]
    fn car_car_labels_recorded_flag_without_sim() {
        // Recorded-events default path: empty sim events still get RL support.
        let mut touch = quiet_tick(1, 10.0);
        touch.car_records[0].is_touching_car = true;
        let labels = classify_tick(&touch, 0, SimContactEvents::default());
        assert!(labels.car_car);
        assert!(!labels.is_quiet());
        assert!(labels.contains(ContactCategory::CarCar));
    }

    #[test]
    fn car_car_labels_either_source() {
        for (recorded, sim_flag, want) in [
            (false, false, false),
            (true, false, true),
            (false, true, true),
            (true, true, true),
        ] {
            let mut tick = quiet_tick(1, 10.0);
            tick.car_records[0].is_touching_car = recorded;
            let sim = SimContactEvents {
                car_car: sim_flag,
                ..SimContactEvents::default()
            };
            assert_eq!(
                classify_tick(&tick, 0, sim).car_car,
                want,
                "recorded={recorded} sim_flag={sim_flag}",
            );
        }
    }

    #[test]
    fn car_car_labels_are_per_car_asymmetric() {
        let mut tick = quiet_tick(1, 10.0);
        let mut other = blank_car();
        other.phys.physics_frame = 1;
        tick.car_records.push(other);
        tick.car_records[0].is_touching_car = false;
        tick.car_records[1].is_touching_car = true;
        let car0 = classify_tick(&tick, 0, SimContactEvents::default());
        let car1 = classify_tick(&tick, 1, SimContactEvents::default());
        assert!(!car0.car_car);
        assert!(car1.car_car);
        assert!(car0.is_quiet());
        assert!(!car1.is_quiet());
        // Sim events also apply per scored car.
        let sim = SimContactEvents {
            car_car: true,
            ..SimContactEvents::default()
        };
        assert!(classify_tick(&tick, 0, sim).car_car);
    }

    #[test]
    fn splits_on_car_teleport() {
        let mut ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        // Tick 2 snaps 5000 UU away with contiguous frames: a reset, not play.
        ticks[2].car_records[0].phys.pos = vec(5000.0, 0.0, 100.0);
        ticks[3].car_records[0].phys.pos = vec(5010.0, 0.0, 100.0);
        let segments = split_segments(&ticks, config(8, 1));
        for segment in &segments {
            let range = segment.start..segment.end();
            assert!(!(range.contains(&1) && range.contains(&2)));
        }
    }

    #[test]
    fn splits_on_ball_teleport() {
        let mut ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        ticks[2].ball_record.pos = vec(0.0, 5000.0, 100.0);
        ticks[3].ball_record.pos = vec(0.0, 5010.0, 100.0);
        let segments = split_segments(&ticks, config(8, 1));
        for segment in &segments {
            let range = segment.start..segment.end();
            assert!(!(range.contains(&1) && range.contains(&2)));
        }
    }

    #[test]
    fn fast_legal_motion_keeps_run() {
        // 400 UU per tick is fast but legal: no teleport split.
        let ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 400.0)).collect();
        let segments = split_segments(&ticks, config(8, 1));
        assert_eq!(segments, vec![Segment { start: 0, len: 4 }]);
    }

    #[test]
    fn reconstructs_handbrake_with_sixty_tick_fall_and_run_boundary() {
        let mut ticks: Vec<_> = (0..84).map(|i| quiet_tick(i, i as f32)).collect();
        for tick in ticks.iter_mut().take(24) {
            tick.car_records[0].prev_controls.handbrake = true;
        }
        assert_eq!(reconstruct_handbrake(&ticks, 0, 23, 0), Some(1.0));
        let washed = reconstruct_handbrake(&ticks, 0, 83, 0).unwrap();
        assert!(washed.abs() < 1e-6, "washout {washed}");

        ticks[0].car_records[0].prev_controls.handbrake = true;
        ticks[1].car_records[0].prev_controls.handbrake = false;
        let bounded = reconstruct_handbrake(&ticks, 1, 1, 0).unwrap();
        assert_eq!(bounded, 0.0);
    }

    #[test]
    fn reconstructs_handbrake_from_history_older_than_sixty_ticks() {
        // 70 alternating ticks stay below saturation: full run keeps the
        // first 5 press cycles that a 60-tick window drops.
        let mut ticks: Vec<_> = (0..70).map(|i| quiet_tick(i, i as f32)).collect();
        for (i, tick) in ticks.iter_mut().enumerate() {
            tick.car_records[0].prev_controls.handbrake = i % 2 == 0;
        }
        let full = reconstruct_handbrake(&ticks, 0, 69, 0).unwrap();
        let truncated = reconstruct_handbrake(&ticks, 10, 69, 0).unwrap();
        assert!((full - 0.875).abs() < 1e-5, "full run value {full}");
        assert!(
            (truncated - 0.75).abs() < 1e-5,
            "truncated value {truncated}"
        );
        assert!((full - truncated - 0.125).abs() < 1e-5);
    }

    #[test]
    fn splits_on_car_count_change_and_labels_per_car() {
        let mut ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        // Tick 2 gains a second car that touches the ball.
        let mut second = ticks[2].car_records[0].clone();
        second.is_touching_ball = true;
        ticks[2].car_records.push(second);
        let segments = split_segments(&ticks, config(4, 1));
        // No segment spans the count change at index 2.
        assert!(!segments.iter().any(|s| (s.start..s.end()).contains(&2)));
        // Labels are per car.
        let tick2 = &ticks[2];
        assert!(!classify_tick(tick2, 0, SimContactEvents::default()).car_ball);
        assert!(classify_tick(tick2, 1, SimContactEvents::default()).car_ball);
        assert!(snapshot_from_tick(&ticks[2], 1).is_some());
        assert!(snapshot_from_tick(&ticks[2], 2).is_none());
        // Car 0 still evaluates over the clean run.
        let mut backend = MirrorBackend::new(&ticks);
        let report = evaluate(
            &mut backend,
            &ticks,
            &segments,
            1,
            false,
            false,
            true,
            false,
            false,
        )
        .report;
        assert!(report.total.support > 0);
    }

    #[test]
    fn evaluate_combines_car_ticks() {
        // Two-car run: every scored tick counts once per car.
        let mut ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        for tick in &mut ticks {
            tick.car_records.push(tick.car_records[0].clone());
        }
        let mut backend = MirrorBackend::new(&ticks);
        let segments = vec![Segment { start: 0, len: 4 }];
        let report = evaluate(
            &mut backend,
            &ticks,
            &segments,
            1,
            false,
            false,
            true,
            false,
            false,
        )
        .report;
        assert_eq!(report.total.support, 6);
        assert_eq!(report.total.passed, 6);
        assert_eq!(report.no_contact.support, 6);
    }

    #[test]
    fn evaluate_counts_overlapping_support() {
        let mut ticks: Vec<_> = (0..6).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        ticks[4].car_records[0].is_touching_ball = true;
        ticks[4].car_records[0].wheels[0].has_contact = true;
        let mut backend = MirrorBackend::new(&ticks);
        let segments = vec![Segment { start: 0, len: 6 }];
        let report = evaluate(
            &mut backend,
            &ticks,
            &segments,
            1,
            false,
            false,
            true,
            false,
            false,
        )
        .report;
        assert_eq!(report.total.support, 5);
        assert_eq!(report.total.passed, 5);
        assert_eq!(report.car_ball.support, 1);
        assert_eq!(report.wheel_world.support, 1);
        assert_eq!(report.no_contact.support, 4);

        let mut backend = MirrorBackend::new(&ticks);
        let report = evaluate(
            &mut backend,
            &ticks,
            &segments,
            3,
            true,
            false,
            true,
            false,
            false,
        )
        .report;
        assert_eq!(report.total.support, 5);
        assert_eq!(report.total.passed, 5);
    }

    fn kickoff_pair(
        car_move: f32,
        car_hxy: f32,
        car_vz: f32,
        ball_pos: (f32, f32, f32),
        ball_world: bool,
        wheels: bool,
    ) -> (TickRecord, TickRecord) {
        let mut from = make_tick(
            100,
            (2048.0, -2560.0, 17.0),
            ball_pos,
            (car_hxy, 0.0, car_vz),
            (0.0, 0.0, 0.0),
            false,
            ball_world,
            true,
            wheels,
        );
        from.car_records[0].phys.lin_vel = vec(car_hxy, 0.0, car_vz);
        let mut to = make_tick(
            101,
            (2048.0 + car_move, -2560.0, 17.0),
            ball_pos,
            (car_hxy, 0.0, car_vz),
            (0.0, 0.0, 0.0),
            false,
            ball_world,
            true,
            wheels,
        );
        to.car_records[0].phys.lin_vel = vec(car_hxy, 0.0, car_vz);
        (from, to)
    }

    #[test]
    fn stasis_flags_observed_spawn_variants() {
        let spawns = [
            (2048.0, -2560.0),
            (-256.0, -3840.0),
            (256.0, -3840.0),
            (-2048.0, -2560.0),
            (0.0, -4608.0),
        ];
        for (x, y) in spawns {
            let from = make_tick(
                100,
                (x, y, 17.0),
                (0.0, 0.0, 92.75),
                (0.0, 0.0, -79.88),
                (0.0, 0.0, 0.0),
                false,
                true,
                true,
                true,
            );
            let mut to = from.clone();
            to.car_records[0].phys.pos.x += 0.5;
            to.car_records[0].phys.physics_frame = 101;
            to.ball_record.physics_frame = 101;
            assert!(!tick_is_frozen(&from, &to));
            assert!(tick_is_kickoff_stasis(&from, &to));
        }
    }

    #[test]
    fn stasis_rejects_driveoff_idle_and_edges() {
        let (from, to) = kickoff_pair(0.3, 30.0, 0.0, (0.0, 0.0, 92.75), false, true);
        assert!(!tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(0.3, 30.0, 0.0, (0.0, 0.0, 92.75), true, true);
        assert!(!tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(0.3, 0.0, 0.0, (1000.0, 500.0, 92.75), true, true);
        assert!(!tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(5.0, 200.0, 0.0, (0.0, 0.0, 92.75), true, true);
        assert!(!tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(0.99, 0.0, 0.0, (0.0, 0.0, 92.75), true, true);
        assert!(tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(1.01, 0.0, 0.0, (0.0, 0.0, 92.75), true, true);
        assert!(!tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(0.3, 4.9, 0.0, (0.0, 0.0, 92.75), true, true);
        assert!(tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(0.3, 5.1, 0.0, (0.0, 0.0, 92.75), true, true);
        assert!(!tick_is_kickoff_stasis(&from, &to));
        let from = make_tick(
            100,
            (2048.0, -2560.0, 17.0),
            (0.0, 0.0, 92.75),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            true,
            true,
            true,
        );
        let mut to = from.clone();
        to.car_records[0].phys.physics_frame = 101;
        to.ball_record.physics_frame = 101;
        assert!(tick_is_frozen(&from, &to));
        assert!(!tick_is_kickoff_stasis(&from, &to));
    }

    #[test]
    fn stasis_keeps_segmentation_and_seeds() {
        let lead = make_tick(
            99,
            (0.0, 0.0, 17.0),
            (0.0, 0.0, 92.75),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            true,
            true,
            true,
        );
        let from = make_tick(
            100,
            (10.0, 0.0, 17.0),
            (0.0, 0.0, 92.75),
            (0.0, 0.0, -79.88),
            (0.0, 0.0, 0.0),
            false,
            true,
            true,
            true,
        );
        let to = make_tick(
            101,
            (10.5, 0.0, 17.0),
            (0.0, 0.0, 92.75),
            (0.0, 0.0, -79.88),
            (0.0, 0.0, 0.0),
            false,
            true,
            true,
            true,
        );
        let after = make_tick(
            102,
            (20.0, 0.0, 17.0),
            (0.0, 0.0, 92.75),
            (200.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            false,
            true,
            true,
        );
        assert!(tick_is_kickoff_stasis(&from, &to));
        let ticks = vec![lead, from, to, after];
        assert_eq!(
            split_segments(&ticks, config(8, 1)),
            vec![Segment { start: 0, len: 4 }]
        );
        assert_eq!(run_start(&ticks, 2), 0);
    }

    #[test]
    fn evaluate_skips_stasis_but_steps() {
        let ticks: Vec<TickRecord> = [99u32, 100, 101, 102]
            .iter()
            .map(|&f| {
                let x = match f {
                    99 => 0.0,
                    100 => 10.0,
                    101 => 10.5,
                    _ => 20.0,
                };
                let (ball_world, hxy) = if f == 100 || f == 101 {
                    (true, 0.0)
                } else {
                    (false, 200.0)
                };
                let mut t = make_tick(
                    f,
                    (x, 0.0, 17.0),
                    (0.0, 0.0, 92.75),
                    (hxy, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                    false,
                    ball_world,
                    true,
                    true,
                );
                t.car_records[0].phys.lin_vel = vec(hxy, 0.0, 0.0);
                t.car_records.push(t.car_records[0].clone());
                t
            })
            .collect();
        assert!(tick_is_kickoff_stasis(&ticks[1], &ticks[2]));
        assert!(!tick_is_kickoff_stasis(&ticks[2], &ticks[3]));
        let segments = vec![Segment { start: 0, len: 4 }];
        let mut backend = MirrorBackend::new(&ticks);
        let outcome = evaluate(
            &mut backend,
            &ticks,
            &segments,
            1,
            true,
            false,
            true,
            false,
            false,
        );
        assert_eq!(outcome.skipped_transitions, 1);
        assert_eq!(outcome.skipped_car_ticks, 2);
        assert_eq!(outcome.report.total.support, 4);
        assert_eq!(outcome.report.total.passed, 4);
        assert_eq!(kickoff_stasis_targets(&ticks), vec![2]);
    }

    struct StickyGateProbe {
        snaps: Vec<Vec<Snapshot>>,
        cursor: usize,
        refreshes: usize,
        events: Vec<String>,
    }

    impl StickyGateProbe {
        fn new(ticks: &[TickRecord]) -> Self {
            let snaps = vec![
                ticks
                    .iter()
                    .map(|tick| snapshot_from_tick(tick, 0).unwrap())
                    .collect(),
            ];
            Self {
                snaps,
                cursor: 0,
                refreshes: 0,
                events: Vec::new(),
            }
        }
    }

    impl ReplayBackend for StickyGateProbe {
        fn reset(&mut self, start: &TickRecord) {
            self.events.push("reset".to_string());
            let want = snapshot_from_tick(start, 0).unwrap();
            self.cursor = self.snaps[0]
                .iter()
                .position(|snap| snap.car.pos == want.car.pos)
                .unwrap_or(0);
        }

        fn set_state(&mut self, state: &TickRecord) {
            self.events.push("set_state".to_string());
            let want = snapshot_from_tick(state, 0).unwrap();
            self.cursor = self.snaps[0]
                .iter()
                .position(|snap| snap.car.pos == want.car.pos)
                .unwrap_or(0);
        }

        fn set_handbrake_value(&mut self, _car_idx: usize, _value: f32) {
            self.events.push("handbrake".to_string());
        }

        fn set_boost_state(&mut self, _car_idx: usize, _armed: bool, _time: f32) {
            self.events.push("boost".to_string());
        }

        fn refresh_sticky_gates(&mut self) {
            self.refreshes += 1;
            self.events.push("refresh".to_string());
        }

        fn step(&mut self, _controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
            self.events.push("step".to_string());
            self.cursor = (self.cursor + 1).min(self.snaps[0].len() - 1);
            vec![SimContactEvents::default(); self.snaps.len()]
        }

        fn snapshot(&mut self, car_idx: usize) -> Snapshot {
            self.snaps[car_idx][self.cursor]
        }
    }

    #[test]
    fn segmented_refresh_runs_once_per_segment() {
        let ticks: Vec<_> = (0..6).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        let segments = vec![Segment { start: 0, len: 3 }, Segment { start: 3, len: 3 }];
        let mut backend = StickyGateProbe::new(&ticks);
        evaluate(
            &mut backend,
            &ticks,
            &segments,
            1,
            false,
            false,
            true,
            true,
            true,
        );
        assert_eq!(backend.refreshes, segments.len());
        let reset_pos = backend.events.iter().position(|e| e == "reset").unwrap();
        let refresh_pos = backend.events.iter().position(|e| e == "refresh").unwrap();
        let step_pos = backend.events.iter().position(|e| e == "step").unwrap();
        let handbrake_pos = backend
            .events
            .iter()
            .position(|e| e == "handbrake")
            .unwrap();
        let boost_pos = backend.events.iter().position(|e| e == "boost").unwrap();
        assert!(reset_pos < handbrake_pos);
        assert!(handbrake_pos < boost_pos);
        assert!(boost_pos < refresh_pos);
        assert!(refresh_pos < step_pos);
        let mut backend = StickyGateProbe::new(&ticks);
        evaluate(
            &mut backend,
            &ticks,
            &segments,
            1,
            false,
            false,
            true,
            true,
            false,
        );
        assert_eq!(backend.refreshes, segments.len());
        let mut backend = StickyGateProbe::new(&ticks);
        evaluate(
            &mut backend,
            &ticks,
            &segments,
            1,
            true,
            false,
            true,
            true,
            true,
        );
        assert_eq!(backend.refreshes, 0);
        let mut backend = StickyGateProbe::new(&ticks);
        evaluate(
            &mut backend,
            &ticks,
            &segments,
            1,
            true,
            true,
            true,
            true,
            true,
        );
        assert_eq!(backend.refreshes, segments.len());
    }
}
