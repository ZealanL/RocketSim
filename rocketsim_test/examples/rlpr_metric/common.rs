//! Backend-neutral multi-tick metric for one-car RLPR recordings.
//!
//! Thin `v3`/`v2` adapters implement [`ReplayBackend`].
//! The CLI builds [`Segment`]s, runs [`evaluate`], prints [`EvalReport`].
//!
//! Each segment holds `segment_ticks` ticks.
//! The first `warmup_ticks` ticks only advance the sim.
//! The rest are scored. Reset happens only at segment starts.

use glam::Vec3A;
use rocketsim_test::rlpr::{cpp_records::ControlsRecord, tick_record::TickRecord};

// Fixed Soccar geometry in Unreal units.
pub const SOCCAR_HALF_X: f32 = 4096.0;
pub const SOCCAR_HALF_Y: f32 = 5120.0;
pub const SOCCAR_CEIL_Z: f32 = 2048.0;
pub const SOCCAR_BALL_RADIUS: f32 = 91.25;
pub const SOCCAR_CAR_BOUND_RADIUS: f32 = 90.0;
// Distance band around a wall plane that counts as near contact.
pub const WORLD_PROX_MARGIN: f32 = 30.0;
// Minimum normal speed that counts as a velocity flip.
pub const VEL_FLIP_MIN: f32 = 50.0;
// Max car-ball center distance that counts as inferred contact.
pub const CAR_BALL_DIST: f32 = 220.0;
// Min velocity jump on either body that confirms inferred contact.
pub const CAR_BALL_DELTA_VEL: f32 = 300.0;

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

/// Backend adapter. Holds the sim. Resets only at segment starts.
pub trait ReplayBackend {
    fn reset(&mut self, start: &TickRecord);
    fn set_state(&mut self, state: &TickRecord);
    fn step(&mut self, controls: &ControlsRecord);
    fn snapshot(&mut self) -> Snapshot;
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

/// Read ground truth from a one-car tick. `None` without one car.
pub fn snapshot_from_tick(tick: &TickRecord) -> Option<Snapshot> {
    let [car] = tick.car_records.as_slice() else {
        return None;
    };
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

/// Exactly one car in the tick.
pub fn tick_has_single_car(tick: &TickRecord) -> bool {
    tick.car_records.len() == 1
}

/// Physics frames advance by one for car and ball.
pub fn frame_is_contiguous(from: &TickRecord, to: &TickRecord) -> bool {
    let [from_car] = from.car_records.as_slice() else {
        return false;
    };
    let [to_car] = to.car_records.as_slice() else {
        return false;
    };
    to_car.phys.physics_frame == from_car.phys.physics_frame + 1
        && to.ball_record.physics_frame == from.ball_record.physics_frame + 1
}

/// No body moved between ticks (pause or replay stall).
pub fn tick_is_frozen(from: &TickRecord, to: &TickRecord) -> bool {
    let [from_car] = from.car_records.as_slice() else {
        return false;
    };
    let [to_car] = to.car_records.as_slice() else {
        return false;
    };
    from_car.phys.pos == to_car.phys.pos
        && from_car.phys.lin_vel == to_car.phys.lin_vel
        && from_car.phys.ang_vel == to_car.phys.ang_vel
        && from.ball_record.pos == to.ball_record.pos
        && from.ball_record.lin_vel == to.ball_record.lin_vel
        && from.ball_record.ang_vel == to.ball_record.ang_vel
}

/// Split ticks into non-overlapping segments.
///
/// Break runs at frame gaps, frozen transitions, and ticks without one car.
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
        if !tick_has_single_car(tick) {
            flush_run(index, &mut run_start);
            continue;
        }
        match run_start {
            None => run_start = Some(index),
            Some(_) => {
                let prev = &ticks[index - 1];
                if !tick_has_single_car(prev)
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
    BallWorld,
    ChassisWorld,
    WheelWorld,
    NoContact,
    Total,
}

impl ContactCategory {
    /// All categories in CLI column order.
    pub const ALL: [ContactCategory; 6] = [
        ContactCategory::CarBall,
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
    pub ball_world: bool,
    pub chassis_world: bool,
    pub wheel_world: bool,
}

impl ContactLabels {
    /// No label is set.
    pub fn is_quiet(&self) -> bool {
        !(self.car_ball || self.ball_world || self.chassis_world || self.wheel_world)
    }

    /// Membership. `NoContact` holds only when all labels are false.
    pub fn contains(&self, category: ContactCategory) -> bool {
        match category {
            ContactCategory::CarBall => self.car_ball,
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
/// flipped normal velocity also count. Missed car-ball touches count when
/// centers are close and either body velocity jumps.
pub fn classify_tick(tick: &TickRecord, prev: Option<&TickRecord>) -> ContactLabels {
    let [car] = tick.car_records.as_slice() else {
        return ContactLabels::default();
    };
    let car_ball = car.is_touching_ball || infer_car_ball(tick, prev);
    let wheel_world = car.wheels.iter().any(|wheel| wheel.has_contact);
    let ball_world =
        tick.ball_record.has_world_contact || infer_wall_hit(prev_vel(prev, true), tick, true);
    let chassis_world =
        car.phys.has_world_contact || infer_wall_hit(prev_vel(prev, false), tick, false);
    ContactLabels {
        car_ball,
        ball_world,
        chassis_world,
        wheel_world,
    }
}

/// Prev-tick velocity for ball (`is_ball`) or car.
fn prev_vel(prev: Option<&TickRecord>, is_ball: bool) -> Option<Vec3A> {
    let prev = prev?;
    if is_ball {
        Some(prev.ball_record.lin_vel.into())
    } else {
        let [car] = prev.car_records.as_slice() else {
            return None;
        };
        Some(car.phys.lin_vel.into())
    }
}

/// Missed car-ball touch: close centers plus a velocity jump on either body.
fn infer_car_ball(tick: &TickRecord, prev: Option<&TickRecord>) -> bool {
    let prev = match prev {
        Some(prev) => prev,
        None => return false,
    };
    let [car] = tick.car_records.as_slice() else {
        return false;
    };
    let [prev_car] = prev.car_records.as_slice() else {
        return false;
    };
    let car_pos: Vec3A = car.phys.pos.into();
    let ball_pos: Vec3A = tick.ball_record.pos.into();
    if (car_pos - ball_pos).length() >= CAR_BALL_DIST {
        return false;
    }
    let car_vel: Vec3A = car.phys.lin_vel.into();
    let prev_car_vel: Vec3A = prev_car.phys.lin_vel.into();
    let ball_vel: Vec3A = tick.ball_record.lin_vel.into();
    let prev_ball_vel: Vec3A = prev.ball_record.lin_vel.into();
    (car_vel - prev_car_vel).length() >= CAR_BALL_DELTA_VEL
        || (ball_vel - prev_ball_vel).length() >= CAR_BALL_DELTA_VEL
}
/// Missed wall hit: wall proximity plus a flipped normal velocity.
fn infer_wall_hit(prev_vel: Option<Vec3A>, tick: &TickRecord, is_ball: bool) -> bool {
    let Some(prev_vel) = prev_vel else {
        return false;
    };
    let (pos, vel): (Vec3A, Vec3A) = if is_ball {
        (tick.ball_record.pos.into(), tick.ball_record.lin_vel.into())
    } else {
        let [car] = tick.car_records.as_slice() else {
            return false;
        };
        (car.phys.pos.into(), car.phys.lin_vel.into())
    };
    let radius = if is_ball {
        SOCCAR_BALL_RADIUS
    } else {
        SOCCAR_CAR_BOUND_RADIUS
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

/// Normalized physics error over car and ball.
///
/// Each component is divided by its tolerance, then combined as a norm:
/// car/ball position by 10 UU, car/ball velocity by 3 UU/s,
/// car/ball angular velocity by 1 rad/s, car/ball forward/up drift by 1.
/// Passes when norm < 1.
pub fn normalized_error(sim: &Snapshot, truth: &Snapshot) -> f32 {
    let car_pos = (sim.car.pos - truth.car.pos).length() / POS_TOL_UU;
    let ball_pos = (sim.ball.pos - truth.ball.pos).length() / POS_TOL_UU;
    let car_vel = (sim.car.vel - truth.car.vel).length() / VEL_TOL_UU_S;
    let ball_vel = (sim.ball.vel - truth.ball.vel).length() / VEL_TOL_UU_S;
    let car_ang = (sim.car.ang_vel - truth.car.ang_vel).length() / ANG_VEL_TOL_RAD_S;
    let ball_ang = (sim.ball.ang_vel - truth.ball.ang_vel).length() / ANG_VEL_TOL_RAD_S;
    let car_fwd = (sim.car.forward - truth.car.forward).length() / AXIS_TOL;
    let car_up = (sim.car.up - truth.car.up).length() / AXIS_TOL;
    let ball_fwd = (sim.ball.forward - truth.ball.forward).length() / AXIS_TOL;
    let ball_up = (sim.ball.up - truth.ball.up).length() / AXIS_TOL;
    (car_pos * car_pos
        + ball_pos * ball_pos
        + car_vel * car_vel
        + ball_vel * ball_vel
        + car_ang * car_ang
        + ball_ang * ball_ang
        + car_fwd * car_fwd
        + car_up * car_up
        + ball_fwd * ball_fwd
        + ball_up * ball_up)
        .sqrt()
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
}

/// Per-category report. Contact categories overlap.
#[derive(Clone, Debug, Default)]
pub struct EvalReport {
    pub total: CategoryStats,
    pub car_ball: CategoryStats,
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
}

/// Run each segment open-loop and aggregate errors.
/// Resets at each segment start, steps with target `prev_controls`,
/// skips `warmup_ticks` ticks, labels scored ticks from ground truth..
pub fn evaluate<B: ReplayBackend>(
    backend: &mut B,
    ticks: &[TickRecord],
    segments: &[Segment],
    warmup_ticks: usize,
    reset_each_tick: bool,
) -> EvalReport {
    let mut report = EvalReport::default();
    for segment in segments {
        if segment.end() > ticks.len() {
            continue;
        }
        if !reset_each_tick {
            backend.reset(&ticks[segment.start]);
        }
        for offset in 1..segment.len {
            let target_index = segment.start + offset;
            let target = &ticks[target_index];
            if reset_each_tick {
                backend.set_state(&ticks[target_index - 1]);
            }
            let [car] = target.car_records.as_slice() else {
                continue;
            };
            backend.step(&car.prev_controls);
            let Some(truth) = snapshot_from_tick(target) else {
                continue;
            };
            if !reset_each_tick && offset < warmup_ticks {
                continue;
            }
            let norm = normalized_error(&backend.snapshot(), &truth);
            let prev = ticks.get(target_index.wrapping_sub(1));
            report.add(classify_tick(target, prev), target_index, norm);
        }
    }
    report
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
        let labels = classify_tick(&tick, None);
        assert!(labels.car_ball && labels.wheel_world);
        assert!(labels.contains(ContactCategory::CarBall));
        assert!(labels.contains(ContactCategory::WheelWorld));
        assert!(labels.contains(ContactCategory::Total));
        assert!(!labels.contains(ContactCategory::NoContact));
    }

    #[test]
    fn no_contact_only_when_all_labels_false() {
        let quiet = quiet_tick(0, 0.0);
        let labels = classify_tick(&quiet, None);
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
        let labels = classify_tick(&noisy, None);
        assert!(!labels.contains(ContactCategory::NoContact));
    }

    #[test]
    fn infers_missed_car_ball_hit() {
        let prev = make_tick(
            0,
            (0.0, 0.0, 100.0),
            (100.0, 0.0, 100.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            false,
            false,
            false,
        );
        let tick = make_tick(
            1,
            (0.0, 0.0, 100.0),
            (100.0, 0.0, 100.0),
            (400.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            false,
            false,
            false,
        );
        assert!(classify_tick(&tick, Some(&prev)).car_ball);
    }

    #[test]
    fn skips_inferred_car_ball_without_velocity_jump() {
        let prev = make_tick(
            0,
            (0.0, 0.0, 100.0),
            (100.0, 0.0, 100.0),
            (10.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
            false,
            false,
            false,
            false,
        );
        let tick = make_tick(
            1,
            (5.0, 0.0, 100.0),
            (100.0, 0.0, 100.0),
            (10.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
            false,
            false,
            false,
            false,
        );
        assert!(!classify_tick(&tick, Some(&prev)).car_ball);
    }

    #[test]
    fn infers_missed_side_wall_hit() {
        let prev = make_tick(
            0,
            (0.0, 0.0, 500.0),
            (SOCCAR_HALF_X - SOCCAR_BALL_RADIUS - 5.0, 0.0, 500.0),
            (0.0, 0.0, 0.0),
            (400.0, 0.0, 0.0),
            false,
            false,
            false,
            false,
        );
        let tick = make_tick(
            1,
            (0.0, 0.0, 500.0),
            (SOCCAR_HALF_X - SOCCAR_BALL_RADIUS - 5.0, 0.0, 500.0),
            (0.0, 0.0, 0.0),
            (-400.0, 0.0, 0.0),
            false,
            false,
            false,
            false,
        );
        let labels = classify_tick(&tick, Some(&prev));
        assert!(labels.ball_world);
    }

    #[test]
    fn norm_error_matches_strict_thresholds() {
        let tick = quiet_tick(0, 0.0);
        let truth = snapshot_from_tick(&tick).unwrap();
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
        snaps: Vec<Snapshot>,
        cursor: usize,
    }

    impl MirrorBackend {
        fn new(ticks: &[TickRecord]) -> Self {
            Self {
                snaps: ticks
                    .iter()
                    .map(|tick| snapshot_from_tick(tick).unwrap())
                    .collect(),
                cursor: 0,
            }
        }
    }

    impl ReplayBackend for MirrorBackend {
        fn reset(&mut self, start: &TickRecord) {
            let want = snapshot_from_tick(start).unwrap();
            self.cursor = self
                .snaps
                .iter()
                .position(|snap| snap.car.pos == want.car.pos)
                .unwrap_or(0);
        }

        fn set_state(&mut self, state: &TickRecord) {
            self.reset(state);
        }

        fn step(&mut self, _controls: &ControlsRecord) {
            self.cursor = (self.cursor + 1).min(self.snaps.len() - 1);
        }

        fn snapshot(&mut self) -> Snapshot {
            self.snaps[self.cursor]
        }
    }

    #[test]
    fn evaluate_counts_overlapping_support() {
        let mut ticks: Vec<_> = (0..6).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        ticks[4].car_records[0].is_touching_ball = true;
        ticks[4].car_records[0].wheels[0].has_contact = true;
        let mut backend = MirrorBackend::new(&ticks);
        let segments = vec![Segment { start: 0, len: 6 }];
        let report = evaluate(&mut backend, &ticks, &segments, 1, false);
        assert_eq!(report.total.support, 5);
        assert_eq!(report.total.passed, 5);
        assert_eq!(report.car_ball.support, 1);
        assert_eq!(report.wheel_world.support, 1);
        assert_eq!(report.no_contact.support, 4);

        let mut backend = MirrorBackend::new(&ticks);
        let report = evaluate(&mut backend, &ticks, &segments, 3, true);
        assert_eq!(report.total.support, 5);
        assert_eq!(report.total.passed, 5);
    }
}
