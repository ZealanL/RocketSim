use glam::Vec3A;
use rocketsim_test::rlpr::{
    cpp_records::{CarRecord, ControlsRecord, Mat3Record, PhysRecord, VecRecord, WheelRecord},
    tick_record::TickRecord,
};

use crate::common::{
    BodySnapshot, ReplayBackend, SegmentConfig, SimContactEvents, Snapshot, snapshot_from_tick,
};

/// Build a minimal one-car physics tick for restore tests.
///
/// The car is grounded at `car_pos` with neutral controls and zero velocity.
/// All other recorded fields are zero. Use it to check which fields a state
/// setter changes and which fields it leaves alone.
#[cfg(test)]
pub fn restore_test_tick(frame: u32, car_pos: Vec3A, ball_pos: Vec3A) -> TickRecord {
    use rocketsim_test::rlpr::{
        ROW_FLAGS_BALL_AVAILABLE,
        cpp_records::{CarRecord, Mat3Record, PhysRecord, VecRecord, WheelRecord},
    };

    fn vec3(value: Vec3A) -> VecRecord {
        VecRecord::new(value.x, value.y, value.z)
    }

    let rot = Mat3Record {
        rows: [vec3(Vec3A::X), vec3(Vec3A::Y), vec3(Vec3A::Z)],
    };
    let mut car_phys: PhysRecord = unsafe { std::mem::zeroed() };
    car_phys.physics_frame = frame;
    car_phys.pos = vec3(car_pos);
    car_phys.rot = rot;
    let mut ball_phys: PhysRecord = unsafe { std::mem::zeroed() };
    ball_phys.physics_frame = frame;
    ball_phys.pos = vec3(ball_pos);
    ball_phys.rot = rot;

    let mut wheel: WheelRecord = unsafe { std::mem::zeroed() };
    wheel.contact_normal = vec3(Vec3A::Z);
    wheel.has_contact = true;
    let mut car: CarRecord = unsafe { std::mem::zeroed() };
    car.phys = car_phys;
    car.is_on_ground = true;
    car.wheels = [wheel; 4];
    car.physics_available = 1;

    TickRecord {
        level_tick: u64::from(frame),
        row_flags: ROW_FLAGS_BALL_AVAILABLE,
        reserved0: 0,
        car_records: vec![car],
        ball_record: ball_phys,
        replay_state: None,
    }
}

pub fn vec(x: f32, y: f32, z: f32) -> VecRecord {
    VecRecord::new(x, y, z)
}

pub fn ident_rot() -> Mat3Record {
    Mat3Record {
        rows: [vec(1.0, 0.0, 0.0), vec(0.0, 1.0, 0.0), vec(0.0, 0.0, 1.0)],
    }
}

pub fn blank_phys() -> PhysRecord {
    let mut phys: PhysRecord = unsafe { std::mem::zeroed() };
    phys.rot = ident_rot();
    phys
}

pub fn blank_car() -> CarRecord {
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
        last_ball_touch_frame: rocketsim_test::rlpr::TOUCH_FRAME_UNKNOWN,
        last_demolish_frame: rocketsim_test::rlpr::DEMOLISH_FRAME_UNKNOWN,
        physics_available: 1,
        _availability_pad: [0; 3],
    }
}

#[allow(clippy::too_many_arguments)]
pub fn make_tick(
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
        level_tick: u64::from(frame),
        row_flags: rocketsim_test::rlpr::ROW_FLAGS_BALL_AVAILABLE,
        reserved0: 0,
        car_records: vec![car],
        ball_record: ball,
        replay_state: None,
    }
}

pub fn quiet_tick(frame: u32, x: f32) -> TickRecord {
    // Physically self-consistent: the car advances 10 uu per tick, so its
    // velocity is 10 uu per tick length (1200 uu/s at 120 Hz); the ball
    // is stationary.
    make_tick(
        frame,
        (x, 0.0, 100.0),
        (0.0, 0.0, 500.0),
        (1200.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        false,
        false,
        false,
        false,
    )
}

pub fn config(segment_ticks: usize) -> SegmentConfig {
    SegmentConfig { segment_ticks }
}

/// One tick with `count` cars. Slots outside `mask` are placeholders.
pub fn multi_tick(frame: u32, base_x: f32, count: usize, mask: u32) -> TickRecord {
    let mut tick = quiet_tick(frame, base_x);
    tick.car_records = (0..count)
        .map(|slot| {
            let available = mask & (1 << slot) != 0;
            let mut car = blank_car();
            car.physics_available = u8::from(available);
            car.phys.physics_frame = if available {
                frame
            } else {
                rocketsim_test::rlpr::NATIVE_FRAME_UNKNOWN
            };
            car.phys.pos = vec(base_x + slot as f32 * 10.0, 0.0, 100.0);
            car.phys.lin_vel = vec(1200.0, 0.0, 0.0);
            car
        })
        .collect();
    tick
}

/// A guard row: timestamp only, no physics for any slot or the ball.
pub fn guard_tick(level_tick: u64, count: usize) -> TickRecord {
    let mut tick = multi_tick(0, 0.0, count, 0);
    tick.level_tick = level_tick;
    tick.row_flags = rocketsim_test::rlpr::ROW_FLAGS_NO_PHYSICS;
    tick.ball_record = blank_phys();
    tick.ball_record.physics_frame = rocketsim_test::rlpr::NATIVE_FRAME_UNKNOWN;
    for car in &mut tick.car_records {
        car.phys = blank_phys();
        car.phys.physics_frame = rocketsim_test::rlpr::NATIVE_FRAME_UNKNOWN;
    }
    tick
}
/// Backend with no history. Support counts still increment; passes do not.
pub struct ConstantBackend;

impl ReplayBackend for ConstantBackend {
    fn reset(&mut self, _start: &TickRecord) {}
    fn set_state(&mut self, _state: &TickRecord) {}
    fn step(&mut self, controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
        vec![SimContactEvents::default(); controls.len()]
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
pub struct MirrorBackend {
    pub snaps: Vec<Vec<Snapshot>>,
    pub cursor: usize,
    pub supports_cooldown: bool,
    pub suppress_calls: Vec<(usize, bool)>,
    pub supports_bump: bool,
    pub bump_calls: Vec<(usize, usize, f32)>,
    pub controls: Vec<Vec<ControlsRecord>>,
}

pub struct BoostProbe {
    pub boost_calls: Vec<(usize, bool, f32)>,
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
pub struct HandbrakeProbe {
    pub brake_calls: Vec<(usize, f32)>,
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
impl MirrorBackend {
    pub fn new(ticks: &[TickRecord]) -> Self {
        // Mirror every car slot seen anywhere in the run. Slots missing
        // on a tick fall back to car 0's snapshot there (uniform runs
        // never trigger the fallback, so their snaps are unchanged).
        let num_cars = ticks
            .iter()
            .map(|tick| tick.car_records.len())
            .max()
            .unwrap_or(1)
            .max(1);
        let car_zero: Vec<Snapshot> = ticks
            .iter()
            .map(|tick| snapshot_from_tick(tick, 0).unwrap())
            .collect();
        let snaps = (0..num_cars)
            .map(|car_idx| {
                ticks
                    .iter()
                    .enumerate()
                    .map(|(i, tick)| snapshot_from_tick(tick, car_idx).unwrap_or(car_zero[i]))
                    .collect()
            })
            .collect();
        Self {
            snaps,
            cursor: 0,
            supports_cooldown: false,
            suppress_calls: Vec::new(),
            supports_bump: false,
            bump_calls: Vec::new(),
            controls: Vec::new(),
        }
    }
}

impl ReplayBackend for MirrorBackend {
    fn supports_cooldown_restore(&self) -> bool {
        self.supports_cooldown
    }

    fn suppress_next_extra_hit(&mut self, car_idx: usize, suppress: bool) {
        self.suppress_calls.push((car_idx, suppress));
    }

    fn supports_bump_restore(&self) -> bool {
        self.supports_bump
    }

    fn set_bump_cooldown(&mut self, car_idx: usize, victim_idx: usize, seconds: f32) {
        self.bump_calls.push((car_idx, victim_idx, seconds));
    }

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

    fn step(&mut self, controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
        self.controls.push(controls.to_vec());
        self.cursor = (self.cursor + 1).min(self.snaps[0].len() - 1);
        vec![SimContactEvents::default(); self.snaps.len()]
    }

    fn snapshot(&mut self, car_idx: usize) -> Snapshot {
        self.snaps[car_idx][self.cursor]
    }
}
/// One consistent step: position delta matches velocity over TICK_TIME.
pub fn steady_tick(frame: u32, car_x: f32, car_vx: f32) -> TickRecord {
    make_tick(
        frame,
        (car_x, 0.0, 100.0),
        (0.0, 0.0, 500.0),
        (car_vx, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        false,
        false,
        false,
        false,
    )
}

/// One car moving steadily, ball kicked on the target tick, touch frame
/// supplied per car. Cars stay drift-clean so only the ball speaks.
pub fn kick_tick(
    frame: u32,
    car_x: f32,
    ball_vel: (f32, f32, f32),
    touch_frame: u32,
) -> TickRecord {
    let mut tick = steady_tick(frame, car_x, 1200.0);
    tick.ball_record.lin_vel = vec(ball_vel.0, ball_vel.1, ball_vel.2);
    tick.car_records[0].last_ball_touch_frame = touch_frame;
    tick
}
pub struct StickyGateProbe {
    pub snaps: Vec<Vec<Snapshot>>,
    pub cursor: usize,
    pub refreshes: usize,
    pub events: Vec<String>,
}

impl StickyGateProbe {
    pub fn new(ticks: &[TickRecord]) -> Self {
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

    fn set_jump_hold_broken(&mut self, _car_idx: usize, _broken: bool) {
        self.events.push("jump".to_string());
    }

    fn restore_body_state(&mut self, _state: &TickRecord) {
        self.events.push("body".to_string());
    }

    fn restore_auxiliary_state(&mut self, _ticks: &[TickRecord], row: usize) {
        assert_eq!(
            row, self.cursor,
            "restore must use the state row, not the target"
        );
        self.events.push("auxiliary".to_string());
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
/// Jump history: each entry is `(is_jumping, prev_controls.jump)`.
/// `prev_controls[u]` drove step u-1 -> u, matching the recorder phase.
pub fn jump_history(flags: &[(bool, bool)]) -> Vec<TickRecord> {
    flags
        .iter()
        .enumerate()
        .map(|(i, &(jumping, held))| {
            let mut tick = quiet_tick(i as u32, i as f32 * 10.0);
            tick.car_records[0].is_jumping = jumping;
            tick.car_records[0].prev_controls.jump = held;
            tick
        })
        .collect()
}
/// Captures `set_jump_hold_broken` writes (never fires otherwise).
pub struct LatchProbe {
    pub calls: Vec<(usize, bool)>,
}

impl ReplayBackend for LatchProbe {
    fn reset(&mut self, _start: &TickRecord) {}
    fn set_state(&mut self, _state: &TickRecord) {}
    fn set_jump_hold_broken(&mut self, car_idx: usize, broken: bool) {
        self.calls.push((car_idx, broken));
    }
    fn step(&mut self, _controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
        Vec::new()
    }
    fn snapshot(&mut self, _car_idx: usize) -> Snapshot {
        Snapshot {
            car: BodySnapshot {
                pos: Vec3A::ZERO,
                vel: Vec3A::ZERO,
                ang_vel: Vec3A::ZERO,
                forward: Vec3A::X,
                up: Vec3A::Z,
            },
            ball: BodySnapshot {
                pos: Vec3A::ZERO,
                vel: Vec3A::ZERO,
                ang_vel: Vec3A::ZERO,
                forward: Vec3A::X,
                up: Vec3A::Z,
            },
        }
    }
}
pub fn blank_snapshot() -> Snapshot {
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
/// Schedule probe for motion-only mode: mirrors truth for scoring and
/// counts every restoration call.
pub struct MotionScheduleProbe {
    pub snaps: Vec<Vec<Snapshot>>,
    pub level_ticks: Vec<u64>,
    pub cursor: usize,
    pub resets: usize,
    pub set_states: usize,
    pub motions: usize,
    pub body_restores: usize,
    pub boost_calls: Vec<(usize, bool, f32)>,
    pub brake_calls: Vec<(usize, f32)>,
    pub suppress_calls: Vec<(usize, bool)>,
    pub bump_calls: Vec<(usize, usize, f32)>,
    pub jump_calls: Vec<(usize, bool)>,
    pub refreshes: usize,
    pub steps: usize,
    pub reset_rows: Vec<usize>,
    pub step_controls: Vec<Vec<ControlsRecord>>,
    pub supports_cooldown: bool,
    pub supports_bump: bool,
}

impl MotionScheduleProbe {
    pub fn new(ticks: &[TickRecord]) -> Self {
        let num_cars = ticks
            .iter()
            .map(|tick| tick.car_records.len())
            .max()
            .unwrap_or(1)
            .max(1);
        let car_zero: Vec<Snapshot> = ticks
            .iter()
            .map(|tick| snapshot_from_tick(tick, 0).unwrap_or_else(blank_snapshot))
            .collect();
        let snaps = (0..num_cars)
            .map(|car_idx| {
                ticks
                    .iter()
                    .enumerate()
                    .map(|(i, tick)| snapshot_from_tick(tick, car_idx).unwrap_or(car_zero[i]))
                    .collect()
            })
            .collect();
        Self {
            snaps,
            level_ticks: ticks.iter().map(|tick| tick.level_tick).collect(),
            cursor: 0,
            resets: 0,
            set_states: 0,
            motions: 0,
            body_restores: 0,
            boost_calls: Vec::new(),
            brake_calls: Vec::new(),
            suppress_calls: Vec::new(),
            bump_calls: Vec::new(),
            jump_calls: Vec::new(),
            refreshes: 0,
            steps: 0,
            reset_rows: Vec::new(),
            step_controls: Vec::new(),
            supports_cooldown: true,
            supports_bump: true,
        }
    }

    pub fn seek(&mut self, state: &TickRecord) {
        // Level ticks are unique in these tests. A frozen row repeats
        // the previous pose, so a position match would be ambiguous.
        if let Some(index) = self
            .level_ticks
            .iter()
            .position(|&level_tick| level_tick == state.level_tick)
        {
            self.cursor = index;
            return;
        }
        let want = snapshot_from_tick(state, 0).unwrap();
        self.cursor = self.snaps[0]
            .iter()
            .position(|snap| snap.car.pos == want.car.pos)
            .unwrap_or(0);
    }
}

impl ReplayBackend for MotionScheduleProbe {
    fn reset(&mut self, start: &TickRecord) {
        self.resets += 1;
        self.seek(start);
        self.reset_rows.push(self.cursor);
    }

    fn set_state(&mut self, state: &TickRecord) {
        self.set_states += 1;
        self.seek(state);
    }

    fn reset_motion(&mut self, state: &TickRecord) {
        self.motions += 1;
        self.seek(state);
    }

    fn set_boost_state(&mut self, car_idx: usize, armed: bool, time: f32) {
        self.boost_calls.push((car_idx, armed, time));
    }

    fn set_handbrake_value(&mut self, car_idx: usize, value: f32) {
        self.brake_calls.push((car_idx, value));
    }

    fn supports_cooldown_restore(&self) -> bool {
        self.supports_cooldown
    }

    fn suppress_next_extra_hit(&mut self, car_idx: usize, suppress: bool) {
        self.suppress_calls.push((car_idx, suppress));
    }

    fn supports_bump_restore(&self) -> bool {
        self.supports_bump
    }

    fn set_bump_cooldown(&mut self, car_idx: usize, victim_idx: usize, seconds: f32) {
        self.bump_calls.push((car_idx, victim_idx, seconds));
    }

    fn set_jump_hold_broken(&mut self, car_idx: usize, broken: bool) {
        self.jump_calls.push((car_idx, broken));
    }

    fn refresh_sticky_gates(&mut self) {
        self.refreshes += 1;
    }

    fn restore_body_state(&mut self, _state: &TickRecord) {
        self.body_restores += 1;
    }

    fn step(&mut self, controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
        self.steps += 1;
        self.step_controls.push(controls.to_vec());
        self.cursor = (self.cursor + 1).min(self.snaps[0].len() - 1);
        vec![SimContactEvents::default(); self.snaps.len()]
    }

    fn snapshot(&mut self, car_idx: usize) -> Snapshot {
        self.snaps[car_idx][self.cursor]
    }
}
/// Five-row two-car sequence: lead, epsilon stasis, last countdown row,
/// immediate live release, live continuation. No exact-frozen separator.
pub fn stasis_release_ticks() -> Vec<TickRecord> {
    [99u32, 100, 101, 102, 103]
        .iter()
        .map(|&frame| {
            let x = match frame {
                99 => 0.0,
                100 => 10.0,
                101 => 10.5,
                102 => 20.0,
                _ => 30.0,
            };
            let (ball_world, hxy) = if frame == 100 || frame == 101 {
                (true, 0.0)
            } else {
                (false, 200.0)
            };
            let vel = match frame {
                100 => 1200.0,
                102 => 1140.0,
                103 => 1200.0,
                _ => hxy,
            };
            let mut tick = make_tick(
                frame,
                (x, 0.0, 17.0),
                (0.0, 0.0, 92.75),
                (hxy, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                false,
                ball_world,
                true,
                true,
            );
            tick.car_records[0].phys.lin_vel = vec(vel, 0.0, 0.0);
            // Distinct controls per row and car prove that the release
            // step uses the target row controls.
            tick.car_records[0].prev_controls.throttle = frame as f32 / 1000.0;
            tick.car_records[0].prev_controls.steer = -(frame as f32) / 2000.0;
            let mut peer = tick.car_records[0];
            peer.prev_controls.throttle = -(frame as f32) / 1000.0;
            peer.prev_controls.steer = frame as f32 / 3000.0;
            tick.car_records.push(peer);
            tick
        })
        .collect()
}

pub fn target_controls(tick: &TickRecord) -> Vec<ControlsRecord> {
    tick.car_records
        .iter()
        .map(|car| car.prev_controls)
        .collect()
}
