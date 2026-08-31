use std::{collections::HashSet, env, fs, mem::size_of};

use glam::{Mat3A, Vec3A};
use rocketsim::consts::{TICK_RATE, TICK_TIME};
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, PhysState, Team};

#[repr(C)]
#[derive(Copy, Clone)]
struct VecRecord {
    x: f32,
    y: f32,
    z: f32,
}
#[repr(C)]
#[derive(Copy, Clone)]
struct Mat3Record {
    rows: [VecRecord; 3],
}
impl Mat3Record {
    fn column(&self, idx: usize) -> VecRecord {
        VecRecord {
            x: self.rows[0].x * (idx == 0) as u8 as f32
                + self.rows[0].y * (idx == 1) as u8 as f32
                + self.rows[0].z * (idx == 2) as u8 as f32,
            y: self.rows[1].x * (idx == 0) as u8 as f32
                + self.rows[1].y * (idx == 1) as u8 as f32
                + self.rows[1].z * (idx == 2) as u8 as f32,
            z: self.rows[2].x * (idx == 0) as u8 as f32
                + self.rows[2].y * (idx == 1) as u8 as f32
                + self.rows[2].z * (idx == 2) as u8 as f32,
        }
    }
}
#[repr(C)]
#[derive(Copy, Clone)]
struct WheelRecord {
    susp_length: f32,
    susp_rel_vel: f32,
    has_contact: bool,
    contact_normal: VecRecord,
    steer_amount: f32,
    engine_force: f32,
    brake: f32,
    lat_friction: f32,
    long_friction: f32,
    extra_pushback: f32,
}
#[repr(C)]
#[derive(Copy, Clone)]
struct ControlsRecord {
    throttle: f32,
    steer: f32,
    pitch: f32,
    yaw: f32,
    roll: f32,
    jump: bool,
    boost: bool,
    handbrake: bool,
}
#[repr(C)]
#[derive(Copy, Clone)]
struct ImpulseRecord {
    lin: VecRecord,
    ang: VecRecord,
    impulse_type: u8,
    is_accum: bool,
}
#[repr(C)]
#[derive(Copy, Clone)]
struct PhysRecord {
    physics_frame: u32,
    pos: VecRecord,
    rot: Mat3Record,
    lin_vel: VecRecord,
    ang_vel: VecRecord,
    has_world_contact: bool,
    world_contact_point: VecRecord,
    world_contact_normal: VecRecord,
    impulse_records_data: [ImpulseRecord; 8],
    num_impulse_records: u32,
}
#[repr(C)]
#[derive(Copy, Clone)]
struct CarRecord {
    phys: PhysRecord,
    is_on_ground: bool,
    is_jumping: bool,
    is_flipping: bool,
    jump_time: f32,
    flip_time: f32,
    has_jumped: bool,
    double_jumped_or_flipped: bool,
    has_flip: bool,
    flip_rel_torque: VecRecord,
    boost_amount: f32,
    is_touching_ball: bool,
    prev_controls: ControlsRecord,
    wheels: [WheelRecord; 4],
}
#[repr(C)]
#[derive(Copy, Clone)]
struct RecInfo {
    num_cars: u32,
    hitbox_rel_min_bt: VecRecord,
    hitbox_rel_max_bt: VecRecord,
}
#[derive(Clone)]
struct Tick {
    cars: Vec<CarRecord>,
    ball: PhysRecord,
}
struct Recording {
    ticks: Vec<Tick>,
    num_cars: usize,
}

struct Reader {
    bytes: Vec<u8>,
    off: usize,
}
impl Reader {
    fn new(bytes: Vec<u8>) -> Self {
        Self { bytes, off: 0 }
    }
    fn bytes(&mut self, n: usize) -> &[u8] {
        let s = self.off;
        self.off += n;
        &self.bytes[s..s + n]
    }
    fn u8(&mut self) -> u8 {
        self.bytes(1)[0]
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.bytes(4).try_into().unwrap())
    }
    fn read<T: Copy>(&mut self) -> T {
        assert!(self.off + size_of::<T>() <= self.bytes.len());
        let value =
            unsafe { std::ptr::read_unaligned(self.bytes.as_ptr().add(self.off) as *const T) };
        self.off += size_of::<T>();
        value
    }
}

fn parse(path: &str) -> Recording {
    assert_eq!(size_of::<PhysRecord>(), 332);
    assert_eq!(size_of::<CarRecord>(), 584);
    assert_eq!(size_of::<ControlsRecord>(), 24);
    let mut r = Reader::new(fs::read(path).expect("read RLPR"));
    assert_eq!(r.bytes(4), b"RLPR");
    assert_eq!(r.u8(), 0);
    assert_eq!(r.u32(), 2);
    assert_eq!(r.u32(), size_of::<RecInfo>() as u32);
    let info: RecInfo = r.read();
    assert!(info.num_cars > 0 && info.num_cars <= 2);
    let n = r.u32() as usize;
    let mut ticks = Vec::with_capacity(n);
    for _ in 0..n {
        let mut cars = Vec::with_capacity(info.num_cars as usize);
        for _ in 0..info.num_cars {
            assert_eq!(r.u32(), size_of::<CarRecord>() as u32);
            cars.push(r.read());
        }
        assert_eq!(r.u32(), size_of::<PhysRecord>() as u32);
        let ball = r.read();
        ticks.push(Tick { cars, ball });
    }
    assert_eq!(r.off, r.bytes.len());
    Recording {
        ticks,
        num_cars: info.num_cars as usize,
    }
}

fn v(v: VecRecord) -> Vec3A {
    Vec3A::new(v.x, v.y, v.z)
}
fn phys(p: PhysRecord) -> PhysState {
    PhysState {
        pos: v(p.pos),
        vel: v(p.lin_vel),
        ang_vel: v(p.ang_vel),
        rot_mat: Mat3A::from_cols(
            v(VecRecord {
                x: p.rot.rows[0].x,
                y: p.rot.rows[1].x,
                z: p.rot.rows[2].x,
            }),
            v(VecRecord {
                x: p.rot.rows[0].y,
                y: p.rot.rows[1].y,
                z: p.rot.rows[2].y,
            }),
            v(VecRecord {
                x: p.rot.rows[0].z,
                y: p.rot.rows[1].z,
                z: p.rot.rows[2].z,
            }),
        ),
    }
}
fn controls(c: ControlsRecord) -> CarControls {
    CarControls {
        throttle: c.throttle,
        steer: c.steer,
        pitch: c.pitch,
        yaw: c.yaw,
        roll: c.roll,
        jump: c.jump,
        boost: c.boost,
        handbrake: c.handbrake,
    }
}

fn set_car(arena: &mut Arena, idx: usize, rec: &CarRecord, c: ControlsRecord) {
    let mut s = *arena.get_car_state(idx);
    let p = phys(rec.phys);
    s.phys = p;
    s.is_on_ground = rec.is_on_ground;
    s.is_jumping = rec.is_jumping;
    s.is_flipping = rec.is_flipping;
    s.jump_ticks = (rec.jump_time * TICK_RATE).round() as u32;
    s.flip_time = rec.flip_time;
    s.has_jumped = rec.has_jumped;
    if s.has_flip_or_jump() {
        s.has_double_jumped = rec.double_jumped_or_flipped;
    }
    s.flip_rel_torque = v(rec.flip_rel_torque);
    s.boost = rec.boost_amount;
    s.controls = controls(c);
    arena.set_car_state(idx, s);
}
fn set_ball(arena: &mut Arena, rec: &PhysRecord) {
    let mut s = *arena.get_ball_state();
    s.phys = phys(*rec);
    arena.set_ball_state(s);
}

fn add_arena(num_cars: usize) -> (Arena, Vec<usize>) {
    // V2's default ArenaConfig disables ball rotation tracking. Match that
    // configuration so RLPR comparisons do not treat the visual-only sphere
    // orientation as a physics difference.
    let mut a = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar).with_no_ball_rot(true));
    let mut ids = Vec::with_capacity(num_cars);
    for i in 0..num_cars {
        ids.push(a.add_car(
            if i % 2 == 0 { Team::Blue } else { Team::Orange },
            CarBodyConfig::OCTANE,
        ));
    }
    (a, ids)
}

fn add_vec(s: &mut f64, p: Vec3A, r: VecRecord, t: f32) {
    *s += f64::from((p - v(r)).length() / t).powi(2);
}
fn add_bool(s: &mut f64, p: bool, r: bool) {
    *s += if p == r { 0.0 } else { 1.0 };
}
fn add_float(s: &mut f64, p: f32, r: f32, t: f32) {
    *s += f64::from(((p - r).abs() / t).powi(2));
}
fn compare_phys(sum: &mut f64, p: &PhysState, r: &PhysRecord) {
    add_vec(sum, p.pos, r.pos, 10.0);
    add_vec(sum, p.vel, r.lin_vel, 3.0);
    add_vec(sum, p.ang_vel, r.ang_vel, 1.0);
    add_vec(sum, p.get_forward_dir(), r.rot.column(0), 1.0);
    add_vec(sum, p.get_up_dir(), r.rot.column(2), 1.0);
}
fn compare_tick(arena: &Arena, ids: &[usize], tick: &Tick) -> f32 {
    let mut sum = 0.0;
    for (i, &id) in ids.iter().enumerate() {
        let s = arena.get_car_state(id);
        let r = &tick.cars[i];
        compare_phys(&mut sum, &s.phys, &r.phys);
        add_bool(&mut sum, s.is_on_ground, r.is_on_ground);
        add_bool(&mut sum, s.is_jumping, r.is_jumping);
        add_bool(&mut sum, s.has_jumped, r.has_jumped);
        if s.has_jumped {
            add_float(&mut sum, s.jump_time(), r.jump_time, TICK_TIME * 2.0);
        }
        add_bool(&mut sum, s.is_flipping, r.is_flipping);
        if s.is_flipping {
            add_float(&mut sum, s.flip_time, r.flip_time, TICK_TIME * 2.0);
            add_vec(&mut sum, s.flip_rel_torque, r.flip_rel_torque, 0.05);
        }
    }
    compare_phys(&mut sum, &arena.get_ball_state().phys, &tick.ball);
    sum.sqrt() as f32
}

struct Outcome {
    attempted: usize,
    passed: usize,
    failed: usize,
    first_fail: Option<usize>,
}
fn replay_from(
    rec: &Recording,
    start: usize,
    stop_on_fail: bool,
    reset_each_tick: bool,
) -> Outcome {
    if let Ok(v) = env::var("RSIM_DEBUG_ONLY_START") {
        if v.parse::<usize>().ok() != Some(start) {
            return Outcome {
                attempted: 0,
                passed: 0,
                failed: 0,
                first_fail: None,
            };
        }
    }
    let (mut arena, ids) = add_arena(rec.num_cars);
    let initial = &rec.ticks[start];
    for c in 0..rec.num_cars {
        set_car(
            &mut arena,
            ids[c],
            &initial.cars[c],
            initial.cars[c].prev_controls,
        );
    }
    set_ball(&mut arena, &initial.ball);
    arena.step_tick();
    for c in 0..rec.num_cars {
        set_car(
            &mut arena,
            ids[c],
            &initial.cars[c],
            rec.ticks[start + 1].cars[c].prev_controls,
        );
    }
    set_ball(&mut arena, &initial.ball);
    let mut out = Outcome {
        attempted: 0,
        passed: 0,
        failed: 0,
        first_fail: None,
    };
    for i in start..rec.ticks.len() - 1 {
        let from = &rec.ticks[i];
        let to = &rec.ticks[i + 1];
        if to.cars[0].phys.physics_frame - from.cars[0].phys.physics_frame != 1 {
            break;
        }
        if reset_each_tick {
            for c in 0..rec.num_cars {
                set_car(&mut arena, ids[c], &from.cars[c], to.cars[c].prev_controls);
            }
            set_ball(&mut arena, &from.ball);
        } else {
            for c in 0..rec.num_cars {
                arena.set_car_controls(ids[c], controls(to.cars[c].prev_controls));
            }
        }
        arena.step_tick();
        out.attempted += 1;
        if env::var("RSIM_TRACE_START")
            .ok()
            .and_then(|v| v.parse::<usize>().ok())
            == Some(start)
            && (env::var("RSIM_TRACE_ALL").is_ok() || out.attempted <= 20)
        {
            for (c, &id) in ids.iter().enumerate() {
                let p = arena.get_car_state(id);
                print!("trace,{},{}", i + 1, c);
                for x in [
                    p.phys.pos.x,
                    p.phys.pos.y,
                    p.phys.pos.z,
                    p.phys.vel.x,
                    p.phys.vel.y,
                    p.phys.vel.z,
                    p.phys.ang_vel.x,
                    p.phys.ang_vel.y,
                    p.phys.ang_vel.z,
                    p.phys.rot_mat.x_axis.x,
                    p.phys.rot_mat.x_axis.y,
                    p.phys.rot_mat.x_axis.z,
                    p.phys.rot_mat.z_axis.x,
                    p.phys.rot_mat.z_axis.y,
                    p.phys.rot_mat.z_axis.z,
                ] {
                    print!(",{x:.9}");
                }
                print!(
                    ",{},{},{},{},{:.9},{:.9}",
                    p.is_flipping as u8,
                    p.is_jumping as u8,
                    p.has_jumped as u8,
                    p.has_flipped as u8,
                    p.flip_time,
                    p.air_time_since_jump
                );
                println!();
            }
        }
        if compare_tick(&arena, &ids, to) < 1.0 {
            out.passed += 1;
        } else {
            out.failed += 1;
            if out.first_fail.is_none() {
                out.first_fail = Some(i + 1);
                if env::var("RSIM_DEBUG_START")
                    .ok()
                    .and_then(|v| v.parse::<usize>().ok())
                    == Some(start)
                {
                    println!(
                        "debug_first_fail,start={},transition={},norm={}",
                        start,
                        i,
                        compare_tick(&arena, &ids, to)
                    );
                    for (c, &id) in ids.iter().enumerate() {
                        let p = arena.get_car_state(id);
                        let r = &to.cars[c];
                        println!(
                            "debug_car={},pred_pos={:?},rec_pos={:?},pred_vel={:?},rec_vel={:?},pred_ang={:?},rec_ang={:?},pred_forward={:?},rec_forward={:?},pred_up={:?},rec_up={:?},pred_ground={},rec_ground={},pred_jump={},rec_jump={},pred_has_jump={},rec_has_jump={}",
                            c,
                            p.phys.pos,
                            v(r.phys.pos),
                            p.phys.vel,
                            v(r.phys.lin_vel),
                            p.phys.ang_vel,
                            v(r.phys.ang_vel),
                            p.get_forward_dir(),
                            v(r.phys.rot.column(0)),
                            p.get_up_dir(),
                            v(r.phys.rot.column(2)),
                            p.is_on_ground,
                            r.is_on_ground,
                            p.is_jumping,
                            r.is_jumping,
                            p.has_jumped,
                            r.has_jumped
                        );
                    }
                    let p = arena.get_ball_state();
                    println!(
                        "debug_ball,pred_pos={:?},rec_pos={:?},pred_vel={:?},rec_vel={:?},pred_ang={:?},rec_ang={:?},pred_forward={:?},rec_forward={:?},pred_up={:?},rec_up={:?}",
                        p.phys.pos,
                        v(to.ball.pos),
                        p.phys.vel,
                        v(to.ball.lin_vel),
                        p.phys.ang_vel,
                        v(to.ball.ang_vel),
                        p.get_forward_dir(),
                        v(to.ball.rot.column(0)),
                        p.get_up_dir(),
                        v(to.ball.rot.column(2))
                    );
                }
            }
            if stop_on_fail {
                break;
            }
        }
        if env::var("RSIM_DEBUG_TICKS")
            .ok()
            .and_then(|v| v.parse::<usize>().ok())
            .is_some_and(|n| out.attempted >= n)
        {
            break;
        }
    }
    out
}

fn splitmix(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E3779B97F4A7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
    z ^ (z >> 31)
}
fn starts(limit: usize, count: usize, seed: u64) -> Vec<usize> {
    let mut set = HashSet::new();
    let mut s = seed;
    while set.len() < count.min(limit) {
        set.insert((splitmix(&mut s) as usize) % limit);
    }
    let mut v: Vec<_> = set.into_iter().collect();
    v.sort_unstable();
    v
}
fn percentile(v: &[usize], p: f64) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    let mut x = v.to_vec();
    x.sort_unstable();
    let q = p * (x.len() - 1) as f64;
    let lo = q.floor() as usize;
    let hi = (lo + 1).min(x.len() - 1);
    x[lo] as f64 + (q - lo as f64) * (x[hi] - x[lo]) as f64
}

fn main() {
    let args: Vec<String> = env::args().collect();
    assert!(
        args.len() >= 2,
        "usage: best_random_benchmark <recording.rlpr>"
    );
    rocketsim::init_from_default(true).expect("RocketSim init");
    let rec = parse(&args[1]);
    let seed = 0xD1A2B3C4E5F60718u64;
    let sample_count = env::var("RSIM_SAMPLE_COUNT")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(1000);
    let ss = starts(rec.ticks.len() - 1, sample_count, seed);
    if let Ok(start) = env::var("RSIM_DEBUG_START") {
        println!("debug_start,{}", start);
    }
    println!(
        "engine,v3_rust\nrecording,{}\nticks,{}\ncars,{}\nsample_seed,{}\nsamples,{}",
        args[1],
        rec.ticks.len(),
        rec.num_cars,
        seed,
        ss.len()
    );
    println!(
        "sample,start_record_idx,start_physics_frame,first_divergence_record_idx,tick_before_divergence,ticks_before_divergence,pass_to_end"
    );
    let mut survived = Vec::with_capacity(ss.len());
    let mut sample_pass = 0usize;
    for (n, &start) in ss.iter().enumerate() {
        let o = replay_from(&rec, start, env::var("RSIM_TRACE_START").is_err(), false);
        let pass = o.failed == 0;
        if pass {
            sample_pass += 1;
        }
        survived.push(o.passed);
        let before = o.first_fail.map(|i| i - 1);
        match o.first_fail {
            Some(i) => println!(
                "{n},{start},{},{i},{},{},0",
                rec.ticks[start].cars[0].phys.physics_frame,
                before.unwrap(),
                o.passed
            ),
            None => println!(
                "{n},{start},{},-,-,{},1",
                rec.ticks[start].cars[0].phys.physics_frame, o.passed
            ),
        }
    }
    let full = replay_from(&rec, 0, false, false);
    let pct = 100.0 * full.passed as f64 / full.attempted.max(1) as f64;
    let reset = if env::var("RSIM_SKIP_RESET").is_ok() {
        Outcome {
            attempted: 0,
            passed: 0,
            failed: 0,
            first_fail: None,
        }
    } else {
        replay_from(&rec, 0, false, true)
    };
    let reset_pct = 100.0 * reset.passed as f64 / reset.attempted.max(1) as f64;
    println!(
        "random_mean_ticks_before_divergence,{}",
        survived.iter().sum::<usize>() as f64 / survived.len().max(1) as f64
    );
    println!(
        "random_median_ticks_before_divergence,{}",
        percentile(&survived, 0.5)
    );
    println!(
        "random_p90_ticks_before_divergence,{}",
        percentile(&survived, 0.9)
    );
    println!(
        "random_min_ticks_before_divergence,{}",
        survived.iter().min().unwrap_or(&0)
    );
    println!(
        "random_max_ticks_before_divergence,{}",
        survived.iter().max().unwrap_or(&0)
    );
    println!(
        "random_01_pass_rate,{}",
        100.0 * sample_pass as f64 / ss.len().max(1) as f64
    );
    println!(
        "full_attempted_frames,{}\nfull_passed_frames,{}\nfull_failed_frames,{}\nfull_frame_pass_rate,{}\nfull_recording_01,{}\nreset_attempted_frames,{}\nreset_passed_frames,{}\nreset_failed_frames,{}\nreset_frame_pass_rate,{}\nreset_recording_01,{}",
        full.attempted,
        full.passed,
        full.failed,
        pct,
        if full.failed == 0 { 1 } else { 0 },
        reset.attempted,
        reset.passed,
        reset.failed,
        reset_pct,
        if reset.failed == 0 { 1 } else { 0 }
    );
}
