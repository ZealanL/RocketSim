//! Re-record a Daizen-vs-Daizen RLPR stream with RocketSim v3 Rust.
//!
//! The source tape contains the controls emitted by the two DaizenBotV1
//! players.  This example uses those controls as the policy stream, runs the
//! physics transitions in the Rust v3 engine, and writes a fresh RLPR file
//! containing the v3-generated state frames.

use std::{
    env,
    fs::{self, File},
    io::{self, BufWriter, Write},
    mem::size_of,
    path::{Path, PathBuf},
};

use glam::{Mat3A, Vec3A};
use rocketsim::{
    Arena, CarBodyConfig, CarControls, CarState, GameMode, PhysState, Team, consts::TICK_RATE,
    init_from_default,
};

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

#[repr(u8)]
#[derive(Copy, Clone)]
enum ImpulseRecordType {
    WheelsSuspension = 0,
}

#[repr(C)]
#[derive(Copy, Clone)]
struct ImpulseRecord {
    lin_impulse: VecRecord,
    ang_impulse: VecRecord,
    impulse_type: ImpulseRecordType,
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
struct SourceTick {
    cars: Vec<CarRecord>,
    ball: PhysRecord,
}

struct SourceRecording {
    info: RecInfo,
    ticks: Vec<SourceTick>,
}

struct Reader {
    bytes: Vec<u8>,
    offset: usize,
}

impl Reader {
    fn new(bytes: Vec<u8>) -> Self {
        Self { bytes, offset: 0 }
    }

    fn take(&mut self, count: usize) -> &[u8] {
        let end = self
            .offset
            .checked_add(count)
            .expect("RLPR offset overflow");
        assert!(end <= self.bytes.len(), "truncated RLPR recording");
        let start = self.offset;
        self.offset = end;
        &self.bytes[start..end]
    }

    fn u8(&mut self) -> u8 {
        self.take(1)[0]
    }

    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take(4).try_into().unwrap())
    }

    fn pod<T: Copy>(&mut self) -> T {
        let bytes = self.take(size_of::<T>());
        // All RLPR structs are #[repr(C)] and the file is little-endian.
        unsafe { std::ptr::read_unaligned(bytes.as_ptr().cast::<T>()) }
    }
}

fn read_size_prefixed<T: Copy>(reader: &mut Reader) -> T {
    let serialized_size = reader.u32() as usize;
    assert_eq!(
        serialized_size,
        size_of::<T>(),
        "RLPR struct-size mismatch (serialized {serialized_size}, local {})",
        size_of::<T>()
    );
    reader.pod()
}

fn read_source(path: &Path) -> SourceRecording {
    assert_eq!(size_of::<PhysRecord>(), 332);
    assert_eq!(size_of::<CarRecord>(), 584);
    assert_eq!(size_of::<ControlsRecord>(), 24);
    assert_eq!(size_of::<RecInfo>(), 28);

    let mut reader = Reader::new(
        fs::read(path)
            .unwrap_or_else(|e| panic!("cannot read source RLPR {}: {e}", path.display())),
    );
    assert_eq!(reader.take(4), b"RLPR", "source is not an RLPR file");
    assert_eq!(reader.u8(), 0, "source RLPR is not little-endian");
    assert_eq!(reader.u32(), 2, "source RLPR version is not 2");
    assert_eq!(reader.u32() as usize, size_of::<RecInfo>());
    let info: RecInfo = reader.pod();
    assert_eq!(
        info.num_cars, 2,
        "Daizen self-play source must contain two cars"
    );

    let num_ticks = reader.u32() as usize;
    let mut ticks = Vec::with_capacity(num_ticks);
    for _ in 0..num_ticks {
        let mut cars = Vec::with_capacity(info.num_cars as usize);
        for _ in 0..info.num_cars {
            cars.push(read_size_prefixed(&mut reader));
        }
        let ball = read_size_prefixed(&mut reader);
        ticks.push(SourceTick { cars, ball });
    }
    assert_eq!(
        reader.offset,
        reader.bytes.len(),
        "source RLPR has trailing bytes"
    );
    SourceRecording { info, ticks }
}

fn vec_record(v: Vec3A) -> VecRecord {
    VecRecord {
        x: v.x,
        y: v.y,
        z: v.z,
    }
}

fn vec3(v: VecRecord) -> Vec3A {
    Vec3A::new(v.x, v.y, v.z)
}

fn phys_from_record(record: PhysRecord) -> PhysState {
    PhysState {
        pos: vec3(record.pos),
        rot_mat: Mat3A::from_cols(
            vec3(VecRecord {
                x: record.rot.rows[0].x,
                y: record.rot.rows[1].x,
                z: record.rot.rows[2].x,
            }),
            vec3(VecRecord {
                x: record.rot.rows[0].y,
                y: record.rot.rows[1].y,
                z: record.rot.rows[2].y,
            }),
            vec3(VecRecord {
                x: record.rot.rows[0].z,
                y: record.rot.rows[1].z,
                z: record.rot.rows[2].z,
            }),
        ),
        vel: vec3(record.lin_vel),
        ang_vel: vec3(record.ang_vel),
    }
}

fn controls_from_record(record: ControlsRecord) -> CarControls {
    CarControls {
        throttle: record.throttle,
        steer: record.steer,
        pitch: record.pitch,
        yaw: record.yaw,
        roll: record.roll,
        jump: record.jump,
        boost: record.boost,
        handbrake: record.handbrake,
    }
}

fn controls_record(controls: CarControls) -> ControlsRecord {
    ControlsRecord {
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

fn car_state_from_source(record: &CarRecord, controls: CarControls) -> CarState {
    let mut state = CarState::default();
    state.phys = phys_from_record(record.phys);
    state.controls = controls;
    state.prev_controls = controls_from_record(record.prev_controls);
    state.is_on_ground = record.is_on_ground;
    state.is_jumping = record.is_jumping;
    state.is_flipping = record.is_flipping;
    state.jump_ticks = (record.jump_time * TICK_RATE).round() as u32;
    state.flip_time = record.flip_time;
    state.has_jumped = record.has_jumped;
    state.has_double_jumped = record.double_jumped_or_flipped;
    state.has_flipped = record.has_flip;
    state.flip_rel_torque = vec3(record.flip_rel_torque);
    state.boost = record.boost_amount;
    state.wheels_with_contact = record.wheels.map(|wheel| wheel.has_contact);
    state.world_contact_normal = record
        .phys
        .has_world_contact
        .then(|| vec3(record.phys.world_contact_normal));
    state
}

fn set_source_state(
    arena: &mut Arena,
    car_ids: &[usize],
    tick: &SourceTick,
    controls: &[CarControls],
) {
    for (i, &car_id) in car_ids.iter().enumerate() {
        arena.set_car_state(car_id, car_state_from_source(&tick.cars[i], controls[i]));
    }
    let mut ball = *arena.get_ball_state();
    ball.phys = phys_from_record(tick.ball);
    arena.set_ball_state(ball);
}

fn empty_phys(frame: u32, state: &PhysState) -> PhysRecord {
    let f = state.rot_mat.x_axis;
    let r = state.rot_mat.y_axis;
    let u = state.rot_mat.z_axis;
    let mut record: PhysRecord = unsafe { std::mem::zeroed() };
    record.physics_frame = frame;
    record.pos = vec_record(state.pos);
    record.rot = Mat3Record {
        rows: [
            VecRecord {
                x: f.x,
                y: r.x,
                z: u.x,
            },
            VecRecord {
                x: f.y,
                y: r.y,
                z: u.y,
            },
            VecRecord {
                x: f.z,
                y: r.z,
                z: u.z,
            },
        ],
    };
    record.lin_vel = vec_record(state.vel);
    record.ang_vel = vec_record(state.ang_vel);
    record.has_world_contact = false;
    record
}

fn car_record_from_state(arena: &Arena, car_id: usize, frame: u32) -> CarRecord {
    let state = arena.get_car_state(car_id);
    let mut record: CarRecord = unsafe { std::mem::zeroed() };
    record.phys = empty_phys(frame, &state.phys);
    record.is_on_ground = state.is_on_ground;
    record.is_jumping = state.is_jumping;
    record.is_flipping = state.is_flipping;
    record.jump_time = state.jump_time();
    record.flip_time = state.flip_time;
    record.has_jumped = state.has_jumped;
    record.double_jumped_or_flipped = state.has_double_jumped;
    record.has_flip = state.has_flipped;
    record.flip_rel_torque = vec_record(state.flip_rel_torque);
    record.boost_amount = state.boost;
    record.is_touching_ball = false;
    record.prev_controls = controls_record(state.prev_controls);

    for (i, (contact, susp, rel_vel)) in arena.get_car_wheel_debug(car_id).into_iter().enumerate() {
        record.wheels[i].has_contact = contact;
        record.wheels[i].susp_length = susp;
        record.wheels[i].susp_rel_vel = rel_vel;
    }
    record
}

fn write_pod<T: Copy>(writer: &mut BufWriter<File>, value: &T) -> io::Result<()> {
    let bytes =
        unsafe { std::slice::from_raw_parts((value as *const T).cast::<u8>(), size_of::<T>()) };
    writer.write_all(bytes)
}

fn write_u8(writer: &mut BufWriter<File>, value: u8) -> io::Result<()> {
    writer.write_all(&[value])
}

fn write_u32(writer: &mut BufWriter<File>, value: u32) -> io::Result<()> {
    writer.write_all(&value.to_le_bytes())
}

fn write_size_prefixed<T: Copy>(writer: &mut BufWriter<File>, value: &T) -> io::Result<()> {
    write_u32(writer, size_of::<T>() as u32)?;
    write_pod(writer, value)
}

fn write_metadata(
    output: &Path,
    source: &Path,
    source_ticks: usize,
    output_bytes: u64,
    final_frame: u32,
) -> io::Result<()> {
    let directory = output.parent().unwrap_or_else(|| Path::new("."));
    let metadata = directory.join("DaizenVsDaizen_RocketSimV3Rust.md");
    let text = format!(
        "# Daizen vs Daizen — RocketSim v3 Rust\n\n\
Engine: RocketSim v3 Rust (`rocketsim` crate)\n\
Players: Blue DaizenBotV1 vs Orange DaizenBotV1\n\
State frames: simulated and serialized by the Rust v3 engine\n\
Policy controls: copied from the verified Daizen-vs-Daizen source tape `{}`\n\
Source control ticks: {}\n\
Output ticks: {}\n\
Final physics frame: {}\n\
RLPR bytes: {}\n\n\
The source tape supplies the two Daizen action streams; this file replays those\
actions through RocketSim v3 Rust and records the resulting state at every tick.\n",
        source.display(),
        source_ticks,
        source_ticks,
        final_frame,
        output_bytes
    );
    fs::write(metadata, text)
}

fn main() -> io::Result<()> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: daizen_v3_rust_rlpr <source_controls.rlpr> <output.rlpr> [max_ticks]");
        return Ok(());
    }
    let source = PathBuf::from(&args[1]);
    let output = PathBuf::from(&args[2]);
    let max_ticks = args.get(3).map(|value| {
        value
            .parse::<usize>()
            .unwrap_or_else(|_| panic!("invalid max tick count: {value}"))
    });

    let source_recording = read_source(&source);
    let num_ticks = max_ticks
        .unwrap_or(source_recording.ticks.len())
        .min(source_recording.ticks.len());
    assert!(num_ticks >= 2, "recording must contain at least two ticks");

    init_from_default(true).expect("RocketSim v3 initialization failed");
    let mut arena = Arena::new(GameMode::Soccar);
    let car_ids = vec![
        arena.add_car(Team::Blue, CarBodyConfig::OCTANE),
        arena.add_car(Team::Orange, CarBodyConfig::OCTANE),
    ];

    // Match the v3 comparison harness: one warm-up establishes contact caches,
    // then the initial source state is restored before the first real action.
    let warmup_controls: Vec<CarControls> = source_recording.ticks[0]
        .cars
        .iter()
        .map(|car| controls_from_record(car.prev_controls))
        .collect();
    set_source_state(
        &mut arena,
        &car_ids,
        &source_recording.ticks[0],
        &warmup_controls,
    );
    arena.step_tick();

    let first_controls: Vec<CarControls> = source_recording.ticks[1]
        .cars
        .iter()
        .map(|car| controls_from_record(car.prev_controls))
        .collect();
    set_source_state(
        &mut arena,
        &car_ids,
        &source_recording.ticks[0],
        &first_controls,
    );

    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)?;
    }
    let temp = output.with_extension("rlpr.tmp");
    let file = File::create(&temp)?;
    let mut writer = BufWriter::with_capacity(1 << 20, file);
    writer.write_all(b"RLPR")?;
    write_u8(&mut writer, 0)?;
    write_u32(&mut writer, 2)?;
    write_u32(&mut writer, size_of::<RecInfo>() as u32)?;
    write_pod(&mut writer, &source_recording.info)?;
    write_u32(&mut writer, num_ticks as u32)?;

    let mut frame = 0u32;
    for i in 0..num_ticks {
        for &car_id in &car_ids {
            let record = car_record_from_state(&arena, car_id, frame);
            write_size_prefixed(&mut writer, &record)?;
        }
        let ball_record = empty_phys(frame, &arena.get_ball_state().phys);
        write_size_prefixed(&mut writer, &ball_record)?;

        if i + 1 == num_ticks {
            break;
        }

        let next_controls: Vec<CarControls> = source_recording.ticks[i + 1]
            .cars
            .iter()
            .map(|car| controls_from_record(car.prev_controls))
            .collect();
        for (idx, &car_id) in car_ids.iter().enumerate() {
            arena.set_car_controls(car_id, next_controls[idx]);
        }

        let source_frame = source_recording.ticks[i].cars[0].phys.physics_frame;
        let next_source_frame = source_recording.ticks[i + 1].cars[0].phys.physics_frame;
        let frame_delta = next_source_frame.saturating_sub(source_frame);
        if frame_delta == 1 {
            arena.step_tick();
        } else {
            // Goal/reset transitions are represented by a deliberate frame
            // gap in RLPR. Keep the gap and start a fresh Rust kickoff.
            arena.reset_to_random_kickoff(Some(0xDA1CE_u64.wrapping_add(i as u64)));
            for (idx, &car_id) in car_ids.iter().enumerate() {
                arena.set_car_controls(car_id, next_controls[idx]);
            }
        }
        frame = frame.wrapping_add(frame_delta.max(1));

        if (i + 1) % 1200 == 0 || i + 1 == num_ticks - 1 {
            eprintln!("recorded {}/{} ticks", i + 1, num_ticks);
        }
    }

    writer.flush()?;
    drop(writer);
    if output.exists() {
        fs::remove_file(&output)?;
    }
    fs::rename(&temp, &output)?;
    let bytes = fs::metadata(&output)?.len();
    write_metadata(&output, &source, source_recording.ticks.len(), bytes, frame)?;
    eprintln!(
        "finished: {} ticks, {} bytes, final physics frame {}, output {}",
        num_ticks,
        bytes,
        frame,
        output.display()
    );
    Ok(())
}
