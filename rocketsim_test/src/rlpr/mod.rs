pub mod cpp_records;
mod data_reader;
pub mod tick_record;

use std::{
    io::{self, ErrorKind, Read},
    mem::size_of,
    path::Path,
};

use cpp_records::*;
use data_reader::DataReader;

use crate::rlpr::tick_record::TickRecord;

const RLPR_MAGIC_BYTES: [u8; 4] = [82, 76, 80, 82];
const RLPR_MIN_VERSION: u32 = 2;
const RLPR_MAX_VERSION: u32 = 11;
const RLPR_MAX_CARS: usize = 8;
/// First version carrying explicit per-car physics availability.
const RLPR_AVAILABILITY_VERSION: u32 = 11;
/// First version carrying recorded boost latch state.
const RLPR_BOOST_STATE_VERSION: u32 = 7;
/// First version carrying recorded handbrake integrator state.
const RLPR_HANDBRAKE_STATE_VERSION: u32 = 8;
/// First version carrying the raw last ball touch frame.
const RLPR_TOUCH_FRAME_VERSION: u32 = 9;
/// First version carrying the event-first last demolition frame.
const RLPR_DEMOLISH_FRAME_VERSION: u32 = 10;
const CAR_RECORD_PREFIX_SIZE: usize = std::mem::offset_of!(CarRecord, wheels);
const WHEEL_CONTACT_OFFSET: usize = std::mem::offset_of!(WheelRecord, has_contact);
/// Legacy v2 file size. Stays explicit because `CarRecord` is larger.
const CAR_RECORD_V2_SIZE: usize = 584;
/// V6 file size: legacy bytes plus trailing bool plus padding.
const CAR_RECORD_V6_SIZE: usize = 588;
/// File offset of the v6 `is_touching_car` byte.
const CAR_TOUCH_OFFSET: usize = 584;
/// V7 file size: 588 legacy bytes plus boost bit, time, and padding.
const CAR_RECORD_V7_SIZE: usize = 596;
/// File offset of the v7 `is_boosting` byte.
const CAR_BOOST_BIT_OFFSET: usize = 588;
/// File offset of the v7 `boosting_time` float.
const CAR_BOOST_TIME_OFFSET: usize = 592;
/// V8 file size: 596 legacy bytes plus handbrake value.
const CAR_RECORD_V8_SIZE: usize = 600;
/// File offset of the v8 `handbrake_val` float.
const CAR_HANDBRAKE_OFFSET: usize = 596;
/// V9 file size: 600 legacy bytes plus raw last ball touch frame.
const CAR_RECORD_V9_SIZE: usize = 604;
/// File offset of the v9 `last_ball_touch_frame` u32.
const CAR_TOUCH_FRAME_OFFSET: usize = 600;
/// V10 file size: 604 legacy bytes plus last demolition event frame.
const CAR_RECORD_V10_SIZE: usize = 608;
/// File offset of the v10 `last_demolish_frame` u32.
const CAR_DEMOLISH_FRAME_OFFSET: usize = 604;
/// V11 file size: v10 bytes plus availability byte and 3 reserved bytes.
const CAR_RECORD_V11_SIZE: usize = 612;
/// File offset of the v11 `physics_available` byte.
const CAR_PHYSICS_AVAILABLE_OFFSET: usize = 608;
/// File offset of the v11 reserved 3 bytes.
const CAR_AVAILABILITY_PAD_OFFSET: usize = 609;
/// V11 row prefix: u64 level_tick, u32 row_flags, u32 reserved0.
pub const ROW_HEADER_V11_SIZE: usize = 16;
/// Row flag bit 0: the shared ball is available.
pub const ROW_FLAGS_BALL_AVAILABLE: u32 = 1 << 0;
/// Row flag bit 1: the row carries no physics (all slots unavailable).
pub const ROW_FLAGS_NO_PHYSICS: u32 = 1 << 1;
const ROW_FLAGS_KNOWN: u32 = ROW_FLAGS_BALL_AVAILABLE | ROW_FLAGS_NO_PHYSICS;
/// Native frame sentinel for an unavailable car or ball.
pub const NATIVE_FRAME_UNKNOWN: u32 = u32::MAX;
/// Absent-field sentinel: no real physics frame reaches this value.
/// Used for `last_ball_touch_frame` on files older than v9.
pub const TOUCH_FRAME_UNKNOWN: u32 = u32::MAX;
/// Sentinel for `last_demolish_frame`. The value means unknown, or no
/// confirmed event was observed and resolved for this slot.
pub const DEMOLISH_FRAME_UNKNOWN: u32 = u32::MAX;

const _: () = assert!(size_of::<CarRecord>() == CAR_RECORD_V11_SIZE);
const _: () = assert!(CAR_DEMOLISH_FRAME_OFFSET + size_of::<u32>() == CAR_RECORD_V10_SIZE);
const _: () = assert!(CAR_PHYSICS_AVAILABLE_OFFSET + 1 == CAR_AVAILABILITY_PAD_OFFSET);
const _: () = assert!(CAR_AVAILABILITY_PAD_OFFSET + 3 == CAR_RECORD_V11_SIZE);

/// True when the recording version carries recorded boost latch state.
///
/// Older versions default `is_boosting`/`boosting_time` to false/0.0 and
/// must keep live-latch evolution instead of forcing those defaults.
pub fn recording_has_boost_state(version: u32) -> bool {
    version >= RLPR_BOOST_STATE_VERSION
}

/// True when the recording version carries recorded handbrake state.
///
/// Older versions default `handbrake_val` to 0.0 and must keep the
/// reconstruction fallback instead of forcing that default.
pub fn recording_has_handbrake_state(version: u32) -> bool {
    version >= RLPR_HANDBRAKE_STATE_VERSION
}

/// True when the recording version carries the raw last ball touch frame.
///
/// Older versions leave `last_ball_touch_frame` at [`TOUCH_FRAME_UNKNOWN`]
/// and must keep the legacy exclusion behavior instead of inferring touch
/// recency from it.
pub fn recording_has_touch_frames(version: u32) -> bool {
    version >= RLPR_TOUCH_FRAME_VERSION
}

/// True when the recording version carries the last demolition event frame.
///
/// This reports field presence only. It does not report hook health, event
/// coverage, or an active demolition state. A `true` result does not mean
/// that any event was observed.
pub fn recording_has_demolish_frames(version: u32) -> bool {
    version >= RLPR_DEMOLISH_FRAME_VERSION
}

/// True when the recording carries explicit per-car physics availability.
///
/// Older versions default every car and the ball to available.
pub fn recording_has_availability(version: u32) -> bool {
    version >= RLPR_AVAILABILITY_VERSION
}

#[allow(dead_code)]
pub struct Recording {
    pub name: String,
    pub version: u32,
    pub info: RecordingInfo,
    pub ticks: Vec<TickRecord>,
}

/// Max decompressed RLPR size. Bounds zstd decode of untrusted files.
const RLPR_MAX_DECODED_SIZE: u64 = 1 << 30;

/// Wrap a zstd failure as an `InvalidData` recording error.
fn decode_failed(path: &Path, err: std::io::Error) -> std::io::Error {
    std::io::Error::new(
        ErrorKind::InvalidData,
        format!("RLPR zstd decode failed for {}: {err}", path.display()),
    )
}

/// Decode one zstd frame with a 1 GiB output bound.
///
/// The streaming decoder rejects windows over 128 MiB by default
/// (`ZSTD_WINDOWLOG_LIMIT_DEFAULT`). Captures compressed with `--long`
/// and `--ultra` exceed that, so allow the library maximum (31 on 64-bit,
/// 30 on 32-bit).
fn decode_bounded(raw: &[u8], path: &Path) -> std::io::Result<Vec<u8>> {
    let mut decoder = zstd::Decoder::new(raw).map_err(|err| decode_failed(path, err))?;
    let max_log = if cfg!(target_pointer_width = "64") {
        31
    } else {
        30
    };
    decoder
        .window_log_max(max_log)
        .map_err(|err| decode_failed(path, err))?;
    let mut out = Vec::new();
    decoder
        .by_ref()
        .take(RLPR_MAX_DECODED_SIZE)
        .read_to_end(&mut out)
        .map_err(|err| decode_failed(path, err))?;
    if out.len() as u64 >= RLPR_MAX_DECODED_SIZE {
        let mut extra = [0u8; 1];
        if decoder
            .read(&mut extra)
            .map_err(|err| decode_failed(path, err))?
            > 0
        {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!(
                    "RLPR zstd output exceeds {} bytes for {}",
                    RLPR_MAX_DECODED_SIZE,
                    path.display()
                ),
            ));
        }
    }
    Ok(out)
}

impl Recording {
    /// Read a plain `.rlpr` file or a compressed `.rlpr.zst` file.
    ///
    /// A compressed `x.rlpr.zst` names the recording `x`, as plain `x.rlpr` does.
    pub fn from_file(path: &Path) -> Result<Recording, std::io::Error> {
        // Compressed recordings end in `.rlpr.zst`. Others stay plain.
        let is_compressed = path.extension().is_some_and(|ext| ext == "zst");
        let raw = std::fs::read(path)?;
        let bytes = if is_compressed {
            decode_bounded(&raw, path)?
        } else {
            raw
        };
        let logical_path;
        let name_path = if is_compressed {
            let stem = path.file_stem().ok_or_else(|| {
                std::io::Error::new(
                    ErrorKind::InvalidInput,
                    "Recording path has no valid file name",
                )
            })?;
            logical_path = Path::new(stem).to_path_buf();
            &logical_path
        } else {
            path
        };
        let name = name_path
            .file_stem()
            .and_then(|stem| stem.to_str())
            .ok_or_else(|| {
                std::io::Error::new(
                    ErrorKind::InvalidInput,
                    "Recording path has no valid file name",
                )
            })?;
        Self::from_bytes(name, &bytes)
    }

    pub fn from_bytes(name: &str, bytes: &[u8]) -> Result<Recording, std::io::Error> {
        let mut reader = DataReader::new(bytes);

        for magic_byte in RLPR_MAGIC_BYTES {
            if reader.read_u8()? != magic_byte {
                return Err(std::io::Error::new(
                    ErrorKind::InvalidData,
                    "File is not a valid recording (wrong magic)",
                ));
            }
        }

        let are_we_big_endian = cfg!(target_endian = "big");
        let is_file_big_endian = reader.read_bool()?;
        if is_file_big_endian != are_we_big_endian {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                "File has wrong endianness",
            ));
        }

        let version = reader.read_u32()?;
        if !(RLPR_MIN_VERSION..=RLPR_MAX_VERSION).contains(&version) {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!(
                    "RLPR version is not supported (expected {RLPR_MIN_VERSION}..={RLPR_MAX_VERSION}, got {version})"
                ),
            ));
        }

        let info = unsafe { reader.read_struct_unsafe::<RecordingInfo>() }?;
        let num_cars = info.num_cars as usize;
        if num_cars > RLPR_MAX_CARS {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR recording has too many cars (max: {RLPR_MAX_CARS}, got: {num_cars})"),
            ));
        }

        let num_ticks = reader.read_u32()?;
        let mut ticks = Vec::with_capacity(num_ticks as usize);

        for _ in 0..num_ticks {
            let (level_tick, row_flags, reserved0) = if recording_has_availability(version) {
                let level_tick = reader.read_u64()?;
                let row_flags = reader.read_u32()?;
                let reserved0 = reader.read_u32()?;
                if reserved0 != 0 {
                    return Err(std::io::Error::new(
                        ErrorKind::InvalidData,
                        format!("RLPR v11 row reserved word must be 0, got {reserved0}"),
                    ));
                }
                if row_flags & !ROW_FLAGS_KNOWN != 0 {
                    return Err(std::io::Error::new(
                        ErrorKind::InvalidData,
                        format!("RLPR v11 row flags have unknown bits: {row_flags:#x}"),
                    ));
                }
                if row_flags & ROW_FLAGS_NO_PHYSICS != 0
                    && row_flags & ROW_FLAGS_BALL_AVAILABLE != 0
                {
                    return Err(std::io::Error::new(
                        ErrorKind::InvalidData,
                        "RLPR v11 no-physics row must not mark the ball available",
                    ));
                }
                (level_tick, row_flags, reserved0)
            } else {
                (0, ROW_FLAGS_BALL_AVAILABLE, 0)
            };

            let (car_records, ball_record) = if recording_has_demolish_frames(version) {
                let mut car_records = Vec::with_capacity(num_cars);
                for _ in 0..num_cars {
                    let bytes = reader.read_sized_bytes()?;
                    car_records.push(read_car_record(bytes, version)?);
                }
                let ball_bytes = reader.read_sized_bytes()?;
                (car_records, read_v10_phys_record(ball_bytes)?)
            } else {
                let mut car_records = Vec::with_capacity(num_cars);
                let ball_record = loop {
                    let bytes = reader.read_sized_bytes()?;
                    if bytes.len() == size_of::<PhysRecord>() {
                        break read_record(bytes)?;
                    }

                    if car_records.len() == RLPR_MAX_CARS {
                        return Err(std::io::Error::new(
                            ErrorKind::InvalidData,
                            format!("RLPR tick has more than {RLPR_MAX_CARS} cars"),
                        ));
                    }
                    car_records.push(read_car_record(bytes, version)?);
                };
                (car_records, ball_record)
            };

            let mut car_records = car_records;
            if !recording_has_availability(version) {
                for record in &mut car_records {
                    record.physics_available = 1;
                    record._availability_pad = [0; 3];
                }
            } else {
                validate_v11_row(row_flags, &car_records, &ball_record)?;
            }

            ticks.push(TickRecord {
                level_tick,
                row_flags,
                reserved0,
                car_records,
                ball_record,
            });
        }

        if reader.num_bytes_left() > 0 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!(
                    "RLPR recording still has {} bytes left after reading all ticks",
                    reader.num_bytes_left()
                ),
            ));
        }

        Ok(Self {
            name: name.to_string(),
            version,
            info,
            ticks,
        })
    }
}

fn read_record<T: Copy>(bytes: &[u8]) -> std::io::Result<T> {
    if bytes.len() != size_of::<T>() {
        return Err(std::io::Error::new(
            ErrorKind::InvalidData,
            format!(
                "RLPR record size mismatch for {} (expected {}, got {})",
                std::any::type_name::<T>(),
                size_of::<T>(),
                bytes.len()
            ),
        ));
    }

    let mut record = std::mem::MaybeUninit::<T>::zeroed();
    unsafe {
        std::ptr::copy_nonoverlapping(
            bytes.as_ptr(),
            record.as_mut_ptr().cast::<u8>(),
            bytes.len(),
        );
        Ok(record.assume_init())
    }
}

/// Validate the four legacy wheel-contact bytes before a raw copy.
///
/// v2/v6 wheel stride matches `WheelRecord`. Copying an unchecked byte
/// into a Rust `bool` would create an invalid `bool`.
fn validate_legacy_wheel_contacts(bytes: &[u8]) -> std::io::Result<()> {
    for wheel_idx in 0..4 {
        let offset =
            CAR_RECORD_PREFIX_SIZE + wheel_idx * size_of::<WheelRecord>() + WHEEL_CONTACT_OFFSET;
        let contact = bytes[offset];
        if contact > 1 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR wheel contact byte must be 0 or 1, got {contact}"),
            ));
        }
    }
    Ok(())
}

/// Reject a boolean byte other than 0 or 1 before a typed copy.
fn validate_bool_byte(bytes: &[u8], offset: usize, field: &str) -> io::Result<()> {
    let value = bytes[offset];
    if value > 1 {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            format!("RLPR {field} byte must be 0 or 1, got {value}"),
        ));
    }
    Ok(())
}

/// Validate a `PhysRecord` before a typed copy.
///
/// `base` is the record offset inside `bytes`. The call checks every boolean
/// and enum, and it bounds the impulse count.
fn validate_phys_record_fields(bytes: &[u8], base: usize) -> io::Result<()> {
    validate_bool_byte(
        bytes,
        base + std::mem::offset_of!(PhysRecord, has_world_contact),
        "has_world_contact",
    )?;

    let impulses = base + PhysRecord::IMPULSE_RECORDS_OFFSET;
    let max_type = ImpulseRecordType::Boost as u8;
    for idx in 0..8 {
        let record = impulses + idx * size_of::<ImpulseRecord>();
        let impulse_type = bytes[record + std::mem::offset_of!(ImpulseRecord, impulse_type)];
        if impulse_type > max_type {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR impulse type must be 0..={max_type}, got {impulse_type}"),
            ));
        }
        validate_bool_byte(
            bytes,
            record + std::mem::offset_of!(ImpulseRecord, is_accum),
            "impulse is_accum",
        )?;
    }

    let count_offset = base + PhysRecord::NUM_IMPULSE_RECORDS_OFFSET;
    let count = u32::from_le_bytes(
        bytes[count_offset..count_offset + size_of::<u32>()]
            .try_into()
            .expect("impulse count slice has four bytes"),
    );
    if count as usize > 8 {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            format!("RLPR num_impulse_records must be 0..=8, got {count}"),
        ));
    }
    Ok(())
}

/// Validate a v10 car record before the typed copy.
fn validate_v10_car_record(bytes: &[u8]) -> io::Result<()> {
    if bytes.len() != CAR_RECORD_V10_SIZE {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            format!(
                "RLPR v10 CarRecord size mismatch (expected {CAR_RECORD_V10_SIZE}, got {})",
                bytes.len()
            ),
        ));
    }

    validate_phys_record_fields(bytes, std::mem::offset_of!(CarRecord, phys))?;

    for (offset, field) in [
        (
            std::mem::offset_of!(CarRecord, is_on_ground),
            "is_on_ground",
        ),
        (std::mem::offset_of!(CarRecord, is_jumping), "is_jumping"),
        (std::mem::offset_of!(CarRecord, is_flipping), "is_flipping"),
        (std::mem::offset_of!(CarRecord, has_jumped), "has_jumped"),
        (
            std::mem::offset_of!(CarRecord, double_jumped_or_flipped),
            "double_jumped_or_flipped",
        ),
        (std::mem::offset_of!(CarRecord, has_flip), "has_flip"),
        (
            std::mem::offset_of!(CarRecord, is_touching_ball),
            "is_touching_ball",
        ),
        (
            std::mem::offset_of!(CarRecord, is_touching_car),
            "is_touching_car",
        ),
        (std::mem::offset_of!(CarRecord, is_boosting), "is_boosting"),
    ] {
        validate_bool_byte(bytes, offset, field)?;
    }

    let controls = std::mem::offset_of!(CarRecord, prev_controls);
    for (offset, field) in [
        (
            controls + std::mem::offset_of!(ControlsRecord, jump),
            "controls jump",
        ),
        (
            controls + std::mem::offset_of!(ControlsRecord, boost),
            "controls boost",
        ),
        (
            controls + std::mem::offset_of!(ControlsRecord, handbrake),
            "controls handbrake",
        ),
    ] {
        validate_bool_byte(bytes, offset, field)?;
    }

    for wheel_idx in 0..4 {
        let offset =
            CAR_RECORD_PREFIX_SIZE + wheel_idx * size_of::<WheelRecord>() + WHEEL_CONTACT_OFFSET;
        validate_bool_byte(bytes, offset, "wheel has_contact")?;
    }

    Ok(())
}

/// Validate a v11 car record before the typed copy.
fn validate_v11_car_record(bytes: &[u8]) -> io::Result<()> {
    if bytes.len() != CAR_RECORD_V11_SIZE {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            format!(
                "RLPR v11 CarRecord size mismatch (expected {CAR_RECORD_V11_SIZE}, got {})",
                bytes.len()
            ),
        ));
    }
    validate_v10_car_record(&bytes[..CAR_RECORD_V10_SIZE])?;
    let available = bytes[CAR_PHYSICS_AVAILABLE_OFFSET];
    if available > 1 {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            format!("RLPR v11 physics_available must be 0 or 1, got {available}"),
        ));
    }
    if bytes[CAR_AVAILABILITY_PAD_OFFSET..CAR_RECORD_V11_SIZE] != [0, 0, 0] {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            "RLPR v11 car reserved bytes must be 0",
        ));
    }
    Ok(())
}

/// Validate v11 row-level availability consistency.
fn validate_v11_row(
    row_flags: u32,
    car_records: &[CarRecord],
    ball_record: &PhysRecord,
) -> io::Result<()> {
    let no_physics = row_flags & ROW_FLAGS_NO_PHYSICS != 0;
    let ball_available = row_flags & ROW_FLAGS_BALL_AVAILABLE != 0;
    if no_physics {
        if car_records.iter().any(CarRecord::is_available) {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                "RLPR v11 no-physics row has an available car",
            ));
        }
        if ball_record.physics_frame != NATIVE_FRAME_UNKNOWN {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                "RLPR v11 no-physics row must use the unknown ball frame",
            ));
        }
        return Ok(());
    }
    for (slot, car) in car_records.iter().enumerate() {
        if !car.is_available() && car.phys.physics_frame != NATIVE_FRAME_UNKNOWN {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR v11 unavailable car {slot} must use the unknown frame"),
            ));
        }
    }
    if !ball_available && ball_record.physics_frame != NATIVE_FRAME_UNKNOWN {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            "RLPR v11 unavailable ball must use the unknown frame",
        ));
    }
    if car_records.iter().all(|car| !car.is_available()) {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            "RLPR v11 row without an available car must set the no-physics flag",
        ));
    }
    Ok(())
}

/// Read and validate a v10 ball record.
fn read_v10_phys_record(bytes: &[u8]) -> io::Result<PhysRecord> {
    if bytes.len() != size_of::<PhysRecord>() {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            format!(
                "RLPR v10 ball record size mismatch (expected {}, got {})",
                size_of::<PhysRecord>(),
                bytes.len()
            ),
        ));
    }
    validate_phys_record_fields(bytes, 0)?;
    read_record(bytes)
}

fn read_car_record(bytes: &[u8], version: u32) -> std::io::Result<CarRecord> {
    let expected_size = match version {
        2 => CAR_RECORD_V2_SIZE,
        3 => 744,
        4 => 864,
        5 => 872,
        6 => CAR_RECORD_V6_SIZE,
        7 => CAR_RECORD_V7_SIZE,
        8 => CAR_RECORD_V8_SIZE,
        9 => CAR_RECORD_V9_SIZE,
        10 => CAR_RECORD_V10_SIZE,
        11 => CAR_RECORD_V11_SIZE,
        _ => unreachable!(),
    };
    if bytes.len() != expected_size {
        return Err(std::io::Error::new(
            ErrorKind::InvalidData,
            format!(
                "RLPR v{version} CarRecord size mismatch (expected {expected_size}, got {})",
                bytes.len()
            ),
        ));
    }

    if version == 2 {
        // Legacy bytes hold 584 bytes. The struct now holds more.
        // Validate wheel bools first, then copy and leave new fields at defaults.
        validate_legacy_wheel_contacts(bytes)?;
        let mut record = std::mem::MaybeUninit::<CarRecord>::zeroed();
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                record.as_mut_ptr().cast::<u8>(),
                CAR_RECORD_V2_SIZE,
            );
            let mut record = record.assume_init();
            record.last_ball_touch_frame = TOUCH_FRAME_UNKNOWN;
            record.last_demolish_frame = DEMOLISH_FRAME_UNKNOWN;
            Ok(record)
        }
    } else if version == 6 {
        let touch = bytes[CAR_TOUCH_OFFSET];
        if touch > 1 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR car touch byte must be 0 or 1, got {touch}"),
            ));
        }
        // Copy the 584 legacy bytes. Ignore the 3 padding bytes.
        validate_legacy_wheel_contacts(bytes)?;
        let mut record = std::mem::MaybeUninit::<CarRecord>::zeroed();
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                record.as_mut_ptr().cast::<u8>(),
                CAR_RECORD_V2_SIZE,
            );
            let mut record = record.assume_init();
            record.is_touching_car = touch == 1;
            record.is_boosting = false;
            record.boosting_time = 0.0;
            record.handbrake_val = 0.0;
            record.last_ball_touch_frame = TOUCH_FRAME_UNKNOWN;
            record.last_demolish_frame = DEMOLISH_FRAME_UNKNOWN;
            Ok(record)
        }
    } else if version == 7 {
        let touch = bytes[CAR_TOUCH_OFFSET];
        if touch > 1 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR car touch byte must be 0 or 1, got {touch}"),
            ));
        }
        let boost_bit = bytes[CAR_BOOST_BIT_OFFSET];
        if boost_bit > 1 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR boost bit byte must be 0 or 1, got {boost_bit}"),
            ));
        }
        let mut time_bytes = [0u8; 4];
        time_bytes.copy_from_slice(&bytes[CAR_BOOST_TIME_OFFSET..CAR_BOOST_TIME_OFFSET + 4]);
        let boosting_time = f32::from_le_bytes(time_bytes);
        if !boosting_time.is_finite() {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR boosting_time must be finite, got {boosting_time}"),
            ));
        }
        // Copy the 584 legacy bytes. Ignore all padding bytes.
        validate_legacy_wheel_contacts(bytes)?;
        let mut record = std::mem::MaybeUninit::<CarRecord>::zeroed();
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                record.as_mut_ptr().cast::<u8>(),
                CAR_RECORD_V2_SIZE,
            );
            let mut record = record.assume_init();
            record.is_touching_car = touch == 1;
            record.is_boosting = boost_bit == 1;
            record.boosting_time = boosting_time;
            record.handbrake_val = 0.0;
            record.last_ball_touch_frame = TOUCH_FRAME_UNKNOWN;
            record.last_demolish_frame = DEMOLISH_FRAME_UNKNOWN;
            Ok(record)
        }
    } else if version == 8 {
        let touch = bytes[CAR_TOUCH_OFFSET];
        if touch > 1 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR car touch byte must be 0 or 1, got {touch}"),
            ));
        }
        let boost_bit = bytes[CAR_BOOST_BIT_OFFSET];
        if boost_bit > 1 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR boost bit byte must be 0 or 1, got {boost_bit}"),
            ));
        }
        let mut boost_bytes = [0u8; 4];
        boost_bytes.copy_from_slice(&bytes[CAR_BOOST_TIME_OFFSET..CAR_BOOST_TIME_OFFSET + 4]);
        let boosting_time = f32::from_le_bytes(boost_bytes);
        if !boosting_time.is_finite() {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR boosting_time must be finite, got {boosting_time}"),
            ));
        }
        let mut brake_bytes = [0u8; 4];
        brake_bytes.copy_from_slice(&bytes[CAR_HANDBRAKE_OFFSET..CAR_HANDBRAKE_OFFSET + 4]);
        let handbrake_val = f32::from_le_bytes(brake_bytes);
        if !handbrake_val.is_finite() {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR handbrake_val must be finite, got {handbrake_val}"),
            ));
        }
        // Copy the 584 legacy bytes. Ignore all padding bytes.
        validate_legacy_wheel_contacts(bytes)?;
        let mut record = std::mem::MaybeUninit::<CarRecord>::zeroed();
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                record.as_mut_ptr().cast::<u8>(),
                CAR_RECORD_V2_SIZE,
            );
            let mut record = record.assume_init();
            record.is_touching_car = touch == 1;
            record.is_boosting = boost_bit == 1;
            record.boosting_time = boosting_time;
            record.handbrake_val = handbrake_val;
            record.last_ball_touch_frame = TOUCH_FRAME_UNKNOWN;
            record.last_demolish_frame = DEMOLISH_FRAME_UNKNOWN;
            Ok(record)
        }
    } else if version == 9 {
        let touch = bytes[CAR_TOUCH_OFFSET];
        if touch > 1 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR car touch byte must be 0 or 1, got {touch}"),
            ));
        }
        let boost_bit = bytes[CAR_BOOST_BIT_OFFSET];
        if boost_bit > 1 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR boost bit byte must be 0 or 1, got {boost_bit}"),
            ));
        }
        let mut boost_bytes = [0u8; 4];
        boost_bytes.copy_from_slice(&bytes[CAR_BOOST_TIME_OFFSET..CAR_BOOST_TIME_OFFSET + 4]);
        let boosting_time = f32::from_le_bytes(boost_bytes);
        if !boosting_time.is_finite() {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR boosting_time must be finite, got {boosting_time}"),
            ));
        }
        let mut brake_bytes = [0u8; 4];
        brake_bytes.copy_from_slice(&bytes[CAR_HANDBRAKE_OFFSET..CAR_HANDBRAKE_OFFSET + 4]);
        let handbrake_val = f32::from_le_bytes(brake_bytes);
        if !handbrake_val.is_finite() {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR handbrake_val must be finite, got {handbrake_val}"),
            ));
        }
        // Raw engine frame number: every bit pattern is a valid reading
        // (including the never-touched encoding), so no validation.
        let mut touch_frame_bytes = [0u8; 4];
        touch_frame_bytes
            .copy_from_slice(&bytes[CAR_TOUCH_FRAME_OFFSET..CAR_TOUCH_FRAME_OFFSET + 4]);
        let last_ball_touch_frame = u32::from_le_bytes(touch_frame_bytes);
        // Copy the 584 legacy bytes. Ignore all padding bytes.
        validate_legacy_wheel_contacts(bytes)?;
        let mut record = std::mem::MaybeUninit::<CarRecord>::zeroed();
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                record.as_mut_ptr().cast::<u8>(),
                CAR_RECORD_V2_SIZE,
            );
            let mut record = record.assume_init();
            record.is_touching_car = touch == 1;
            record.is_boosting = boost_bit == 1;
            record.boosting_time = boosting_time;
            record.handbrake_val = handbrake_val;
            record.last_ball_touch_frame = last_ball_touch_frame;
            record.last_demolish_frame = DEMOLISH_FRAME_UNKNOWN;
            Ok(record)
        }
    } else if version == 10 {
        validate_v10_car_record(bytes)?;
        let mut record = std::mem::MaybeUninit::<CarRecord>::zeroed();
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                record.as_mut_ptr().cast::<u8>(),
                CAR_RECORD_V10_SIZE,
            );
            let mut record = record.assume_init();
            // The wire keeps reserved padding for v6/v7 field alignment. Ignore
            // those bytes so the parsed record keeps its zero invariant.
            record._touch_pad = [0; 3];
            record._boost_pad = [0; 3];
            Ok(record)
        }
    } else if version == 11 {
        validate_v11_car_record(bytes)?;
        let mut record: CarRecord = read_record(bytes)?;
        record._touch_pad = [0; 3];
        record._boost_pad = [0; 3];
        record._availability_pad = [0; 3];
        Ok(record)
    } else {
        let wheel_stride = match version {
            3 => 88,
            4 | 5 => 112,
            _ => unreachable!(),
        };
        let mut record = std::mem::MaybeUninit::<CarRecord>::zeroed();
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                record.as_mut_ptr().cast::<u8>(),
                CAR_RECORD_PREFIX_SIZE,
            );
            let mut record = record.assume_init();
            for (wheel_idx, wheel) in record.wheels.iter_mut().enumerate() {
                let contact_offset =
                    CAR_RECORD_PREFIX_SIZE + wheel_idx * wheel_stride + WHEEL_CONTACT_OFFSET;
                let contact = bytes[contact_offset];
                if contact > 1 {
                    return Err(std::io::Error::new(
                        ErrorKind::InvalidData,
                        format!("RLPR wheel contact byte must be 0 or 1, got {contact}"),
                    ));
                }
                wheel.has_contact = contact == 1;
            }
            record.last_ball_touch_frame = TOUCH_FRAME_UNKNOWN;
            record.last_demolish_frame = DEMOLISH_FRAME_UNKNOWN;
            Ok(record)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v9_bytes(touch_frame: u32) -> Vec<u8> {
        let mut bytes = vec![0u8; CAR_RECORD_V9_SIZE];
        bytes[CAR_TOUCH_FRAME_OFFSET..CAR_TOUCH_FRAME_OFFSET + 4]
            .copy_from_slice(&touch_frame.to_le_bytes());
        bytes
    }

    fn v10_car_bytes(demolish_frame: u32) -> Vec<u8> {
        let mut bytes = vec![0u8; CAR_RECORD_V10_SIZE];
        bytes[CAR_DEMOLISH_FRAME_OFFSET..CAR_DEMOLISH_FRAME_OFFSET + 4]
            .copy_from_slice(&demolish_frame.to_le_bytes());
        bytes
    }

    fn v10_ball_bytes() -> Vec<u8> {
        vec![0u8; size_of::<PhysRecord>()]
    }

    fn push_sized(bytes: &mut Vec<u8>, record: &[u8]) {
        bytes.extend_from_slice(&(record.len() as u32).to_le_bytes());
        bytes.extend_from_slice(record);
    }

    /// Build a v10 file with the given car and ball records per tick.
    fn v10_file(num_cars: u32, ticks: &[(Vec<Vec<u8>>, Vec<u8>)]) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&RLPR_MAGIC_BYTES);
        bytes.push(0);
        bytes.extend_from_slice(&10u32.to_le_bytes());
        let mut info = Vec::new();
        info.extend_from_slice(&num_cars.to_le_bytes());
        info.extend_from_slice(&[0u8; 24]);
        push_sized(&mut bytes, &info);
        bytes.extend_from_slice(&(ticks.len() as u32).to_le_bytes());
        for (cars, ball) in ticks {
            for car in cars {
                push_sized(&mut bytes, car);
            }
            push_sized(&mut bytes, ball);
        }
        bytes
    }

    /// Build a v11 car record. `frame` is the native physics frame.
    fn v11_car_bytes(available: bool, frame: u32) -> Vec<u8> {
        let mut bytes = vec![0u8; CAR_RECORD_V11_SIZE];
        bytes[..4].copy_from_slice(&frame.to_le_bytes());
        bytes[CAR_PHYSICS_AVAILABLE_OFFSET] = u8::from(available);
        bytes
    }

    /// Build a v11 ball record. Unavailable balls use the unknown frame.
    fn v11_ball_bytes(available: bool) -> Vec<u8> {
        let mut bytes = vec![0u8; size_of::<PhysRecord>()];
        let frame = if available { 0 } else { NATIVE_FRAME_UNKNOWN };
        bytes[..4].copy_from_slice(&frame.to_le_bytes());
        bytes
    }

    /// One v11 row: `(level_tick, row_flags, cars, ball)`.
    type V11Row = (u64, u32, Vec<Vec<u8>>, Vec<u8>);

    /// Build a v11 file. Each row carries the 16-byte prefix.
    fn v11_file(num_cars: u32, rows: &[V11Row]) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&RLPR_MAGIC_BYTES);
        bytes.push(0);
        bytes.extend_from_slice(&11u32.to_le_bytes());
        let mut info = Vec::new();
        info.extend_from_slice(&num_cars.to_le_bytes());
        info.extend_from_slice(&[0u8; 24]);
        push_sized(&mut bytes, &info);
        bytes.extend_from_slice(&(rows.len() as u32).to_le_bytes());
        for (level_tick, row_flags, cars, ball) in rows {
            bytes.extend_from_slice(&level_tick.to_le_bytes());
            bytes.extend_from_slice(&row_flags.to_le_bytes());
            bytes.extend_from_slice(&0u32.to_le_bytes());
            for car in cars {
                push_sized(&mut bytes, car);
            }
            push_sized(&mut bytes, ball);
        }
        bytes
    }

    /// Six-car v11 row with the given availability mask and frame base.
    fn v11_six_row(level_tick: u64, mask: u32, frame: u32) -> V11Row {
        let cars = (0..6)
            .map(|slot| {
                let available = mask & (1 << slot) != 0;
                let car_frame = if available {
                    frame
                } else {
                    NATIVE_FRAME_UNKNOWN
                };
                v11_car_bytes(available, car_frame)
            })
            .collect();
        (
            level_tick,
            ROW_FLAGS_BALL_AVAILABLE,
            cars,
            v11_ball_bytes(true),
        )
    }

    /// Six-car v11 no-physics guard row.
    fn v11_guard_row(level_tick: u64) -> V11Row {
        let cars = (0..6)
            .map(|_| v11_car_bytes(false, NATIVE_FRAME_UNKNOWN))
            .collect();
        (
            level_tick,
            ROW_FLAGS_NO_PHYSICS,
            cars,
            v11_ball_bytes(false),
        )
    }

    #[test]
    fn v11_reads_prefix_and_availability() {
        let bytes = v11_file(6, &[v11_six_row(77, 0b11_1111, 900)]);
        let recording = Recording::from_bytes("v11", &bytes).unwrap();
        let tick = &recording.ticks[0];
        assert_eq!(tick.level_tick, 77);
        assert_eq!(tick.row_flags, ROW_FLAGS_BALL_AVAILABLE);
        assert_eq!(tick.reserved0, 0);
        assert!(tick.has_physics());
        assert!(tick.ball_available());
        assert!(tick.all_cars_available());
        assert_eq!(tick.availability_mask(), 0b11_1111);
        assert_eq!(tick.car_records[0].phys.physics_frame, 900);
    }

    #[test]
    fn v11_partial_topology_parses() {
        // Native full row, a guard row with no physics, then a five-car row
        // (slot 3 unavailable), then a full row again.
        let bytes = v11_file(
            6,
            &[
                v11_six_row(100, 0b11_1111, 10),
                v11_guard_row(101),
                v11_six_row(102, 0b11_0111, 12),
                v11_six_row(103, 0b11_1111, 13),
            ],
        );
        let recording = Recording::from_bytes("v11", &bytes).unwrap();
        let ticks = &recording.ticks;
        assert_eq!(ticks.len(), 4);
        assert!(ticks[0].all_cars_available());
        assert!(!ticks[1].has_physics());
        assert_eq!(ticks[1].availability_mask(), 0);
        assert!(!ticks[1].ball_available());
        assert_eq!(ticks[1].level_tick, 101);
        assert_eq!(ticks[2].availability_mask(), 0b11_0111);
        assert!(!ticks[2].car_available(3));
        assert_eq!(ticks[2].car_records[3].phys.physics_frame, u32::MAX);
        assert!(ticks[3].all_cars_available());
    }

    #[test]
    fn v11_rejects_inconsistent_rows() {
        let full = v11_six_row(1, 0b11_1111, 1);
        let guard = v11_guard_row(2);
        let cases: Vec<(&str, Vec<u8>)> = vec![
            (
                "physics_available must be 0 or 1",
                v11_file(
                    6,
                    &[mutate_car(full.clone(), |car| {
                        car[CAR_PHYSICS_AVAILABLE_OFFSET] = 2;
                    })],
                ),
            ),
            (
                "car reserved bytes must be 0",
                v11_file(
                    6,
                    &[mutate_car(full.clone(), |car| {
                        car[CAR_AVAILABILITY_PAD_OFFSET] = 1;
                    })],
                ),
            ),
            (
                "CarRecord size mismatch",
                v11_file(6, &[mutate_car(full.clone(), |car| car.push(0))]),
            ),
            (
                "unavailable car 3 must use the unknown frame",
                v11_file(
                    6,
                    &[mutate_car(v11_six_row(1, 0b11_0111, 1), |car| {
                        car[..4].copy_from_slice(&7u32.to_le_bytes());
                    })],
                ),
            ),
            (
                "reserved word must be 0",
                v11_file_with_reserved(6, &full, 9),
            ),
            (
                "unknown bits",
                v11_file_with_flags(6, &full, ROW_FLAGS_BALL_AVAILABLE | 0b100),
            ),
            (
                "must not mark the ball available",
                v11_file_with_flags(6, &guard.clone(), ROW_FLAGS_NO_PHYSICS | 1),
            ),
            (
                "no-physics row has an available car",
                v11_file_with_flags(6, &full.clone(), ROW_FLAGS_NO_PHYSICS),
            ),
            (
                "unavailable ball must use the unknown frame",
                v11_file_with_ball(
                    6,
                    &full.clone(),
                    v11_ball_bytes(true),
                    ROW_FLAGS_BALL_AVAILABLE & !1,
                ),
            ),
            (
                "without an available car must set the no-physics flag",
                v11_file(6, &[v11_six_row(3, 0, 3)]),
            ),
        ];
        for (expected, bytes) in cases {
            let err = match Recording::from_bytes("v11", &bytes) {
                Ok(_) => panic!("expected an error containing {expected:?}"),
                Err(err) => err,
            };
            assert!(
                err.to_string().contains(expected),
                "expected {expected:?}, got {err}"
            );
        }
    }

    /// Mutate one car record (slot 3) inside a single-row v11 file.
    fn mutate_car(mut row: V11Row, mutate: impl FnOnce(&mut Vec<u8>)) -> V11Row {
        mutate(&mut row.2[3]);
        row
    }

    fn v11_file_with_reserved(num_cars: u32, row: &V11Row, reserved: u32) -> Vec<u8> {
        let mut bytes = v11_file(num_cars, std::slice::from_ref(row));
        // The row prefix starts after the header: magic(4) + endian(1) +
        // version(4) + sized info(4 + 28) + tick count(4).
        let prefix = 4 + 1 + 4 + 4 + 28 + 4;
        bytes[prefix + 12..prefix + 16].copy_from_slice(&reserved.to_le_bytes());
        bytes
    }

    fn v11_file_with_flags(num_cars: u32, row: &V11Row, flags: u32) -> Vec<u8> {
        let mut bytes = v11_file(num_cars, std::slice::from_ref(row));
        let prefix = 4 + 1 + 4 + 4 + 28 + 4;
        bytes[prefix + 8..prefix + 12].copy_from_slice(&flags.to_le_bytes());
        bytes
    }

    fn v11_file_with_ball(num_cars: u32, row: &V11Row, ball: Vec<u8>, flags: u32) -> Vec<u8> {
        let mut bytes = v11_file(num_cars, std::slice::from_ref(row));
        let prefix = 4 + 1 + 4 + 4 + 28 + 4;
        bytes[prefix + 8..prefix + 12].copy_from_slice(&flags.to_le_bytes());
        // Replace the trailing sized ball payload (keep its u32 size prefix).
        let ball_size = size_of::<PhysRecord>();
        let ball_start = bytes.len() - ball_size;
        bytes[ball_start..].copy_from_slice(&ball);
        bytes
    }

    #[test]
    fn old_versions_default_to_available() {
        let bytes = v10_file(2, &[(vec![v10_car_bytes(0); 2], v10_ball_bytes())]);
        let recording = Recording::from_bytes("v10", &bytes).unwrap();
        let tick = &recording.ticks[0];
        assert_eq!(tick.level_tick, 0);
        assert_eq!(tick.row_flags, ROW_FLAGS_BALL_AVAILABLE);
        assert!(tick.has_physics());
        assert!(tick.ball_available());
        assert!(tick.all_cars_available());
        assert_eq!(tick.availability_mask(), 0b11);
    }

    #[test]
    fn v9_parses_touch_frame_verbatim() {
        let record = read_car_record(&v9_bytes(1234), 9).unwrap();
        assert_eq!(record.last_ball_touch_frame, 1234);
        assert!(!record.is_touching_car);
        assert_eq!(
            std::mem::offset_of!(CarRecord, last_ball_touch_frame),
            CAR_TOUCH_FRAME_OFFSET
        );
    }

    #[test]
    fn v9_preserves_never_touched_bits() {
        // Negative int32 from the engine arrives bit-preserving.
        let record = read_car_record(&v9_bytes(u32::MAX), 9).unwrap();
        assert_eq!(record.last_ball_touch_frame, u32::MAX);
    }

    #[test]
    fn v9_rejects_wrong_size() {
        assert!(read_car_record(&vec![0u8; CAR_RECORD_V8_SIZE], 9).is_err());
        assert!(read_car_record(&v9_bytes(0), 8).is_err());
    }

    #[test]
    fn legacy_versions_default_events_unknown() {
        for (version, size) in [
            (2, CAR_RECORD_V2_SIZE),
            (3, 744),
            (4, 864),
            (5, 872),
            (6, CAR_RECORD_V6_SIZE),
            (7, CAR_RECORD_V7_SIZE),
            (8, CAR_RECORD_V8_SIZE),
            (9, CAR_RECORD_V9_SIZE),
        ] {
            let record = read_car_record(&vec![0u8; size], version).unwrap();
            if recording_has_touch_frames(version) {
                assert_eq!(record.last_ball_touch_frame, 0);
            } else {
                assert_eq!(
                    record.last_ball_touch_frame, TOUCH_FRAME_UNKNOWN,
                    "v{version} must not invent touch data"
                );
            }
            assert_eq!(
                record.last_demolish_frame, DEMOLISH_FRAME_UNKNOWN,
                "v{version} must not invent event data"
            );
        }
    }

    #[test]
    fn v10_reads_demolish_frame_verbatim() {
        for frame in [0, 1, 1234, 0x8000_0001, DEMOLISH_FRAME_UNKNOWN] {
            let bytes = v10_file(1, &[(vec![v10_car_bytes(frame)], v10_ball_bytes())]);
            let recording = Recording::from_bytes("v10", &bytes).unwrap();
            let cars = &recording.ticks[0].car_records;
            assert_eq!(cars.len(), 1);
            assert_eq!(cars[0].last_demolish_frame, frame);
        }

        let mut car = v10_car_bytes(7);
        let touch_pad = std::mem::offset_of!(CarRecord, _touch_pad);
        car[touch_pad..touch_pad + 3].fill(0xff);
        let boost_pad = std::mem::offset_of!(CarRecord, _boost_pad);
        car[boost_pad..boost_pad + 3].fill(0xff);
        let record = read_car_record(&car, 10).unwrap();
        assert_eq!(record._touch_pad, [0; 3]);
        assert_eq!(record._boost_pad, [0; 3]);
        assert_eq!(record.last_demolish_frame, 7);
    }

    #[test]
    fn v10_requires_the_declared_car_count() {
        let valid = v10_file(
            2,
            &[(vec![v10_car_bytes(3), v10_car_bytes(4)], v10_ball_bytes())],
        );
        let recording = Recording::from_bytes("v10", &valid).unwrap();
        let cars = &recording.ticks[0].car_records;
        assert_eq!(cars.len(), 2);
        assert_eq!(
            [cars[0].last_demolish_frame, cars[1].last_demolish_frame],
            [3, 4]
        );

        let short = v10_file(2, &[(vec![v10_car_bytes(3)], v10_ball_bytes())]);
        assert!(Recording::from_bytes("short", &short).is_err());

        let long = v10_file(
            1,
            &[(vec![v10_car_bytes(3), v10_car_bytes(4)], v10_ball_bytes())],
        );
        assert!(Recording::from_bytes("long", &long).is_err());
    }

    #[test]
    fn v10_rejects_truncated_and_wrong_sized_records() {
        for size in [
            CAR_RECORD_V9_SIZE,
            CAR_RECORD_V10_SIZE - 1,
            CAR_RECORD_V10_SIZE + 4,
        ] {
            assert!(read_car_record(&vec![0u8; size], 10).is_err());
        }

        let bytes = v10_file(1, &[(vec![v10_car_bytes(0)], v10_ball_bytes())]);
        assert!(Recording::from_bytes("truncated", &bytes[..bytes.len() - 1]).is_err());
    }

    #[test]
    fn v10_rejects_malformed_bool_and_enum_bytes() {
        let mut bool_offsets = vec![
            std::mem::offset_of!(CarRecord, phys)
                + std::mem::offset_of!(PhysRecord, has_world_contact),
            std::mem::offset_of!(CarRecord, is_on_ground),
            std::mem::offset_of!(CarRecord, is_jumping),
            std::mem::offset_of!(CarRecord, is_flipping),
            std::mem::offset_of!(CarRecord, has_jumped),
            std::mem::offset_of!(CarRecord, double_jumped_or_flipped),
            std::mem::offset_of!(CarRecord, has_flip),
            std::mem::offset_of!(CarRecord, is_touching_ball),
            std::mem::offset_of!(CarRecord, is_touching_car),
            std::mem::offset_of!(CarRecord, is_boosting),
            std::mem::offset_of!(CarRecord, prev_controls)
                + std::mem::offset_of!(ControlsRecord, jump),
            std::mem::offset_of!(CarRecord, prev_controls)
                + std::mem::offset_of!(ControlsRecord, boost),
            std::mem::offset_of!(CarRecord, prev_controls)
                + std::mem::offset_of!(ControlsRecord, handbrake),
        ];
        bool_offsets.extend((0..4).map(|wheel_idx| {
            CAR_RECORD_PREFIX_SIZE + wheel_idx * size_of::<WheelRecord>() + WHEEL_CONTACT_OFFSET
        }));

        for offset in bool_offsets {
            let mut car = v10_car_bytes(DEMOLISH_FRAME_UNKNOWN);
            car[offset] = 2;
            assert!(
                read_car_record(&car, 10).is_err(),
                "bool at offset {offset} must be rejected"
            );
        }

        let impulse_base =
            std::mem::offset_of!(CarRecord, phys) + PhysRecord::IMPULSE_RECORDS_OFFSET;
        for impulse_idx in [0, 7] {
            let mut car = v10_car_bytes(DEMOLISH_FRAME_UNKNOWN);
            let offset = impulse_base
                + impulse_idx * size_of::<ImpulseRecord>()
                + std::mem::offset_of!(ImpulseRecord, impulse_type);
            car[offset] = ImpulseRecordType::Boost as u8 + 1;
            assert!(
                read_car_record(&car, 10).is_err(),
                "enum at offset {offset} must be rejected"
            );
        }

        let mut car = v10_car_bytes(DEMOLISH_FRAME_UNKNOWN);
        let offset = impulse_base
            + 3 * size_of::<ImpulseRecord>()
            + std::mem::offset_of!(ImpulseRecord, is_accum);
        car[offset] = 2;
        assert!(read_car_record(&car, 10).is_err());

        let mut car = v10_car_bytes(DEMOLISH_FRAME_UNKNOWN);
        let offset = std::mem::offset_of!(CarRecord, phys) + PhysRecord::NUM_IMPULSE_RECORDS_OFFSET;
        car[offset..offset + 4].copy_from_slice(&9u32.to_le_bytes());
        assert!(read_car_record(&car, 10).is_err());

        for (offset, value) in [
            (std::mem::offset_of!(PhysRecord, has_world_contact), 2u8),
            (
                PhysRecord::IMPULSE_RECORDS_OFFSET
                    + std::mem::offset_of!(ImpulseRecord, impulse_type),
                ImpulseRecordType::Boost as u8 + 1,
            ),
            (
                PhysRecord::IMPULSE_RECORDS_OFFSET
                    + 3 * size_of::<ImpulseRecord>()
                    + std::mem::offset_of!(ImpulseRecord, is_accum),
                2,
            ),
        ] {
            let mut ball = v10_ball_bytes();
            ball[offset] = value;
            let bytes = v10_file(1, &[(vec![v10_car_bytes(0)], ball)]);
            assert!(
                Recording::from_bytes("bad-ball", &bytes).is_err(),
                "ball byte at offset {offset} must be rejected"
            );
        }

        let mut ball = v10_ball_bytes();
        let offset = PhysRecord::NUM_IMPULSE_RECORDS_OFFSET;
        ball[offset..offset + 4].copy_from_slice(&9u32.to_le_bytes());
        let bytes = v10_file(1, &[(vec![v10_car_bytes(0)], ball)]);
        assert!(Recording::from_bytes("bad-ball-count", &bytes).is_err());
    }

    #[test]
    fn legacy_recordings_keep_events_unknown() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tests/rl_comparison_test/test_recordings");
        for name in ["cw_single_jump.rlpr", "cw_wall_ramp_jump_rearm.rlpr"] {
            let recording = Recording::from_file(&dir.join(name)).unwrap();
            assert!(recording.version < 10);
            assert!(!recording_has_demolish_frames(recording.version));
            for tick in &recording.ticks {
                for car in &tick.car_records {
                    assert_eq!(car.last_demolish_frame, DEMOLISH_FRAME_UNKNOWN);
                }
            }
        }
    }

    #[test]
    fn touch_frame_gate_follows_version() {
        assert!(!recording_has_touch_frames(2));
        assert!(!recording_has_touch_frames(8));
        assert!(recording_has_touch_frames(9));
    }

    #[test]
    fn demolish_frame_gate_reports_field_presence_only() {
        assert!(!recording_has_demolish_frames(2));
        assert!(!recording_has_demolish_frames(9));
        assert!(recording_has_demolish_frames(10));
        assert!(recording_has_demolish_frames(11));
    }
}
