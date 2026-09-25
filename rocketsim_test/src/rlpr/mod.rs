pub mod cpp_records;
mod data_reader;
pub mod tick_record;

use std::{
    io::{ErrorKind, Read},
    mem::size_of,
    path::Path,
};

use cpp_records::*;
use data_reader::DataReader;

use crate::rlpr::tick_record::TickRecord;

const RLPR_MAGIC_BYTES: [u8; 4] = [82, 76, 80, 82];
const RLPR_MIN_VERSION: u32 = 2;
const RLPR_MAX_VERSION: u32 = 8;
const RLPR_MAX_CARS: usize = 8;
/// First version carrying recorded boost latch state.
const RLPR_BOOST_STATE_VERSION: u32 = 7;
/// First version carrying recorded handbrake integrator state.
const RLPR_HANDBRAKE_STATE_VERSION: u32 = 8;
const CAR_RECORD_PREFIX_SIZE: usize = std::mem::offset_of!(CarRecord, wheels);
const WHEEL_CONTACT_OFFSET: usize = std::mem::offset_of!(WheelRecord, has_contact);
/// Legacy v2 file size. Stays explicit: `CarRecord` is now 588 bytes.
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
fn decode_bounded(raw: &[u8], path: &Path) -> std::io::Result<Vec<u8>> {
    let mut decoder = zstd::Decoder::new(raw).map_err(|err| decode_failed(path, err))?;
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
            ticks.push(TickRecord {
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

fn read_car_record(bytes: &[u8], version: u32) -> std::io::Result<CarRecord> {
    let expected_size = match version {
        2 => CAR_RECORD_V2_SIZE,
        3 => 744,
        4 => 864,
        5 => 872,
        6 => CAR_RECORD_V6_SIZE,
        7 => CAR_RECORD_V7_SIZE,
        8 => CAR_RECORD_V8_SIZE,
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
        // Legacy bytes hold 584 bytes. The struct now holds 596.
        // Validate wheel bools first, then copy and leave new fields at defaults.
        validate_legacy_wheel_contacts(bytes)?;
        let mut record = std::mem::MaybeUninit::<CarRecord>::zeroed();
        unsafe {
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                record.as_mut_ptr().cast::<u8>(),
                CAR_RECORD_V2_SIZE,
            );
            Ok(record.assume_init())
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
            Ok(record)
        }
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
            Ok(record)
        }
    }
}
