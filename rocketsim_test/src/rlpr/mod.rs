pub mod cpp_records;
mod data_reader;
pub mod tick_record;

use std::{io::ErrorKind, mem::size_of, path::Path};

use cpp_records::*;
use data_reader::DataReader;

use crate::rlpr::tick_record::TickRecord;

const RLPR_MAGIC_BYTES: [u8; 4] = [82, 76, 80, 82];
const RLPR_MIN_VERSION: u32 = 2;
const RLPR_MAX_VERSION: u32 = 5;
const RLPR_MAX_CARS: usize = 8;
const CAR_RECORD_PREFIX_SIZE: usize = std::mem::offset_of!(CarRecord, wheels);
const WHEEL_CONTACT_OFFSET: usize = std::mem::offset_of!(WheelRecord, has_contact);

#[allow(dead_code)]
pub struct Recording {
    pub name: String,
    pub version: u32,
    pub info: RecordingInfo,
    pub ticks: Vec<TickRecord>,
}

impl Recording {
    pub fn from_file(path: &Path) -> Result<Recording, std::io::Error> {
        let bytes = std::fs::read(path)?;
        let name = path
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

fn read_car_record(bytes: &[u8], version: u32) -> std::io::Result<CarRecord> {
    let expected_size = match version {
        2 => size_of::<CarRecord>(),
        3 => 744,
        4 => 864,
        5 => 872,
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
        return read_record(bytes);
    }

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
