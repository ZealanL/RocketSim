pub mod cpp_records;
mod data_reader;

use cpp_records::*;
use data_reader::DataReader;
use std::io::{BufReader, ErrorKind};

const RLPR_MAGIC_BYTES: [u8; 4] = [82, 76, 80, 82];
const RLPR_VERSION: u32 = 0;

#[allow(dead_code)]
pub struct Recording {
    pub name: String,
    pub info: RecordingInfo,
    pub ticks: Vec<TickRecord>,
}

impl Recording {
    pub fn from_bytes(name: &str, bytes: &[u8]) -> Result<Recording, std::io::Error> {
        let buffered_stream = BufReader::new(bytes);
        let mut reader = DataReader::new(buffered_stream);

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
        if version != RLPR_VERSION {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR Version mismatch (expected: {version}, got: {version})"),
            ));
        }

        let info = unsafe { reader.read_struct_unsafe::<RecordingInfo>() }?;

        let num_ticks = reader.read_u32()?;
        let mut ticks = Vec::with_capacity(num_ticks as usize);

        for _ in 0..num_ticks {
            let tick = unsafe { reader.read_struct_unsafe::<TickRecord>() }?;
            ticks.push(tick);
        }

        Ok(Self {
            name: name.to_string(),
            info,
            ticks,
        })
    }
}
