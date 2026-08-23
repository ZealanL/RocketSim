use std::io::{self, ErrorKind, Read};

use byteorder::{LittleEndian, ReadBytesExt};

pub struct DataReader<'a> {
    bytes: &'a [u8],
}

impl<'a> DataReader<'a> {
    pub fn new(bytes: &'a [u8]) -> Self {
        Self { bytes }
    }

    pub fn read_u8(&mut self) -> io::Result<u8> {
        self.bytes.read_u8()
    }

    pub fn read_u32(&mut self) -> io::Result<u32> {
        self.bytes.read_u32::<LittleEndian>()
    }

    pub fn read_bool(&mut self) -> io::Result<bool> {
        let byte = self.bytes.read_u8()?;
        if byte > 1 {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                format!("Boolean byte should either be 0 or 1 (got {byte})"),
            ));
        }

        Ok(byte == 1)
    }

    pub unsafe fn read_struct_unsafe<T: Copy>(&mut self) -> io::Result<T> {
        let expected_size = self.bytes.read_u32::<LittleEndian>()? as usize;
        let struct_size = size_of::<T>();

        if expected_size != struct_size {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                format!(
                    "Size mismatch reading struct '{}' (serialized={}, expected={})",
                    std::any::type_name::<T>(),
                    expected_size,
                    struct_size
                ),
            ));
        }

        let mut obj = std::mem::MaybeUninit::<T>::zeroed();

        // Oooooo scary! But not really since the worst case scenario is wrong-value primitives
        unsafe {
            let slice = std::slice::from_raw_parts_mut(obj.as_mut_ptr() as *mut u8, struct_size);
            self.bytes.read_exact(slice)?;
            Ok(obj.assume_init())
        }
    }

    pub fn num_bytes_left(&mut self) -> usize {
        self.bytes.len()
    }
}
