use std::{
    io::Write,
    time::{Duration, Instant},
};

#[derive(Default)]
pub(crate) struct OutgoingBuffer {
    bytes: Vec<u8>,
    offset: usize,
    last_progress: Option<Instant>,
}

impl OutgoingBuffer {
    pub(crate) fn is_empty(&self) -> bool {
        self.offset == self.bytes.len()
    }

    pub(crate) fn queue(&mut self, bytes: Vec<u8>) {
        debug_assert!(self.is_empty());
        self.bytes = bytes;
        self.offset = 0;
        self.last_progress = Some(Instant::now());
    }

    pub(crate) fn flush<W: Write>(&mut self, writer: &mut W) -> std::io::Result<bool> {
        while !self.is_empty() {
            match writer.write(&self.bytes[self.offset..]) {
                Ok(0) => return Err(std::io::ErrorKind::WriteZero.into()),
                Ok(count) => {
                    self.offset += count;
                    self.last_progress = Some(Instant::now());
                }
                Err(error) if error.kind() == std::io::ErrorKind::Interrupted => return Ok(false),
                Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                    if self
                        .last_progress
                        .is_some_and(|at| at.elapsed() >= Duration::from_millis(250))
                    {
                        return Err(std::io::ErrorKind::TimedOut.into());
                    }
                    return Ok(false);
                }
                Err(error) => return Err(error),
            }
        }
        self.bytes.clear();
        self.offset = 0;
        self.last_progress = None;
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use std::io::Write;

    use super::*;

    struct ScriptedWriter {
        steps: Vec<std::io::Result<usize>>,
        bytes: Vec<u8>,
    }

    impl Write for ScriptedWriter {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            let next = self.steps.remove(0)?;
            if next == 0 {
                return Ok(0);
            }
            let count = next.min(buf.len());
            self.bytes.extend_from_slice(&buf[..count]);
            Ok(count)
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn outgoing_buffer_resumes_short_writes_after_would_block() {
        let mut buffer = OutgoingBuffer::default();
        buffer.queue(b"complete-frame".to_vec());
        let mut writer = ScriptedWriter {
            steps: vec![Ok(3), Err(std::io::ErrorKind::WouldBlock.into()), Ok(4096)],
            bytes: Vec::new(),
        };
        assert!(!buffer.flush(&mut writer).unwrap());
        assert_eq!(buffer.offset, 3);
        assert!(buffer.flush(&mut writer).unwrap());
        assert_eq!(writer.bytes, b"complete-frame");
        assert!(buffer.is_empty());
    }

    #[test]
    fn outgoing_buffer_closes_after_a_stalled_write_deadline() {
        let mut buffer = OutgoingBuffer::default();
        buffer.queue(vec![1, 2, 3]);
        buffer.last_progress = Some(Instant::now() - Duration::from_millis(300));
        let mut writer = ScriptedWriter {
            steps: vec![Err(std::io::ErrorKind::WouldBlock.into())],
            bytes: Vec::new(),
        };
        assert_eq!(
            buffer.flush(&mut writer).unwrap_err().kind(),
            std::io::ErrorKind::TimedOut
        );
    }
}
