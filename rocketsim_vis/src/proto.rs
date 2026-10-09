//! Wire protocol: `[u32 LE payload_len][postcard(VisFrame)]` over TCP.
//!
//! The sim dials; the viewer listens (default port [`DEFAULT_PORT`]). Baseline:
//! a 6-car Soccar frame postcards to ~3KB, so [`MAX_FRAME_BYTES`] (4MB) is a
//! sanity cap only, never approached in practice.
//!
//! Bump [`PROTOCOL_VERSION`] (and change port) on any breaking change.

use rocketsim::{ArenaState, CarControls};

/// Wire protocol version. Bump + change port on breaking changes.
pub const PROTOCOL_VERSION: u32 = 1;

/// Default TCP port. Deliberately different from rlviser's 34254.
pub const DEFAULT_PORT: u16 = 39201;

/// Default TCP port for the viewer→sim control channel (driving inputs).
/// The sim listens here; the viewer dials lazily. Kept adjacent to
/// [`DEFAULT_PORT`] so both pass through one firewall rule / SSH forward.
pub const DEFAULT_CONTROL_PORT: u16 = DEFAULT_PORT + 1;

/// Sanity cap on one frame; real frames are a few KB (see size gates).
pub const MAX_FRAME_BYTES: usize = 4 * 1024 * 1024;

/// Sanity cap on one control message; real ones postcard to ~dozens of
/// bytes (8 floats + flags + indices), so this is a corrupt-stream tripwire.
pub const MAX_CONTROL_BYTES: usize = 1024;

/// One streamed tick: full arena snapshot + tick dt (`TICK_TIME`, 1/120).
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct VisFrame {
    pub state: ArenaState,
    pub dt: f32,
}

impl VisFrame {
    /// Snapshots `state` with tick `dt`.
    pub fn new(state: ArenaState, dt: f32) -> Self {
        Self { state, dt }
    }
}

/// One viewer→sim driving update: inputs for `car_index` with a sender-side
/// sequence number (`seq`, starts at 1, +1 per send).
///
/// Same framing as frames (`[u32 LE len][postcard]`); the sim keeps only the
/// NEWEST message per tick and drops stale ones (monotonic `seq` lets it tell
/// a reconnect replay from fresh input). `CarControls` already implements
/// `Serialize` under `rocketsim/serde` (verified in `car_controls.rs`).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ControlMsg {
    /// Which arena car these inputs drive (sim clamps to its car count).
    pub car_index: u32,
    /// Driver inputs (analog `-1..1`, clamped again by `set_car_controls`).
    pub controls: CarControls,
    /// Action buttons as level state (bit0 = reset kickoff, bit1 = dribble
    /// ball, bit2 = launch ball); the sim fires each on its rising edge.
    pub buttons: u8,
    /// Sender sequence (monotonic per connection; gaps = dropped frames).
    pub seq: u64,
}

impl ControlMsg {
    /// Builds message `seq` for `car_index` with `controls` and `buttons`.
    pub fn new(car_index: u32, controls: CarControls, buttons: u8, seq: u64) -> Self {
        Self {
            car_index,
            controls,
            buttons,
            seq,
        }
    }
}

/// Encodes a control message ready for `write_all` (length prefix + payload).
pub fn encode_control(msg: &ControlMsg) -> Vec<u8> {
    let payload = postcard::to_allocvec(msg).expect("ControlMsg must serialize");
    let mut out = Vec::with_capacity(4 + payload.len());
    out.extend_from_slice(&(payload.len() as u32).to_le_bytes());
    out.extend_from_slice(&payload);
    out
}

/// Decodes one control message from the front of `buf` (same contract as
/// [`decode_frame`]: `Ok(None)` = incomplete, `Err` = oversize/corrupt).
pub fn decode_control(buf: &[u8]) -> Result<Option<(ControlMsg, usize)>, postcard::Error> {
    if buf.len() < 4 {
        return Ok(None);
    }
    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
    if len > MAX_CONTROL_BYTES {
        // No bytes to decode; manufacture the error type from empty input.
        return Err(postcard::from_bytes::<ControlMsg>(&[]).unwrap_err());
    }
    if buf.len() < 4 + len {
        return Ok(None);
    }
    postcard::from_bytes::<ControlMsg>(&buf[4..4 + len]).map(|msg| Some((msg, 4 + len)))
}
/// Encodes a frame ready for `write_all` (length prefix + postcard payload).
pub fn encode_frame(frame: &VisFrame) -> Vec<u8> {
    let payload = postcard::to_allocvec(frame).expect("VisFrame must serialize");
    let mut out = Vec::with_capacity(4 + payload.len());
    out.extend_from_slice(&(payload.len() as u32).to_le_bytes());
    out.extend_from_slice(&payload);
    out
}

/// Decodes one frame from the front of `buf`.
///
/// * `Ok(None)` — fewer than 4 header bytes, or the payload is incomplete.
/// * `Ok(Some((frame, consumed)))` — `consumed` covers header + payload.
/// * `Err` — declared length exceeds [`MAX_FRAME_BYTES`] or postcard decode
///   failed (caller should drop the connection and re-accept).
pub fn decode_frame(buf: &[u8]) -> Result<Option<(VisFrame, usize)>, postcard::Error> {
    if buf.len() < 4 {
        return Ok(None);
    }
    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
    if len > MAX_FRAME_BYTES {
        // No bytes to decode; manufacture the error type from empty input.
        return Err(postcard::from_bytes::<VisFrame>(&[]).unwrap_err());
    }
    if buf.len() < 4 + len {
        return Ok(None);
    }
    postcard::from_bytes::<VisFrame>(&buf[4..4 + len]).map(|frame| Some((frame, 4 + len)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::photo;

    fn frame_with_cars(n_blue: usize, n_orange: usize) -> VisFrame {
        photo::init_meshes();
        let mut arena = rocketsim::Arena::new(rocketsim::GameMode::Soccar);
        for _ in 0..n_blue {
            arena.add_car(rocketsim::Team::Blue, rocketsim::CarBodyConfig::OCTANE);
        }
        for _ in 0..n_orange {
            arena.add_car(rocketsim::Team::Orange, rocketsim::CarBodyConfig::DOMINUS);
        }
        arena.reset_to_random_kickoff(Some(0));
        VisFrame::new(arena.get_arena_state(), 1.0 / 120.0)
    }

    #[test]
    fn round_trip_empty_and_full() {
        for (nb, no) in [(0, 0), (1, 1), (3, 3)] {
            let frame = frame_with_cars(nb, no);
            let bytes = encode_frame(&frame);
            // ~3KB bincode baseline for 6 cars; postcard is at/under that.
            assert!(bytes.len() < 16 * 1024, "frame too big: {}", bytes.len());
            let (back, consumed) = decode_frame(&bytes).unwrap().unwrap();
            assert_eq!(consumed, bytes.len());
            assert_eq!(back.state.cars.len(), nb + no);
            assert!((back.dt - 1.0 / 120.0).abs() < 1e-6);
            assert_eq!(back.state.tick_count, frame.state.tick_count);
        }
    }

    #[test]
    fn partial_reads_return_none() {
        let bytes = encode_frame(&frame_with_cars(3, 3));
        assert!(decode_frame(&bytes[..2]).unwrap().is_none());
        assert!(decode_frame(&bytes[..4]).unwrap().is_none());
        assert!(decode_frame(&bytes[..bytes.len() - 1]).unwrap().is_none());
    }

    #[test]
    fn oversize_length_is_error() {
        let mut bad = (MAX_FRAME_BYTES as u32 + 1).to_le_bytes().to_vec();
        bad.extend_from_slice(&[0u8; 8]);
        assert!(decode_frame(&bad).is_err());
    }

    #[test]
    fn control_round_trip() {
        let msg = ControlMsg::new(
            2,
            rocketsim::CarControls {
                throttle: 1.0,
                steer: -0.5,
                pitch: 0.25,
                yaw: -0.5,
                roll: 1.0,
                jump: true,
                boost: true,
                handbrake: false,
            },
            0b101,
            42,
        );
        let bytes = encode_control(&msg);
        // Compact: floats + flags + indices must stay far under the cap.
        assert!(bytes.len() < 128, "control frame too big: {}", bytes.len());
        let (back, consumed) = decode_control(&bytes).unwrap().unwrap();
        assert_eq!(consumed, bytes.len());
        assert_eq!(back, msg);
        assert_eq!(back.buttons, 0b101);
    }

    #[test]
    fn control_partial_and_oversize() {
        let msg = ControlMsg::new(0, rocketsim::CarControls::default(), 0, 1);
        let bytes = encode_control(&msg);
        assert!(decode_control(&bytes[..2]).unwrap().is_none());
        assert!(decode_control(&bytes[..bytes.len() - 1]).unwrap().is_none());
        let mut bad = (MAX_CONTROL_BYTES as u32 + 1).to_le_bytes().to_vec();
        bad.extend_from_slice(&[0u8; 8]);
        assert!(decode_control(&bad).is_err());
    }

    #[test]
    fn control_ports_adjacent() {
        assert_eq!(DEFAULT_CONTROL_PORT, DEFAULT_PORT + 1);
        assert_eq!(DEFAULT_CONTROL_PORT, 39202);
    }
}
