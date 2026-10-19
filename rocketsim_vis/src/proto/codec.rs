use super::messages::*;
use rocketsim::CarControls;

/// Encodes one control message as a length-prefixed protocol message.
///
/// The length is little-endian and includes the version and payload. The
/// version is [`PROTOCOL_VERSION`]. The result can be passed to
/// [`std::io::Write::write_all`]. This function does not validate message values.
pub fn encode_control(msg: &ControlMsg) -> Vec<u8> {
    let payload = postcard::to_allocvec(msg).expect("ControlMsg must serialize");
    let wire_len = payload.len() + 4;
    let mut out = Vec::with_capacity(4 + wire_len);
    out.extend_from_slice(&(wire_len as u32).to_le_bytes());
    out.extend_from_slice(&PROTOCOL_VERSION.to_le_bytes());
    out.extend_from_slice(&payload);
    out
}

/// Decodes one control message from the front of `buf`.
/// Returns `Ok(None)` when the message is incomplete. The consumed byte count
/// includes the length prefix, version, and payload.
///
/// # Errors
///
/// Returns [`ProtocolError::InvalidLength`] for a length outside
/// `4..=MAX_CONTROL_BYTES`, [`ProtocolError::VersionMismatch`] for another
/// version, [`ProtocolError::Deserialize`] for invalid postcard data, or
/// [`ProtocolError::InvalidData`] when analog values are not finite.
pub fn decode_control(buf: &[u8]) -> Result<Option<(ControlMsg, usize)>, ProtocolError> {
    if buf.len() < 4 {
        return Ok(None);
    }
    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
    if !(4..=MAX_CONTROL_BYTES).contains(&len) {
        return Err(ProtocolError::InvalidLength {
            length: len,
            min: 4,
            max: MAX_CONTROL_BYTES,
        });
    }
    if buf.len() < 8 {
        return Ok(None);
    }
    let version = u32::from_le_bytes(buf[4..8].try_into().unwrap());
    if version != PROTOCOL_VERSION {
        return Err(ProtocolError::VersionMismatch {
            received: version,
            expected: PROTOCOL_VERSION,
        });
    }
    if buf.len() < 4 + len {
        return Ok(None);
    }
    let msg = postcard::from_bytes::<ControlMsg>(&buf[8..4 + len])?;
    if ![
        msg.controls.throttle,
        msg.controls.steer,
        msg.controls.pitch,
        msg.controls.yaw,
        msg.controls.roll,
    ]
    .iter()
    .all(|v| v.is_finite())
    {
        return Err(ProtocolError::InvalidData(
            "control analog values must be finite",
        ));
    }
    Ok(Some((msg, 4 + len)))
}
/// Encodes one multiplexed sim-to-viewer message.
///
/// The length is little-endian and includes the version and postcard payload.
/// The version is [`PROTOCOL_VERSION`].
///
/// # Errors
///
/// This function returns no `Result`. Serialization must succeed for
/// [`SimMsg`].
pub fn encode_sim(msg: &SimMsg) -> Vec<u8> {
    let payload = postcard::to_allocvec(msg).expect("SimMsg must serialize");
    let wire_len = payload.len() + 4;
    let mut out = Vec::with_capacity(4 + wire_len);
    out.extend_from_slice(&(wire_len as u32).to_le_bytes());
    out.extend_from_slice(&PROTOCOL_VERSION.to_le_bytes());
    out.extend_from_slice(&payload);
    out
}

/// Decodes one multiplexed sim→viewer message from the front of `buf`.
///
/// Returns `Ok(None)` when the message is incomplete. The consumed byte count
/// includes the length prefix, version, and payload.
///
/// # Errors
///
/// Returns [`ProtocolError::InvalidLength`] for a length outside
/// `4..=MAX_SIM_MSG_BYTES`, [`ProtocolError::VersionMismatch`] for another
/// version, [`ProtocolError::Deserialize`] for invalid postcard data, or
/// [`ProtocolError::InvalidData`] for invalid frame values. A debug chunk can
/// also fail [`DebugChunk::validate_framing`].
pub fn decode_sim(buf: &[u8]) -> Result<Option<(SimMsg, usize)>, ProtocolError> {
    if buf.len() < 4 {
        return Ok(None);
    }
    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
    if !(4..=MAX_SIM_MSG_BYTES).contains(&len) {
        return Err(ProtocolError::InvalidLength {
            length: len,
            min: 4,
            max: MAX_SIM_MSG_BYTES,
        });
    }
    if buf.len() < 8 {
        return Ok(None);
    }
    let version = u32::from_le_bytes(buf[4..8].try_into().unwrap());
    if version != PROTOCOL_VERSION {
        return Err(ProtocolError::VersionMismatch {
            received: version,
            expected: PROTOCOL_VERSION,
        });
    }
    if buf.len() < 4 + len {
        return Ok(None);
    }
    let msg = postcard::from_bytes::<SimMsg>(&buf[8..4 + len])?;
    match &msg {
        SimMsg::Frame(frame) => {
            // Reuse frame validation without re-encoding: decode inner via a
            // temporary re-encode is wasteful, so inline the same checks.
            let finite_vec = |v: glam::Vec3A| v.to_array().iter().all(|x| x.is_finite());
            let valid_phys = |p: &rocketsim::PhysState| {
                finite_vec(p.pos)
                    && finite_vec(p.vel)
                    && finite_vec(p.ang_vel)
                    && finite_vec(p.rot_mat.x_axis)
                    && finite_vec(p.rot_mat.y_axis)
                    && finite_vec(p.rot_mat.z_axis)
            };
            let valid_controls = |c: &CarControls| {
                [c.throttle, c.steer, c.pitch, c.yaw, c.roll]
                    .iter()
                    .all(|v| v.is_finite())
            };
            if !frame.dt.is_finite() || !(0.0..=1.0).contains(&frame.dt) {
                return Err(ProtocolError::InvalidData(
                    "frame dt must be finite and in 0..=1",
                ));
            }
            if !valid_phys(&frame.state.ball.phys)
                || frame.state.cars.len() > 256
                || frame.state.cars.iter().any(|(_, car)| {
                    !valid_phys(&car.phys)
                        || !valid_controls(&car.controls)
                        || !valid_controls(car.prev_controls())
                })
                || frame.state.boost_pads.len() > 4096
            {
                return Err(ProtocolError::InvalidData("arena state has invalid values"));
            }
            if let Some(scene) = frame.scene.as_ref()
                && (scene.game_mode != frame.state.game_mode() || !scene.validate())
            {
                return Err(ProtocolError::InvalidData(
                    "scene metadata failed validation",
                ));
            }
        }
        SimMsg::DebugChunk(chunk) => {
            chunk.validate_framing()?;
        }
    }
    Ok(Some((msg, 4 + len)))
}

/// Encodes a retained-debug publication after validation.
///
/// # Errors
///
/// Returns [`ProtocolError::InvalidData`] if the message fails validation or
/// exceeds [`MAX_DEBUG_SYNC_BYTES`]. Returns
/// [`ProtocolError::Deserialize`] if postcard serialization fails.
pub fn encode_debug_sync(msg: &DebugSyncMsg) -> Result<Vec<u8>, ProtocolError> {
    match msg {
        DebugSyncMsg::Full(v) => v.validate()?,
        DebugSyncMsg::Delta(v) => v.validate()?,
    }
    let bytes = postcard::to_allocvec(msg).map_err(ProtocolError::from)?;
    if bytes.len() > MAX_DEBUG_SYNC_BYTES {
        return Err(ProtocolError::InvalidData("debug sync too large"));
    }
    Ok(bytes)
}

/// Decodes and validates one reassembled [`DebugSyncMsg`] payload.
///
/// # Errors
///
/// Returns [`ProtocolError::InvalidData`] if the payload exceeds
/// [`MAX_DEBUG_SYNC_BYTES`] or its values fail validation. Returns
/// [`ProtocolError::Deserialize`] if postcard cannot decode the payload.
pub fn decode_debug_sync(bytes: &[u8]) -> Result<DebugSyncMsg, ProtocolError> {
    if bytes.len() > MAX_DEBUG_SYNC_BYTES {
        return Err(ProtocolError::InvalidData("debug sync too large"));
    }
    let msg = postcard::from_bytes::<DebugSyncMsg>(bytes)?;
    match &msg {
        DebugSyncMsg::Full(v) => v.validate()?,
        DebugSyncMsg::Delta(v) => v.validate()?,
    }
    Ok(msg)
}

/// Split encoded scene bytes into at most 512 chunk messages.
///
/// Use bytes returned by [`encode_debug_sync`]. Each chunk contains at most
/// [`DEBUG_CHUNK_SIZE`] bytes and the supplied `transfer_id`. Serialize each
/// message with [`encode_sim`] before sending it. Return an empty vector for
/// an empty input. This function does not validate input and truncates bytes
/// beyond [`MAX_DEBUG_SYNC_BYTES`].
#[must_use]
pub fn split_debug_sync(encoded: &[u8], transfer_id: u64) -> Vec<SimMsg> {
    if encoded.is_empty() {
        return Vec::new();
    }
    let total = encoded
        .chunks(DEBUG_CHUNK_SIZE)
        .len()
        .clamp(1, MAX_CHUNKS_PER_TRANSFER);
    encoded
        .chunks(DEBUG_CHUNK_SIZE)
        .take(MAX_CHUNKS_PER_TRANSFER)
        .enumerate()
        .map(|(i, slice)| {
            SimMsg::DebugChunk(DebugChunk {
                transfer_id,
                index: i as u32,
                total: total as u32,
                bytes: slice.to_vec(),
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_codec_round_trips_and_rejects_truncation() {
        let message = ControlMsg::new(2, CarControls::default(), 5, 42);
        let bytes = encode_control(&message);
        assert!(decode_control(&bytes[..2]).unwrap().is_none());
        let (decoded, consumed) = decode_control(&bytes).unwrap().unwrap();
        assert_eq!(decoded, message);
        assert_eq!(consumed, bytes.len());
    }

    #[test]
    fn multiplexed_frame_accepts_large_scene_metadata() {
        let arena = rocketsim::Arena::new(rocketsim::GameMode::TheVoid);
        let mut frame = VisFrame::new(arena.get_arena_state(), 1.0 / 120.0);
        let mut scene = SceneMetadata::empty(rocketsim::GameMode::TheVoid);
        scene.meshes.push(MeshData {
            vertices: (0..6000).map(|index| [index as f32, 0.0, 0.0]).collect(),
            indices: Vec::new(),
        });
        frame.scene = Some(scene);
        let bytes = encode_sim(&SimMsg::Frame(frame));
        assert!(bytes.len() > 64 * 1024);
        assert!(bytes.len() <= MAX_SIM_MSG_BYTES + 4);
        let (message, consumed) = decode_sim(&bytes).unwrap().unwrap();
        assert_eq!(consumed, bytes.len());
        let SimMsg::Frame(decoded) = message else {
            panic!("expected a frame")
        };
        assert_eq!(decoded.scene.unwrap().meshes[0].vertices.len(), 6000);
    }

    #[test]
    fn multiplexed_codec_checks_version_and_length() {
        let arena = rocketsim::Arena::new(rocketsim::GameMode::TheVoid);
        let bytes = encode_sim(&SimMsg::Frame(VisFrame::new(
            arena.get_arena_state(),
            1.0 / 120.0,
        )));
        assert!(decode_sim(&bytes[..4]).unwrap().is_none());
        let mut wrong_version = bytes.clone();
        wrong_version[4..8].copy_from_slice(&2u32.to_le_bytes());
        assert!(matches!(
            decode_sim(&wrong_version),
            Err(ProtocolError::VersionMismatch { .. })
        ));
        let mut bad_length = (MAX_SIM_MSG_BYTES as u32 + 1).to_le_bytes().to_vec();
        bad_length.extend_from_slice(&[0; 8]);
        assert!(matches!(
            decode_sim(&bad_length),
            Err(ProtocolError::InvalidLength { .. })
        ));
    }
}
