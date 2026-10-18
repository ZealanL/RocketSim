//! Wire protocol: `[u32 LE frame_len][u32 LE version][postcard payload]` over TCP.
//!
//! `frame_len` includes the version field and payload, but not its own prefix.
//! The sim dials; the viewer listens (default port [`DEFAULT_PORT`]).
//! Keep [`PROTOCOL_VERSION`] at 1 until the first release.

use rocketsim::{ArenaState, CarControls};

/// Unreleased wire protocol. Keep this at 1 until the first release.
pub const PROTOCOL_VERSION: u32 = 1;

/// Default TCP port. Deliberately different from rlviser's 34254.
pub const DEFAULT_PORT: u16 = 39201;

/// Legacy TCP port for the standalone control server API.
/// Stock viewers send controls on the accepted frame stream and do not use this port.
pub const DEFAULT_CONTROL_PORT: u16 = DEFAULT_PORT + 1;

/// Sanity cap on one frame; real frames are a few KB.
pub const MAX_FRAME_BYTES: usize = 32 * 1024 * 1024;
/// Limit scene geometry separately from snapshots and debug data.
pub const MAX_SCENE_BYTES: usize = 24 * 1024 * 1024;

/// Sanity cap on one control message (corrupt-stream tripwire).
pub const MAX_CONTROL_BYTES: usize = 1024;

/// Cap on one multiplexed sim→viewer message (frame or debug chunk).
/// The first frame includes scene metadata, so it uses the existing frame cap.
pub const MAX_SIM_MSG_BYTES: usize = MAX_FRAME_BYTES;
/// Cap on a reassembled retained-debug transfer (postcard `DebugSyncMsg`).
pub const MAX_DEBUG_SYNC_BYTES: usize = 8 * 1024 * 1024;
/// Payload bytes per debug chunk (frames interleave between chunks so
/// controls/physics keep progressing during large uploads).
pub const DEBUG_CHUNK_SIZE: usize = 16 * 1024;
/// Maximum chunks per debug transfer (`512 * 16 KiB` = `MAX_DEBUG_SYNC_BYTES`).
pub const MAX_CHUNKS_PER_TRANSFER: usize = 512;

#[derive(Debug)]
pub enum ProtocolError {
    VersionMismatch {
        received: u32,
        expected: u32,
    },
    InvalidLength {
        length: usize,
        min: usize,
        max: usize,
    },
    InvalidData(&'static str),
    Deserialize(postcard::Error),
}
impl std::fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::VersionMismatch { received, expected } => write!(
                f,
                "protocol version {received} is unsupported; expected {expected}"
            ),
            Self::InvalidLength { length, min, max } => {
                write!(f, "wire payload length {length} is outside {min}..={max}")
            }
            Self::InvalidData(reason) => write!(f, "invalid protocol data: {reason}"),
            Self::Deserialize(error) => write!(f, "could not decode postcard payload: {error}"),
        }
    }
}
impl std::error::Error for ProtocolError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Deserialize(error) => Some(error),
            _ => None,
        }
    }
}
impl From<postcard::Error> for ProtocolError {
    fn from(error: postcard::Error) -> Self {
        Self::Deserialize(error)
    }
}

/// One streamed tick: full arena snapshot + tick dt (`TICK_TIME`, 1/120).
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct VisFrame {
    pub state: ArenaState,
    pub dt: f32,
    /// Simulation update clock used to evaluate retained debug lifetimes.
    #[serde(default)]
    pub debug_tick: u64,
    #[serde(default)]
    pub debug: DebugDraw,
    /// Scene metadata is optional for raw and synthetic clients.
    #[serde(default)]
    pub scene: Option<SceneMetadata>,
}

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct DebugDraw {
    pub lines: Vec<DebugLine>,
    pub text: Vec<DebugText>,
    pub rects: Vec<DebugRect>,
    pub models: Vec<DebugModel>,
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct DebugLine {
    pub start: [f32; 3],
    pub end: [f32; 3],
    pub color: [f32; 4],
    pub width: f32,
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct DebugText {
    pub text: String,
    pub position: [f32; 2],
    pub color: [f32; 4],
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct DebugRect {
    pub min: [f32; 2],
    pub max: [f32; 2],
    pub color: [f32; 4],
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct DebugModel {
    pub model: String,
    pub texture: String,
    pub position: [f32; 3],
    pub rotation: [[f32; 3]; 3],
    pub scale: [f32; 3],
}
impl Default for DebugLine {
    fn default() -> Self {
        Self {
            start: [0.0; 3],
            end: [0.0; 3],
            color: [1.0; 4],
            width: 2.0,
        }
    }
}
impl Default for DebugText {
    fn default() -> Self {
        Self {
            text: String::new(),
            position: [0.0; 2],
            color: [1.0; 4],
        }
    }
}
impl Default for DebugRect {
    fn default() -> Self {
        Self {
            min: [0.0; 2],
            max: [0.0; 2],
            color: [1.0; 4],
        }
    }
}
impl Default for DebugModel {
    fn default() -> Self {
        Self {
            model: "ball".into(),
            texture: "ball".into(),
            position: [0.0; 3],
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            scale: [1.0; 3],
        }
    }
}

/// Sim→viewer multiplexed message (v4).
///
/// Physics frames stay latest-only; retained-debug transfers split into
/// `16 KiB` chunks so frames interleave and progress during large uploads.
/// A reconnect starts clean: the receiver resets its debug scene and the
/// sender opens with a `Full` transfer, so old deletes are never replayed.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[allow(clippy::large_enum_variant)]
pub enum SimMsg {
    /// Latest-only physics tick (may carry legacy per-tick [`DebugDraw`]).
    Frame(VisFrame),
    /// One chunk of a `DebugSyncMsg` transfer (atomic on reassembly).
    DebugChunk(DebugChunk),
}

/// One chunk of an encoded [`DebugSyncMsg`].
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct DebugChunk {
    /// Transfer id (sender-monotonic per connection).
    pub transfer_id: u64,
    /// Chunk index (`< total`).
    pub index: u32,
    /// Total chunks (`1..=MAX_CHUNKS_PER_TRANSFER`).
    pub total: u32,
    /// Slice of the postcard-encoded [`DebugSyncMsg`] (`<= DEBUG_CHUNK_SIZE`).
    pub bytes: Vec<u8>,
}

/// Retained-debug publication: full resync or per-group/resource delta.
///
/// Deltas carry only changed groups/resources plus removal lists:
/// a transform-only group update resends the small group record, never
/// unchanged mesh bytes (instances reference reusable ids).
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum DebugSyncMsg {
    /// Complete scene (first message after connect).
    Full(DebugFull),
    /// Incremental update within one ordered connection.
    Delta(DebugDelta),
}

/// Complete retained scene for a fresh session.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct DebugFull {
    pub groups: Vec<crate::debug::DebugGroup>,
    pub meshes: Vec<crate::debug::MeshResource>,
    pub textures: Vec<crate::debug::TextureResource>,
    /// Sender global scene revision.
    pub rev: u64,
}

/// Metadata-only update for an existing group's retained geometry.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct DebugGroupPatch {
    pub name: String,
    pub visible: bool,
    pub anchor: crate::debug::Anchor,
    pub transform: crate::debug::GroupTransform,
    pub lifetime: crate::debug::Lifetime,
    pub expires_tick: Option<u64>,
    pub expires_at_ms: Option<u64>,
    pub revision: u64,
}

/// Per-group/resource delta within one ordered stream.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct DebugDelta {
    pub upsert_groups: Vec<crate::debug::DebugGroup>,
    pub group_patches: Vec<DebugGroupPatch>,
    pub removed_groups: Vec<String>,
    pub upsert_meshes: Vec<crate::debug::MeshResource>,
    pub removed_meshes: Vec<String>,
    pub upsert_textures: Vec<crate::debug::TextureResource>,
    pub removed_textures: Vec<String>,
    /// Sender global scene revision after this delta.
    pub rev: u64,
}

impl DebugChunk {
    /// Validate framing only (payload validated after reassembly).
    pub fn validate_framing(&self) -> Result<(), ProtocolError> {
        if self.total == 0 || self.total as usize > MAX_CHUNKS_PER_TRANSFER {
            return Err(ProtocolError::InvalidData("debug chunk total out of range"));
        }
        if self.index >= self.total {
            return Err(ProtocolError::InvalidData("debug chunk index out of range"));
        }
        if self.bytes.len() > DEBUG_CHUNK_SIZE + 1024 {
            return Err(ProtocolError::InvalidData("debug chunk too large"));
        }
        Ok(())
    }
}

impl DebugFull {
    /// Validate counts, names, per-entry data, and total budget.
    pub fn validate(&self) -> Result<(), ProtocolError> {
        validate_debug_wire(&self.groups, &self.meshes, &self.textures, &[], &[], &[])
    }
}

impl DebugDelta {
    /// Validate counts, names, per-entry data, and total budget.
    /// Empty deltas are invalid (sender must not transmit them).
    pub fn validate(&self) -> Result<(), ProtocolError> {
        if self.upsert_groups.is_empty()
            && self.group_patches.is_empty()
            && self.removed_groups.is_empty()
            && self.upsert_meshes.is_empty()
            && self.removed_meshes.is_empty()
            && self.upsert_textures.is_empty()
            && self.removed_textures.is_empty()
        {
            return Err(ProtocolError::InvalidData("empty debug delta"));
        }
        validate_debug_wire(
            &self.upsert_groups,
            &self.upsert_meshes,
            &self.upsert_textures,
            &self.removed_groups,
            &self.removed_meshes,
            &self.removed_textures,
        )
    }

    /// True when nothing changes.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.upsert_groups.is_empty()
            && self.group_patches.is_empty()
            && self.removed_groups.is_empty()
            && self.upsert_meshes.is_empty()
            && self.removed_meshes.is_empty()
            && self.upsert_textures.is_empty()
            && self.removed_textures.is_empty()
    }
}

fn validate_debug_wire(
    groups: &[crate::debug::DebugGroup],
    meshes: &[crate::debug::MeshResource],
    textures: &[crate::debug::TextureResource],
    removed_groups: &[String],
    removed_meshes: &[String],
    removed_textures: &[String],
) -> Result<(), ProtocolError> {
    const BAD: &str = "debug sync failed validation";
    if groups.len() > crate::debug::MAX_GROUPS
        || meshes.len() > crate::debug::MAX_MESHES
        || textures.len() > crate::debug::MAX_TEXTURES
        || removed_groups.len() > crate::debug::MAX_GROUPS
        || removed_meshes.len() > crate::debug::MAX_MESHES
        || removed_textures.len() > crate::debug::MAX_TEXTURES
    {
        return Err(ProtocolError::InvalidData(BAD));
    }
    for name in removed_groups
        .iter()
        .chain(removed_meshes)
        .chain(removed_textures)
    {
        if crate::debug::validate_name(name).is_err() {
            return Err(ProtocolError::InvalidData(BAD));
        }
    }
    let mut bytes = 0usize;
    for g in groups {
        if g.validate().is_err() {
            return Err(ProtocolError::InvalidData(BAD));
        }
        bytes = bytes.saturating_add(g.estimated_bytes());
    }
    for m in meshes {
        if m.validate().is_err() {
            return Err(ProtocolError::InvalidData(BAD));
        }
        bytes = bytes.saturating_add(m.estimated_bytes());
    }
    for t in textures {
        if t.validate().is_err() {
            return Err(ProtocolError::InvalidData(BAD));
        }
        bytes = bytes.saturating_add(t.estimated_bytes());
    }
    if bytes > MAX_DEBUG_SYNC_BYTES {
        return Err(ProtocolError::InvalidData(BAD));
    }
    Ok(())
}

#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct MeshData {
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SceneMetadata {
    pub game_mode: rocketsim::GameMode,
    pub ball_radius: f32,
    pub meshes: Vec<MeshData>,
}

impl DebugDraw {
    pub fn validate(&self) -> bool {
        let count = self
            .lines
            .len()
            .saturating_add(self.text.len())
            .saturating_add(self.rects.len())
            .saturating_add(self.models.len());
        let finite3 = |v: &[f32; 3]| v.iter().all(|x| x.is_finite());
        let finite4 = |v: &[f32; 4]| v.iter().all(|x| x.is_finite());
        let estimated_bytes = self
            .lines
            .len()
            .saturating_mul(32)
            .saturating_add(self.text.iter().fold(0usize, |n, v| {
                n.saturating_add(v.text.len()).saturating_add(32)
            }))
            .saturating_add(self.rects.len().saturating_mul(40))
            .saturating_add(self.models.iter().fold(0usize, |n, v| {
                n.saturating_add(v.model.len())
                    .saturating_add(v.texture.len())
                    .saturating_add(100)
            }));
        count <= 100_000
            && estimated_bytes <= 8 * 1024 * 1024
            && self.lines.iter().all(|v| {
                finite3(&v.start)
                    && finite3(&v.end)
                    && finite4(&v.color)
                    && v.width.is_finite()
                    && v.width >= 0.0
            })
            && self.text.iter().all(|v| {
                v.text.len() <= 4096
                    && v.position.iter().all(|x| (0.0..=1.0).contains(x))
                    && finite4(&v.color)
            })
            && self.rects.iter().all(|v| {
                v.min.iter().all(|x| (0.0..=1.0).contains(x))
                    && v.max.iter().all(|x| (0.0..=1.0).contains(x))
                    && finite4(&v.color)
            })
            && self.models.iter().all(|v| {
                v.model.len() <= 256
                    && v.texture.len() <= 256
                    && finite3(&v.position)
                    && finite3(&v.scale)
                    && v.rotation.iter().all(finite3)
            })
    }
}

impl SceneMetadata {
    pub fn empty(game_mode: rocketsim::GameMode) -> Self {
        Self {
            game_mode,
            ball_radius: rocketsim::consts::ball::get_radius(game_mode),
            meshes: Vec::new(),
        }
    }

    /// Build metadata from the arena's loaded collision meshes.
    pub fn from_arena(arena: &rocketsim::Arena) -> Self {
        let game_mode = arena.get_arena_state().game_mode();
        let meshes = rocketsim::get_arena_collision_mesh_files(game_mode)
            .into_iter()
            .map(|mesh| MeshData {
                vertices: mesh
                    .get_vertices()
                    .iter()
                    .map(|v| (v * rocketsim::consts::BT_TO_UU).to_array())
                    .collect(),
                indices: mesh
                    .get_indices()
                    .iter()
                    .map(|&i| u32::try_from(i).unwrap_or(u32::MAX))
                    .collect(),
            })
            .collect();
        let metadata = Self {
            game_mode,
            ball_radius: arena.mutator_config().ball_radius,
            meshes,
        };
        assert!(
            metadata.validate(),
            "loaded arena meshes contain invalid data"
        );
        metadata
    }

    pub fn validate(&self) -> bool {
        if !self.ball_radius.is_finite() || self.ball_radius <= 0.0 || self.meshes.len() > 256 {
            return false;
        }
        let mut bytes = 0usize;
        for mesh in &self.meshes {
            if mesh.vertices.len() > 1_000_000
                || mesh.indices.len() > 3_000_000
                || mesh.indices.len() % 3 != 0
                || mesh.vertices.iter().flatten().any(|v| !v.is_finite())
                || mesh
                    .indices
                    .iter()
                    .any(|&i| (i as usize) >= mesh.vertices.len())
            {
                return false;
            }
            bytes = bytes
                .saturating_add(mesh.vertices.len().saturating_mul(12))
                .saturating_add(mesh.indices.len().saturating_mul(4));
            if bytes > MAX_SCENE_BYTES {
                return false;
            }
        }
        true
    }
}

impl VisFrame {
    /// Snapshots `state` with tick `dt`.
    pub fn new(state: ArenaState, dt: f32) -> Self {
        Self {
            debug_tick: state.tick_count,
            state,
            dt,
            debug: DebugDraw::default(),
            scene: None,
        }
    }
}

/// Level-state mutator bits for [`ControlMsg::mutators`] (1 = on).
/// Unlike `buttons`, these are NOT edge-triggered: the sim applies the
/// current level every tick (see `mutators` module for the emulation).
pub const MUT_UNLIMITED_BOOST: u8 = 0b0000_0001;
/// Passive boost regen while the sim runs.
pub const MUT_BOOST_RECHARGE: u8 = 0b0000_0010;
/// Re-arm flips while airborne.
pub const MUT_UNLIMITED_FLIPS: u8 = 0b0000_0100;
/// Re-arm double jumps while airborne.
pub const MUT_UNLIMITED_DOUBLE_JUMPS: u8 = 0b0000_1000;
/// Bits the sim honors; the rest are reserved (ignored on receive).
pub const MUT_KNOWN_MASK: u8 =
    MUT_UNLIMITED_BOOST | MUT_BOOST_RECHARGE | MUT_UNLIMITED_FLIPS | MUT_UNLIMITED_DOUBLE_JUMPS;

/// One viewer→sim driving update: inputs for `car_index` with a sender-side
/// sequence number (`seq`, starts at 1, +1 per send).
///
/// Same framing as frames (`[u32 LE len][postcard]`); the sim keeps only the
/// NEWEST message per tick and drops stale ones (monotonic `seq` tells a
/// reconnect replay from fresh input).
///
/// ```rust
/// use rocketsim::CarControls;
/// use rocketsim_vis::proto::{ControlMsg, decode_control, encode_control};
///
/// let msg = ControlMsg::new(0, CarControls::default(), 0b001, 1).with_mutators(0b0001);
/// let bytes = encode_control(&msg);
/// let (back, consumed) = decode_control(&bytes).unwrap().unwrap();
/// assert_eq!(back, msg);
/// assert_eq!(consumed, bytes.len());
/// ```
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ControlMsg {
    /// Which arena car these inputs drive (sim clamps to its car count).
    pub car_index: u32,
    /// Driver inputs (analog `-1..1`, clamped again by `set_car_controls`).
    pub controls: CarControls,
    /// Action buttons as level state (bit0 = reset kickoff, bit1 = dribble
    /// ball, bit2 = launch ball); the sim fires each on its rising edge.
    pub buttons: u8,
    /// Live mutators as level state (`MUT_*` bits); the sim applies the
    /// current level every tick. On link loss the sim freezes the last
    /// received level (controls still failsafe to neutral).
    pub mutators: u8,
    /// Sender sequence (monotonic per connection; gaps = dropped frames).
    pub seq: u64,
    /// Connection generation. Zero is reserved for legacy standalone APIs.
    #[serde(default)]
    pub generation: u64,
}

impl ControlMsg {
    /// Builds message `seq` for `car_index` with `controls` and `buttons`.
    /// Mutators default to off; set them with [`with_mutators`](Self::with_mutators).
    pub fn new(car_index: u32, controls: CarControls, buttons: u8, seq: u64) -> Self {
        Self {
            car_index,
            controls,
            buttons,
            mutators: 0,
            seq,
            generation: 0,
        }
    }

    /// Sets the level-state mutator bitmask (`MUT_*` bits).
    #[must_use]
    pub fn with_mutators(mut self, mutators: u8) -> Self {
        self.mutators = mutators;
        self
    }
}

/// Encodes a control message ready for `write_all` (length prefix + payload).
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
/// `Ok(None)` means the message is incomplete. `Err` gives a framing or data reason.
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
/// Encodes a frame ready for `write_all` (length prefix + postcard payload).
pub fn encode_frame(frame: &VisFrame) -> Vec<u8> {
    let payload = postcard::to_allocvec(frame).expect("VisFrame must serialize");
    let wire_len = payload.len() + 4;
    let mut out = Vec::with_capacity(4 + wire_len);
    out.extend_from_slice(&(wire_len as u32).to_le_bytes());
    out.extend_from_slice(&PROTOCOL_VERSION.to_le_bytes());
    out.extend_from_slice(&payload);
    out
}

/// Decodes one frame from the front of `buf`.
///
/// * `Ok(None)` — fewer than 4 header bytes, or the payload is incomplete.
/// * `Ok(Some((frame, consumed)))` — `consumed` covers header + payload.
/// * `Err` — framing, version, deserialization, or value validation failed.
pub fn decode_frame(buf: &[u8]) -> Result<Option<(VisFrame, usize)>, ProtocolError> {
    if buf.len() < 4 {
        return Ok(None);
    }
    let len = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
    if !(4..=MAX_FRAME_BYTES).contains(&len) {
        return Err(ProtocolError::InvalidLength {
            length: len,
            min: 4,
            max: MAX_FRAME_BYTES,
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
    let frame = postcard::from_bytes::<VisFrame>(&buf[8..4 + len])?;
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
        || frame
            .state
            .boost_pads
            .iter()
            .any(|(pad, state)| !finite_vec(pad.pos) || !state.cooldown.is_finite())
    {
        return Err(ProtocolError::InvalidData(
            "arena state has invalid numeric values or counts",
        ));
    }
    if let Some(scene) = frame.scene.as_ref() {
        if scene.game_mode != frame.state.game_mode() {
            return Err(ProtocolError::InvalidData(
                "scene game mode does not match frame state",
            ));
        }
        if !scene.validate() {
            return Err(ProtocolError::InvalidData(
                "scene metadata failed validation",
            ));
        }
    }
    if !frame.debug.validate() {
        return Err(ProtocolError::InvalidData("debug draw failed validation"));
    }
    Ok(Some((frame, 4 + len)))
}

/// Encodes one multiplexed sim→viewer message (`Frame` or `DebugChunk`).
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
/// `Ok(None)` means incomplete; `Err` is framing/version/data.
/// Malformed payloads never panic (bounded checks first).
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
            if !frame.debug.validate() {
                return Err(ProtocolError::InvalidData("debug draw failed validation"));
            }
        }
        SimMsg::DebugChunk(chunk) => {
            chunk.validate_framing()?;
        }
    }
    Ok(Some((msg, 4 + len)))
}

/// Encodes a retained-debug publication (validated first).
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

/// Decodes and validates one reassembled `DebugSyncMsg` payload.
/// Never panics on malformed input.
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

/// Splits an encoded sync payload into `<= DEBUG_CHUNK_SIZE` chunks.
/// Returns wire-ready `SimMsg::DebugChunk` messages with `transfer_id`.
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
    fn arena_metadata_uses_world_units_and_valid_indices() {
        photo::init_meshes();
        let arena = photo::build_arena();
        let metadata = SceneMetadata::from_arena(&arena);
        assert!(metadata.validate());
        assert_eq!(metadata.game_mode, rocketsim::GameMode::Soccar);
        assert!((metadata.ball_radius - 91.25).abs() < 0.001);
        assert!(!metadata.meshes.is_empty());
        assert!(
            metadata
                .meshes
                .iter()
                .all(|m| m.indices.iter().all(|&i| (i as usize) < m.vertices.len()))
        );
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
    fn protocol_round_trips_car_tuning_and_contact_cache() {
        let mut frame = frame_with_cars(1, 0);
        let car = &mut frame.state.cars[0].1;
        car.handbrake_val = 0.375;
        car.time_since_boosted = 1.75;
        car.set_contact_flags(true, [true, false, true, false]);
        car.set_prev_controls(CarControls {
            jump: true,
            throttle: 0.25,
            ..CarControls::default()
        });

        let encoded = encode_frame(&frame);
        assert_eq!(
            u32::from_le_bytes(encoded[4..8].try_into().unwrap()),
            PROTOCOL_VERSION
        );
        let (decoded, _) = decode_frame(&encoded).unwrap().unwrap();
        let car = &decoded.state.cars[0].1;
        assert_eq!(car.handbrake_val, 0.375);
        assert_eq!(car.time_since_boosted, 1.75);
        assert!(car.is_on_ground());
        assert_eq!(car.num_wheels_in_contact(), 2);
        assert!(car.prev_controls().jump);
        assert_eq!(car.prev_controls().throttle, 0.25);
    }

    #[test]
    fn frame_rejects_nonfinite_positions_and_rotations() {
        let mut frame = frame_with_cars(1, 0);
        frame.state.ball.phys.pos.x = f32::NAN;
        assert!(decode_frame(&encode_frame(&frame)).is_err());
        let mut frame = frame_with_cars(1, 0);
        frame.state.cars[0].1.phys.rot_mat =
            glam::Mat3A::from_cols(glam::Vec3A::splat(f32::NAN), glam::Vec3A::Y, glam::Vec3A::Z);
        assert!(decode_frame(&encode_frame(&frame)).is_err());
    }

    #[test]
    fn scene_bearing_sim_frame_round_trips_above_64k() {
        let mut frame = frame_with_cars(0, 0);
        let mut scene = SceneMetadata::empty(rocketsim::GameMode::Soccar);
        scene.meshes.push(MeshData {
            vertices: (0..6000).map(|i| [i as f32, 0.0, 0.0]).collect(),
            indices: Vec::new(),
        });
        assert!(scene.validate());
        frame.scene = Some(scene);
        let bytes = encode_sim(&SimMsg::Frame(frame.clone()));
        assert!(bytes.len() > 64 * 1024);
        assert!(bytes.len() <= MAX_SIM_MSG_BYTES + 4);
        let (decoded, consumed) = decode_sim(&bytes).unwrap().unwrap();
        assert_eq!(consumed, bytes.len());
        let SimMsg::Frame(decoded) = decoded else {
            panic!("expected frame")
        };
        assert_eq!(decoded.scene.unwrap().meshes[0].vertices.len(), 6000);
    }

    #[test]
    fn frame_framing_edge_cases() {
        let bytes = encode_frame(&frame_with_cars(3, 3));
        assert!(decode_frame(&bytes[..2]).unwrap().is_none());
        assert!(decode_frame(&bytes[..4]).unwrap().is_none());
        assert!(decode_frame(&bytes[..bytes.len() - 1]).unwrap().is_none());
        let mut bad = (MAX_FRAME_BYTES as u32 + 1).to_le_bytes().to_vec();
        bad.extend_from_slice(&[0u8; 8]);
        assert!(decode_frame(&bad).is_err());
    }

    #[test]
    fn control_round_trip() {
        assert_eq!(DEFAULT_CONTROL_PORT, DEFAULT_PORT + 1);
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
        assert_eq!(back.mutators, 0);
    }

    #[test]
    fn wire_errors_distinguish_version_length_and_numeric_data() {
        let mut frame = encode_frame(&frame_with_cars(0, 0));
        frame[4..8].copy_from_slice(&2u32.to_le_bytes());
        assert!(matches!(
            decode_frame(&frame),
            Err(ProtocolError::VersionMismatch {
                received: 2,
                expected: PROTOCOL_VERSION
            })
        ));
        frame[4] = (PROTOCOL_VERSION as u8).wrapping_add(1);
        assert!(matches!(
            decode_frame(&frame),
            Err(ProtocolError::VersionMismatch { .. })
        ));
        let mut bad_length = 0u32.to_le_bytes().to_vec();
        bad_length.extend_from_slice(&[0; 8]);
        assert!(matches!(
            decode_frame(&bad_length),
            Err(ProtocolError::InvalidLength { .. })
        ));
        let mut malformed = 5u32.to_le_bytes().to_vec();
        malformed.extend_from_slice(&PROTOCOL_VERSION.to_le_bytes());
        malformed.push(0xff);
        assert!(matches!(
            decode_frame(&malformed),
            Err(ProtocolError::Deserialize(_))
        ));
        let mut controls = encode_control(&ControlMsg::new(0, CarControls::default(), 0, 1));
        controls[4] = (PROTOCOL_VERSION as u8).wrapping_add(1);
        assert!(matches!(
            decode_control(&controls),
            Err(ProtocolError::VersionMismatch { .. })
        ));
        let mut malformed_control = 5u32.to_le_bytes().to_vec();
        malformed_control.extend_from_slice(&PROTOCOL_VERSION.to_le_bytes());
        malformed_control.push(0xff);
        assert!(matches!(
            decode_control(&malformed_control),
            Err(ProtocolError::Deserialize(_))
        ));
        let invalid_controls = encode_control(&ControlMsg::new(
            0,
            CarControls {
                throttle: f32::NAN,
                ..CarControls::default()
            },
            0,
            1,
        ));
        assert!(matches!(
            decode_control(&invalid_controls),
            Err(ProtocolError::InvalidData(_))
        ));
        let mut scene = SceneMetadata::empty(rocketsim::GameMode::Soccar);
        scene.meshes.push(MeshData {
            vertices: vec![[0.0, 0.0, 0.0]],
            indices: vec![1],
        });
        assert!(!scene.validate());
        scene.meshes[0].indices = vec![0, 0];
        assert!(!scene.validate());
    }

    #[test]
    fn optional_scene_must_match_frame_mode() {
        let mut frame = frame_with_cars(0, 0);
        frame.scene = Some(SceneMetadata::empty(rocketsim::GameMode::Hoops));
        assert!(matches!(
            decode_frame(&encode_frame(&frame)),
            Err(ProtocolError::InvalidData(_))
        ));
    }

    #[test]
    fn debug_model_default_is_an_identity_ball() {
        let model = DebugModel::default();
        assert_eq!(model.model, "ball");
        assert_eq!(model.texture, "ball");
        assert_eq!(
            model.rotation,
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        );
        assert_eq!(model.scale, [1.0; 3]);
        assert_eq!(DebugLine::default().color, [1.0; 4]);
        assert_eq!(DebugLine::default().width, 2.0);
        assert_eq!(DebugText::default().color, [1.0; 4]);
        assert_eq!(DebugRect::default().color, [1.0; 4]);
        assert!(
            DebugDraw {
                models: vec![model],
                ..DebugDraw::default()
            }
            .validate()
        );
    }

    #[test]
    fn control_mutators_round_trip_stays_compact() {
        let msg = ControlMsg::new(0, rocketsim::CarControls::default(), 0, 1)
            .with_mutators(MUT_UNLIMITED_BOOST | MUT_UNLIMITED_FLIPS);
        let bytes = encode_control(&msg);
        // One bitmask byte on the wire: still far under the cap.
        assert!(bytes.len() < 128, "control frame too big: {}", bytes.len());
        let (back, consumed) = decode_control(&bytes).unwrap().unwrap();
        assert_eq!(consumed, bytes.len());
        assert_eq!(back, msg);
        assert_eq!(back.mutators, MUT_UNLIMITED_BOOST | MUT_UNLIMITED_FLIPS);
        // Builder only touches the mutator field.
        let plain = ControlMsg::new(0, rocketsim::CarControls::default(), 0, 1);
        assert_eq!(plain.mutators, 0);
    }

    #[test]
    fn control_framing_edge_cases() {
        let msg = ControlMsg::new(0, rocketsim::CarControls::default(), 0, 1);
        let bytes = encode_control(&msg);
        assert!(decode_control(&bytes[..2]).unwrap().is_none());
        assert!(decode_control(&bytes[..bytes.len() - 1]).unwrap().is_none());
        let mut bad = (MAX_CONTROL_BYTES as u32 + 1).to_le_bytes().to_vec();
        bad.extend_from_slice(&[0u8; 8]);
        assert!(decode_control(&bad).is_err());
    }

    fn debug_test_group(name: &str) -> crate::debug::DebugGroup {
        use crate::debug::{Color, DepthMode, DrawList, GroupOptions, Width};
        let mut list = DrawList::new();
        list.line(
            [0.0, 0.0, 0.0],
            [100.0, 0.0, 0.0],
            Color::RED,
            Width::pixels(2.0),
            DepthMode::Test,
        );
        list.label_world(
            "hi",
            [0.0, 0.0, 100.0],
            14.0,
            crate::debug::TextAlign::Center,
            Color::WHITE,
            None,
        );
        crate::debug::DebugGroup::new(name, list, GroupOptions::default()).unwrap()
    }

    #[test]
    fn debug_sync_full_roundtrip() {
        let full = DebugFull {
            groups: vec![debug_test_group("a"), debug_test_group("b")],
            meshes: vec![],
            textures: vec![],
            rev: 7,
        };
        let msg = DebugSyncMsg::Full(full);
        let encoded = encode_debug_sync(&msg).unwrap();
        let back = decode_debug_sync(&encoded).unwrap();
        match back {
            DebugSyncMsg::Full(v) => {
                assert_eq!(v.groups.len(), 2);
                assert_eq!(v.rev, 7);
            }
            DebugSyncMsg::Delta(_) => panic!("expected Full"),
        }
    }

    #[test]
    fn debug_sync_delta_roundtrip_and_chunk_split() {
        let delta = DebugDelta {
            upsert_groups: vec![debug_test_group("only")],
            group_patches: vec![],
            removed_groups: vec!["old".into()],
            upsert_meshes: vec![],
            removed_meshes: vec![],
            upsert_textures: vec![],
            removed_textures: vec![],
            rev: 9,
        };
        let msg = DebugSyncMsg::Delta(delta);
        let encoded = encode_debug_sync(&msg).unwrap();
        // Split + reassemble preserves bytes exactly.
        let chunks = split_debug_sync(&encoded, 42);
        assert!(!chunks.is_empty());
        let mut parts: Vec<(u32, Vec<u8>)> = chunks
            .iter()
            .map(|m| match m {
                SimMsg::DebugChunk(c) => {
                    assert_eq!(c.transfer_id, 42);
                    (c.index, c.bytes.clone())
                }
                SimMsg::Frame(_) => panic!("expected chunk"),
            })
            .collect();
        parts.sort_by_key(|(i, _)| *i);
        let mut joined = Vec::new();
        for (_, b) in parts {
            joined.extend_from_slice(&b);
        }
        assert_eq!(joined, encoded);
        let back = decode_debug_sync(&joined).unwrap();
        match back {
            DebugSyncMsg::Delta(v) => {
                assert_eq!(v.upsert_groups.len(), 1);
                assert_eq!(v.removed_groups, vec!["old".to_string()]);
            }
            _ => panic!("expected Delta"),
        }
        // Each chunk rides as a bounded SimMsg.
        for m in split_debug_sync(&encoded, 1) {
            let bytes = encode_sim(&m);
            assert!(bytes.len() <= MAX_SIM_MSG_BYTES + 4);
            let (back, consumed) = decode_sim(&bytes).unwrap().unwrap();
            assert_eq!(consumed, bytes.len());
            assert!(matches!(back, SimMsg::DebugChunk(_)));
        }
    }

    #[test]
    fn debug_sync_rejects_empty_delta_and_bad_names_without_panic() {
        let empty = DebugSyncMsg::Delta(DebugDelta::default());
        assert!(encode_debug_sync(&empty).is_err());
        let mut bad = DebugDelta::default();
        bad.removed_groups.push("bad name!".into());
        assert!(encode_debug_sync(&DebugSyncMsg::Delta(bad)).is_err());
        // Garbage bytes never panic, only error.
        assert!(decode_debug_sync(&[0xff, 0x00, 0x01]).is_err());
        assert!(decode_debug_sync(&vec![0u8; MAX_DEBUG_SYNC_BYTES + 1]).is_err());
        // Sim framing tripwires.
        assert!(decode_sim(&[0, 1]).unwrap().is_none());
        let mut bad_len = (MAX_SIM_MSG_BYTES as u32 + 1).to_le_bytes().to_vec();
        bad_len.extend_from_slice(&[0u8; 8]);
        assert!(decode_sim(&bad_len).is_err());
    }

    #[test]
    fn sim_frame_roundtrip_through_multiplexed_envelope() {
        let frame = frame_with_cars(1, 1);
        let bytes = encode_sim(&SimMsg::Frame(frame));
        let (back, consumed) = decode_sim(&bytes).unwrap().unwrap();
        assert_eq!(consumed, bytes.len());
        match back {
            SimMsg::Frame(f) => assert_eq!(f.state.num_cars(), 2),
            SimMsg::DebugChunk(_) => panic!("expected Frame"),
        }
    }
}
