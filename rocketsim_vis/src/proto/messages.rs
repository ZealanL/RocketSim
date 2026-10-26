use rocketsim::{ArenaState, CarControls};

/// Unreleased wire protocol. Keep this at 1 until the first release.
/// Rebuild both ends when serialized state fields change.
pub const PROTOCOL_VERSION: u32 = 1;

/// Default TCP listener port for the detached viewer: `39201`.
pub const DEFAULT_PORT: u16 = 39201;

/// Limit scene geometry separately from snapshots and debug data.
pub const MAX_SCENE_BYTES: usize = 24 * 1024 * 1024;

/// Maximum framed length for one control message, including its version field.
pub const MAX_CONTROL_BYTES: usize = 1024;

/// Cap on one multiplexed sim-to-viewer message, including scene metadata.
pub const MAX_SIM_MSG_BYTES: usize = 32 * 1024 * 1024;
/// Cap on a reassembled retained-scene transfer (postcard `DebugSyncMsg`).
pub const MAX_DEBUG_SYNC_BYTES: usize = 8 * 1024 * 1024;
/// Payload bytes per debug chunk (frames interleave between chunks so
/// controls/physics keep progressing during large uploads).
pub const DEBUG_CHUNK_SIZE: usize = 16 * 1024;
/// Maximum chunks per debug transfer (`512 * 16 KiB` = `MAX_DEBUG_SYNC_BYTES`).
pub const MAX_CHUNKS_PER_TRANSFER: usize = 512;

/// An error while reading or validating a protocol message.
#[derive(Debug)]
pub enum ProtocolError {
    /// The message uses a different wire version.
    VersionMismatch {
        /// Version found in the message.
        received: u32,
        /// Version supported by this crate.
        expected: u32,
    },
    /// The framed payload length is outside its allowed range.
    InvalidLength {
        /// Length declared by the message.
        length: usize,
        /// Smallest allowed length.
        min: usize,
        /// Largest allowed length.
        max: usize,
    },
    /// The message contains invalid values.
    InvalidData(&'static str),
    /// The postcard payload could not be decoded.
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
    /// Arena snapshot for this tick. Positions and velocities use RocketSim world units.
    pub state: ArenaState,
    /// Tick duration in seconds. The decoder accepts finite values in `0..=1`.
    pub dt: f32,
    /// Simulation update clock used to evaluate retained debug lifetimes.
    #[serde(default)]
    pub debug_tick: u64,
    /// Scene metadata is optional for raw and synthetic clients.
    #[serde(default)]
    pub scene: Option<SceneMetadata>,
}

/// Sim-to-viewer multiplexed message for protocol version 1.
///
/// Physics frames stay latest-only; retained-scene transfers split into
/// `16 KiB` chunks so frames interleave and progress during large uploads.
/// A reconnect starts clean: the receiver resets its debug scene and the
/// sender opens with a `Full` transfer, so old deletes are never replayed.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[allow(clippy::large_enum_variant)]
pub enum SimMsg {
    /// Latest-only physics tick and optional scene metadata.
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
    /// Part of the encoded [`DebugSyncMsg`].
    ///
    /// [`split_debug_sync`](super::split_debug_sync) emits at most [`DEBUG_CHUNK_SIZE`] bytes per part.
    /// Framing validation permits an additional 1024 bytes.
    pub bytes: Vec<u8>,
}

/// Retained-scene publication: full resync or per-group/resource delta.
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
    /// Complete retained groups for the receiver to replace its group table.
    pub groups: Vec<crate::debug::DebugGroup>,
    /// Mesh resources required by the groups.
    pub meshes: Vec<crate::debug::MeshResource>,
    /// Texture resources required by the groups.
    pub textures: Vec<crate::debug::TextureResource>,
    /// Sender global scene revision.
    pub rev: u64,
}

/// Metadata-only update for an existing group's retained geometry.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct DebugGroupPatch {
    /// Name of the existing group to update.
    pub name: String,
    /// Whether the group is visible.
    pub visible: bool,
    /// Coordinate anchor for the group.
    pub anchor: crate::debug::Anchor,
    /// Updated group transform.
    pub transform: crate::debug::GroupTransform,
    /// Lifetime policy for the group.
    pub lifetime: crate::debug::Lifetime,
    /// Optional simulation-tick expiry.
    pub expires_tick: Option<u64>,
    /// Optional wall-clock expiry in milliseconds since the Unix epoch.
    pub expires_at_ms: Option<u64>,
    /// Revision of this group record.
    pub revision: u64,
}

/// Per-group/resource delta within one ordered stream.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct DebugDelta {
    /// Groups to add or replace.
    pub upsert_groups: Vec<crate::debug::DebugGroup>,
    /// Existing groups whose metadata changes without geometry changes.
    pub group_patches: Vec<DebugGroupPatch>,
    /// Group names to remove.
    pub removed_groups: Vec<String>,
    /// Mesh resources to add or replace.
    pub upsert_meshes: Vec<crate::debug::MeshResource>,
    /// Mesh resource names to remove.
    pub removed_meshes: Vec<String>,
    /// Texture resources to add or replace.
    pub upsert_textures: Vec<crate::debug::TextureResource>,
    /// Texture resource names to remove.
    pub removed_textures: Vec<String>,
    /// Sender global scene revision after this delta.
    pub rev: u64,
}

impl DebugChunk {
    /// Check chunk count, index, and byte length.
    ///
    /// Accept `1..=MAX_CHUNKS_PER_TRANSFER` chunks and an index below `total`.
    /// Permit at most `DEBUG_CHUNK_SIZE + 1024` bytes. Do not validate the
    /// encoded scene or compare this chunk with other chunks.
    ///
    /// # Errors
    ///
    /// Return [`ProtocolError::InvalidData`] when any framing limit fails.
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

/// One collision mesh in scene metadata.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct MeshData {
    /// Vertex positions in RocketSim world units.
    pub vertices: Vec<[f32; 3]>,
    /// Triangle indices into `vertices`. The count must be a multiple of three.
    pub indices: Vec<u32>,
}
/// Arena collision data and ball settings for a frame's game mode.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SceneMetadata {
    /// Game mode for the scene. It must match the frame state.
    pub game_mode: rocketsim::GameMode,
    /// Ball radius in world units. Must be finite and greater than zero.
    pub ball_radius: f32,
    /// Collision meshes used to draw the arena.
    pub meshes: Vec<MeshData>,
}

impl SceneMetadata {
    /// Creates metadata with the mode's default ball radius and no meshes.
    pub fn empty(game_mode: rocketsim::GameMode) -> Self {
        Self {
            game_mode,
            ball_radius: rocketsim::consts::ball::get_radius(game_mode),
            meshes: Vec::new(),
        }
    }

    /// Checks mesh counts, indices, finite coordinates, and the scene byte limit.
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
    /// Create a frame from an owned arena state and a duration in seconds.
    ///
    /// Set `debug_tick` to the state's tick count. Start with no scene metadata.
    /// This constructor does not validate `dt` or the state.
    pub fn new(state: ArenaState, dt: f32) -> Self {
        Self {
            debug_tick: state.tick_count,
            state,
            dt,
            scene: None,
        }
    }
}

/// Unlimited-boost flag for [`ControlMsg::mutators`].
///
/// Mutator flags are level values, not one-time actions. The application
/// decides whether to apply them through [`crate::mutators::Mutators`].
pub const MUT_UNLIMITED_BOOST: u8 = 0b0000_0001;
/// Boost-recharge flag for [`ControlMsg::mutators`].
pub const MUT_BOOST_RECHARGE: u8 = 0b0000_0010;
/// Unlimited-flips flag for [`ControlMsg::mutators`].
pub const MUT_UNLIMITED_FLIPS: u8 = 0b0000_0100;
/// Unlimited-double-jumps flag for [`ControlMsg::mutators`].
pub const MUT_UNLIMITED_DOUBLE_JUMPS: u8 = 0b0000_1000;
/// Mask of the four supported mutator flags. Other bits are reserved.
pub const MUT_KNOWN_MASK: u8 =
    MUT_UNLIMITED_BOOST | MUT_BOOST_RECHARGE | MUT_UNLIMITED_FLIPS | MUT_UNLIMITED_DOUBLE_JUMPS;

/// Driving inputs and action requests from a viewer to a simulation.
///
/// The normal client keeps the latest unsent analog inputs. It separately
/// accumulates action bits until transmission. The simulation exposes the
/// latest controls and pending actions through [`crate::ArenaVisExt`]. It
/// does not apply them automatically.
///
/// `seq` is a sender-supplied sequence number. The current transport does not
/// reject messages based on that number. Use
/// [`crate::transport::viewer::ViewerListener::send_controls_for_generation`]
/// to avoid submitting inputs from a previous connection.
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
    /// Requested car index. The application must check that the car exists.
    ///
    /// The stock viewer sends `u32::MAX` in spectate-only mode. Ignore driving
    /// controls for an invalid index. Keep the car's existing policy controls.
    pub car_index: u32,
    /// Requested controls. Analog inputs normally use `-1..=1`.
    ///
    /// Decoding requires finite values. [`rocketsim::Arena::step_tick`]
    /// restricts applied analog controls to their valid range.
    pub controls: CarControls,
    /// One-time action requests: bit 0 reset, bit 1 dribble, bit 2 launch.
    ///
    /// Send a set bit for each request, not continuously while a key is held.
    /// Repeated pending requests can combine. [`crate::ArenaVisExt::take_vis_actions`]
    /// consumes the accumulated bits. It does not perform the actions.
    pub buttons: u8,
    /// Requested mutator level values, using the `MUT_*` flags.
    ///
    /// The application decides whether to apply or retain them on link loss.
    /// The drive example keeps the last selection and releases driving inputs.
    pub mutators: u8,
    /// Sender sequence number. The stock viewer increases it for each update.
    pub seq: u64,
    /// Connection generation. The combined listener starts at generation one.
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
