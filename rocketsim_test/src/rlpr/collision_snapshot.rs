//! RLPR v12 group 4 collision snapshot reader.
//!
//! The payload is a portable wire format. It has no C++ structs and no
//! pointers. The exact layout is `docs/collision-snapshot-wire.md` in the
//! recorder repository. All integers and floats are little-endian.
//!
//! A reader uses only fields whose mask bit is set. Unknown point mask bits
//! are rejected. An unknown schema version makes the reader skip the whole
//! group, because the group is length-bounded.

use std::io::{self, ErrorKind};

use super::data_reader::DataReader;

/// Collision snapshot schema version written by the recorder.
pub const SCHEMA_VERSION: u16 = 3;
/// Fixed payload header size.
pub const HEADER_SIZE: usize = 88;
/// Fixed section header size.
pub const SECTION_HEADER_SIZE: usize = 40;
/// Hard payload cap. A larger payload is rejected, not truncated.
pub const MAX_PAYLOAD_SIZE: usize = 1024 * 1024;

/// Section bit: broadphase handles.
pub const SECTION_HANDLES: u16 = 1 << 0;
/// Section bit: broadphase edge axes.
pub const SECTION_EDGES: u16 = 1 << 1;
/// Section bit: free handle list.
pub const SECTION_FREE_HANDLES: u16 = 1 << 2;
/// Section bit: rigid bodies.
pub const SECTION_BODIES: u16 = 1 << 3;
/// Section bit: broadphase pairs.
pub const SECTION_PAIRS: u16 = 1 << 4;
/// Section bit: persistent manifolds.
pub const SECTION_MANIFOLDS: u16 = 1 << 5;
/// All known section bits.
pub const KNOWN_SECTION_MASK: u16 = 0x3f;

/// Known header flag bits.
pub const KNOWN_HEADER_FLAGS: u16 = 0x03;
/// Known free-handle section flag bits.
pub const KNOWN_SECTION_FLAGS: u8 = 0x07;

/// Maximum handle pool entries and maximum inclusive handle id.
///
/// The native pool stores 16385 entries: 16384 entries plus one sentinel.
/// A handle id of `16384` is valid.
pub const MAX_HANDLES: u32 = 16384;
/// Fixed number of edge axis records.
pub const MAX_EDGE_AXES: u32 = 3;
/// Maximum edges in one axis.
pub const MAX_EDGES_PER_AXIS: u32 = 32768;
/// Maximum conceptual free handles in one snapshot.
pub const MAX_FREE_HANDLES: u32 = 16384;
/// Maximum free-handle runs in one snapshot.
pub const MAX_FREE_RUNS: u32 = 16384;
/// Maximum bodies in one snapshot.
pub const MAX_BODIES: u32 = 4096;
/// Maximum pairs in one snapshot.
pub const MAX_PAIRS: u32 = 16384;
/// Maximum manifolds in one snapshot.
pub const MAX_MANIFOLDS: u32 = 1024;
/// Maximum points in one manifold.
pub const MAX_POINTS_PER_MANIFOLD: u8 = 4;
/// Maximum scene slots in the current build.
pub const MAX_SCENE_COUNT: u32 = 16;

/// Point field bit: local point A.
pub const POINT_FIELD_LOCAL_A: u32 = 1 << 0;
/// Point field bit: local point B.
pub const POINT_FIELD_LOCAL_B: u32 = 1 << 1;
/// Point field bit: world point A.
pub const POINT_FIELD_WORLD_A: u32 = 1 << 2;
/// Point field bit: world point B.
pub const POINT_FIELD_WORLD_B: u32 = 1 << 3;
/// Point field bit: normal on B.
pub const POINT_FIELD_NORMAL_B: u32 = 1 << 4;
/// Point field bit: signed distance.
pub const POINT_FIELD_DISTANCE: u32 = 1 << 5;
/// Point field bit: friction values.
pub const POINT_FIELD_FRICTION: u32 = 1 << 6;
/// Point field bit: part identifiers and indices.
pub const POINT_FIELD_PART_IDS: u32 = 1 << 7;
/// Point field bit: lifetime.
pub const POINT_FIELD_LIFETIME: u32 = 1 << 8;
/// Point field bit: applied impulses.
pub const POINT_FIELD_APPLIED_IMPULSE: u32 = 1 << 9;
/// Point field bit: contact motion and CFM values.
pub const POINT_FIELD_CONTACT_MOTION: u32 = 1 << 10;
/// Point field bit: lateral friction directions.
pub const POINT_FIELD_LATERAL_DIRS: u32 = 1 << 11;
/// All known point field bits.
pub const POINT_FIELD_KNOWN_MASK: u32 = 0x0fff;
/// Point fields with a verified offset on the current build.
pub const POINT_FIELD_CORE_MASK: u32 = 0x013f;

macro_rules! wire_enum {
    ($name:ident, $default:ident { $($(#[$meta:meta])* $variant:ident = $value:expr),+ $(,)? }) => {
        #[repr(u8)]
        #[derive(Clone, Copy, Debug, PartialEq, Eq)]
        pub enum $name {
            $($(#[$meta])* $variant = $value),+
        }

        impl Default for $name {
            fn default() -> Self {
                Self::$default
            }
        }

        impl TryFrom<u8> for $name {
            type Error = u8;

            fn try_from(value: u8) -> Result<Self, Self::Error> {
                match value {
                    $($value => Ok(Self::$variant),)+
                    other => Err(other),
                }
            }
        }
    };
}

wire_enum!(CollisionLayoutId, None {
    /// No verified layout.
    None = 0,
    /// Rocket League 2.73, executable ecbc3f8a...1795c3b.
    Ecbc20261019 = 1,
    /// Current build, executable 56b499a8...185bc7.
    Current20261102 = 2,
});

wire_enum!(CollisionSourcePath, None {
    /// No source path.
    None = 0,
    /// At least one pointer came from the caller.
    ExplicitPointers = 1,
    /// The pointers came from the verified engine-scene chain.
    EngineSceneChain = 2,
});

wire_enum!(CollisionSectionId, Handles {
    /// Broadphase handles.
    Handles = 0,
    /// Broadphase edge axes.
    Edges = 1,
    /// Free handle list.
    FreeHandles = 2,
    /// Rigid bodies.
    Bodies = 3,
    /// Broadphase pairs.
    Pairs = 4,
    /// Persistent manifolds.
    Manifolds = 5,
});

wire_enum!(CollisionSectionStatus, NotRequested {
    /// The caller did not request this section.
    NotRequested = 0,
    /// The complete section was captured.
    Ok = 1,
    /// Some entries are present. At least one entry was dropped or unreadable.
    Partial = 2,
    /// No entries. The error states why.
    Unavailable = 3,
});

wire_enum!(CollisionSnapshotError, None {
    /// No error.
    None = 0,
    /// The caller did not request this section.
    NotRequested = 1,
    /// The executable hash has no verified layout for this section.
    UnverifiedLayout = 2,
    /// A required source pointer is null.
    SourceMissing = 3,
    /// A required header or array read failed.
    SourceUnreadable = 4,
    /// Structural checks contradict the verified layout.
    LayoutMismatch = 5,
    /// A count or index is outside its valid range.
    BoundsInvalid = 6,
    /// The native count exceeds the request or hard limit.
    LimitReached = 7,
    /// Some entries were readable and some were not.
    EntryUnreadable = 8,
    /// The free-handle chain has an invalid link or a cycle.
    FreeListInvalid = 9,
});

wire_enum!(CollisionBodyRole, Unknown {
    /// Role was not identified.
    Unknown = 0,
    /// The shared ball.
    Ball = 1,
    /// A car body.
    Car = 2,
    /// A world static body.
    WorldStatic = 3,
    /// Another dynamic body.
    OtherDynamic = 4,
});

wire_enum!(CollisionResolveStage, NotAttempted {
    /// The resolver was not attempted.
    NotAttempted = 0,
    /// The engine pointer is null.
    EngineNull = 1,
    /// The engine scene field read failed.
    EngineReadFailed = 2,
    /// The scene manager is null.
    SceneManagerNull = 3,
    /// The scene array read failed.
    SceneArrayReadFailed = 4,
    /// The scene array is empty.
    SceneArrayEmpty = 5,
    /// The scene index or array bounds are invalid.
    SceneArrayBounds = 6,
    /// The scene slot read failed.
    SceneSlotReadFailed = 7,
    /// The scene wrapper is null.
    SceneWrapperNull = 8,
    /// The scene wrapper read failed.
    SceneWrapperReadFailed = 9,
    /// The world, dispatcher, or broadphase pointer is null.
    ScenePointerNull = 10,
    /// The world cross-link read failed.
    WorldCrossReadFailed = 11,
    /// The world cross-link does not match.
    WorldCrossMismatch = 12,
    /// The resolver succeeded.
    Ok = 13,
});

wire_enum!(CollisionManifoldOrder, Unknown {
    /// The order was not identified.
    Unknown = 0,
    /// Dispatcher array order. This is append order with swap-with-last
    /// removal. It is not creation order.
    DispatcherStored = 1,
    /// Island-sorted solve-input order. It is not creation order.
    SolveInput = 2,
});

/// One section report.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionSectionReport {
    /// Section identifier from the wire.
    pub section_id: CollisionSectionId,
    /// Section status.
    pub status: CollisionSectionStatus,
    /// Section error.
    pub error: CollisionSnapshotError,
    /// Section flag bits. Nonzero only for free handles.
    pub section_flags: u8,
    /// Native count of entries in the source.
    pub stored_count: u32,
    /// Entries present in the output vector.
    pub captured_count: u32,
    /// Entries that were in range but unreadable or rejected.
    pub skipped_count: u32,
    /// Entries that were in range but beyond the effective limit.
    pub dropped_count: u32,
    /// Section-specific auxiliary value 0.
    pub aux0: u32,
    /// Section-specific auxiliary value 1.
    pub aux1: u32,
    /// Section-specific auxiliary value 2.
    pub aux2: u32,
    /// Number of records after this report.
    pub record_count: u32,
    /// True when the source count was truncated.
    pub truncated: bool,
}

/// One broadphase handle.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionHandle {
    /// Native unique identifier.
    pub unique_id: u16,
    /// Native filter group.
    pub filter_group: u16,
    /// Native filter mask.
    pub filter_mask: u16,
    /// Minimum edge index per axis.
    pub min_edge_index: [u16; 3],
    /// Maximum edge index per axis.
    pub max_edge_index: [u16; 3],
    /// Catalog index of the client body, or a negative value.
    pub client_catalog_index: i32,
    /// Client body index, or a negative value.
    pub client_body_index: i32,
    /// Stored handle AABB minimum. The value is not recomputed.
    pub aabb_min: [f32; 3],
    /// Stored handle AABB maximum. The value is not recomputed.
    pub aabb_max: [f32; 3],
}

/// One edge axis block.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionEdgeAxis {
    /// True when the axis data is available.
    pub available: bool,
    /// True when the edge cardinality does not match.
    pub cardinality_mismatch: bool,
    /// Native valid edge count.
    pub valid_edge_count: u32,
    /// Expected cardinality.
    pub cardinality_expected: u32,
    /// Native edge capacity.
    pub edge_capacity: u32,
    /// Edges in stored axis order. Equal positions stay in stored order.
    pub edges: Vec<CollisionEdge>,
}

/// One broadphase edge.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CollisionEdge {
    /// Raw position. The low bit is the max flag.
    pub position_raw: u16,
    /// Unique identifier of the referenced handle.
    pub handle_unique_id: u16,
}

/// One compact free-handle run.
///
/// A run with length `L` represents the links:
/// `(start + 0, start + 1)`, `(start + 1, start + 2)`, ...,
/// `(start + L - 1, next)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CollisionFreeHandleRun {
    /// First unique identifier in the run.
    pub start_unique_id: u16,
    /// Number of conceptual links in the run.
    pub length: u16,
    /// Successor of the last link in the run.
    pub next_unique_id: u16,
}

/// One conceptual free-list link.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CollisionFreeChainLink {
    /// Native unique identifier.
    pub unique_id: u16,
    /// Next unique identifier in the free-list link order.
    pub next_unique_id: u16,
}

/// Lazy iterator over the conceptual free-list links.
///
/// The iterator yields exactly `report.captured_count` links. It does not
/// expand the runs into a retained array.
#[derive(Clone, Debug)]
pub struct CollisionFreeChainIter<'a> {
    runs: &'a [CollisionFreeHandleRun],
    run_index: usize,
    offset: u16,
    remaining: usize,
}

impl Iterator for CollisionFreeChainIter<'_> {
    type Item = CollisionFreeChainLink;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }
        let run = self.runs.get(self.run_index)?;
        let unique_id =
            u16::try_from(u32::from(run.start_unique_id) + u32::from(self.offset)).ok()?;
        let next_unique_id = if self.offset + 1 < run.length {
            u16::try_from(u32::from(unique_id) + 1).ok()?
        } else {
            run.next_unique_id
        };

        self.offset += 1;
        if self.offset >= run.length {
            self.run_index += 1;
            self.offset = 0;
        }
        self.remaining -= 1;

        Some(CollisionFreeChainLink {
            unique_id,
            next_unique_id,
        })
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}

impl ExactSizeIterator for CollisionFreeChainIter<'_> {}

/// One rigid body.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionBody {
    /// Catalog index.
    pub catalog_index: i32,
    /// Native broadphase unique identifier.
    pub broadphase_unique_id: i32,
    /// Native activation state.
    pub activation_state: i32,
    /// Native island tag.
    pub island_tag: i32,
    /// Native companion identifier.
    pub companion_id: i32,
    /// Native internal type.
    pub internal_type: i32,
    /// Native collision flags.
    pub collision_flags: u32,
    /// Body role.
    pub role: CollisionBodyRole,
    /// Car slot, or `0xFF` when the body is not a car.
    pub car_slot: u8,
    /// True when the body has a broadphase handle.
    pub has_broadphase_handle: bool,
    /// Stored AABB minimum. The value is not recomputed.
    pub stored_aabb_min: [f32; 3],
    /// Stored AABB maximum. The value is not recomputed.
    pub stored_aabb_max: [f32; 3],
}

/// One broadphase pair.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CollisionPair {
    /// Unique identifier of proxy 0.
    pub proxy0_unique_id: i32,
    /// Unique identifier of proxy 1.
    pub proxy1_unique_id: i32,
    /// Catalog index of client 0.
    pub client0_catalog_index: i32,
    /// Catalog index of client 1.
    pub client1_catalog_index: i32,
    /// True when an algorithm is present.
    pub algorithm_present: bool,
}

/// One manifold point.
///
/// Each field is `Some` only when its bit is set in the parent manifold
/// `point_field_mask`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionManifoldPoint {
    /// Local point A.
    pub local_point_a: Option<[f32; 3]>,
    /// Local point B.
    pub local_point_b: Option<[f32; 3]>,
    /// World point A.
    pub position_world_on_a: Option<[f32; 3]>,
    /// World point B.
    pub position_world_on_b: Option<[f32; 3]>,
    /// Normal on B.
    pub normal_world_on_b: Option<[f32; 3]>,
    /// Signed distance. A positive value is a separating contact point. It
    /// does not prove when the manifold was created.
    pub distance1: Option<f32>,
    /// Combined friction.
    pub combined_friction: Option<f32>,
    /// Combined rolling friction.
    pub combined_rolling_friction: Option<f32>,
    /// Combined restitution.
    pub combined_restitution: Option<f32>,
    /// Part identifier 0.
    pub part_id0: Option<i32>,
    /// Part identifier 1.
    pub part_id1: Option<i32>,
    /// Point index 0.
    pub index0: Option<i32>,
    /// Point index 1.
    pub index1: Option<i32>,
    /// Point lifetime.
    pub lifetime: Option<i32>,
    /// Applied impulse.
    pub applied_impulse: Option<f32>,
    /// Applied lateral impulse 1.
    pub applied_impulse_lateral1: Option<f32>,
    /// Applied lateral impulse 2.
    pub applied_impulse_lateral2: Option<f32>,
    /// Contact motion 1.
    pub contact_motion1: Option<f32>,
    /// Contact motion 2.
    pub contact_motion2: Option<f32>,
    /// Contact CFM 1.
    pub contact_cfm1: Option<f32>,
    /// Contact CFM 2.
    pub contact_cfm2: Option<f32>,
    /// Lateral friction direction 1.
    pub lateral_friction_dir1: Option<[f32; 3]>,
    /// Lateral friction direction 2.
    pub lateral_friction_dir2: Option<[f32; 3]>,
}

/// One persistent manifold.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionManifold {
    /// Index in the manifold source array. Records are in stored order.
    pub source_slot: u32,
    /// Native dispatcher array index, or -1 when unknown.
    pub dispatcher_index: i32,
    /// Catalog index of body 0.
    pub body0_catalog_index: i32,
    /// Catalog index of body 1.
    pub body1_catalog_index: i32,
    /// Snapshot index of body 0.
    pub body0_snapshot_index: i32,
    /// Snapshot index of body 1.
    pub body1_snapshot_index: i32,
    /// Island tag of body 0.
    pub island_tag_body0: i32,
    /// Island tag of body 1.
    pub island_tag_body1: i32,
    /// Contact breaking threshold.
    pub contact_breaking_threshold: f32,
    /// Contact processing threshold.
    pub contact_processing_threshold: f32,
    /// Number of retained points. Zero is an empty persisted slot. It is not
    /// a predictive manifold.
    pub point_count: u8,
    /// Points with `distance1 > 0`. A positive distance is a separating
    /// contact point. It is not a creation-order signal.
    pub positive_distance_point_count: u8,
    /// Points with `distance1 <= 0`.
    pub non_positive_distance_point_count: u8,
    /// True when both island tag reads succeeded.
    pub island_tags_valid: bool,
    /// Point fields with a verified offset. Only these fields are present.
    pub point_field_mask: u32,
    /// Retained points in stored order.
    pub points: Vec<CollisionManifoldPoint>,
}

/// Handles section.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionHandlesSection {
    /// Section report.
    pub report: CollisionSectionReport,
    /// Native active handle count.
    pub active_count: u32,
    /// Native handle pool entry count.
    pub pool_entry_count: u32,
    /// Native edge capacity.
    pub edge_capacity: u32,
    /// Handles in stored pool order.
    pub records: Vec<CollisionHandle>,
}

/// Edges section.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionEdgesSection {
    /// Section report.
    pub report: CollisionSectionReport,
    /// The three axis blocks in axis order 0 to 2.
    pub axes: [CollisionEdgeAxis; 3],
}

/// Free handles section.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionFreeHandlesSection {
    /// Section report. `captured_count` is the conceptual link count and
    /// `record_count` is the run count.
    pub report: CollisionSectionReport,
    /// True when the free list terminated at 0.
    pub terminated: bool,
    /// True when a cycle was detected.
    pub cycle_detected: bool,
    /// True when an invalid link was found.
    pub invalid_link: bool,
    /// Compact runs in free-list link order.
    pub runs: Vec<CollisionFreeHandleRun>,
}

impl CollisionFreeHandlesSection {
    /// Conceptual link count. Equal to the sum of the run lengths.
    pub fn entry_count(&self) -> u32 {
        self.report.captured_count
    }

    /// Serialized run count.
    pub fn run_count(&self) -> u32 {
        self.report.record_count
    }

    /// Lazy iterator over the conceptual free-list links.
    pub fn iter_chain(&self) -> CollisionFreeChainIter<'_> {
        CollisionFreeChainIter {
            runs: &self.runs,
            run_index: 0,
            offset: 0,
            remaining: self.entry_count() as usize,
        }
    }
}

/// Bodies section.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionBodiesSection {
    /// Section report.
    pub report: CollisionSectionReport,
    /// Bodies in catalog order.
    pub records: Vec<CollisionBody>,
}

/// Pairs section.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionPairsSection {
    /// Section report.
    pub report: CollisionSectionReport,
    /// Native pair array size.
    pub array_size: u32,
    /// Native pair array capacity.
    pub array_capacity: u32,
    /// Pairs in stored pair-array order. This is not a sorted cache.
    pub records: Vec<CollisionPair>,
}

/// Manifolds section.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionManifoldsSection {
    /// Section report.
    pub report: CollisionSectionReport,
    /// Manifold order metadata. This is not creation order.
    pub order: CollisionManifoldOrder,
    /// Native manifold source-slot count.
    pub source_slot_count: u32,
    /// Native null slot count.
    pub null_slots: u32,
    /// Manifolds in stored order.
    pub records: Vec<CollisionManifold>,
}

/// Resolver trace for the engine-scene source chain.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionResolverTrace {
    /// Final resolver stage.
    pub stage: CollisionResolveStage,
    /// True when the engine pointer was present.
    pub engine_pointer_present: bool,
    /// True when the scene manager was present.
    pub scene_manager_present: bool,
    /// True when the scene array was present.
    pub scene_array_present: bool,
    /// True when the world cross-link was checked.
    pub cross_link_checked: bool,
    /// True when the world and dispatcher pointers matched.
    pub world_dispatcher_match: bool,
    /// True when the world and broadphase pointers matched.
    pub world_broadphase_match: bool,
    /// Requested scene index.
    pub requested_scene: u32,
    /// Resolved scene index, or `0xFFFF_FFFF` when none resolved.
    pub resolved_scene: u32,
    /// Native scene count.
    pub scene_count: u32,
    /// Native scene capacity.
    pub scene_capacity: u32,
    /// First bad scene slot, or `0xFFFF_FFFF` when none failed.
    pub first_bad_index: u32,
    /// Number of scene slots scanned.
    pub scene_slots_scanned: u32,
    /// Number of null scene slots.
    pub scene_slots_null: u32,
    /// Number of scene slots that failed to read.
    pub scene_slots_read_failed: u32,
}

/// One RLPR v12 group 4 collision snapshot.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CollisionSnapshot {
    /// Wire schema version.
    pub schema_version: u16,
    /// Requested section mask.
    pub requested_sections: u16,
    /// Sections whose offsets are verified for the build.
    pub verified_sections: u16,
    /// True when the executable hash matched a verified build.
    pub build_identified: bool,
    /// True when at least one limit clamped a native count.
    pub limits_clamped: bool,
    /// Layout identifier.
    pub layout_id: CollisionLayoutId,
    /// Source path for the world, dispatcher, and broadphase pointers.
    pub source_path: CollisionSourcePath,
    /// Engine scene index, or `0xFF` when the engine chain was not used.
    pub engine_scene_index: u8,
    /// Number of sections in the payload.
    pub section_count: u8,
    /// Exact executable SHA-256 bytes.
    pub executable_sha256: [u8; 32],
    /// True when the executable SHA-256 is present.
    pub have_executable_sha256: bool,
    /// Resolver trace from the schema 3 header.
    pub resolver: CollisionResolverTrace,
    /// Handles section when requested.
    pub handles: Option<CollisionHandlesSection>,
    /// Edges section when requested.
    pub edges: Option<CollisionEdgesSection>,
    /// Free handles section when requested.
    pub free_handles: Option<CollisionFreeHandlesSection>,
    /// Bodies section when requested.
    pub bodies: Option<CollisionBodiesSection>,
    /// Pairs section when requested.
    pub pairs: Option<CollisionPairsSection>,
    /// Manifolds section when requested.
    pub manifolds: Option<CollisionManifoldsSection>,
}

fn invalid_data(message: impl Into<String>) -> io::Error {
    io::Error::new(ErrorKind::InvalidData, message.into())
}

fn read_bool(reader: &mut DataReader<'_>, field: &str) -> io::Result<bool> {
    let value = reader.read_u8()?;
    match value {
        0 => Ok(false),
        1 => Ok(true),
        _ => Err(invalid_data(format!(
            "RLPR v12 collision {field} byte must be 0 or 1, got {value}"
        ))),
    }
}

fn read_finite_f32(reader: &mut DataReader<'_>, field: &str) -> io::Result<f32> {
    let value = reader.read_f32()?;
    if !value.is_finite() {
        return Err(invalid_data(format!(
            "RLPR v12 collision {field} must be finite, got {value}"
        )));
    }
    Ok(value)
}

fn read_finite_vec3(reader: &mut DataReader<'_>, field: &str) -> io::Result<[f32; 3]> {
    Ok([
        read_finite_f32(reader, field)?,
        read_finite_f32(reader, field)?,
        read_finite_f32(reader, field)?,
    ])
}

fn read_wire_enum<T>(reader: &mut DataReader<'_>, field: &str) -> io::Result<T>
where
    T: TryFrom<u8>,
{
    let raw = reader.read_u8()?;
    T::try_from(raw)
        .map_err(|_| invalid_data(format!("RLPR v12 collision {field} value {raw} is unknown")))
}

fn require_consumed(reader: &mut DataReader<'_>, context: &str) -> io::Result<()> {
    let remaining = reader.num_bytes_left();
    if remaining != 0 {
        return Err(invalid_data(format!(
            "RLPR v12 collision {context} has {remaining} trailing bytes"
        )));
    }
    Ok(())
}

fn bounded_count(value: u32, maximum: u32, name: &str) -> io::Result<u32> {
    if value > maximum {
        return Err(invalid_data(format!(
            "RLPR v12 collision {name} count {value} is above {maximum}"
        )));
    }
    Ok(value)
}

fn count_mismatch(name: &str, report: &CollisionSectionReport, expected: u32) -> io::Error {
    invalid_data(format!(
        "RLPR v12 collision {name} captured_count {} does not match decoded {expected} (record_count {})",
        report.captured_count, report.record_count
    ))
}

fn read_report(
    reader: &mut DataReader<'_>,
    section_id: CollisionSectionId,
) -> io::Result<CollisionSectionReport> {
    let status = read_wire_enum::<CollisionSectionStatus>(reader, "section status")?;
    let error = read_wire_enum::<CollisionSnapshotError>(reader, "section error")?;
    let section_flags = reader.read_u8()?;
    let stored_count = reader.read_u32()?;
    let captured_count = reader.read_u32()?;
    let skipped_count = reader.read_u32()?;
    let dropped_count = reader.read_u32()?;
    let aux0 = reader.read_u32()?;
    let aux1 = reader.read_u32()?;
    let aux2 = reader.read_u32()?;
    let record_count = reader.read_u32()?;
    let truncated = read_bool(reader, "section truncated")?;
    for _ in 0..3 {
        reader.read_u8()?;
    }

    if status == CollisionSectionStatus::NotRequested {
        return Err(invalid_data(
            "RLPR v12 collision requested section has status not-requested",
        ));
    }
    if error == CollisionSnapshotError::NotRequested {
        return Err(invalid_data(
            "RLPR v12 collision requested section has error not-requested",
        ));
    }
    if (status == CollisionSectionStatus::Ok) != (error == CollisionSnapshotError::None) {
        return Err(invalid_data(
            "RLPR v12 collision section status and error do not agree",
        ));
    }
    match section_id {
        CollisionSectionId::FreeHandles => {
            if section_flags & !KNOWN_SECTION_FLAGS != 0 {
                return Err(invalid_data(format!(
                    "RLPR v12 collision free-handle flags {section_flags:#x} are unknown"
                )));
            }
        }
        _ => {
            if section_flags != 0 {
                return Err(invalid_data(format!(
                    "RLPR v12 collision section {section_id:?} has unexpected flags {section_flags:#x}"
                )));
            }
        }
    }

    Ok(CollisionSectionReport {
        section_id,
        status,
        error,
        section_flags,
        stored_count,
        captured_count,
        skipped_count,
        dropped_count,
        aux0,
        aux1,
        aux2,
        record_count,
        truncated,
    })
}

fn read_handle(reader: &mut DataReader<'_>) -> io::Result<CollisionHandle> {
    let unique_id = reader.read_u16()?;
    if u32::from(unique_id) > MAX_HANDLES {
        return Err(invalid_data(format!(
            "RLPR v12 collision handle unique_id {unique_id} is above {MAX_HANDLES}"
        )));
    }
    let filter_group = reader.read_u16()?;
    let filter_mask = reader.read_u16()?;
    let mut min_edge_index = [0u16; 3];
    for value in &mut min_edge_index {
        *value = reader.read_u16()?;
    }
    let mut max_edge_index = [0u16; 3];
    for value in &mut max_edge_index {
        *value = reader.read_u16()?;
    }
    let client_catalog_index = reader.read_i32()?;
    let client_body_index = reader.read_i32()?;
    let in_free_list = reader.read_u8()?;
    if in_free_list != 0 {
        return Err(invalid_data(
            "RLPR v12 collision handle in_free_list byte must be 0",
        ));
    }
    for _ in 0..3 {
        reader.read_u8()?;
    }
    let aabb_min = read_finite_vec3(reader, "handle aabb_min")?;
    let aabb_max = read_finite_vec3(reader, "handle aabb_max")?;

    Ok(CollisionHandle {
        unique_id,
        filter_group,
        filter_mask,
        min_edge_index,
        max_edge_index,
        client_catalog_index,
        client_body_index,
        aabb_min,
        aabb_max,
    })
}

fn read_edge_axis(reader: &mut DataReader<'_>) -> io::Result<CollisionEdgeAxis> {
    let available = read_bool(reader, "edge axis available")?;
    let cardinality_mismatch = read_bool(reader, "edge axis cardinality_mismatch")?;
    reader.read_u16()?;
    let valid_edge_count = reader.read_u32()?;
    let cardinality_expected = reader.read_u32()?;
    let edge_capacity = reader.read_u32()?;
    let edge_count = bounded_count(reader.read_u32()?, MAX_EDGES_PER_AXIS, "edge")?;
    let mut edges = Vec::with_capacity(edge_count as usize);
    for _ in 0..edge_count {
        edges.push(CollisionEdge {
            position_raw: reader.read_u16()?,
            handle_unique_id: reader.read_u16()?,
        });
    }

    Ok(CollisionEdgeAxis {
        available,
        cardinality_mismatch,
        valid_edge_count,
        cardinality_expected,
        edge_capacity,
        edges,
    })
}

fn read_body(reader: &mut DataReader<'_>) -> io::Result<CollisionBody> {
    let catalog_index = reader.read_i32()?;
    let broadphase_unique_id = reader.read_i32()?;
    let activation_state = reader.read_i32()?;
    let island_tag = reader.read_i32()?;
    let companion_id = reader.read_i32()?;
    let internal_type = reader.read_i32()?;
    let collision_flags = reader.read_u32()?;
    let role = read_wire_enum::<CollisionBodyRole>(reader, "body role")?;
    let car_slot = reader.read_u8()?;
    let has_broadphase_handle = read_bool(reader, "body has_broadphase_handle")?;
    reader.read_u8()?;
    let stored_aabb_min = read_finite_vec3(reader, "body stored_aabb_min")?;
    let stored_aabb_max = read_finite_vec3(reader, "body stored_aabb_max")?;

    Ok(CollisionBody {
        catalog_index,
        broadphase_unique_id,
        activation_state,
        island_tag,
        companion_id,
        internal_type,
        collision_flags,
        role,
        car_slot,
        has_broadphase_handle,
        stored_aabb_min,
        stored_aabb_max,
    })
}

fn read_pair(reader: &mut DataReader<'_>) -> io::Result<CollisionPair> {
    let proxy0_unique_id = reader.read_i32()?;
    let proxy1_unique_id = reader.read_i32()?;
    let client0_catalog_index = reader.read_i32()?;
    let client1_catalog_index = reader.read_i32()?;
    let algorithm_present = read_bool(reader, "pair algorithm_present")?;
    for _ in 0..3 {
        reader.read_u8()?;
    }

    Ok(CollisionPair {
        proxy0_unique_id,
        proxy1_unique_id,
        client0_catalog_index,
        client1_catalog_index,
        algorithm_present,
    })
}

fn read_point(reader: &mut DataReader<'_>, mask: u32) -> io::Result<CollisionManifoldPoint> {
    let mut point = CollisionManifoldPoint::default();

    if mask & POINT_FIELD_LOCAL_A != 0 {
        point.local_point_a = Some(read_finite_vec3(reader, "point local_point_a")?);
    }
    if mask & POINT_FIELD_LOCAL_B != 0 {
        point.local_point_b = Some(read_finite_vec3(reader, "point local_point_b")?);
    }
    if mask & POINT_FIELD_WORLD_A != 0 {
        point.position_world_on_a = Some(read_finite_vec3(reader, "point position_world_on_a")?);
    }
    if mask & POINT_FIELD_WORLD_B != 0 {
        point.position_world_on_b = Some(read_finite_vec3(reader, "point position_world_on_b")?);
    }
    if mask & POINT_FIELD_NORMAL_B != 0 {
        point.normal_world_on_b = Some(read_finite_vec3(reader, "point normal_world_on_b")?);
    }
    if mask & POINT_FIELD_DISTANCE != 0 {
        point.distance1 = Some(read_finite_f32(reader, "point distance1")?);
    }
    if mask & POINT_FIELD_FRICTION != 0 {
        point.combined_friction = Some(read_finite_f32(reader, "point combined_friction")?);
        point.combined_rolling_friction =
            Some(read_finite_f32(reader, "point combined_rolling_friction")?);
        point.combined_restitution = Some(read_finite_f32(reader, "point combined_restitution")?);
    }
    if mask & POINT_FIELD_PART_IDS != 0 {
        point.part_id0 = Some(reader.read_i32()?);
        point.part_id1 = Some(reader.read_i32()?);
        point.index0 = Some(reader.read_i32()?);
        point.index1 = Some(reader.read_i32()?);
    }
    if mask & POINT_FIELD_LIFETIME != 0 {
        point.lifetime = Some(reader.read_i32()?);
    }
    if mask & POINT_FIELD_APPLIED_IMPULSE != 0 {
        point.applied_impulse = Some(read_finite_f32(reader, "point applied_impulse")?);
        point.applied_impulse_lateral1 =
            Some(read_finite_f32(reader, "point applied_impulse_lateral1")?);
        point.applied_impulse_lateral2 =
            Some(read_finite_f32(reader, "point applied_impulse_lateral2")?);
    }
    if mask & POINT_FIELD_CONTACT_MOTION != 0 {
        point.contact_motion1 = Some(read_finite_f32(reader, "point contact_motion1")?);
        point.contact_motion2 = Some(read_finite_f32(reader, "point contact_motion2")?);
        point.contact_cfm1 = Some(read_finite_f32(reader, "point contact_cfm1")?);
        point.contact_cfm2 = Some(read_finite_f32(reader, "point contact_cfm2")?);
    }
    if mask & POINT_FIELD_LATERAL_DIRS != 0 {
        point.lateral_friction_dir1 =
            Some(read_finite_vec3(reader, "point lateral_friction_dir1")?);
        point.lateral_friction_dir2 =
            Some(read_finite_vec3(reader, "point lateral_friction_dir2")?);
    }

    Ok(point)
}

fn read_manifold(reader: &mut DataReader<'_>) -> io::Result<CollisionManifold> {
    let source_slot = reader.read_u32()?;
    let dispatcher_index = reader.read_i32()?;
    let body0_catalog_index = reader.read_i32()?;
    let body1_catalog_index = reader.read_i32()?;
    let body0_snapshot_index = reader.read_i32()?;
    let body1_snapshot_index = reader.read_i32()?;
    let island_tag_body0 = reader.read_i32()?;
    let island_tag_body1 = reader.read_i32()?;
    let contact_breaking_threshold =
        read_finite_f32(reader, "manifold contact_breaking_threshold")?;
    let contact_processing_threshold =
        read_finite_f32(reader, "manifold contact_processing_threshold")?;
    let point_count = reader.read_u8()?;
    let positive_distance_point_count = reader.read_u8()?;
    let non_positive_distance_point_count = reader.read_u8()?;
    let island_tags_valid = read_bool(reader, "manifold island_tags_valid")?;
    let point_field_mask = reader.read_u32()?;

    if point_count > MAX_POINTS_PER_MANIFOLD {
        return Err(invalid_data(format!(
            "RLPR v12 collision point count {point_count} is above {MAX_POINTS_PER_MANIFOLD}"
        )));
    }
    if point_field_mask & !POINT_FIELD_KNOWN_MASK != 0 {
        return Err(invalid_data(format!(
            "RLPR v12 collision point field mask {point_field_mask:#x} has unknown bits"
        )));
    }
    if u16::from(positive_distance_point_count) + u16::from(non_positive_distance_point_count)
        != u16::from(point_count)
    {
        return Err(invalid_data(
            "RLPR v12 collision point distance counts do not match point_count",
        ));
    }

    let mut points = Vec::with_capacity(point_count as usize);
    for _ in 0..point_count {
        points.push(read_point(reader, point_field_mask)?);
    }

    Ok(CollisionManifold {
        source_slot,
        dispatcher_index,
        body0_catalog_index,
        body1_catalog_index,
        body0_snapshot_index,
        body1_snapshot_index,
        island_tag_body0,
        island_tag_body1,
        contact_breaking_threshold,
        contact_processing_threshold,
        point_count,
        positive_distance_point_count,
        non_positive_distance_point_count,
        island_tags_valid,
        point_field_mask,
        points,
    })
}

fn read_handles_section(
    reader: &mut DataReader<'_>,
    report: CollisionSectionReport,
) -> io::Result<CollisionHandlesSection> {
    let count = bounded_count(report.record_count, MAX_HANDLES, "handle")?;
    if report.captured_count != count {
        return Err(count_mismatch("handles", &report, count));
    }
    if report.aux0 != count {
        return Err(invalid_data(format!(
            "RLPR v12 collision handles active_count {} does not match record_count {count}",
            report.aux0
        )));
    }
    if report.aux1 > MAX_HANDLES {
        return Err(invalid_data(format!(
            "RLPR v12 collision handles pool_entry_count {} is above {MAX_HANDLES}",
            report.aux1
        )));
    }
    if report.aux1 != 0 && count > report.aux1 {
        return Err(invalid_data(format!(
            "RLPR v12 collision handles record_count {count} is above pool_entry_count {}",
            report.aux1
        )));
    }
    let active_count = report.aux0;
    let pool_entry_count = report.aux1;
    let edge_capacity = report.aux2;
    let mut records = Vec::with_capacity(count as usize);
    for _ in 0..count {
        records.push(read_handle(reader)?);
    }
    Ok(CollisionHandlesSection {
        report,
        active_count,
        pool_entry_count,
        edge_capacity,
        records,
    })
}

fn read_edges_section(
    reader: &mut DataReader<'_>,
    report: CollisionSectionReport,
) -> io::Result<CollisionEdgesSection> {
    if report.record_count != MAX_EDGE_AXES {
        return Err(invalid_data(format!(
            "RLPR v12 collision edges record_count {} does not match {MAX_EDGE_AXES}",
            report.record_count
        )));
    }
    let mut axes: [CollisionEdgeAxis; 3] = std::array::from_fn(|_| CollisionEdgeAxis::default());
    let mut total = 0u32;
    for axis in &mut axes {
        *axis = read_edge_axis(reader)?;
        total += axis.edges.len() as u32;
    }
    if report.captured_count != total {
        return Err(invalid_data(format!(
            "RLPR v12 collision edges captured_count {} does not match decoded {total}",
            report.captured_count
        )));
    }
    Ok(CollisionEdgesSection { report, axes })
}

fn read_free_handles_section(
    reader: &mut DataReader<'_>,
    report: CollisionSectionReport,
) -> io::Result<CollisionFreeHandlesSection> {
    let run_count = bounded_count(report.record_count, MAX_FREE_RUNS, "free handle run")?;
    if report.captured_count > MAX_FREE_HANDLES {
        return Err(invalid_data(format!(
            "RLPR v12 collision free handle entry count {} is above {MAX_FREE_HANDLES}",
            report.captured_count
        )));
    }
    if report.aux0 != report.captured_count {
        return Err(invalid_data(format!(
            "RLPR v12 collision free handle entry_count {} does not match captured_count {}",
            report.aux0, report.captured_count
        )));
    }
    if run_count > report.captured_count {
        return Err(invalid_data(
            "RLPR v12 collision free handle run count is above the entry count",
        ));
    }

    let terminated = report.section_flags & 0x01 != 0;
    let cycle_detected = report.section_flags & 0x02 != 0;
    let invalid_link = report.section_flags & 0x04 != 0;

    let mut runs = Vec::with_capacity(run_count as usize);
    let mut entries = 0u32;
    for _ in 0..run_count {
        let start_unique_id = reader.read_u16()?;
        let length = reader.read_u16()?;
        let next_unique_id = reader.read_u16()?;
        if length == 0 {
            return Err(invalid_data(
                "RLPR v12 collision free handle run has zero length",
            ));
        }
        if start_unique_id == 0 {
            return Err(invalid_data(format!(
                "RLPR v12 collision free handle run start {start_unique_id} is invalid"
            )));
        }
        let end = u32::from(start_unique_id) + u32::from(length) - 1;
        if end > MAX_HANDLES {
            return Err(invalid_data(format!(
                "RLPR v12 collision free handle run end {end} is above {MAX_HANDLES}"
            )));
        }
        if u32::from(next_unique_id) > MAX_HANDLES {
            return Err(invalid_data(format!(
                "RLPR v12 collision free handle successor {next_unique_id} is above {MAX_HANDLES}"
            )));
        }
        entries = entries
            .checked_add(u32::from(length))
            .ok_or_else(|| invalid_data("RLPR v12 collision free handle entry count overflow"))?;
        if entries > report.captured_count {
            return Err(invalid_data(format!(
                "RLPR v12 collision free handle run lengths exceed captured_count {}",
                report.captured_count
            )));
        }
        runs.push(CollisionFreeHandleRun {
            start_unique_id,
            length,
            next_unique_id,
        });
    }

    if entries != report.captured_count {
        return Err(invalid_data(format!(
            "RLPR v12 collision free handle run lengths sum to {entries}, captured_count is {}",
            report.captured_count
        )));
    }
    for pair in runs.windows(2) {
        if pair[0].next_unique_id != pair[1].start_unique_id {
            return Err(invalid_data(
                "RLPR v12 collision free handle runs do not chain",
            ));
        }
    }
    if let Some(last) = runs.last()
        && terminated
        && last.next_unique_id != 0
    {
        return Err(invalid_data(
            "RLPR v12 collision free handle chain is terminated but does not end at 0",
        ));
    }

    Ok(CollisionFreeHandlesSection {
        report,
        terminated,
        cycle_detected,
        invalid_link,
        runs,
    })
}

fn read_bodies_section(
    reader: &mut DataReader<'_>,
    report: CollisionSectionReport,
) -> io::Result<CollisionBodiesSection> {
    let count = bounded_count(report.record_count, MAX_BODIES, "body")?;
    if report.captured_count != count {
        return Err(count_mismatch("bodies", &report, count));
    }
    let mut records = Vec::with_capacity(count as usize);
    for _ in 0..count {
        records.push(read_body(reader)?);
    }
    Ok(CollisionBodiesSection { report, records })
}

fn read_pairs_section(
    reader: &mut DataReader<'_>,
    report: CollisionSectionReport,
) -> io::Result<CollisionPairsSection> {
    let count = bounded_count(report.record_count, MAX_PAIRS, "pair")?;
    if report.captured_count != count {
        return Err(count_mismatch("pairs", &report, count));
    }
    let array_size = report.aux0;
    let array_capacity = report.aux1;
    if array_capacity != 0 && array_size > array_capacity {
        return Err(invalid_data(format!(
            "RLPR v12 collision pair array_size {array_size} is above array_capacity {array_capacity}"
        )));
    }
    let mut records = Vec::with_capacity(count as usize);
    for _ in 0..count {
        records.push(read_pair(reader)?);
    }
    Ok(CollisionPairsSection {
        report,
        array_size,
        array_capacity,
        records,
    })
}

fn read_manifolds_section(
    reader: &mut DataReader<'_>,
    report: CollisionSectionReport,
) -> io::Result<CollisionManifoldsSection> {
    let count = bounded_count(report.record_count, MAX_MANIFOLDS, "manifold")?;
    if report.captured_count != count {
        return Err(count_mismatch("manifolds", &report, count));
    }
    let order = match report.aux2 {
        0 => CollisionManifoldOrder::Unknown,
        1 => CollisionManifoldOrder::DispatcherStored,
        2 => CollisionManifoldOrder::SolveInput,
        other => {
            return Err(invalid_data(format!(
                "RLPR v12 collision manifold order {other} is unknown"
            )));
        }
    };
    let source_slot_count = report.aux0;
    let null_slots = report.aux1;
    let mut records = Vec::with_capacity(count as usize);
    for _ in 0..count {
        records.push(read_manifold(reader)?);
    }
    Ok(CollisionManifoldsSection {
        report,
        order,
        source_slot_count,
        null_slots,
        records,
    })
}

fn validate_complete_handle_partition(
    handles: &CollisionHandlesSection,
    free_handles: &CollisionFreeHandlesSection,
) -> io::Result<()> {
    let active_count = handles.active_count;
    let free_count = free_handles.report.captured_count;
    let pool_count = handles.pool_entry_count;
    if u64::from(active_count) + u64::from(free_count) != u64::from(pool_count) {
        return Err(invalid_data(format!(
            "RLPR v12 collision complete partition mismatch: active {active_count} + free {free_count} != pool {pool_count}"
        )));
    }

    let mut active_ids: Vec<u16> = handles
        .records
        .iter()
        .map(|record| record.unique_id)
        .collect();
    active_ids.sort_unstable();
    for pair in active_ids.windows(2) {
        if pair[0] == pair[1] {
            return Err(invalid_data(format!(
                "RLPR v12 collision complete partition has duplicate active handle id {}",
                pair[0]
            )));
        }
    }

    let mut intervals: Vec<(u32, u32)> = free_handles
        .runs
        .iter()
        .map(|run| {
            let start = u32::from(run.start_unique_id);
            (start, start + u32::from(run.length) - 1)
        })
        .collect();
    intervals.sort_unstable_by_key(|&(start, _)| start);
    for pair in intervals.windows(2) {
        if pair[1].0 <= pair[0].1 {
            return Err(invalid_data(format!(
                "RLPR v12 collision complete partition has overlapping free runs {}..={} and {}..={}",
                pair[0].0, pair[0].1, pair[1].0, pair[1].1
            )));
        }
    }

    for &id in &active_ids {
        let id = u32::from(id);
        let index = intervals.partition_point(|&(start, _)| start <= id);
        if index > 0 {
            let (_, end) = intervals[index - 1];
            if id <= end {
                return Err(invalid_data(format!(
                    "RLPR v12 collision active handle id {id} is inside a free run"
                )));
            }
        }
    }

    Ok(())
}

fn read_resolver_trace(reader: &mut DataReader<'_>) -> io::Result<CollisionResolverTrace> {
    let stage = read_wire_enum::<CollisionResolveStage>(reader, "resolver stage")?;
    let engine_pointer_present = read_bool(reader, "resolver engine_pointer_present")?;
    let scene_manager_present = read_bool(reader, "resolver scene_manager_present")?;
    let scene_array_present = read_bool(reader, "resolver scene_array_present")?;
    let cross_link_checked = read_bool(reader, "resolver cross_link_checked")?;
    let world_dispatcher_match = read_bool(reader, "resolver world_dispatcher_match")?;
    let world_broadphase_match = read_bool(reader, "resolver world_broadphase_match")?;
    reader.read_u8()?;
    let requested_scene = reader.read_u32()?;
    let resolved_scene = reader.read_u32()?;
    let scene_count = reader.read_u32()?;
    let scene_capacity = reader.read_u32()?;
    let first_bad_index = reader.read_u32()?;
    let scene_slots_scanned = reader.read_u32()?;
    let scene_slots_null = reader.read_u32()?;
    let scene_slots_read_failed = reader.read_u32()?;

    if scene_count > MAX_SCENE_COUNT {
        return Err(invalid_data(format!(
            "RLPR v12 collision resolver scene_count {scene_count} is above {MAX_SCENE_COUNT}"
        )));
    }
    if scene_capacity > MAX_SCENE_COUNT {
        return Err(invalid_data(format!(
            "RLPR v12 collision resolver scene_capacity {scene_capacity} is above {MAX_SCENE_COUNT}"
        )));
    }
    if scene_count > 0 && scene_capacity < scene_count {
        return Err(invalid_data(format!(
            "RLPR v12 collision resolver scene_capacity {scene_capacity} is below scene_count {scene_count}"
        )));
    }
    if resolved_scene != u32::MAX && resolved_scene >= MAX_SCENE_COUNT {
        return Err(invalid_data(format!(
            "RLPR v12 collision resolver resolved_scene {resolved_scene} is above {MAX_SCENE_COUNT}"
        )));
    }
    if first_bad_index != u32::MAX && first_bad_index >= MAX_SCENE_COUNT {
        return Err(invalid_data(format!(
            "RLPR v12 collision resolver first_bad_index {first_bad_index} is above {MAX_SCENE_COUNT}"
        )));
    }

    Ok(CollisionResolverTrace {
        stage,
        engine_pointer_present,
        scene_manager_present,
        scene_array_present,
        cross_link_checked,
        world_dispatcher_match,
        world_broadphase_match,
        requested_scene,
        resolved_scene,
        scene_count,
        scene_capacity,
        first_bad_index,
        scene_slots_scanned,
        scene_slots_null,
        scene_slots_read_failed,
    })
}

/// Read one group 4 payload.
///
/// The caller has already bounded the payload by `group_size`. An unknown
/// schema version returns `Ok(None)` so the caller skips the whole group.
pub(crate) fn read_collision_ext(bytes: &[u8]) -> io::Result<Option<CollisionSnapshot>> {
    if bytes.len() > MAX_PAYLOAD_SIZE {
        return Err(invalid_data(format!(
            "RLPR v12 collision payload is {} bytes, above {MAX_PAYLOAD_SIZE}",
            bytes.len()
        )));
    }
    if bytes.len() < 2 {
        return Err(invalid_data(
            "RLPR v12 collision payload is truncated before the schema version",
        ));
    }

    let mut reader = DataReader::new(bytes);
    let schema_version = reader.read_u16()?;
    if schema_version != SCHEMA_VERSION {
        return Ok(None);
    }

    let requested_sections = reader.read_u16()?;
    let verified_sections = reader.read_u16()?;
    let flags = reader.read_u16()?;
    let layout_id = read_wire_enum::<CollisionLayoutId>(&mut reader, "layout_id")?;
    let source_path = read_wire_enum::<CollisionSourcePath>(&mut reader, "source_path")?;
    let engine_scene_index = reader.read_u8()?;
    let section_count = reader.read_u8()?;
    let mut executable_sha256 = [0u8; 32];
    for byte in &mut executable_sha256 {
        *byte = reader.read_u8()?;
    }
    let have_executable_sha256 = read_bool(&mut reader, "have_executable_sha256")?;
    for _ in 0..3 {
        reader.read_u8()?;
    }
    let resolver = read_resolver_trace(&mut reader)?;

    if flags & !KNOWN_HEADER_FLAGS != 0 {
        return Err(invalid_data(format!(
            "RLPR v12 collision header flags {flags:#x} are unknown"
        )));
    }
    if requested_sections & !KNOWN_SECTION_MASK != 0 {
        return Err(invalid_data(format!(
            "RLPR v12 collision requested mask {requested_sections:#x} is unknown"
        )));
    }
    if verified_sections & !KNOWN_SECTION_MASK != 0 {
        return Err(invalid_data(format!(
            "RLPR v12 collision verified mask {verified_sections:#x} is unknown"
        )));
    }
    if section_count != requested_sections.count_ones() as u8 {
        return Err(invalid_data(
            "RLPR v12 collision section_count does not match requested_sections",
        ));
    }

    let mut snapshot = CollisionSnapshot {
        schema_version,
        requested_sections,
        verified_sections,
        build_identified: flags & 0x01 != 0,
        limits_clamped: flags & 0x02 != 0,
        layout_id,
        source_path,
        engine_scene_index,
        section_count,
        executable_sha256,
        have_executable_sha256,
        resolver,
        ..CollisionSnapshot::default()
    };

    for id in 0..6u8 {
        if requested_sections & (1u16 << id) == 0 {
            continue;
        }
        let section_id = read_wire_enum::<CollisionSectionId>(&mut reader, "section_id")?;
        if section_id as u8 != id {
            return Err(invalid_data(format!(
                "RLPR v12 collision section id {section_id:?} does not match expected {id}"
            )));
        }
        let report = read_report(&mut reader, section_id)?;
        match section_id {
            CollisionSectionId::Handles => {
                snapshot.handles = Some(read_handles_section(&mut reader, report)?);
            }
            CollisionSectionId::Edges => {
                snapshot.edges = Some(read_edges_section(&mut reader, report)?);
            }
            CollisionSectionId::FreeHandles => {
                snapshot.free_handles = Some(read_free_handles_section(&mut reader, report)?);
            }
            CollisionSectionId::Bodies => {
                snapshot.bodies = Some(read_bodies_section(&mut reader, report)?);
            }
            CollisionSectionId::Pairs => {
                snapshot.pairs = Some(read_pairs_section(&mut reader, report)?);
            }
            CollisionSectionId::Manifolds => {
                snapshot.manifolds = Some(read_manifolds_section(&mut reader, report)?);
            }
        }
    }

    if let (Some(handles), Some(free_handles)) = (&snapshot.handles, &snapshot.free_handles)
        && handles.report.status == CollisionSectionStatus::Ok
        && free_handles.report.status == CollisionSectionStatus::Ok
    {
        validate_complete_handle_partition(handles, free_handles)?;
    }

    require_consumed(&mut reader, "payload")?;
    Ok(Some(snapshot))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn push_u16(out: &mut Vec<u8>, value: u16) {
        out.extend_from_slice(&value.to_le_bytes());
    }

    fn push_u32(out: &mut Vec<u8>, value: u32) {
        out.extend_from_slice(&value.to_le_bytes());
    }

    fn push_i32(out: &mut Vec<u8>, value: i32) {
        out.extend_from_slice(&value.to_le_bytes());
    }

    fn push_f32(out: &mut Vec<u8>, value: f32) {
        out.extend_from_slice(&value.to_le_bytes());
    }

    fn push_vec3(out: &mut Vec<u8>, values: [f32; 3]) {
        for value in values {
            push_f32(out, value);
        }
    }

    fn base_header(requested: u16) -> Vec<u8> {
        let mut out = Vec::new();
        push_u16(&mut out, SCHEMA_VERSION);
        push_u16(&mut out, requested);
        push_u16(&mut out, requested);
        push_u16(&mut out, KNOWN_HEADER_FLAGS);
        out.push(2);
        out.push(2);
        out.push(0);
        out.push(requested.count_ones() as u8);
        out.extend_from_slice(&[0u8; 32]);
        out.push(1);
        out.extend_from_slice(&[0, 0, 0]);
        out.push(13);
        out.extend_from_slice(&[1, 1, 1, 1, 1, 1, 0]);
        push_u32(&mut out, 1);
        push_u32(&mut out, 1);
        push_u32(&mut out, 2);
        push_u32(&mut out, 4);
        push_u32(&mut out, u32::MAX);
        push_u32(&mut out, 2);
        push_u32(&mut out, 0);
        push_u32(&mut out, 0);
        out
    }

    #[derive(Clone, Copy)]
    struct Report {
        status: u8,
        error: u8,
        section_flags: u8,
        captured_count: u32,
        record_count: u32,
        aux0: u32,
        aux1: u32,
        aux2: u32,
        truncated: u8,
    }

    impl Default for Report {
        fn default() -> Self {
            Self {
                status: 1,
                error: 0,
                section_flags: 0,
                captured_count: 0,
                record_count: 0,
                aux0: 0,
                aux1: 0,
                aux2: 0,
                truncated: 0,
            }
        }
    }

    fn section_block(section_id: u8, report: Report, records: &[u8]) -> Vec<u8> {
        let mut out = vec![
            section_id,
            report.status,
            report.error,
            report.section_flags,
        ];
        push_u32(&mut out, report.captured_count);
        push_u32(&mut out, report.captured_count);
        push_u32(&mut out, 0);
        push_u32(&mut out, 0);
        push_u32(&mut out, report.aux0);
        push_u32(&mut out, report.aux1);
        push_u32(&mut out, report.aux2);
        push_u32(&mut out, report.record_count);
        out.push(report.truncated);
        out.extend_from_slice(&[0, 0, 0]);
        out.extend_from_slice(records);
        out
    }

    fn section_payload(section_id: u8, report: Report, records: &[u8]) -> Vec<u8> {
        let mut out = base_header(1u16 << section_id);
        out.extend_from_slice(&section_block(section_id, report, records));
        out
    }

    fn multi_section_payload(requested: u16, sections: &[(u8, Report, Vec<u8>)]) -> Vec<u8> {
        let mut out = base_header(requested);
        for (section_id, report, records) in sections {
            out.extend_from_slice(&section_block(*section_id, *report, records));
        }
        out
    }

    fn free_runs_bytes(runs: &[(u16, u16, u16)]) -> Vec<u8> {
        let mut out = Vec::new();
        for (start, length, next) in runs {
            out.extend_from_slice(&free_run(*start, *length, *next));
        }
        out
    }

    fn body_record(role: u8, has_handle: u8, aabb_min: [f32; 3]) -> Vec<u8> {
        let mut out = Vec::new();
        push_i32(&mut out, 0);
        push_i32(&mut out, 1);
        push_i32(&mut out, 2);
        push_i32(&mut out, 3);
        push_i32(&mut out, 4);
        push_i32(&mut out, 5);
        push_u32(&mut out, 6);
        out.push(role);
        out.push(0);
        out.push(has_handle);
        out.push(0);
        push_vec3(&mut out, aabb_min);
        push_vec3(&mut out, [4.0, 5.0, 6.0]);
        out
    }

    fn handle_record(reserved: u8) -> Vec<u8> {
        handle_record_with_id(1, reserved)
    }

    fn handle_record_with_id(unique_id: u16, reserved: u8) -> Vec<u8> {
        let mut out = Vec::new();
        push_u16(&mut out, unique_id);
        push_u16(&mut out, 2);
        push_u16(&mut out, 3);
        for value in 4u16..10 {
            push_u16(&mut out, value);
        }
        push_i32(&mut out, -1);
        push_i32(&mut out, -2);
        out.push(reserved);
        out.extend_from_slice(&[0, 0, 0]);
        push_vec3(&mut out, [1.0, 2.0, 3.0]);
        push_vec3(&mut out, [4.0, 5.0, 6.0]);
        out
    }

    fn edge_axis(available: u8, mismatch: u8, edge_count: u32, edges: &[(u16, u16)]) -> Vec<u8> {
        let mut out = Vec::new();
        out.push(available);
        out.push(mismatch);
        push_u16(&mut out, 0);
        push_u32(&mut out, 0);
        push_u32(&mut out, 0);
        push_u32(&mut out, 0);
        push_u32(&mut out, edge_count);
        for (position, handle) in edges {
            push_u16(&mut out, *position);
            push_u16(&mut out, *handle);
        }
        out
    }

    fn pair_record(algorithm_present: u8) -> Vec<u8> {
        let mut out = Vec::new();
        push_i32(&mut out, 1);
        push_i32(&mut out, 2);
        push_i32(&mut out, 3);
        push_i32(&mut out, 4);
        out.push(algorithm_present);
        out.extend_from_slice(&[0, 0, 0]);
        out
    }

    fn free_run(start: u16, length: u16, next: u16) -> Vec<u8> {
        let mut out = Vec::new();
        push_u16(&mut out, start);
        push_u16(&mut out, length);
        push_u16(&mut out, next);
        out
    }

    fn free_handles_payload(
        runs: &[(u16, u16, u16)],
        entry_count: u32,
        section_flags: u8,
    ) -> Vec<u8> {
        section_payload(
            2,
            Report {
                captured_count: entry_count,
                record_count: runs.len() as u32,
                aux0: entry_count,
                section_flags,
                ..Report::default()
            },
            &free_runs_bytes(runs),
        )
    }

    fn manifold_record(
        point_count: u8,
        positive: u8,
        non_positive: u8,
        island_valid: u8,
        mask: u32,
        points: &[u8],
    ) -> Vec<u8> {
        let mut out = Vec::new();
        push_u32(&mut out, 0);
        push_i32(&mut out, -1);
        push_i32(&mut out, -1);
        push_i32(&mut out, -1);
        push_i32(&mut out, -1);
        push_i32(&mut out, -1);
        push_i32(&mut out, 0);
        push_i32(&mut out, 0);
        push_f32(&mut out, 0.02);
        push_f32(&mut out, 0.04);
        out.push(point_count);
        out.push(positive);
        out.push(non_positive);
        out.push(island_valid);
        push_u32(&mut out, mask);
        out.extend_from_slice(points);
        out
    }

    fn assert_rejects(expected: &str, bytes: &[u8]) {
        let err = read_collision_ext(bytes)
            .expect_err(&format!("expected an error containing {expected:?}"));
        assert!(
            err.to_string().contains(expected),
            "expected {expected:?}, got {err}"
        );
    }

    #[test]
    fn minimal_bodies_payload_parses() {
        let records = body_record(2, 1, [1.0, 2.0, 3.0]);
        let payload = section_payload(
            3,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &records,
        );
        let snapshot = read_collision_ext(&payload).unwrap().unwrap();
        assert_eq!(snapshot.schema_version, SCHEMA_VERSION);
        assert_eq!(snapshot.requested_sections, SECTION_BODIES);
        assert_eq!(snapshot.section_count, 1);
        assert_eq!(snapshot.resolver.stage, CollisionResolveStage::Ok);
        assert_eq!(snapshot.resolver.scene_count, 2);
        assert_eq!(snapshot.resolver.scene_capacity, 4);
        let bodies = snapshot.bodies.as_ref().unwrap();
        assert_eq!(bodies.records.len(), 1);
        assert_eq!(bodies.records[0].role, CollisionBodyRole::Car);
        assert!(bodies.records[0].has_broadphase_handle);
        assert!(snapshot.handles.is_none());
        assert!(snapshot.edges.is_none());
        assert!(snapshot.free_handles.is_none());
        assert!(snapshot.pairs.is_none());
        assert!(snapshot.manifolds.is_none());
    }

    #[test]
    fn unknown_schema_version_skips_the_group() {
        let records = body_record(2, 1, [1.0, 2.0, 3.0]);
        let payload = section_payload(
            3,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &records,
        );
        for version in [1u16, 2] {
            let mut historical = payload.clone();
            historical[0..2].copy_from_slice(&version.to_le_bytes());
            assert_eq!(read_collision_ext(&historical).unwrap(), None);
        }
    }

    #[test]
    fn rejects_payload_cap_and_truncation() {
        assert_rejects("above", &vec![0u8; MAX_PAYLOAD_SIZE + 1]);
        assert_rejects("truncated", &[]);
        assert_rejects("truncated", &[0u8]);
    }

    #[test]
    fn rejects_bad_header() {
        let base = || {
            section_payload(
                3,
                Report {
                    captured_count: 1,
                    record_count: 1,
                    ..Report::default()
                },
                &body_record(2, 1, [1.0, 2.0, 3.0]),
            )
        };

        let mut cases: Vec<(&str, Vec<u8>)> = Vec::new();
        let mut payload = base();
        payload[6..8].copy_from_slice(&0x04u16.to_le_bytes());
        cases.push(("header flags", payload));
        let mut payload = base();
        payload[2..4].copy_from_slice(&0x40u16.to_le_bytes());
        cases.push(("requested mask", payload));
        let mut payload = base();
        payload[4..6].copy_from_slice(&0x40u16.to_le_bytes());
        cases.push(("verified mask", payload));
        let mut payload = base();
        payload[11] = 2;
        cases.push(("section_count", payload));
        let mut payload = base();
        payload[8] = 3;
        cases.push(("layout_id", payload));
        let mut payload = base();
        payload[9] = 3;
        cases.push(("source_path", payload));
        let mut payload = base();
        payload[44] = 2;
        cases.push(("have_executable_sha256", payload));
        let mut payload = base();
        payload[48] = 14;
        cases.push(("resolver stage", payload));
        let mut payload = base();
        payload[49] = 2;
        cases.push(("resolver engine_pointer_present", payload));
        let mut payload = base();
        payload[64..68].copy_from_slice(&(MAX_SCENE_COUNT + 1).to_le_bytes());
        cases.push(("resolver scene_count", payload));
        let mut payload = base();
        payload[68..72].copy_from_slice(&(MAX_SCENE_COUNT + 1).to_le_bytes());
        cases.push(("resolver scene_capacity", payload));
        let mut payload = base();
        payload[64..68].copy_from_slice(&3u32.to_le_bytes());
        payload[68..72].copy_from_slice(&2u32.to_le_bytes());
        cases.push(("resolver scene_capacity", payload));
        let mut payload = base();
        payload[60..64].copy_from_slice(&MAX_SCENE_COUNT.to_le_bytes());
        cases.push(("resolver resolved_scene", payload));
        let mut payload = base();
        payload[72..76].copy_from_slice(&MAX_SCENE_COUNT.to_le_bytes());
        cases.push(("resolver first_bad_index", payload));

        for (expected, payload) in cases {
            assert_rejects(expected, &payload);
        }
    }

    #[test]
    fn rejects_bad_report() {
        let base = || {
            section_payload(
                3,
                Report {
                    captured_count: 1,
                    record_count: 1,
                    ..Report::default()
                },
                &body_record(2, 1, [1.0, 2.0, 3.0]),
            )
        };

        let mut cases: Vec<(&str, Vec<u8>)> = Vec::new();
        let mut payload = base();
        payload[HEADER_SIZE + 1] = 0;
        cases.push(("status not-requested", payload));
        let mut payload = base();
        payload[HEADER_SIZE + 2] = 1;
        cases.push(("error not-requested", payload));
        let mut payload = base();
        payload[HEADER_SIZE + 2] = 2;
        cases.push(("status and error do not agree", payload));
        let mut payload = base();
        payload[HEADER_SIZE + 36] = 2;
        cases.push(("truncated", payload));
        let mut payload = base();
        payload[HEADER_SIZE + 3] = 1;
        cases.push(("unexpected flags", payload));

        for (expected, payload) in cases {
            assert_rejects(expected, &payload);
        }

        let free_handles = free_handles_payload(&[(5, 1, 0)], 1, 0x08);
        assert_rejects("free-handle flags", &free_handles);
    }

    #[test]
    fn rejects_bad_counts_and_caps() {
        let mut cases: Vec<(&str, Vec<u8>)> = Vec::new();
        for (section_id, cap, name) in [
            (0u8, MAX_HANDLES, "handle count"),
            (2u8, MAX_FREE_RUNS, "free handle run count"),
            (3u8, MAX_BODIES, "body count"),
            (4u8, MAX_PAIRS, "pair count"),
            (5u8, MAX_MANIFOLDS, "manifold count"),
        ] {
            let payload = section_payload(
                section_id,
                Report {
                    captured_count: cap + 1,
                    record_count: cap + 1,
                    ..Report::default()
                },
                &[],
            );
            cases.push((name, payload));
        }

        let edges_cap = section_payload(
            1,
            Report {
                captured_count: 0,
                record_count: 3,
                ..Report::default()
            },
            &edge_axis(1, 0, MAX_EDGES_PER_AXIS + 1, &[]),
        );
        cases.push(("edge count", edges_cap));

        let handles_mismatch = section_payload(
            0,
            Report {
                captured_count: 2,
                record_count: 1,
                ..Report::default()
            },
            &[],
        );
        cases.push(("captured_count", handles_mismatch));

        let handles_pool = section_payload(
            0,
            Report {
                captured_count: 1,
                record_count: 1,
                aux0: 1,
                aux1: MAX_HANDLES + 1,
                ..Report::default()
            },
            &handle_record(0),
        );
        cases.push(("pool_entry_count", handles_pool));

        let handle_id = section_payload(
            0,
            Report {
                captured_count: 1,
                record_count: 1,
                aux0: 1,
                aux1: 1,
                ..Report::default()
            },
            &handle_record_with_id((MAX_HANDLES + 1) as u16, 0),
        );
        cases.push(("handle unique_id", handle_id));

        let edges_record_count = section_payload(
            1,
            Report {
                captured_count: 0,
                record_count: 2,
                ..Report::default()
            },
            &[],
        );
        cases.push(("edges record_count", edges_record_count));

        let mut axes = Vec::new();
        for _ in 0..3 {
            axes.extend(edge_axis(1, 0, 0, &[]));
        }
        let edges_captured = section_payload(
            1,
            Report {
                captured_count: 1,
                record_count: 3,
                ..Report::default()
            },
            &axes,
        );
        cases.push(("edges captured_count", edges_captured));

        let pairs_array = section_payload(
            4,
            Report {
                captured_count: 0,
                record_count: 0,
                aux0: 2,
                aux1: 1,
                ..Report::default()
            },
            &[],
        );
        cases.push(("array_size", pairs_array));

        let manifold_order = section_payload(
            5,
            Report {
                captured_count: 0,
                record_count: 0,
                aux2: 3,
                ..Report::default()
            },
            &[],
        );
        cases.push(("manifold order", manifold_order));

        for (expected, payload) in cases {
            assert_rejects(expected, &payload);
        }
    }

    #[test]
    fn free_handle_runs_iterate_the_exact_chain() {
        let runs = [(5, 1, 9), (9, 2, 11), (11, 1, 2), (2, 1, 3), (3, 1, 0)];
        let payload = free_handles_payload(&runs, 6, 0x01);
        let snapshot = read_collision_ext(&payload).unwrap().unwrap();
        let section = snapshot.free_handles.as_ref().unwrap();

        assert_eq!(section.entry_count(), 6);
        assert_eq!(section.run_count(), 5);
        assert_eq!(section.report.captured_count, 6);
        assert_eq!(section.report.record_count, 5);
        assert!(section.terminated);
        assert_eq!(section.runs.len(), 5);

        let links: Vec<_> = section.iter_chain().collect();
        assert_eq!(
            links,
            vec![
                CollisionFreeChainLink {
                    unique_id: 5,
                    next_unique_id: 9,
                },
                CollisionFreeChainLink {
                    unique_id: 9,
                    next_unique_id: 10,
                },
                CollisionFreeChainLink {
                    unique_id: 10,
                    next_unique_id: 11,
                },
                CollisionFreeChainLink {
                    unique_id: 11,
                    next_unique_id: 2,
                },
                CollisionFreeChainLink {
                    unique_id: 2,
                    next_unique_id: 3,
                },
                CollisionFreeChainLink {
                    unique_id: 3,
                    next_unique_id: 0,
                },
            ]
        );
        assert_eq!(section.iter_chain().len(), 6);
    }

    #[test]
    fn inclusive_handle_bound_is_accepted() {
        let payload = section_payload(
            0,
            Report {
                captured_count: 1,
                record_count: 1,
                aux0: 1,
                aux1: MAX_HANDLES,
                ..Report::default()
            },
            &handle_record_with_id(MAX_HANDLES as u16, 0),
        );
        let snapshot = read_collision_ext(&payload).unwrap().unwrap();
        assert_eq!(
            snapshot.handles.as_ref().unwrap().records[0].unique_id,
            MAX_HANDLES as u16
        );

        let payload = free_handles_payload(&[(MAX_HANDLES as u16, 1, 0)], 1, 0x01);
        let snapshot = read_collision_ext(&payload).unwrap().unwrap();
        let section = snapshot.free_handles.as_ref().unwrap();
        assert_eq!(
            section.iter_chain().collect::<Vec<_>>(),
            vec![CollisionFreeChainLink {
                unique_id: MAX_HANDLES as u16,
                next_unique_id: 0,
            }]
        );
    }

    #[test]
    fn free_handle_runs_allow_a_cycle_without_termination() {
        let payload = free_handles_payload(&[(5, 1, 9), (9, 1, 5)], 2, 0x02);
        let snapshot = read_collision_ext(&payload).unwrap().unwrap();
        let section = snapshot.free_handles.as_ref().unwrap();

        assert!(!section.terminated);
        assert!(section.cycle_detected);
        assert_eq!(
            section.iter_chain().collect::<Vec<_>>(),
            vec![
                CollisionFreeChainLink {
                    unique_id: 5,
                    next_unique_id: 9,
                },
                CollisionFreeChainLink {
                    unique_id: 9,
                    next_unique_id: 5,
                },
            ]
        );
    }

    #[test]
    fn rejects_bad_free_handle_runs() {
        let mut cases: Vec<(&str, Vec<u8>)> = vec![
            ("zero length", free_handles_payload(&[(5, 0, 9)], 1, 0)),
            ("start 0", free_handles_payload(&[(0, 1, 9)], 1, 0)),
            ("run end", free_handles_payload(&[(16384, 2, 0)], 2, 0x01)),
            ("successor", free_handles_payload(&[(5, 1, 16385)], 1, 0)),
            ("sum", free_handles_payload(&[(5, 1, 9)], 2, 0)),
            (
                "run count",
                free_handles_payload(&[(5, 1, 9), (9, 1, 0)], 1, 0x01),
            ),
            (
                "do not chain",
                free_handles_payload(&[(5, 1, 7), (9, 1, 0)], 2, 0x01),
            ),
            ("terminated", free_handles_payload(&[(5, 1, 9)], 1, 0x01)),
        ];

        let mut entry_count = free_handles_payload(&[(5, 1, 9)], 1, 0);
        let aux0 = HEADER_SIZE + 20;
        entry_count[aux0..aux0 + 4].copy_from_slice(&2u32.to_le_bytes());
        cases.push(("entry_count", entry_count));

        let too_many_entries = free_handles_payload(&[(5, 1, 9)], MAX_FREE_HANDLES + 1, 0);
        cases.push(("entry count", too_many_entries));

        let too_many_runs = section_payload(
            2,
            Report {
                captured_count: MAX_FREE_RUNS + 1,
                record_count: MAX_FREE_RUNS + 1,
                aux0: MAX_FREE_RUNS + 1,
                ..Report::default()
            },
            &[],
        );
        cases.push(("free handle run count", too_many_runs));

        for (expected, payload) in cases {
            assert_rejects(expected, &payload);
        }
    }

    #[test]
    fn complete_partition_accepts_valid_handles_and_free_runs() {
        let payload = multi_section_payload(
            SECTION_HANDLES | SECTION_FREE_HANDLES,
            &[
                (
                    0,
                    Report {
                        captured_count: 2,
                        record_count: 2,
                        aux0: 2,
                        aux1: 5,
                        ..Report::default()
                    },
                    [handle_record_with_id(0, 0), handle_record_with_id(1, 0)].concat(),
                ),
                (
                    2,
                    Report {
                        captured_count: 3,
                        record_count: 1,
                        aux0: 3,
                        section_flags: 0x01,
                        ..Report::default()
                    },
                    free_runs_bytes(&[(2, 3, 0)]),
                ),
            ],
        );
        let snapshot = read_collision_ext(&payload).unwrap().unwrap();
        assert!(snapshot.handles.is_some());
        assert!(snapshot.free_handles.is_some());
    }

    #[test]
    fn complete_partition_rejects_lying_counts() {
        let payload = multi_section_payload(
            SECTION_HANDLES | SECTION_FREE_HANDLES,
            &[
                (
                    0,
                    Report {
                        captured_count: 2,
                        record_count: 2,
                        aux0: 2,
                        aux1: 6,
                        ..Report::default()
                    },
                    [handle_record_with_id(0, 0), handle_record_with_id(1, 0)].concat(),
                ),
                (
                    2,
                    Report {
                        captured_count: 3,
                        record_count: 1,
                        aux0: 3,
                        section_flags: 0x01,
                        ..Report::default()
                    },
                    free_runs_bytes(&[(2, 3, 0)]),
                ),
            ],
        );
        assert_rejects("partition mismatch", &payload);
    }

    #[test]
    fn complete_partition_rejects_active_free_overlap() {
        let payload = multi_section_payload(
            SECTION_HANDLES | SECTION_FREE_HANDLES,
            &[
                (
                    0,
                    Report {
                        captured_count: 2,
                        record_count: 2,
                        aux0: 2,
                        aux1: 5,
                        ..Report::default()
                    },
                    [handle_record_with_id(1, 0), handle_record_with_id(3, 0)].concat(),
                ),
                (
                    2,
                    Report {
                        captured_count: 3,
                        record_count: 1,
                        aux0: 3,
                        section_flags: 0x01,
                        ..Report::default()
                    },
                    free_runs_bytes(&[(2, 3, 0)]),
                ),
            ],
        );
        assert_rejects("active handle id 3 is inside a free run", &payload);

        let payload = multi_section_payload(
            SECTION_HANDLES | SECTION_FREE_HANDLES,
            &[
                (
                    0,
                    Report {
                        captured_count: 2,
                        record_count: 2,
                        aux0: 2,
                        aux1: 5,
                        ..Report::default()
                    },
                    [handle_record_with_id(1, 0), handle_record_with_id(1, 0)].concat(),
                ),
                (
                    2,
                    Report {
                        captured_count: 3,
                        record_count: 1,
                        aux0: 3,
                        section_flags: 0x01,
                        ..Report::default()
                    },
                    free_runs_bytes(&[(2, 3, 0)]),
                ),
            ],
        );
        assert_rejects("duplicate active handle id", &payload);
    }

    #[test]
    fn complete_partition_rejects_overlapping_free_runs() {
        let payload = multi_section_payload(
            SECTION_HANDLES | SECTION_FREE_HANDLES,
            &[
                (
                    0,
                    Report {
                        captured_count: 0,
                        record_count: 0,
                        aux0: 0,
                        aux1: 9,
                        ..Report::default()
                    },
                    Vec::new(),
                ),
                (
                    2,
                    Report {
                        captured_count: 9,
                        record_count: 2,
                        aux0: 9,
                        section_flags: 0x01,
                        ..Report::default()
                    },
                    free_runs_bytes(&[(10, 2, 5), (5, 7, 0)]),
                ),
            ],
        );
        assert_rejects("overlapping free runs", &payload);
    }

    #[test]
    fn complete_partition_does_not_expand_large_free_runs() {
        let payload = multi_section_payload(
            SECTION_HANDLES | SECTION_FREE_HANDLES,
            &[
                (
                    0,
                    Report {
                        captured_count: 0,
                        record_count: 0,
                        aux0: 0,
                        aux1: 16000,
                        ..Report::default()
                    },
                    Vec::new(),
                ),
                (
                    2,
                    Report {
                        captured_count: 16000,
                        record_count: 1,
                        aux0: 16000,
                        section_flags: 0x01,
                        ..Report::default()
                    },
                    free_runs_bytes(&[(1, 16000, 0)]),
                ),
            ],
        );
        let snapshot = read_collision_ext(&payload).unwrap().unwrap();
        let section = snapshot.free_handles.as_ref().unwrap();
        assert_eq!(section.runs.len(), 1);
        assert_eq!(section.entry_count(), 16000);
        assert_eq!(
            section.iter_chain().take(3).collect::<Vec<_>>(),
            vec![
                CollisionFreeChainLink {
                    unique_id: 1,
                    next_unique_id: 2,
                },
                CollisionFreeChainLink {
                    unique_id: 2,
                    next_unique_id: 3,
                },
                CollisionFreeChainLink {
                    unique_id: 3,
                    next_unique_id: 4,
                },
            ]
        );
    }

    #[test]
    fn partial_partition_is_advisory() {
        let payload = multi_section_payload(
            SECTION_HANDLES | SECTION_FREE_HANDLES,
            &[
                (
                    0,
                    Report {
                        status: 2,
                        error: 8,
                        captured_count: 1,
                        record_count: 1,
                        aux0: 1,
                        aux1: 5,
                        ..Report::default()
                    },
                    handle_record_with_id(1, 0),
                ),
                (
                    2,
                    Report {
                        captured_count: 1,
                        record_count: 1,
                        aux0: 1,
                        section_flags: 0x01,
                        ..Report::default()
                    },
                    free_runs_bytes(&[(2, 1, 0)]),
                ),
            ],
        );
        let snapshot = read_collision_ext(&payload).unwrap().unwrap();
        assert_eq!(
            snapshot.handles.as_ref().unwrap().report.status,
            CollisionSectionStatus::Partial
        );
        assert!(snapshot.free_handles.is_some());
    }

    #[test]
    fn rejects_bad_manifold_points() {
        let mut cases: Vec<(&str, Vec<u8>)> = Vec::new();

        let unknown_mask = section_payload(
            5,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &manifold_record(0, 0, 0, 0, 0x1000, &[]),
        );
        cases.push(("point field mask", unknown_mask));

        let too_many_points = section_payload(
            5,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &manifold_record(5, 0, 0, 0, 0, &[]),
        );
        cases.push(("point count", too_many_points));

        let distance_counts = section_payload(
            5,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &manifold_record(1, 0, 0, 0, 0, &[]),
        );
        cases.push(("distance counts", distance_counts));

        let mut points = Vec::new();
        push_f32(&mut points, f32::NAN);
        let nonfinite_distance = section_payload(
            5,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &manifold_record(1, 1, 0, 1, POINT_FIELD_DISTANCE, &points),
        );
        cases.push(("point distance1", nonfinite_distance));

        for (expected, payload) in cases {
            assert_rejects(expected, &payload);
        }

        let mut points = Vec::new();
        push_f32(&mut points, 0.5);
        let payload = section_payload(
            5,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &manifold_record(1, 1, 0, 1, POINT_FIELD_DISTANCE, &points),
        );
        let snapshot = read_collision_ext(&payload).unwrap().unwrap();
        let point = &snapshot.manifolds.as_ref().unwrap().records[0].points[0];
        assert_eq!(point.distance1, Some(0.5));
        assert_eq!(point.local_point_a, None);
        assert_eq!(point.normal_world_on_b, None);
    }

    #[test]
    fn rejects_nonfinite_floats_and_bad_booleans() {
        let mut cases: Vec<(&str, Vec<u8>)> = Vec::new();

        let body_nan = section_payload(
            3,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &body_record(2, 1, [f32::NAN, 0.0, 0.0]),
        );
        cases.push(("body stored_aabb_min", body_nan));

        let body_role = section_payload(
            3,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &body_record(5, 1, [1.0, 2.0, 3.0]),
        );
        cases.push(("body role", body_role));

        let body_bool = section_payload(
            3,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &body_record(2, 2, [1.0, 2.0, 3.0]),
        );
        cases.push(("body has_broadphase_handle", body_bool));

        let handle_bool = section_payload(
            0,
            Report {
                captured_count: 1,
                record_count: 1,
                aux0: 1,
                ..Report::default()
            },
            &handle_record(2),
        );
        cases.push(("handle in_free_list", handle_bool));

        let mut axes = edge_axis(2, 0, 0, &[]);
        for _ in 0..2 {
            axes.extend(edge_axis(1, 0, 0, &[]));
        }
        let edge_bool = section_payload(
            1,
            Report {
                captured_count: 0,
                record_count: 3,
                ..Report::default()
            },
            &axes,
        );
        cases.push(("edge axis available", edge_bool));

        let pair_bool = section_payload(
            4,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &pair_record(2),
        );
        cases.push(("pair algorithm_present", pair_bool));

        let manifold_bool = section_payload(
            5,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &manifold_record(0, 0, 0, 2, 0, &[]),
        );
        cases.push(("manifold island_tags_valid", manifold_bool));

        let mut manifold = manifold_record(0, 0, 0, 0, 0, &[]);
        manifold[32..36].copy_from_slice(&f32::INFINITY.to_le_bytes());
        let manifold_threshold = section_payload(
            5,
            Report {
                captured_count: 1,
                record_count: 1,
                ..Report::default()
            },
            &manifold,
        );
        cases.push(("manifold contact_breaking_threshold", manifold_threshold));

        for (expected, payload) in cases {
            assert_rejects(expected, &payload);
        }
    }

    #[test]
    fn rejects_trailing_payload_and_bad_section_id() {
        let base = || {
            section_payload(
                3,
                Report {
                    captured_count: 1,
                    record_count: 1,
                    ..Report::default()
                },
                &body_record(2, 1, [1.0, 2.0, 3.0]),
            )
        };

        let mut trailing = base();
        trailing.push(0);
        assert_rejects("trailing bytes", &trailing);

        let mut wrong_section = base();
        wrong_section[HEADER_SIZE] = 4;
        assert_rejects("section id", &wrong_section);
    }
}
