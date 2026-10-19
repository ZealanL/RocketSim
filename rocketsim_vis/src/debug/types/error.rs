use std::fmt;

/// Typed rejection reason for a debug submission.
///
/// Fallible submissions return this error before network delivery. Wire
/// decoding reports invalid data through [`crate::proto::ProtocolError`].
///
/// ```
/// use rocketsim_vis::debug::{Color, DebugError, Width};
///
/// assert_eq!(Width::pixels(0.0).validate(), Err(DebugError::InvalidWidth));
/// let color = Color::rgb(2.0, 0.0, 0.0);
/// assert_eq!(color.validate(), Err(DebugError::InvalidColor));
/// ```
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum DebugError {
    /// Name or resource id is empty, too long, or contains rejected characters.
    InvalidName(String),
    /// A user resource uses an identifier reserved for a built-in asset.
    ReservedResourceId(String),
    /// A resource handle cannot insert a different resource id.
    ResourceIdMismatch {
        /// Expected resource id from the handle.
        expected: String,
        /// Resource id supplied by the caller.
        actual: String,
    },
    /// Too many groups in the scene.
    TooManyGroups {
        /// Observed group count.
        count: usize,
        /// Maximum allowed group count.
        max: usize,
    },
    /// Too many items in one group.
    TooManyItems {
        /// Name of the group that exceeded its item limit.
        group: String,
        /// Observed item count.
        count: usize,
        /// Maximum allowed item count.
        max: usize,
    },
    /// Too many items in the whole scene.
    TooManyTotalItems {
        /// Observed item count.
        count: usize,
        /// Maximum allowed item count.
        max: usize,
    },
    /// Too many points in a polyline.
    TooManyPoints {
        /// Observed point count.
        count: usize,
        /// Maximum allowed point count.
        max: usize,
    },
    /// Too many vertices in a mesh.
    TooManyVertices {
        /// Observed vertex count.
        count: usize,
        /// Maximum allowed vertex count.
        max: usize,
    },
    /// Too many indices in a mesh.
    TooManyIndices {
        /// Observed index count.
        count: usize,
        /// Maximum allowed index count.
        max: usize,
    },
    /// Index count is not a triangle list (`% 3 != 0`).
    InvalidIndexCount {
        /// Supplied index count.
        count: usize,
    },
    /// Index points outside the vertex buffer.
    InvalidIndex {
        /// Index that is outside the vertex buffer.
        index: u32,
        /// Number of available vertices.
        vertices: usize,
    },
    /// Too many reusable meshes.
    TooManyMeshes {
        /// Observed mesh count.
        count: usize,
        /// Maximum allowed mesh count.
        max: usize,
    },
    /// Too many reusable textures.
    TooManyTextures {
        /// Observed texture count.
        count: usize,
        /// Maximum allowed texture count.
        max: usize,
    },
    /// Texture dimensions are zero or exceed [`crate::debug::MAX_TEXTURE_DIM`].
    InvalidTextureDim {
        /// Texture width in pixels.
        width: u32,
        /// Texture height in pixels.
        height: u32,
    },
    /// RGBA length does not equal `width * height * 4`.
    TextureSizeMismatch {
        /// Required RGBA byte count.
        expected: usize,
        /// Supplied RGBA byte count.
        got: usize,
    },
    /// Texture byte size exceeds [`crate::debug::MAX_TEXTURE_BYTES`].
    TextureTooLarge {
        /// Required RGBA byte count.
        bytes: usize,
        /// Maximum allowed byte count.
        max: usize,
    },
    /// Label text exceeds [`crate::debug::MAX_TEXT_LEN`].
    TextTooLong {
        /// Text length in UTF-8 bytes.
        len: usize,
        /// Maximum allowed byte count.
        max: usize,
    },
    /// A float field is NaN/infinite or out of its documented range.
    InvalidNumber(&'static str),
    /// Color components must be finite `0..=1`.
    InvalidColor,
    /// Width must be finite and positive within its cap.
    InvalidWidth,
    /// An anchor is invalid. See [`crate::debug::Anchor::validate`] for current checks.
    InvalidAnchor,
    /// Group or instance transform is not finite or has an invalid scale.
    InvalidTransform,
    /// Lifetime value is outside its valid range.
    InvalidLifetime,
    /// Referenced mesh id does not exist (checked on submit when feasible).
    MeshNotFound(String),
    /// Referenced texture id does not exist.
    TextureNotFound(String),
    /// Resource is still referenced by a retained group.
    ResourceInUse(String),
    /// Named group does not exist for a targeted update.
    GroupNotFound(String),
    /// Item handle refers to a different group generation.
    StaleHandle,
    /// Estimated scene payload exceeds [`crate::debug::MAX_DEBUG_BYTES`].
    PayloadTooLarge {
        /// Estimated scene byte count.
        bytes: usize,
        /// Maximum allowed byte count.
        max: usize,
    },
}

impl fmt::Display for DebugError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidName(n) => write!(f, "invalid debug name {n:?}"),
            Self::ReservedResourceId(id) => {
                write!(f, "resource id {id:?} is reserved for a built-in asset")
            }
            Self::ResourceIdMismatch { expected, actual } => write!(
                f,
                "resource id {actual:?} does not match handle {expected:?}"
            ),
            Self::TooManyGroups { count, max } => {
                write!(f, "too many debug groups: {count} > {max}")
            }
            Self::TooManyItems { group, count, max } => {
                write!(f, "group {group:?} has too many items: {count} > {max}")
            }
            Self::TooManyTotalItems { count, max } => {
                write!(f, "too many debug items: {count} > {max}")
            }
            Self::TooManyPoints { count, max } => {
                write!(f, "polyline has too many points: {count} > {max}")
            }
            Self::TooManyVertices { count, max } => {
                write!(f, "mesh has too many vertices: {count} > {max}")
            }
            Self::TooManyIndices { count, max } => {
                write!(f, "mesh has too many indices: {count} > {max}")
            }
            Self::InvalidIndexCount { count } => {
                write!(f, "mesh index count {count} is not a triangle list")
            }
            Self::InvalidIndex { index, vertices } => {
                write!(f, "mesh index {index} out of {vertices} vertices")
            }
            Self::TooManyMeshes { count, max } => {
                write!(f, "too many meshes: {count} > {max}")
            }
            Self::TooManyTextures { count, max } => {
                write!(f, "too many textures: {count} > {max}")
            }
            Self::InvalidTextureDim { width, height } => {
                write!(f, "invalid texture dimensions {width}x{height}")
            }
            Self::TextureSizeMismatch { expected, got } => {
                write!(
                    f,
                    "texture size mismatch: expected {expected} bytes, got {got}"
                )
            }
            Self::TextureTooLarge { bytes, max } => {
                write!(f, "texture too large: {bytes} > {max} bytes")
            }
            Self::TextTooLong { len, max } => {
                write!(f, "label text too long: {len} > {max} bytes")
            }
            Self::InvalidNumber(what) => write!(f, "invalid number in {what}"),
            Self::InvalidColor => write!(f, "color components must be finite 0..=1"),
            Self::InvalidWidth => write!(f, "width must be finite and positive"),
            Self::InvalidAnchor => write!(f, "anchor offset must be finite"),
            Self::InvalidTransform => write!(f, "transform must be finite"),
            Self::InvalidLifetime => write!(f, "lifetime is out of range"),
            Self::MeshNotFound(id) => write!(f, "mesh {id:?} not found"),
            Self::TextureNotFound(id) => write!(f, "texture {id:?} not found"),
            Self::ResourceInUse(id) => write!(f, "resource {id:?} is still in use"),
            Self::GroupNotFound(name) => write!(f, "group {name:?} not found"),
            Self::StaleHandle => write!(f, "item handle is stale"),
            Self::PayloadTooLarge { bytes, max } => {
                write!(f, "debug payload too large: {bytes} > {max} bytes")
            }
        }
    }
}

impl std::error::Error for DebugError {}

/// Convenience alias for debug submissions.
pub type DebugResult<T> = Result<T, DebugError>;
