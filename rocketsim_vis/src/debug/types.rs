//! Core debug-rendering primitives: [`Color`], [`Anchor`], [`Width`],
//! [`DepthMode`], [`Lifetime`], [`DebugItem`], and [`DrawList`] builders.
//!
//! Drawing records use arrays and support `serde` serialization. Three-
//! dimensional positions use Unreal units in group-local space. Group and
//! anchor transforms place them in the world. Screen positions use fractions
//! of the viewport, with `[0.0, 0.0]` at its top-left corner and `[1.0, 1.0]`
//! at its bottom-right corner. Finite values outside that range are accepted.
//! Colors use separate red, green, blue, and alpha components in `0..=1`.
//! RGB values are not multiplied by alpha before submission.
//!
//! Group transform order:
//!
//! 1. Item local coordinates.
//! 2. Instance transform (for [`MeshInstanceItem`]): `p1 = ipos + irot * (iscale * p0)`.
//! 3. Group transform: `p2 = gpos + grot * (gscale * p1)`.
//! 4. Anchor offset (in anchor frame): `p3 = p2 + offset`.
//! 5. Anchor frame: `World` is identity; `Ball`/`Car` prepend the live
//!    rigid transform `p_world = apos + arot * p3` sampled from the newest
//!    physics frame by the renderer.
//!
//! Scale applies first, then rotation, then translation at each stage.

use std::fmt;

use glam::Vec3A;

// ---------------------------------------------------------------------------
// Bounds (memory / count caps; enforced on submit and on wire decode)
// ---------------------------------------------------------------------------

/// Maximum retained groups per scene.
pub const MAX_GROUPS: usize = 256;
/// Maximum items in one group.
pub const MAX_ITEMS_PER_GROUP: usize = 4096;
/// Maximum total items across a scene snapshot.
pub const MAX_TOTAL_ITEMS: usize = 65_536;
/// Maximum points in one polyline.
pub const MAX_POLYLINE_POINTS: usize = 4096;
/// Maximum vertices in one inline mesh or mesh resource.
pub const MAX_MESH_VERTICES: usize = 65_536;
/// Maximum indices in one inline mesh or mesh resource (must be `% 3 == 0`).
pub const MAX_MESH_INDICES: usize = 262_144;
/// Maximum reusable meshes per scene.
pub const MAX_MESHES: usize = 128;
/// Maximum reusable textures per scene.
pub const MAX_TEXTURES: usize = 64;
/// Maximum texture width or height in pixels.
pub const MAX_TEXTURE_DIM: u32 = 1024;
/// Maximum decoded RGBA bytes for one texture (`1024*1024*4`).
pub const MAX_TEXTURE_BYTES: usize = 4 * 1024 * 1024;
/// Maximum UTF-8 bytes for one label.
pub const MAX_TEXT_LEN: usize = 1024;
/// Maximum bytes for a group / mesh / texture name or id.
pub const MAX_NAME_LEN: usize = 64;
/// Maximum estimated bytes for retained groups and resources in one scene.
pub const MAX_DEBUG_BYTES: usize = 8 * 1024 * 1024;

// ---------------------------------------------------------------------------
// Validation error
// ---------------------------------------------------------------------------

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
    /// Texture dimensions are zero or exceed [`MAX_TEXTURE_DIM`].
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
    /// Texture byte size exceeds [`MAX_TEXTURE_BYTES`].
    TextureTooLarge {
        /// Required RGBA byte count.
        bytes: usize,
        /// Maximum allowed byte count.
        max: usize,
    },
    /// Label text exceeds [`MAX_TEXT_LEN`].
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
    /// An anchor is invalid. See [`Anchor::validate`] for current checks.
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
    /// Estimated scene payload exceeds [`MAX_DEBUG_BYTES`].
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

// ---------------------------------------------------------------------------
// Names
// ---------------------------------------------------------------------------

/// Validate a group name or resource identifier.
///
/// Accept 1 through [`MAX_NAME_LEN`] UTF-8 bytes. Each slash-separated segment
/// must be nonempty, must not be `.` or `..`, and must not contain whitespace
/// or control characters. Other Unicode characters are accepted.
///
/// # Errors
///
/// Return [`DebugError::InvalidName`] with the supplied name if a rule fails.
///
/// ```
/// use rocketsim_vis::debug::validate_name;
///
/// assert!(validate_name("planner/target").is_ok());
/// assert!(validate_name("planner/../target").is_err());
/// assert!(validate_name("target name").is_err());
/// ```
pub fn validate_name(name: &str) -> DebugResult<()> {
    let len = name.len();
    if len == 0 || len > MAX_NAME_LEN {
        return Err(DebugError::InvalidName(name.to_string()));
    }
    if name.split('/').any(|segment| {
        segment.is_empty()
            || segment == "."
            || segment == ".."
            || segment.chars().any(|c| c.is_control() || c.is_whitespace())
    }) {
        return Err(DebugError::InvalidName(name.to_string()));
    }
    Ok(())
}

fn finite3(v: &[f32; 3], what: &'static str) -> DebugResult<()> {
    if v.iter().all(|x| x.is_finite()) {
        Ok(())
    } else {
        Err(DebugError::InvalidNumber(what))
    }
}

fn finite2(v: &[f32; 2], what: &'static str) -> DebugResult<()> {
    if v.iter().all(|x| x.is_finite()) {
        Ok(())
    } else {
        Err(DebugError::InvalidNumber(what))
    }
}

// ---------------------------------------------------------------------------
// Color
// ---------------------------------------------------------------------------

/// Typed RGBA color, components `0..=1` (linear, alpha forward).
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Color {
    /// Red component in linear 0..=1 RGBA.
    pub r: f32,
    /// Green component in linear 0..=1 RGBA.
    pub g: f32,
    /// Blue component in linear 0..=1 RGBA.
    pub b: f32,
    /// Alpha component in 0..=1; 0 is clear and 1 is opaque.
    pub a: f32,
}

impl Color {
    /// Opaque white, with all RGB components 1.0.
    pub const WHITE: Self = Self {
        r: 1.0,
        g: 1.0,
        b: 1.0,
        a: 1.0,
    };
    /// Opaque black.
    pub const BLACK: Self = Self {
        r: 0.0,
        g: 0.0,
        b: 0.0,
        a: 1.0,
    };
    /// Black with alpha 0.0.
    pub const TRANSPARENT: Self = Self {
        r: 0.0,
        g: 0.0,
        b: 0.0,
        a: 0.0,
    };
    /// Opaque red.
    pub const RED: Self = Self {
        r: 1.0,
        g: 0.0,
        b: 0.0,
        a: 1.0,
    };
    /// Opaque green.
    pub const GREEN: Self = Self {
        r: 0.0,
        g: 1.0,
        b: 0.0,
        a: 1.0,
    };
    /// Opaque blue with components (0, 0.5, 1).
    pub const BLUE: Self = Self {
        r: 0.0,
        g: 0.5,
        b: 1.0,
        a: 1.0,
    };
    /// Opaque yellow.
    pub const YELLOW: Self = Self {
        r: 1.0,
        g: 1.0,
        b: 0.0,
        a: 1.0,
    };
    /// Opaque cyan.
    pub const CYAN: Self = Self {
        r: 0.0,
        g: 1.0,
        b: 1.0,
        a: 1.0,
    };
    /// Opaque magenta.
    pub const MAGENTA: Self = Self {
        r: 1.0,
        g: 0.0,
        b: 1.0,
        a: 1.0,
    };
    /// Opaque orange with components (1, 0.5, 0).
    pub const ORANGE: Self = Self {
        r: 1.0,
        g: 0.5,
        b: 0.0,
        a: 1.0,
    };

    /// Create an RGBA color without validation.
    ///
    /// Supply finite components in `0..=1`. Submission checks them with
    /// [`Self::validate`]. An alpha of `0.0` is transparent; `1.0` is opaque.
    #[must_use]
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Create an RGB color with alpha `1.0`, without validation.
    #[must_use]
    pub const fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b, a: 1.0 }
    }

    /// Create an RGBA color without validation. Equivalent to [`Self::new`].
    #[must_use]
    pub const fn rgba(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Return this color with the supplied alpha, without validation.
    #[must_use]
    pub const fn with_alpha(self, a: f32) -> Self {
        Self { a, ..self }
    }

    /// Check finite `0..=1` on all components.
    pub fn validate(&self) -> DebugResult<()> {
        if [self.r, self.g, self.b, self.a]
            .iter()
            .all(|x| x.is_finite() && (0.0..=1.0).contains(x))
        {
            Ok(())
        } else {
            Err(DebugError::InvalidColor)
        }
    }

    /// Return components in red, green, blue, alpha order.
    #[must_use]
    pub const fn to_array(self) -> [f32; 4] {
        [self.r, self.g, self.b, self.a]
    }

    /// Create a color from red, green, blue, alpha order, without validation.
    #[must_use]
    pub const fn from_array(v: [f32; 4]) -> Self {
        Self {
            r: v[0],
            g: v[1],
            b: v[2],
            a: v[3],
        }
    }
}

impl Default for Color {
    fn default() -> Self {
        Self::WHITE
    }
}

/// Common colors and allocation-free palette helpers.
pub mod palette {
    use super::Color;

    /// Opaque white, with all RGB components 1.0.
    pub const WHITE: Color = Color::WHITE;
    /// Opaque black.
    pub const BLACK: Color = Color::BLACK;
    /// Opaque red.
    pub const RED: Color = Color::RED;
    /// Opaque green.
    pub const GREEN: Color = Color::GREEN;
    /// Opaque blue with components (0, 0.5, 1).
    pub const BLUE: Color = Color::BLUE;
    /// Opaque yellow.
    pub const YELLOW: Color = Color::YELLOW;
    /// Opaque cyan.
    pub const CYAN: Color = Color::CYAN;
    /// Opaque magenta.
    pub const MAGENTA: Color = Color::MAGENTA;
    /// Opaque orange with components (1, 0.5, 0).
    pub const ORANGE: Color = Color::ORANGE;
    /// Opaque purple with components (0.6, 0.2, 0.9).
    pub const PURPLE: Color = Color::rgb(0.6, 0.2, 0.9);
    /// Opaque lime with components (0.6, 1, 0.2).
    pub const LIME: Color = Color::rgb(0.6, 1.0, 0.2);
    /// Team blue color with components (0.15, 0.45, 1).
    pub const TEAM_BLUE: Color = Color::rgb(0.15, 0.45, 1.0);
    /// Team orange color with components (1, 0.55, 0.1).
    pub const TEAM_ORANGE: Color = Color::rgb(1.0, 0.55, 0.1);
    /// Opaque white ball color.
    pub const BALL_WHITE: Color = Color::WHITE;
    /// White at 25 percent alpha.
    pub const GHOST: Color = Color::rgba(1.0, 1.0, 1.0, 0.25);

    /// Return [`TEAM_BLUE`] if `blue` is `true`, or [`TEAM_ORANGE`] otherwise.
    #[must_use]
    pub const fn team(blue: bool) -> Color {
        if blue { TEAM_BLUE } else { TEAM_ORANGE }
    }

    /// Return `c` with the supplied alpha, without validation.
    #[must_use]
    pub const fn with_alpha(c: Color, a: f32) -> Color {
        c.with_alpha(a)
    }

    /// Return an opaque rainbow color for a finite hue in turns.
    ///
    /// The hue repeats at each integer. Saturation and value are both `1.0`.
    #[must_use]
    pub fn rainbow(t: f32) -> Color {
        let h = (t.fract() + 1.0).fract() * 6.0;
        let c = 1.0;
        let x = c * (1.0 - ((h % 2.0) - 1.0).abs());
        let (r, g, b) = match h as u32 {
            0 => (c, x, 0.0),
            1 => (x, c, 0.0),
            2 => (0.0, c, x),
            3 => (0.0, x, c),
            4 => (x, 0.0, c),
            _ => (c, 0.0, x),
        };
        Color::rgb(r, g, b)
    }
}

// ---------------------------------------------------------------------------
// Anchor / transform / width / depth / lifetime
// ---------------------------------------------------------------------------

/// Reference frame for a group.
///
/// Offsets are in the anchor frame: `p_world = apos + arot * (p_group + offset)`.
/// `World` is identity (`apos = 0`, `arot = I`).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Anchor {
    /// Fixed world coordinates (Unreal units).
    #[default]
    World,
    /// Ball rigid transform plus a local offset.
    Ball {
        /// Offset in ball-local space (Unreal units).
        offset: [f32; 3],
    },
    /// Car transform by stable `CarInfo` identifier plus a local offset.
    /// If the car is gone the renderer falls back to world identity.
    Car {
        /// Stable `CarInfo` identifier.
        car_id: u32,
        /// Offset in car-local space (Unreal units).
        offset: [f32; 3],
    },
}

impl Anchor {
    /// Return the fixed world reference frame.
    #[must_use]
    pub const fn world() -> Self {
        Self::World
    }

    /// Return a ball anchor with zero local offset.
    #[must_use]
    pub const fn ball() -> Self {
        Self::Ball { offset: [0.0; 3] }
    }

    /// Return a car anchor with zero local offset for a stable car identifier.
    #[must_use]
    pub const fn car(car_id: u32) -> Self {
        Self::Car {
            car_id,
            offset: [0.0; 3],
        }
    }

    /// Check that the local offset is finite.
    ///
    /// This does not check whether a car identifier exists in a simulation.
    pub fn validate(&self) -> DebugResult<()> {
        match self {
            Self::World => Ok(()),
            Self::Ball { offset } => finite3(offset, "anchor offset"),
            Self::Car { offset, .. } => finite3(offset, "anchor offset"),
        }
    }
}

/// Group placement after the instance transform and before anchor placement.
///
/// Compute `p2 = position + rotation * (scale * p1)`. Store the matrix as
/// three rows: `rotation[row][column]`. Values must be finite. Validation
/// does not require an orthogonal matrix. The default is identity placement.
///
/// Convert a column-major `glam` matrix to rows before storing it:
///
/// ```
/// use glam::Mat3A;
/// use rocketsim_vis::debug::GroupTransform;
///
/// let rotation = Mat3A::from_rotation_z(std::f32::consts::FRAC_PI_2);
/// let transform = GroupTransform {
///     position: [100.0, 0.0, 0.0],
///     rotation: rotation.transpose().to_cols_array_2d(),
///     scale: [1.0; 3],
/// };
/// transform.validate()?;
/// assert!((transform.rotation[0][1] + 1.0).abs() < 1e-6);
/// # Ok::<(), rocketsim_vis::debug::DebugError>(())
/// ```
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GroupTransform {
    /// Translation in Unreal units.
    pub position: [f32; 3],
    /// Finite 3 by 3 row-major matrix. Each inner array is one row.
    pub rotation: [[f32; 3]; 3],
    /// Positive scale for X, Y, and Z; each value must be greater than 1e-6 and less than 1e6.
    pub scale: [f32; 3],
}

impl Default for GroupTransform {
    fn default() -> Self {
        Self::identity()
    }
}

impl GroupTransform {
    /// Return zero translation, identity rotation, and unit scale.
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            position: [0.0; 3],
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            scale: [1.0; 3],
        }
    }

    /// Return identity rotation and scale with a translation in Unreal units.
    #[must_use]
    pub const fn from_position(position: [f32; 3]) -> Self {
        Self {
            position,
            ..Self::identity()
        }
    }

    /// Return identity rotation with a translation and a uniform scale.
    #[must_use]
    pub const fn from_position_scale(position: [f32; 3], scale: f32) -> Self {
        Self {
            position,
            scale: [scale; 3],
            ..Self::identity()
        }
    }

    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        finite3(&self.position, "group position")?;
        finite3(&self.scale, "group scale")?;
        for row in &self.rotation {
            finite3(row, "group rotation")?;
        }
        if self.scale.iter().all(|s| *s > 1e-6 && *s < 1e6) {
            Ok(())
        } else {
            Err(DebugError::InvalidTransform)
        }
    }
}

/// Explicit line/point thickness.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Width {
    /// Screen-space pixels. Valid values are finite and in `(0, 64]`; the default is 2 pixels.
    Pixels(f32),
    /// Unreal units. Valid values are finite and in `(0, 10000]`.
    World(f32),
}

impl Default for Width {
    fn default() -> Self {
        Self::Pixels(2.0)
    }
}

impl Width {
    /// Create a pixel width without validation. Valid values are in `(0, 64]`.
    #[must_use]
    pub const fn pixels(px: f32) -> Self {
        Self::Pixels(px)
    }

    /// Create a width in Unreal units without validation.
    ///
    /// Valid values are finite and in `(0, 10000]`.
    #[must_use]
    pub const fn world(w: f32) -> Self {
        Self::World(w)
    }

    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        let v = match *self {
            Self::Pixels(v) => {
                if !(v.is_finite() && v > 0.0 && v <= 64.0) {
                    return Err(DebugError::InvalidWidth);
                }
                v
            }
            Self::World(v) => {
                if !(v.is_finite() && v > 0.0 && v <= 10_000.0) {
                    return Err(DebugError::InvalidWidth);
                }
                v
            }
        };
        let _ = v;
        Ok(())
    }
}

/// Depth handling per item.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum DepthMode {
    /// Depth-tested (hidden behind arena geometry).
    #[default]
    Test,
    /// Always drawn on top.
    Always,
    /// Draw hidden parts with reduced opacity and visible parts normally.
    Xray,
}

/// Expiry rule for a retained group.
///
/// Group replacement starts a new deadline. Transform, visibility, anchor,
/// and item updates preserve the deadline. Tick deadlines follow simulation
/// updates, not rendered frames or wall-clock time. Seconds deadlines use
/// absolute Unix time, so clock differences can affect remote viewers.
///
/// ```
/// use rocketsim_vis::debug::{DebugRenderer, GroupOptions, Lifetime};
///
/// let renderer = DebugRenderer::new_detached();
/// renderer.group("temporary").draw_with(
///     GroupOptions::new().lifetime(Lifetime::Frame),
///     |draw| { draw.point([0.0; 3]); },
/// )?;
/// renderer.advance_tick();
/// assert!(renderer.snapshot().groups.contains_key("temporary"));
/// renderer.advance_tick();
/// assert!(!renderer.snapshot().groups.contains_key("temporary"));
/// # Ok::<(), rocketsim_vis::debug::DebugError>(())
/// ```
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Lifetime {
    /// Kept until explicitly replaced or cleared.
    #[default]
    Persistent,
    /// Show with the next simulation snapshot and expire at the following update.
    ///
    /// The viewer keeps it if the simulation pauses at that snapshot.
    /// This is not a wall-clock timeout.
    Frame,
    /// Remain through the next `n` simulation updates, then expire.
    ///
    /// Accept `1..=1_000_000`. At update `t`, the deadline is `t + n + 1`.
    Ticks(u32),
    /// Expires after finite wall-clock seconds in 0.001..=3600.
    Seconds(f32),
}

impl Lifetime {
    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        match *self {
            Self::Persistent | Self::Frame => Ok(()),
            Self::Ticks(n) => {
                if (1..=1_000_000).contains(&n) {
                    Ok(())
                } else {
                    Err(DebugError::InvalidLifetime)
                }
            }
            Self::Seconds(s) => {
                if s.is_finite() && (0.001..=3600.0).contains(&s) {
                    Ok(())
                } else {
                    Err(DebugError::InvalidLifetime)
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Items
// ---------------------------------------------------------------------------

/// Single straight segment.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LineItem {
    /// Start point in group-local coordinates, in Unreal units.
    pub start: [f32; 3],
    /// End point in group-local coordinates, in Unreal units.
    pub end: [f32; 3],
    /// Straight-alpha linear RGBA color with finite components in 0..=1.
    pub color: Color,
    /// Line width in pixels or Unreal units; see [`Width`].
    pub width: Width,
    /// Depth test mode for this item.
    pub depth: DepthMode,
}

impl LineItem {
    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        finite3(&self.start, "line start")?;
        finite3(&self.end, "line end")?;
        self.color.validate()?;
        self.width.validate()?;
        Ok(())
    }
}

/// Connected strip (`closed` joins last to first).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PolylineItem {
    /// Group-local points in Unreal units. Accept 2 through [`MAX_POLYLINE_POINTS`].
    pub points: Vec<[f32; 3]>,
    /// Straight-alpha linear RGBA color with finite components in 0..=1.
    pub color: Color,
    /// Line width in pixels or Unreal units; see [`Width`].
    pub width: Width,
    /// Depth test mode for this item.
    pub depth: DepthMode,
    /// If true, connect the final point to the first point.
    pub closed: bool,
}

impl PolylineItem {
    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        if self.points.len() > MAX_POLYLINE_POINTS {
            return Err(DebugError::TooManyPoints {
                count: self.points.len(),
                max: MAX_POLYLINE_POINTS,
            });
        }
        if self.points.len() < 2 {
            return Err(DebugError::InvalidNumber("polyline needs >= 2 points"));
        }
        for p in &self.points {
            finite3(p, "polyline point")?;
        }
        self.color.validate()?;
        self.width.validate()?;
        Ok(())
    }
}

/// Single world point (square sprite, sized by [`Width`]).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PointItem {
    /// Point position in group-local coordinates, in Unreal units.
    pub pos: [f32; 3],
    /// Straight-alpha linear RGBA color with finite components in 0..=1.
    pub color: Color,
    /// Point size as a [`Width`].
    pub size: Width,
    /// Depth test mode for this item.
    pub depth: DepthMode,
}

impl PointItem {
    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        finite3(&self.pos, "point pos")?;
        self.color.validate()?;
        self.size.validate()?;
        Ok(())
    }
}

/// Inline colored triangle mesh (normals/UV optional, texture optional).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct InlineMeshItem {
    /// Vertex positions in local Unreal units.
    pub positions: Vec<[f32; 3]>,
    /// One finite 0..=1 RGBA color per vertex.
    pub colors: Vec<Color>,
    /// Optional finite normal vectors, one per vertex.
    pub normals: Option<Vec<[f32; 3]>>,
    /// Optional finite texture coordinates, one `[u, v]` pair per vertex.
    pub uvs: Option<Vec<[f32; 2]>>,
    /// Triangle-list indices; count must be divisible by 3 and each index must be less than vertex count.
    pub indices: Vec<u32>,
    /// Reusable texture id, or `None` for vertex colors only.
    pub texture: Option<String>,
    /// Depth test mode for this item.
    pub depth: DepthMode,
}

impl InlineMeshItem {
    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        validate_mesh_buffers(
            self.positions.len(),
            self.indices.len(),
            self.normals.as_ref().map(Vec::len),
            self.uvs.as_ref().map(Vec::len),
            Some(self.colors.len()),
        )?;
        for p in &self.positions {
            finite3(p, "mesh position")?;
        }
        for c in &self.colors {
            c.validate()?;
        }
        if let Some(normals) = &self.normals {
            for n in normals {
                finite3(n, "mesh normal")?;
            }
        }
        if let Some(uvs) = &self.uvs {
            for uv in uvs {
                finite2(uv, "mesh uv")?;
            }
        }
        for &i in &self.indices {
            if (i as usize) >= self.positions.len() {
                return Err(DebugError::InvalidIndex {
                    index: i,
                    vertices: self.positions.len(),
                });
            }
        }
        if let Some(id) = &self.texture {
            validate_name(id)?;
        }
        Ok(())
    }
}

fn validate_mesh_buffers(
    verts: usize,
    indices: usize,
    normals: Option<usize>,
    uvs: Option<usize>,
    colors: Option<usize>,
) -> DebugResult<()> {
    if verts > MAX_MESH_VERTICES {
        return Err(DebugError::TooManyVertices {
            count: verts,
            max: MAX_MESH_VERTICES,
        });
    }
    if indices > MAX_MESH_INDICES {
        return Err(DebugError::TooManyIndices {
            count: indices,
            max: MAX_MESH_INDICES,
        });
    }
    if !indices.is_multiple_of(3) {
        return Err(DebugError::InvalidIndexCount { count: indices });
    }
    if verts == 0 || indices == 0 {
        return Err(DebugError::InvalidNumber("mesh needs vertices and indices"));
    }
    if let Some(n) = normals
        && n != verts
    {
        return Err(DebugError::InvalidNumber(
            "mesh normals length must match vertices",
        ));
    }
    if let Some(n) = uvs
        && n != verts
    {
        return Err(DebugError::InvalidNumber(
            "mesh uvs length must match vertices",
        ));
    }
    if let Some(n) = colors
        && n != verts
    {
        return Err(DebugError::InvalidNumber(
            "mesh colors length must match vertices",
        ));
    }
    Ok(())
}

/// Transform-only instance of a reusable mesh id.
///
/// Vertex data travels once as a [`MeshResource`](crate::debug::MeshResource); later updates send only
/// this small record (group delta, no mesh bytes).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MeshInstanceItem {
    /// Reusable mesh id (see `insert_mesh`).
    pub mesh: String,
    /// Translation in Unreal units.
    pub position: [f32; 3],
    /// Finite 3 by 3 row-major matrix. Each inner array is one row.
    pub rotation: [[f32; 3]; 3],
    /// Finite scale for X, Y, and Z. Zero and negative values are accepted.
    pub scale: [f32; 3],
    /// Vertex color multiplier for this instance.
    pub tint: Color,
    /// Reusable texture identifier, or `None` for vertex colors only.
    pub texture: Option<String>,
    /// Depth test mode for this item.
    pub depth: DepthMode,
}

impl MeshInstanceItem {
    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        validate_name(&self.mesh)?;
        finite3(&self.position, "instance position")?;
        finite3(&self.scale, "instance scale")?;
        for row in &self.rotation {
            finite3(row, "instance rotation")?;
        }
        self.tint.validate()?;
        if let Some(id) = &self.texture {
            validate_name(id)?;
        }
        Ok(())
    }
}

/// Placement for a screen label or a projected three-dimensional label.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum LabelPos {
    /// Screen coordinates as viewport fractions. The origin is top-left.
    ///
    /// Values must be finite. Values outside `0..=1` can place text off-screen.
    Screen([f32; 2]),
    /// Group-local position in Unreal units. The viewer projects it to the screen.
    World([f32; 3]),
}

impl LabelPos {
    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        match self {
            Self::Screen(p) => {
                if !p.iter().all(|x| x.is_finite()) {
                    return Err(DebugError::InvalidNumber("label screen pos"));
                }
                Ok(())
            }
            Self::World(p) => finite3(p, "label world pos"),
        }
    }
}

/// Horizontal text alignment.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum TextAlign {
    /// Align text to the left.
    Left,
    #[default]
    /// Center text.
    Center,
    /// Align text to the right.
    Right,
}

/// A text label with horizontal alignment and an optional background.
///
/// The viewer draws labels as an overlay. A projected world label is not
/// hidden by arena or mesh depth. Text is vertically centered at its position.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LabelItem {
    /// Non-empty UTF-8 label text of at most [`MAX_TEXT_LEN`] bytes.
    pub text: String,
    /// Screen coordinates or a projected group-local position.
    pub pos: LabelPos,
    /// Font size in viewer logical points. Accept finite values in `4..=128`.
    pub size: f32,
    /// Horizontal text alignment.
    pub align: TextAlign,
    /// Straight-alpha linear RGBA color with finite components in 0..=1.
    pub color: Color,
    /// Optional background fill (`None` = transparent).
    pub bg: Option<Color>,
}

impl LabelItem {
    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        if self.text.len() > MAX_TEXT_LEN {
            return Err(DebugError::TextTooLong {
                len: self.text.len(),
                max: MAX_TEXT_LEN,
            });
        }
        if self.text.is_empty() {
            return Err(DebugError::InvalidNumber("label text is empty"));
        }
        if !(self.size.is_finite() && (4.0..=128.0).contains(&self.size)) {
            return Err(DebugError::InvalidNumber("label size"));
        }
        self.pos.validate()?;
        self.color.validate()?;
        if let Some(bg) = &self.bg {
            bg.validate()?;
        }
        Ok(())
    }
}

/// Screen rectangle border.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RectBorder {
    /// Straight-alpha linear RGBA color with finite components in 0..=1.
    pub color: Color,
    /// Border width in viewer logical points with [`Width::Pixels`].
    ///
    /// The current viewer uses a fixed one-point border for [`Width::World`].
    pub width: Width,
}

/// A rectangle placed with viewport fractions, with the origin at top-left.
///
/// `[0.0, 0.0]` and `[1.0, 1.0]` span the viewport. Validation accepts other
/// finite coordinates. The viewer clips the rectangle to the viewport and
/// draws it as an overlay. Group and anchor transforms do not move it.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RectItem {
    /// Top-left normalized screen coordinate; components must be finite.
    pub min: [f32; 2],
    /// Bottom-right normalized screen coordinate; components must be finite.
    pub max: [f32; 2],
    /// Rectangle fill color with finite components in 0..=1.
    pub fill: Color,
    /// Optional outline color and width.
    pub border: Option<RectBorder>,
    /// Optional reusable texture id stretched over the rect.
    pub texture: Option<String>,
}

impl RectItem {
    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        finite2(&self.min, "rect min")?;
        finite2(&self.max, "rect max")?;
        self.fill.validate()?;
        if let Some(b) = &self.border {
            b.color.validate()?;
            b.width.validate()?;
        }
        if let Some(id) = &self.texture {
            validate_name(id)?;
        }
        Ok(())
    }
}

/// One drawable primitive inside a [`DrawList`].
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum DebugItem {
    /// A single line segment.
    Line(LineItem),
    /// A connected series of points.
    Polyline(PolylineItem),
    /// A square point sprite.
    Point(PointItem),
    /// An inline indexed triangle mesh.
    Mesh(InlineMeshItem),
    /// An instance of a reusable mesh.
    Instance(MeshInstanceItem),
    /// A text label.
    Label(LabelItem),
    /// A normalized screen rectangle.
    Rect(RectItem),
}

impl DebugItem {
    /// Validate all stored values and return the first validation error.
    pub fn validate(&self) -> DebugResult<()> {
        match self {
            Self::Line(v) => v.validate(),
            Self::Polyline(v) => v.validate(),
            Self::Point(v) => v.validate(),
            Self::Mesh(v) => v.validate(),
            Self::Instance(v) => v.validate(),
            Self::Label(v) => v.validate(),
            Self::Rect(v) => v.validate(),
        }
    }

    /// Rough byte estimate for budget checks.
    pub fn estimated_bytes(&self) -> usize {
        match self {
            Self::Line(_) => 64,
            Self::Point(_) => 48,
            Self::Rect(_) => 64,
            Self::Label(v) => v.text.len().saturating_add(64),
            Self::Polyline(v) => v.points.len().saturating_mul(12).saturating_add(48),
            Self::Mesh(v) => v
                .positions
                .len()
                .saturating_mul(28)
                .saturating_add(v.indices.len().saturating_mul(4))
                .saturating_add(64),
            Self::Instance(_) => 128,
        }
    }
}

// ---------------------------------------------------------------------------
// DrawList + builder helpers
// ---------------------------------------------------------------------------

/// An ordered list of drawing items for one group.
///
/// Builder methods append items and return the list for chaining. They do not
/// validate input. Call [`validate`](Self::validate) before use; submission
/// also validates the list.
///
/// # Example
///
/// ```
/// use rocketsim_vis::debug::{Color, DebugItem, DepthMode, DrawList, Width};
///
/// let mut draw = DrawList::new();
/// draw.line([0.0, 0.0, 0.0], [100.0, 0.0, 0.0],
///     Color::RED, Width::pixels(2.0), DepthMode::Test);
/// assert_eq!(draw.len(), 1);
/// assert!(matches!(draw.items[0], DebugItem::Line(_)));
/// ```
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DrawList {
    /// Items in draw order before renderer batching.
    pub items: Vec<DebugItem>,
}

impl DrawList {
    /// Create an empty draw list.
    #[must_use]
    pub fn new() -> Self {
        Self { items: Vec::new() }
    }

    /// Create an empty list with capacity for at most [`MAX_ITEMS_PER_GROUP`] items.
    ///
    /// This limits the initial allocation, not the number of later pushes.
    /// Submission rejects lists that exceed the item limit.
    #[must_use]
    pub fn with_capacity(n: usize) -> Self {
        Self {
            items: Vec::with_capacity(n.min(MAX_ITEMS_PER_GROUP)),
        }
    }

    /// Return the number of items.
    #[must_use]
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// Return `true` if the list has no items.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Clear all items.
    pub fn clear(&mut self) {
        self.items.clear();
    }

    /// Push a prebuilt item.
    pub fn push(&mut self, item: DebugItem) -> &mut Self {
        self.items.push(item);
        self
    }

    /// Append a line between two group-local points in Unreal units.
    pub fn line(
        &mut self,
        start: [f32; 3],
        end: [f32; 3],
        color: Color,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        self.items.push(DebugItem::Line(LineItem {
            start,
            end,
            color,
            width,
            depth,
        }));
        self
    }

    /// Append a connected line strip. Set `closed` to join the last point to the first.
    pub fn polyline(
        &mut self,
        points: Vec<[f32; 3]>,
        color: Color,
        width: Width,
        depth: DepthMode,
        closed: bool,
    ) -> &mut Self {
        self.items.push(DebugItem::Polyline(PolylineItem {
            points,
            color,
            width,
            depth,
            closed,
        }));
        self
    }

    /// Append a square point sprite at a group-local position in Unreal units.
    pub fn point(
        &mut self,
        pos: [f32; 3],
        color: Color,
        size: Width,
        depth: DepthMode,
    ) -> &mut Self {
        self.items.push(DebugItem::Point(PointItem {
            pos,
            color,
            size,
            depth,
        }));
        self
    }

    /// Append an inline indexed triangle mesh in group-local coordinates.
    ///
    /// Supply one color per vertex. Optional normals and UV coordinates must
    /// match the vertex count. Indices form triangles and must be in range.
    /// `texture` names an existing reusable texture. See [`InlineMeshItem`]
    /// for validation rules. Prefer a [`MeshResource`](crate::debug::MeshResource)
    /// and [`Self::instance`] when several items share the same geometry.
    #[allow(clippy::too_many_arguments)]
    pub fn mesh(
        &mut self,
        positions: Vec<[f32; 3]>,
        colors: Vec<Color>,
        normals: Option<Vec<[f32; 3]>>,
        uvs: Option<Vec<[f32; 2]>>,
        indices: Vec<u32>,
        texture: Option<String>,
        depth: DepthMode,
    ) -> &mut Self {
        self.items.push(DebugItem::Mesh(InlineMeshItem {
            positions,
            colors,
            normals,
            uvs,
            indices,
            texture,
            depth,
        }));
        self
    }

    /// Append an instance of an existing reusable mesh.
    ///
    /// Apply scale, then row-major rotation, then translation in Unreal
    /// units. `tint` multiplies vertex colors. `texture` optionally names a
    /// reusable texture; the mesh resource does not store a texture itself.
    /// See the [module example](crate::debug) for efficient shared geometry.
    #[allow(clippy::too_many_arguments)]
    pub fn instance(
        &mut self,
        mesh: impl Into<String>,
        position: [f32; 3],
        rotation: [[f32; 3]; 3],
        scale: [f32; 3],
        tint: Color,
        texture: Option<String>,
        depth: DepthMode,
    ) -> &mut Self {
        self.items.push(DebugItem::Instance(MeshInstanceItem {
            mesh: mesh.into(),
            position,
            rotation,
            scale,
            tint,
            texture,
            depth,
        }));
        self
    }

    /// Append a screen label at viewport fractions, with a top-left origin.
    ///
    /// `size` uses viewer logical points. See [`LabelItem`] for size limits
    /// and [`LabelPos::Screen`] for coordinates outside the viewport.
    pub fn label_screen(
        &mut self,
        text: impl Into<String>,
        pos: [f32; 2],
        size: f32,
        align: TextAlign,
        color: Color,
        bg: Option<Color>,
    ) -> &mut Self {
        self.items.push(DebugItem::Label(LabelItem {
            text: text.into(),
            pos: LabelPos::Screen(pos),
            size,
            align,
            color,
            bg,
        }));
        self
    }

    /// Append a projected label at a group-local position in Unreal units.
    ///
    /// `size` uses viewer logical points. The label remains an overlay and
    /// is not hidden by world geometry. See [`LabelItem`].
    pub fn label_world(
        &mut self,
        text: impl Into<String>,
        pos: [f32; 3],
        size: f32,
        align: TextAlign,
        color: Color,
        bg: Option<Color>,
    ) -> &mut Self {
        self.items.push(DebugItem::Label(LabelItem {
            text: text.into(),
            pos: LabelPos::World(pos),
            size,
            align,
            color,
            bg,
        }));
        self
    }

    /// Append a screen rectangle using viewport fractions.
    ///
    /// The viewer clips the rectangle to the viewport. `texture` must name
    /// an existing resource. See [`RectItem`] and [`RectBorder`] for details.
    pub fn rect(
        &mut self,
        min: [f32; 2],
        max: [f32; 2],
        fill: Color,
        border: Option<RectBorder>,
        texture: Option<String>,
    ) -> &mut Self {
        self.items.push(DebugItem::Rect(RectItem {
            min,
            max,
            fill,
            border,
            texture,
        }));
        self
    }

    // -- shape helpers (at least box / sphere / arrows / grid) -------------

    /// Append 12 line items for an axis-aligned box.
    ///
    /// `center` and per-axis half-extents `half` use group-local Unreal units.
    pub fn box_edges(
        &mut self,
        center: [f32; 3],
        half: [f32; 3],
        color: Color,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        let c = center;
        let h = half;
        let v = [
            [c[0] - h[0], c[1] - h[1], c[2] - h[2]],
            [c[0] + h[0], c[1] - h[1], c[2] - h[2]],
            [c[0] + h[0], c[1] + h[1], c[2] - h[2]],
            [c[0] - h[0], c[1] + h[1], c[2] - h[2]],
            [c[0] - h[0], c[1] - h[1], c[2] + h[2]],
            [c[0] + h[0], c[1] - h[1], c[2] + h[2]],
            [c[0] + h[0], c[1] + h[1], c[2] + h[2]],
            [c[0] - h[0], c[1] + h[1], c[2] + h[2]],
        ];
        for &(a, b) in &[
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ] {
            self.line(v[a], v[b], color, width, depth);
        }
        self
    }

    /// Append one inline mesh for an axis-aligned box.
    ///
    /// `center` and per-axis half-extents `half` use group-local Unreal units.
    /// The mesh has 8 vertices, 12 triangles, and one color for all vertices.
    pub fn box_solid(
        &mut self,
        center: [f32; 3],
        half: [f32; 3],
        color: Color,
        depth: DepthMode,
    ) -> &mut Self {
        let c = center;
        let h = half;
        let positions = vec![
            [c[0] - h[0], c[1] - h[1], c[2] - h[2]],
            [c[0] + h[0], c[1] - h[1], c[2] - h[2]],
            [c[0] + h[0], c[1] + h[1], c[2] - h[2]],
            [c[0] - h[0], c[1] + h[1], c[2] - h[2]],
            [c[0] - h[0], c[1] - h[1], c[2] + h[2]],
            [c[0] + h[0], c[1] - h[1], c[2] + h[2]],
            [c[0] + h[0], c[1] + h[1], c[2] + h[2]],
            [c[0] - h[0], c[1] + h[1], c[2] + h[2]],
        ];
        let indices = vec![
            0, 1, 2, 0, 2, 3, // -z
            4, 6, 5, 4, 7, 6, // +z
            0, 4, 5, 0, 5, 1, // -y
            2, 6, 7, 2, 7, 3, // +y
            0, 3, 7, 0, 7, 4, // -x
            1, 5, 6, 1, 6, 2, // +x
        ];
        self.mesh(positions, vec![color; 8], None, None, indices, None, depth)
    }

    /// Append a closed circle polyline in the plane of `axis_u` and `axis_v`.
    ///
    /// Use perpendicular unit axes for a circular result. The helper does
    /// not normalize them. `center` and `radius` use group-local Unreal units.
    /// Restrict the segment count to `8..=128` before generating points.
    #[allow(clippy::too_many_arguments)]
    pub fn circle(
        &mut self,
        center: [f32; 3],
        radius: f32,
        axis_u: [f32; 3],
        axis_v: [f32; 3],
        segments: usize,
        color: Color,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        let n = segments.clamp(8, 128);
        let center = Vec3A::from_array(center);
        let axis_u = Vec3A::from_array(axis_u);
        let axis_v = Vec3A::from_array(axis_v);
        let mut points = Vec::with_capacity(n);
        for i in 0..n {
            let t = (i as f32) / (n as f32) * std::f32::consts::TAU;
            let (s, c) = t.sin_cos();
            points.push((center + (axis_u * c + axis_v * s) * radius).to_array());
        }
        self.polyline(points, color, width, depth, true)
    }

    /// Wireframe sphere (3 great circles).
    pub fn sphere_lines(
        &mut self,
        center: [f32; 3],
        radius: f32,
        color: Color,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        self.circle(
            center,
            radius,
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            32,
            color,
            width,
            depth,
        );
        self.circle(
            center,
            radius,
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            32,
            color,
            width,
            depth,
        );
        self.circle(
            center,
            radius,
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            32,
            color,
            width,
            depth,
        );
        self
    }

    /// Append one solid sphere mesh at a group-local center, in Unreal units.
    ///
    /// Use 12 rings and 24 segments: 325 vertices and 576 triangles. Include
    /// one color for all vertices, with no explicit normals or UV coordinates.
    pub fn sphere_solid(
        &mut self,
        center: [f32; 3],
        radius: f32,
        color: Color,
        depth: DepthMode,
    ) -> &mut Self {
        let rings = 12usize;
        let segments = 24usize;
        let mut positions = Vec::with_capacity((rings + 1) * (segments + 1));
        for r in 0..=rings {
            let theta = (r as f32) / (rings as f32) * std::f32::consts::PI;
            let (st, ct) = theta.sin_cos();
            for s in 0..=segments {
                let phi = (s as f32) / (segments as f32) * std::f32::consts::TAU;
                let (sp, cp) = phi.sin_cos();
                positions.push([
                    center[0] + radius * st * cp,
                    center[1] + radius * st * sp,
                    center[2] + radius * ct,
                ]);
            }
        }
        let mut indices = Vec::with_capacity(rings * segments * 6);
        let row = segments + 1;
        for r in 0..rings {
            for s in 0..segments {
                let a = (r * row + s) as u32;
                let b = (r * row + s + 1) as u32;
                let c = ((r + 1) * row + s) as u32;
                let d = ((r + 1) * row + s + 1) as u32;
                indices.extend_from_slice(&[a, c, b, b, c, d]);
            }
        }
        let colors = vec![color; positions.len()];
        self.mesh(positions, colors, None, None, indices, None, depth)
    }

    /// Append an arrow shaft and wireframe head in group-local Unreal units.
    ///
    /// Add one shaft and eight head edges if the direction is finite and at
    /// least `0.001` units long. Otherwise, append only the shaft. Submission
    /// still rejects non-finite endpoints. Head length is 20 percent of shaft
    /// length, restricted to `5..=40` units.
    pub fn arrow(
        &mut self,
        start: [f32; 3],
        end: [f32; 3],
        color: Color,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        self.line(start, end, color, width, depth);
        let start_vec = Vec3A::from_array(start);
        let end_vec = Vec3A::from_array(end);
        let dir = end_vec - start_vec;
        let len = dir.length();
        if !len.is_finite() || len < 1e-3 {
            return self;
        }
        let d = dir / len;
        let head_len = (len * 0.2).clamp(5.0, 40.0);
        let head_r = head_len * 0.4;
        // Pick a stable perpendicular.
        let up = if d.z.abs() < 0.9 { Vec3A::Z } else { Vec3A::Y };
        let u = d.cross(up);
        let u = u / u.length().max(1e-6);
        let v = d.cross(u);
        let base = end_vec - d * head_len;
        let corners = [
            (base + (u + v) * head_r).to_array(),
            (base + (u - v) * head_r).to_array(),
            (base + (-u - v) * head_r).to_array(),
            (base + (-u + v) * head_r).to_array(),
        ];
        for c in &corners {
            self.line(*c, end, color, width, depth);
        }
        for i in 0..4 {
            self.line(corners[i], corners[(i + 1) % 4], color, width, depth);
        }
        self
    }

    /// Append an XY grid at `origin[2]`. Lines span `origin ± extent` on X and Y. The step count is capped at 128; the helper appends two lines per step slot, including the zero slot.
    pub fn grid(
        &mut self,
        origin: [f32; 3],
        extent: f32,
        step: f32,
        color: Color,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        if !(extent.is_finite() && step.is_finite() && extent > 0.0 && step > 0.0) {
            return self;
        }
        let n = ((extent / step).floor() as usize).min(128);
        for i in 0..=n {
            let o = -extent + (i as f32) * step;
            self.line(
                [origin[0] + o, origin[1] - extent, origin[2]],
                [origin[0] + o, origin[1] + extent, origin[2]],
                color,
                width,
                depth,
            );
            self.line(
                [origin[0] - extent, origin[1] + o, origin[2]],
                [origin[0] + extent, origin[1] + o, origin[2]],
                color,
                width,
                depth,
            );
        }
        self
    }

    /// RGB axes triad (X red, Y green, Z blue).
    pub fn axes(
        &mut self,
        origin: [f32; 3],
        len: f32,
        width: Width,
        depth: DepthMode,
    ) -> &mut Self {
        self.line(
            origin,
            [origin[0] + len, origin[1], origin[2]],
            Color::RED,
            width,
            depth,
        );
        self.line(
            origin,
            [origin[0], origin[1] + len, origin[2]],
            Color::GREEN,
            width,
            depth,
        );
        self.line(
            origin,
            [origin[0], origin[1], origin[2] + len],
            Color::BLUE,
            width,
            depth,
        );
        self
    }

    /// Validate all items plus count caps for `group`.
    pub fn validate(&self, group: &str) -> DebugResult<()> {
        if self.items.len() > MAX_ITEMS_PER_GROUP {
            return Err(DebugError::TooManyItems {
                group: group.to_string(),
                count: self.items.len(),
                max: MAX_ITEMS_PER_GROUP,
            });
        }
        for item in &self.items {
            item.validate()?;
        }
        Ok(())
    }

    /// Rough byte estimate.
    pub fn estimated_bytes(&self) -> usize {
        self.items.iter().map(DebugItem::estimated_bytes).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arrow_has_orthogonal_head_and_skips_degenerate_head() {
        let mut list = DrawList::new();
        list.arrow(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 100.0],
            Color::RED,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        assert_eq!(list.items.len(), 9);
        let DebugItem::Line(first_corner_edge) = &list.items[1] else {
            panic!("expected arrow head line");
        };
        let corner = Vec3A::from_array(first_corner_edge.start);
        let radial = corner - Vec3A::new(0.0, 0.0, 80.0);
        assert!(radial.dot(Vec3A::Z).abs() < 1e-5);
        assert!((radial.length() - 20.0 * 0.4 * 2.0_f32.sqrt()).abs() < 1e-5);

        let mut degenerate = DrawList::new();
        degenerate.arrow(
            [1.0, 2.0, 3.0],
            [1.0, 2.0, 3.0],
            Color::RED,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        assert_eq!(degenerate.items.len(), 1);
    }

    #[test]
    fn grid_lies_on_xy_plane_at_origin_z_with_requested_extents() {
        let origin = [10.0, 20.0, 30.0];
        let extent = 4.0;
        let mut list = DrawList::new();
        list.grid(
            origin,
            extent,
            2.0,
            Color::RED,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        assert_eq!(list.items.len(), 6);
        for item in &list.items {
            let DebugItem::Line(line) = item else {
                panic!("expected grid line");
            };
            assert_eq!(line.start[2], origin[2]);
            assert_eq!(line.end[2], origin[2]);
            assert!(
                line.start
                    .iter()
                    .chain(line.end.iter())
                    .all(|v| v.is_finite())
            );
            for (axis, &center) in origin.iter().take(2).enumerate() {
                assert!(line.start[axis] >= center - extent);
                assert!(line.start[axis] <= center + extent);
                assert!(line.end[axis] >= center - extent);
                assert!(line.end[axis] <= center + extent);
            }
        }
        let DebugItem::Line(first) = &list.items[0] else {
            panic!("expected grid line");
        };
        assert_eq!(
            first.start,
            [origin[0] - extent, origin[1] - extent, origin[2]]
        );
        assert_eq!(
            first.end,
            [origin[0] - extent, origin[1] + extent, origin[2]]
        );
    }

    #[test]
    fn grid_preserves_step_caps_and_ignores_invalid_dimensions() {
        let mut list = DrawList::new();
        list.grid(
            [0.0; 3],
            0.0,
            1.0,
            Color::WHITE,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        list.grid(
            [0.0; 3],
            1.0,
            f32::NAN,
            Color::WHITE,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        assert!(list.is_empty());
        list.grid(
            [0.0; 3],
            1000.0,
            1.0,
            Color::WHITE,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        assert_eq!(list.items.len(), 258);
    }

    #[test]
    fn circle_points_follow_the_supplied_plane() {
        let mut list = DrawList::new();
        list.circle(
            [1.0, 2.0, 3.0],
            2.0,
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            8,
            Color::RED,
            Width::Pixels(1.0),
            DepthMode::Test,
        );
        let DebugItem::Polyline(circle) = &list.items[0] else {
            panic!("expected circle polyline");
        };
        assert_eq!(circle.points.len(), 8);
        assert!(circle.points.iter().all(|point| point[1] == 2.0));
        assert_eq!(circle.points[0], [3.0, 2.0, 3.0]);
    }
}
