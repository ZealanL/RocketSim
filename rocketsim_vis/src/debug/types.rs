//! Core debug-rendering primitives: [`Color`], [`Anchor`], [`Width`],
//! [`DepthMode`], [`Lifetime`], [`DebugItem`], and [`DrawList`] builders.
//!
//! All raw types are `pub`, `Clone`, and `serde` for wire transport.
//! They are renderer-friendly: positions are plain `[f32; N]` arrays in
//! Unreal units (world) or normalized `0..=1` (screen), colors are
//! straight-alpha `0..=1` RGBA.
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
/// Soft cap on estimated live scene bytes (groups + resources).
pub const MAX_DEBUG_BYTES: usize = 8 * 1024 * 1024;

// ---------------------------------------------------------------------------
// Validation error
// ---------------------------------------------------------------------------

/// Typed rejection reason for a debug submission.
///
/// Submitters get `Err(DebugError)` synchronously; nothing is silently
/// dropped. Wire decode maps these to [`crate::proto::ProtocolError`].
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum DebugError {
    /// Name/id is empty, too long, or contains rejected characters.
    InvalidName(String),
    /// A resource handle cannot insert a different resource id.
    ResourceIdMismatch { expected: String, actual: String },
    /// Too many groups in the scene.
    TooManyGroups { count: usize, max: usize },
    /// Too many items in one group.
    TooManyItems {
        group: String,
        count: usize,
        max: usize,
    },
    /// Too many items in the whole scene.
    TooManyTotalItems { count: usize, max: usize },
    /// Too many points in a polyline.
    TooManyPoints { count: usize, max: usize },
    /// Too many vertices in a mesh.
    TooManyVertices { count: usize, max: usize },
    /// Too many indices in a mesh.
    TooManyIndices { count: usize, max: usize },
    /// Index count is not a triangle list (`% 3 != 0`).
    InvalidIndexCount { count: usize },
    /// Index points outside the vertex buffer.
    InvalidIndex { index: u32, vertices: usize },
    /// Too many reusable meshes.
    TooManyMeshes { count: usize, max: usize },
    /// Too many reusable textures.
    TooManyTextures { count: usize, max: usize },
    /// Texture dimensions are zero or exceed [`MAX_TEXTURE_DIM`].
    InvalidTextureDim { width: u32, height: u32 },
    /// RGBA length does not equal `width * height * 4`.
    TextureSizeMismatch { expected: usize, got: usize },
    /// Texture byte size exceeds [`MAX_TEXTURE_BYTES`].
    TextureTooLarge { bytes: usize, max: usize },
    /// Label text exceeds [`MAX_TEXT_LEN`].
    TextTooLong { len: usize, max: usize },
    /// A float field is NaN/infinite or out of its documented range.
    InvalidNumber(&'static str),
    /// Color components must be finite `0..=1`.
    InvalidColor,
    /// Width must be finite and positive within its cap.
    InvalidWidth,
    /// Anchor offset not finite, or car id out of range.
    InvalidAnchor,
    /// Group/instance transform not finite (or rotation malformed).
    InvalidTransform,
    /// Lifetime variant out of range (`Ticks(0)` or non-finite seconds).
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
    PayloadTooLarge { bytes: usize, max: usize },
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

/// Validate a group / mesh / texture name: slash-separated UTF-8 segments,
/// each non-empty and not `.` or `..`; reject control characters.
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
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Color {
    pub const WHITE: Self = Self {
        r: 1.0,
        g: 1.0,
        b: 1.0,
        a: 1.0,
    };
    pub const BLACK: Self = Self {
        r: 0.0,
        g: 0.0,
        b: 0.0,
        a: 1.0,
    };
    pub const TRANSPARENT: Self = Self {
        r: 0.0,
        g: 0.0,
        b: 0.0,
        a: 0.0,
    };
    pub const RED: Self = Self {
        r: 1.0,
        g: 0.0,
        b: 0.0,
        a: 1.0,
    };
    pub const GREEN: Self = Self {
        r: 0.0,
        g: 1.0,
        b: 0.0,
        a: 1.0,
    };
    pub const BLUE: Self = Self {
        r: 0.0,
        g: 0.5,
        b: 1.0,
        a: 1.0,
    };
    pub const YELLOW: Self = Self {
        r: 1.0,
        g: 1.0,
        b: 0.0,
        a: 1.0,
    };
    pub const CYAN: Self = Self {
        r: 0.0,
        g: 1.0,
        b: 1.0,
        a: 1.0,
    };
    pub const MAGENTA: Self = Self {
        r: 1.0,
        g: 0.0,
        b: 1.0,
        a: 1.0,
    };
    pub const ORANGE: Self = Self {
        r: 1.0,
        g: 0.5,
        b: 0.0,
        a: 1.0,
    };

    /// Build from `0..=1` components (validated on submit, not here).
    #[must_use]
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Opaque RGB.
    #[must_use]
    pub const fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b, a: 1.0 }
    }

    /// Full RGBA.
    #[must_use]
    pub const fn rgba(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Copy with a new alpha.
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

    #[must_use]
    pub const fn to_array(self) -> [f32; 4] {
        [self.r, self.g, self.b, self.a]
    }

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

/// Named palette plus small helpers (RLBot-v5-parity basics).
pub mod palette {
    use super::Color;

    pub const WHITE: Color = Color::WHITE;
    pub const BLACK: Color = Color::BLACK;
    pub const RED: Color = Color::RED;
    pub const GREEN: Color = Color::GREEN;
    pub const BLUE: Color = Color::BLUE;
    pub const YELLOW: Color = Color::YELLOW;
    pub const CYAN: Color = Color::CYAN;
    pub const MAGENTA: Color = Color::MAGENTA;
    pub const ORANGE: Color = Color::ORANGE;
    pub const PURPLE: Color = Color::rgb(0.6, 0.2, 0.9);
    pub const LIME: Color = Color::rgb(0.6, 1.0, 0.2);
    pub const TEAM_BLUE: Color = Color::rgb(0.15, 0.45, 1.0);
    pub const TEAM_ORANGE: Color = Color::rgb(1.0, 0.55, 0.1);
    pub const BALL_WHITE: Color = Color::WHITE;
    pub const GHOST: Color = Color::rgba(1.0, 1.0, 1.0, 0.25);

    /// Team color by side (`true` = blue).
    #[must_use]
    pub const fn team(blue: bool) -> Color {
        if blue { TEAM_BLUE } else { TEAM_ORANGE }
    }

    /// Copy with a new alpha.
    #[must_use]
    pub const fn with_alpha(c: Color, a: f32) -> Color {
        c.with_alpha(a)
    }

    /// HSV rainbow (`h` in turns `0..1`, full saturation/value, opaque).
    /// Small pure helper for demos (no allocation).
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
    /// World anchor.
    #[must_use]
    pub const fn world() -> Self {
        Self::World
    }

    /// Ball anchor with zero offset.
    #[must_use]
    pub const fn ball() -> Self {
        Self::Ball { offset: [0.0; 3] }
    }

    /// Car anchor with zero offset.
    #[must_use]
    pub const fn car(car_id: u32) -> Self {
        Self::Car {
            car_id,
            offset: [0.0; 3],
        }
    }

    pub fn validate(&self) -> DebugResult<()> {
        match self {
            Self::World => Ok(()),
            Self::Ball { offset } => finite3(offset, "anchor offset"),
            Self::Car { offset, .. } => finite3(offset, "anchor offset"),
        }
    }
}

/// Rigid-plus-scale group placement (applied before the anchor).
///
/// `p2 = position + rotation * (scale * p1)`; `rotation` is row-major.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GroupTransform {
    pub position: [f32; 3],
    pub rotation: [[f32; 3]; 3],
    pub scale: [f32; 3],
}

impl Default for GroupTransform {
    fn default() -> Self {
        Self::identity()
    }
}

impl GroupTransform {
    /// Identity placement.
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            position: [0.0; 3],
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            scale: [1.0; 3],
        }
    }

    /// Translation only.
    #[must_use]
    pub const fn from_position(position: [f32; 3]) -> Self {
        Self {
            position,
            ..Self::identity()
        }
    }

    /// Uniform scale plus translation.
    #[must_use]
    pub const fn from_position_scale(position: [f32; 3], scale: f32) -> Self {
        Self {
            position,
            scale: [scale; 3],
            ..Self::identity()
        }
    }

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
    /// Screen-space pixels (constant on screen, e.g. `2.0`).
    Pixels(f32),
    /// World units (Unreal units; scales with distance).
    World(f32),
}

impl Default for Width {
    fn default() -> Self {
        Self::Pixels(2.0)
    }
}

impl Width {
    /// Pixels helper.
    #[must_use]
    pub const fn pixels(px: f32) -> Self {
        Self::Pixels(px)
    }

    /// World-units helper.
    #[must_use]
    pub const fn world(w: f32) -> Self {
        Self::World(w)
    }

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
    /// X-ray: occluded fragments drawn translucent, visible solid.
    /// Demo staple (e.g. Zero Two solid + wireframe).
    Xray,
}

/// Item/group lifetime.
///
/// Transform-only or visibility updates preserve the original deadline;
/// only a full `replace_group` with a new [`Lifetime`] restarts it
/// (never silently restarted by the transport).
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Lifetime {
    /// Kept until explicitly replaced or cleared.
    #[default]
    Persistent,
    /// Shown for one physics tick / received frame, then auto-removed.
    Frame,
    /// Expires after `N` physics ticks (`N >= 1`).
    Ticks(u32),
    /// Expires after wall-clock seconds (`0.001..=3600`).
    Seconds(f32),
}

impl Lifetime {
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
    pub start: [f32; 3],
    pub end: [f32; 3],
    pub color: Color,
    pub width: Width,
    pub depth: DepthMode,
}

impl LineItem {
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
    pub points: Vec<[f32; 3]>,
    pub color: Color,
    pub width: Width,
    pub depth: DepthMode,
    pub closed: bool,
}

impl PolylineItem {
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
    pub pos: [f32; 3],
    pub color: Color,
    pub size: Width,
    pub depth: DepthMode,
}

impl PointItem {
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
    pub positions: Vec<[f32; 3]>,
    pub colors: Vec<Color>,
    pub normals: Option<Vec<[f32; 3]>>,
    pub uvs: Option<Vec<[f32; 2]>>,
    pub indices: Vec<u32>,
    /// Reusable texture id, or `None` for vertex colors only.
    pub texture: Option<String>,
    pub depth: DepthMode,
}

impl InlineMeshItem {
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
    pub position: [f32; 3],
    pub rotation: [[f32; 3]; 3],
    pub scale: [f32; 3],
    pub tint: Color,
    /// Optional texture override (reusable texture id).
    pub texture: Option<String>,
    pub depth: DepthMode,
}

impl MeshInstanceItem {
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

/// Screen (`0..=1` normalized) or world label placement.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum LabelPos {
    /// Normalized screen `x/y` in `0..=1` (origin top-left).
    Screen([f32; 2]),
    /// World position (Unreal units, before group/anchor transforms).
    World([f32; 3]),
}

impl LabelPos {
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
    Left,
    #[default]
    Center,
    Right,
}

/// Text label with size/alignment/background.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LabelItem {
    pub text: String,
    pub pos: LabelPos,
    /// Pixel height (`4..=128`).
    pub size: f32,
    pub align: TextAlign,
    pub color: Color,
    /// Optional background fill (`None` = transparent).
    pub bg: Option<Color>,
}

impl LabelItem {
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
    pub color: Color,
    pub width: Width,
}

/// Normalized screen rectangle (`0..=1`, origin top-left).
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct RectItem {
    pub min: [f32; 2],
    pub max: [f32; 2],
    pub fill: Color,
    pub border: Option<RectBorder>,
    /// Optional reusable texture id stretched over the rect.
    pub texture: Option<String>,
}

impl RectItem {
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
    Line(LineItem),
    Polyline(PolylineItem),
    Point(PointItem),
    Mesh(InlineMeshItem),
    Instance(MeshInstanceItem),
    Label(LabelItem),
    Rect(RectItem),
}

impl DebugItem {
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

/// Reusable ordered item list: the payload of one group.
///
/// Builders are infallible for ergonomics; `replace_group` validates and
/// returns [`DebugError`] on rejection.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DrawList {
    pub items: Vec<DebugItem>,
}

impl DrawList {
    /// Empty list.
    #[must_use]
    pub fn new() -> Self {
        Self { items: Vec::new() }
    }

    /// Empty list with capacity.
    #[must_use]
    pub fn with_capacity(n: usize) -> Self {
        Self {
            items: Vec::with_capacity(n.min(MAX_ITEMS_PER_GROUP)),
        }
    }

    /// Number of items.
    #[must_use]
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// True when empty.
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

    /// Straight line.
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

    /// Connected strip.
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

    /// World point.
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

    /// Inline mesh.
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

    /// Instance of a reusable mesh id (transform-only updates).
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

    /// Screen-space label (`pos` normalized `0..=1`).
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

    /// World-space label.
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

    /// Screen rectangle.
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

    /// Wireframe box edges (12 lines).
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

    /// Solid box (8 verts, 12 triangles, per-vertex color).
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

    /// Flat circle in the plane spanned by `axis_u`/`axis_v`.
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

    /// Solid UV sphere (lat/long, per-vertex color, no normals/UVs).
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

    /// Arrow shaft plus a 4-sided pyramid head (wireframe).
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

    /// Ground-plane grid on XY at `origin[2]` height.
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
