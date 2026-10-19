use super::{
    error::{DebugError, DebugResult},
    limits::{
        MAX_MESH_INDICES, MAX_MESH_VERTICES, MAX_POLYLINE_POINTS, MAX_TEXT_LEN, finite2, finite3,
        validate_name,
    },
    style::Color,
    transform::{DepthMode, Width},
};

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

/// One drawable primitive inside a [`crate::debug::DrawList`].
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
            Self::Rect(v) => {
                96usize.saturating_add(v.texture.as_ref().map_or(0, |texture| texture.len()))
            }
            Self::Label(v) => v.text.len().saturating_add(64),
            Self::Polyline(v) => v.points.len().saturating_mul(12).saturating_add(48),
            Self::Mesh(v) => v
                .positions
                .len()
                .saturating_mul(28)
                .saturating_add(
                    v.normals
                        .as_ref()
                        .map_or(0, |normals| normals.len().saturating_mul(12)),
                )
                .saturating_add(v.uvs.as_ref().map_or(0, |uvs| uvs.len().saturating_mul(8)))
                .saturating_add(v.indices.len().saturating_mul(4))
                .saturating_add(v.texture.as_ref().map_or(0, String::len))
                .saturating_add(64),
            Self::Instance(v) => 128usize
                .saturating_add(v.mesh.len())
                .saturating_add(v.texture.as_ref().map_or(0, String::len)),
        }
    }
}
