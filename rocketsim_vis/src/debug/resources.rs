//! Reusable mesh and texture resources for retained debug scenes.

use super::types::{
    Color, DebugError, DebugResult, MAX_MESH_INDICES, MAX_MESH_VERTICES, MAX_TEXTURE_BYTES,
    MAX_TEXTURE_DIM, validate_name,
};

/// A mesh that the viewer provides without a user resource.
///
/// Use [`Self::id`] in a mesh instance. The identifier does not need a
/// [`MeshResource`]. Do not create a resource with a reserved identifier.
///
/// ```
/// use glam::Mat3A;
/// use rocketsim_vis::debug::{BuiltinMesh, BuiltinTexture, Color, DebugRenderer, DepthMode, DrawList, GroupOptions};
///
/// let renderer = DebugRenderer::new_detached();
/// let mut list = DrawList::new();
/// list.instance(
///     BuiltinMesh::Ball.id(), [0.0; 3], Mat3A::IDENTITY.to_cols_array_2d(),
///     [1.0; 3], Color::WHITE, Some(BuiltinTexture::Ball.id().into()),
///     DepthMode::Test,
/// );
/// renderer.replace_group("ball", list, GroupOptions::new())?;
/// assert!(renderer.snapshot().meshes.is_empty());
/// # Ok::<(), rocketsim_vis::debug::DebugError>(())
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BuiltinMesh {
    /// The standard ball mesh. ID: `@builtin/ball`.
    Ball,
    /// The puck mesh. ID: `@builtin/puck`.
    Puck,
    /// The Octane car mesh. ID: `@builtin/car_octane`.
    CarOctane,
    /// The Dominus car mesh. ID: `@builtin/car_dominus`.
    CarDominus,
    /// The Breakout car mesh. ID: `@builtin/car_breakout`.
    CarBreakout,
    /// The Merc car mesh. ID: `@builtin/car_merc`.
    CarMerc,
}

impl BuiltinMesh {
    /// Return the reserved resource identifier for this mesh.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Ball => "@builtin/ball",
            Self::Puck => "@builtin/puck",
            Self::CarOctane => "@builtin/car_octane",
            Self::CarDominus => "@builtin/car_dominus",
            Self::CarBreakout => "@builtin/car_breakout",
            Self::CarMerc => "@builtin/car_merc",
        }
    }
}

/// A texture that the viewer provides without a user resource.
///
/// Use [`Self::id`] for a textured item. The identifier does not need a
/// [`TextureResource`]. Do not create a resource with a reserved identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum BuiltinTexture {
    /// The ball texture. ID: `@builtin/ball`.
    Ball,
    /// The blue car texture. ID: `@builtin/car_blue`.
    CarBlue,
    /// The orange car texture. ID: `@builtin/car_orange`.
    CarOrange,
}

impl BuiltinTexture {
    /// Return the reserved resource identifier for this texture.
    #[must_use]
    pub const fn id(self) -> &'static str {
        match self {
            Self::Ball => "@builtin/ball",
            Self::CarBlue => "@builtin/car_blue",
            Self::CarOrange => "@builtin/car_orange",
        }
    }
}

pub(super) fn is_builtin_mesh_id(id: &str) -> bool {
    matches!(
        id,
        "@builtin/ball"
            | "@builtin/puck"
            | "@builtin/car_octane"
            | "@builtin/car_dominus"
            | "@builtin/car_breakout"
            | "@builtin/car_merc"
    )
}

pub(super) fn is_builtin_texture_id(id: &str) -> bool {
    matches!(
        id,
        "@builtin/ball" | "@builtin/car_blue" | "@builtin/car_orange"
    )
}

fn validate_resource_id(id: &str) -> DebugResult<()> {
    validate_name(id)?;
    if id.starts_with("@builtin/") {
        return Err(DebugError::ReservedResourceId(id.to_owned()));
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Reusable assets
// ---------------------------------------------------------------------------

/// Reusable indexed triangle mesh referenced by its identifier.
///
/// Colors must match the vertex count. Optional normals and UV coordinates
/// must also match that count. Indices describe triangles in listed order.
/// The viewer does not convert winding. It reuses unchanged vertex data until
/// this resource is replaced or the connection resets.
///
/// # Example
///
/// ```
/// use rocketsim_vis::debug::{Color, MeshResource};
///
/// let mesh = MeshResource::new(
///     "triangle",
///     vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
///     vec![Color::RED, Color::GREEN, Color::BLUE],
///     None, None, vec![0, 1, 2],
/// ).unwrap();
/// assert_eq!(mesh.indices.len(), 3);
/// ```
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MeshResource {
    /// Resource key. It must pass [`validate_name`] and must not start with `@builtin/`.
    pub id: String,
    /// Vertex positions in local Unreal units.
    pub positions: Vec<[f32; 3]>,
    /// One finite 0..=1 RGBA color per vertex.
    pub colors: Vec<Color>,
    /// Optional finite normal vectors, one per vertex.
    pub normals: Option<Vec<[f32; 3]>>,
    /// Optional finite `[u, v]` texture coordinates, one per vertex.
    pub uvs: Option<Vec<[f32; 2]>>,
    /// Triangle-list indices. Count must be divisible by 3. Indices must be in range.
    pub indices: Vec<u32>,
    /// Monotonic resource revision. Replacement increases this value.
    pub revision: u64,
}

impl MeshResource {
    /// Create and validate a mesh at revision `0`.
    ///
    /// The renderer assigns the publication revision on insertion. Return an
    /// error for an invalid name, empty geometry, invalid buffer lengths,
    /// non-finite values, invalid colors, or out-of-range indices.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        id: impl Into<String>,
        positions: Vec<[f32; 3]>,
        colors: Vec<Color>,
        normals: Option<Vec<[f32; 3]>>,
        uvs: Option<Vec<[f32; 2]>>,
        indices: Vec<u32>,
    ) -> DebugResult<Self> {
        let id = id.into();
        let out = Self {
            id,
            positions,
            colors,
            normals,
            uvs,
            indices,
            revision: 0,
        };
        out.validate()?;
        Ok(out)
    }

    /// Validate the identifier, mesh counts, buffer lengths, and numeric values.
    ///
    /// Require at least one vertex and one triangle. Normals need not be unit
    /// vectors. UV coordinates need not be in `0..=1`. Return the first error.
    pub fn validate(&self) -> DebugResult<()> {
        validate_resource_id(&self.id)?;
        if self.positions.len() > MAX_MESH_VERTICES {
            return Err(DebugError::TooManyVertices {
                count: self.positions.len(),
                max: MAX_MESH_VERTICES,
            });
        }
        if self.indices.len() > MAX_MESH_INDICES {
            return Err(DebugError::TooManyIndices {
                count: self.indices.len(),
                max: MAX_MESH_INDICES,
            });
        }
        if !self.indices.len().is_multiple_of(3) {
            return Err(DebugError::InvalidIndexCount {
                count: self.indices.len(),
            });
        }
        if self.positions.is_empty() || self.indices.is_empty() {
            return Err(DebugError::InvalidNumber("mesh needs vertices and indices"));
        }
        if self.colors.len() != self.positions.len() {
            return Err(DebugError::InvalidNumber(
                "mesh colors length must match vertices",
            ));
        }
        if let Some(n) = &self.normals
            && n.len() != self.positions.len()
        {
            return Err(DebugError::InvalidNumber(
                "mesh normals length must match vertices",
            ));
        }
        if let Some(u) = &self.uvs
            && u.len() != self.positions.len()
        {
            return Err(DebugError::InvalidNumber(
                "mesh uvs length must match vertices",
            ));
        }
        for p in &self.positions {
            if !p.iter().all(|x| x.is_finite()) {
                return Err(DebugError::InvalidNumber("mesh position"));
            }
        }
        for c in &self.colors {
            c.validate()?;
        }
        if let Some(normals) = &self.normals {
            for n in normals {
                if !n.iter().all(|x| x.is_finite()) {
                    return Err(DebugError::InvalidNumber("mesh normal"));
                }
            }
        }
        if let Some(uvs) = &self.uvs {
            for uv in uvs {
                if !uv.iter().all(|x| x.is_finite()) {
                    return Err(DebugError::InvalidNumber("mesh uv"));
                }
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
        Ok(())
    }

    /// Estimate retained bytes. This estimate is for budget checks, not exact allocation size.
    pub fn estimated_bytes(&self) -> usize {
        self.positions
            .len()
            .saturating_mul(28)
            .saturating_add(
                self.normals
                    .as_ref()
                    .map_or(0, |normals| normals.len().saturating_mul(12)),
            )
            .saturating_add(
                self.uvs
                    .as_ref()
                    .map_or(0, |uvs| uvs.len().saturating_mul(8)),
            )
            .saturating_add(self.indices.len().saturating_mul(4))
            .saturating_add(64)
            .saturating_add(self.id.len())
    }
}

/// Reusable RGBA texture referenced by its identifier.
///
/// Width and height must be in `1..=1024`. Supply exactly four bytes per
/// pixel, with RGB values not multiplied by alpha. The maximum size is 4 MiB.
/// The viewer uses linear filtering and supports transparency.
///
/// # Example
///
/// ```
/// use rocketsim_vis::debug::TextureResource;
///
/// let texture = TextureResource::new("pixel", 1, 1, vec![255, 0, 0, 255]).unwrap();
/// assert_eq!(texture.rgba, [255, 0, 0, 255]);
/// ```
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TextureResource {
    /// Resource key. It must pass [`validate_name`] and must not start with `@builtin/`.
    pub id: String,
    /// Texture width in pixels; valid range is 1..=1024.
    pub width: u32,
    /// Texture height in pixels; valid range is 1..=1024.
    pub height: u32,
    /// Exactly `width * height * 4` bytes in row-major order, from the top-left pixel. Each pixel stores red, green, blue, then alpha.
    pub rgba: Vec<u8>,
    /// Monotonic resource revision. Replacement increases this value.
    pub revision: u64,
}

impl TextureResource {
    /// Create and validate a texture at revision `0`.
    ///
    /// Return an error for an invalid identifier, dimensions, or byte count.
    /// The renderer assigns the publication revision on insertion.
    pub fn new(id: impl Into<String>, width: u32, height: u32, rgba: Vec<u8>) -> DebugResult<Self> {
        let id = id.into();
        let out = Self {
            id,
            width,
            height,
            rgba,
            revision: 0,
        };
        out.validate()?;
        Ok(out)
    }

    /// Validate the identifier, dimensions, and exact RGBA byte count.
    ///
    /// Return the first error. This does not inspect individual pixel values.
    pub fn validate(&self) -> DebugResult<()> {
        validate_resource_id(&self.id)?;
        if self.width == 0
            || self.height == 0
            || self.width > MAX_TEXTURE_DIM
            || self.height > MAX_TEXTURE_DIM
        {
            return Err(DebugError::InvalidTextureDim {
                width: self.width,
                height: self.height,
            });
        }
        let expected = (self.width as usize)
            .saturating_mul(self.height as usize)
            .saturating_mul(4);
        if expected > MAX_TEXTURE_BYTES {
            return Err(DebugError::TextureTooLarge {
                bytes: expected,
                max: MAX_TEXTURE_BYTES,
            });
        }
        if self.rgba.len() != expected {
            return Err(DebugError::TextureSizeMismatch {
                expected,
                got: self.rgba.len(),
            });
        }
        Ok(())
    }

    /// Estimate retained bytes. This estimate is for budget checks, not exact allocation size.
    pub fn estimated_bytes(&self) -> usize {
        self.rgba
            .len()
            .saturating_add(64)
            .saturating_add(self.id.len())
    }
}
