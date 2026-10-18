//! Retained scene: [`DebugGroup`], reusable [`MeshResource`] /
//! [`TextureResource`], and the immutable [`DebugScene`] snapshot.
//!
//! A scene is a set of named groups plus asset tables. Groups replace
//! atomically by name; assets upload once and are referenced by id from
//! [`crate::debug::types::MeshInstanceItem`] and textured items, so later
//! transform-only group updates never resend mesh bytes.

use std::{collections::BTreeMap, sync::Arc};

use super::types::{
    Anchor, Color, DebugError, DebugResult, DrawList, GroupTransform, Lifetime, MAX_DEBUG_BYTES,
    MAX_GROUPS, MAX_MESH_INDICES, MAX_MESH_VERTICES, MAX_MESHES, MAX_TEXTURE_BYTES,
    MAX_TEXTURE_DIM, MAX_TEXTURES, validate_name,
};

// ---------------------------------------------------------------------------
// Group
// ---------------------------------------------------------------------------

/// Placement/visibility options for [`DebugGroup`] creation.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GroupOptions {
    pub visible: bool,
    pub anchor: Anchor,
    pub transform: GroupTransform,
    pub lifetime: Lifetime,
}

impl Default for GroupOptions {
    fn default() -> Self {
        Self {
            visible: true,
            anchor: Anchor::World,
            transform: GroupTransform::identity(),
            lifetime: Lifetime::Persistent,
        }
    }
}

impl GroupOptions {
    /// Visible persistent world group at identity.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Builder: visibility.
    #[must_use]
    pub fn visible(mut self, v: bool) -> Self {
        self.visible = v;
        self
    }

    /// Builder: anchor.
    #[must_use]
    pub fn anchor(mut self, a: Anchor) -> Self {
        self.anchor = a;
        self
    }

    /// Builder: transform.
    #[must_use]
    pub fn transform(mut self, t: GroupTransform) -> Self {
        self.transform = t;
        self
    }

    /// Builder: lifetime.
    #[must_use]
    pub fn lifetime(mut self, l: Lifetime) -> Self {
        self.lifetime = l;
        self
    }

    pub fn validate(&self) -> DebugResult<()> {
        self.anchor.validate()?;
        self.transform.validate()?;
        self.lifetime.validate()?;
        Ok(())
    }
}

/// One named retained drawing.
///
/// Items are stored in group-local space; see the transform-order contract
/// in [`crate::debug::types`].
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DebugGroup {
    /// Group name (`1..=64`, `[A-Za-z0-9._-]`, starts alnum).
    pub name: String,
    pub visible: bool,
    pub anchor: Anchor,
    pub transform: GroupTransform,
    pub lifetime: Lifetime,
    /// Items in submission order. The renderer batches opaque geometry and
    /// depth-sorts translucent mesh instances. Submission order breaks depth ties.
    pub items: Arc<Vec<super::types::DebugItem>>,
    /// Monotonic revision, bumped on every accepted change.
    pub revision: u64,
    /// Changes only when item geometry changes.
    pub geometry_revision: u64,
    /// Absolute sender simulation tick at which this group expires.
    pub expires_tick: Option<u64>,
    /// Absolute Unix time in milliseconds at which a seconds lifetime expires.
    pub expires_at_ms: Option<u64>,
}

impl DebugGroup {
    /// Build a group at revision `0` (renderer assigns the real revision).
    pub fn new(name: impl Into<String>, list: DrawList, opts: GroupOptions) -> DebugResult<Self> {
        let name = name.into();
        validate_name(&name)?;
        opts.validate()?;
        list.validate(&name)?;
        Ok(Self {
            name,
            visible: opts.visible,
            anchor: opts.anchor,
            transform: opts.transform,
            lifetime: opts.lifetime,
            items: Arc::new(list.items),
            revision: 0,
            geometry_revision: 0,
            expires_tick: None,
            expires_at_ms: None,
        })
    }

    pub fn validate(&self) -> DebugResult<()> {
        validate_name(&self.name)?;
        self.anchor.validate()?;
        self.transform.validate()?;
        self.lifetime.validate()?;
        if self.items.len() > super::types::MAX_ITEMS_PER_GROUP {
            return Err(DebugError::TooManyItems {
                group: self.name.clone(),
                count: self.items.len(),
                max: super::types::MAX_ITEMS_PER_GROUP,
            });
        }
        for item in self.items.iter() {
            item.validate()?;
        }
        Ok(())
    }

    /// Rough byte estimate.
    pub fn estimated_bytes(&self) -> usize {
        self.items
            .iter()
            .map(super::types::DebugItem::estimated_bytes)
            .sum::<usize>()
            .saturating_add(self.name.len())
            .saturating_add(64)
    }
}

// ---------------------------------------------------------------------------
// Reusable assets
// ---------------------------------------------------------------------------

/// Reusable indexed triangle mesh uploaded once, referenced by id.
///
/// Vertex colors are required (tint multiplies at instance time);
/// normals/UVs are optional but must match vertex count when present.
/// User-local OBJ importers (examples) build this; no asset is bundled.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MeshResource {
    /// Asset id (same name rules as groups).
    pub id: String,
    pub positions: Vec<[f32; 3]>,
    pub colors: Vec<Color>,
    pub normals: Option<Vec<[f32; 3]>>,
    pub uvs: Option<Vec<[f32; 2]>>,
    pub indices: Vec<u32>,
    /// Monotonic revision, bumped on replace.
    pub revision: u64,
}

impl MeshResource {
    /// Build at revision `0` (renderer assigns the real revision).
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

    pub fn validate(&self) -> DebugResult<()> {
        validate_name(&self.id)?;
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

    /// Rough byte estimate.
    pub fn estimated_bytes(&self) -> usize {
        self.positions
            .len()
            .saturating_mul(28)
            .saturating_add(self.indices.len().saturating_mul(4))
            .saturating_add(64)
    }
}

/// Reusable RGBA texture uploaded once, referenced by id.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct TextureResource {
    /// Asset id.
    pub id: String,
    pub width: u32,
    pub height: u32,
    /// `width * height * 4` RGBA bytes, row-major top-left.
    pub rgba: Vec<u8>,
    /// Monotonic revision, bumped on replace.
    pub revision: u64,
}

impl TextureResource {
    /// Build at revision `0` (renderer assigns the real revision).
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

    pub fn validate(&self) -> DebugResult<()> {
        validate_name(&self.id)?;
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

    /// Rough byte estimate.
    pub fn estimated_bytes(&self) -> usize {
        self.rgba.len().saturating_add(64)
    }
}

// ---------------------------------------------------------------------------
// Immutable snapshot
// ---------------------------------------------------------------------------

/// Immutable retained scene: groups plus reusable assets.
///
/// Shared by `Arc` between sender, receiver, and GPU code; updates publish
/// a new `Arc` atomically (never mutated in place).
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DebugScene {
    pub groups: BTreeMap<String, Arc<DebugGroup>>,
    pub meshes: BTreeMap<String, Arc<MeshResource>>,
    pub textures: BTreeMap<String, Arc<TextureResource>>,
    /// Global revision, bumped on every accepted publication.
    pub revision: u64,
}

impl DebugScene {
    /// Empty scene at revision `0`.
    #[must_use]
    pub fn empty() -> Self {
        Self::default()
    }

    /// True when no groups, meshes, or textures are present.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.groups.is_empty() && self.meshes.is_empty() && self.textures.is_empty()
    }

    /// Number of groups.
    #[must_use]
    pub fn group_count(&self) -> usize {
        self.groups.len()
    }

    /// Fetch a group by name.
    #[must_use]
    pub fn get_group(&self, name: &str) -> Option<&DebugGroup> {
        self.groups.get(name).map(Arc::as_ref)
    }

    /// Fetch a mesh by id.
    #[must_use]
    pub fn get_mesh(&self, id: &str) -> Option<&MeshResource> {
        self.meshes.get(id).map(Arc::as_ref)
    }

    /// Fetch a texture by id.
    #[must_use]
    pub fn get_texture(&self, id: &str) -> Option<&TextureResource> {
        self.textures.get(id).map(Arc::as_ref)
    }

    /// Total item count across groups.
    #[must_use]
    pub fn total_items(&self) -> usize {
        self.groups.values().map(|g| g.items.len()).sum()
    }

    /// Rough byte estimate (groups + resources).
    #[must_use]
    pub fn estimated_bytes(&self) -> usize {
        let mut n = 0usize;
        for g in self.groups.values() {
            n = n.saturating_add(g.estimated_bytes());
        }
        for m in self.meshes.values() {
            n = n.saturating_add(m.estimated_bytes());
        }
        for t in self.textures.values() {
            n = n.saturating_add(t.estimated_bytes());
        }
        n
    }

    /// Validate counts, per-entry validation, and total budget.
    pub fn validate(&self) -> DebugResult<()> {
        if self.groups.len() > MAX_GROUPS {
            return Err(DebugError::TooManyGroups {
                count: self.groups.len(),
                max: MAX_GROUPS,
            });
        }
        if self.meshes.len() > MAX_MESHES {
            return Err(DebugError::TooManyMeshes {
                count: self.meshes.len(),
                max: MAX_MESHES,
            });
        }
        if self.textures.len() > MAX_TEXTURES {
            return Err(DebugError::TooManyTextures {
                count: self.textures.len(),
                max: MAX_TEXTURES,
            });
        }
        let mut total_items = 0usize;
        for (name, group) in &self.groups {
            if name != &group.name {
                return Err(DebugError::InvalidName(group.name.clone()));
            }
            group.validate()?;
            total_items = total_items.saturating_add(group.items.len());
        }
        if total_items > super::types::MAX_TOTAL_ITEMS {
            return Err(DebugError::TooManyTotalItems {
                count: total_items,
                max: super::types::MAX_TOTAL_ITEMS,
            });
        }
        for (id, mesh) in &self.meshes {
            if id != &mesh.id {
                return Err(DebugError::InvalidName(mesh.id.clone()));
            }
            mesh.validate()?;
        }
        for (id, tex) in &self.textures {
            if id != &tex.id {
                return Err(DebugError::InvalidName(tex.id.clone()));
            }
            tex.validate()?;
        }
        let bytes = self.estimated_bytes();
        if bytes > MAX_DEBUG_BYTES {
            return Err(DebugError::PayloadTooLarge {
                bytes,
                max: MAX_DEBUG_BYTES,
            });
        }
        Ok(())
    }

    /// Cheap `Arc` wrap for publication.
    #[must_use]
    pub fn shared(self) -> Arc<Self> {
        Arc::new(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::debug::types::{Color, DepthMode, DrawList, Width};

    #[test]
    fn group_replace_delete_independent() {
        let mut scene = DebugScene::empty();
        let mut a = DrawList::new();
        a.line(
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            Color::RED,
            Width::pixels(2.0),
            DepthMode::Test,
        );
        let mut b = DrawList::new();
        b.point(
            [0.0, 0.0, 0.0],
            Color::BLUE,
            Width::pixels(4.0),
            DepthMode::Always,
        );
        scene.groups.insert(
            "a".into(),
            Arc::new(DebugGroup::new("a", a, GroupOptions::default()).unwrap()),
        );
        scene.groups.insert(
            "b".into(),
            Arc::new(DebugGroup::new("b", b, GroupOptions::default()).unwrap()),
        );
        assert_eq!(scene.group_count(), 2);
        scene.groups.remove("a");
        assert!(scene.get_group("a").is_none());
        assert!(scene.get_group("b").is_some());
        assert!(scene.validate().is_ok());
    }

    #[test]
    fn scene_validation_bounds_memory() {
        let mut scene = DebugScene::empty();
        for i in 0..(MAX_GROUPS + 1) {
            let list = DrawList::new();
            let group = DebugGroup::new(format!("g{i}"), list, GroupOptions::default()).unwrap();
            scene.groups.insert(format!("g{i}"), Arc::new(group));
        }
        assert!(matches!(
            scene.validate(),
            Err(DebugError::TooManyGroups { .. })
        ));
    }
}
