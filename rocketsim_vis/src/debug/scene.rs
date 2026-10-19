//! Named debug groups and immutable retained-scene snapshots.
//!
//! Reusable mesh and texture resources live in the private `resources` module.

use std::{collections::BTreeMap, sync::Arc};

use super::{
    resources::{MeshResource, TextureResource},
    types::{
        Anchor, DebugError, DebugResult, DrawList, GroupTransform, Lifetime, MAX_DEBUG_BYTES,
        MAX_GROUPS, MAX_MESHES, MAX_TEXTURES, validate_name,
    },
};

// ---------------------------------------------------------------------------
// Group
// ---------------------------------------------------------------------------

/// Visibility, anchor, transform, and lifetime for a new [`DebugGroup`].
///
/// The default is visible, persistent, world-anchored, and at identity.
///
/// # Example
///
/// ```
/// use rocketsim_vis::debug::{Anchor, Color, DebugGroup, DrawList, GroupOptions, Lifetime};
///
/// let mut draw = DrawList::new();
/// draw.point(
///     [0.0, 0.0, 0.0],
///     Color::WHITE,
///     Default::default(),
///     Default::default(),
/// );
/// let options = GroupOptions::new()
///     .anchor(Anchor::ball())
///     .lifetime(Lifetime::Ticks(5));
/// let group = DebugGroup::new("ball_marker", draw, options).unwrap();
/// assert_eq!(group.lifetime, Lifetime::Ticks(5));
/// ```
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GroupOptions {
    /// Whether the renderer draws this group.
    pub visible: bool,
    /// Reference frame used after the group transform.
    pub anchor: Anchor,
    /// Group-local scale, rotation, and translation.
    pub transform: GroupTransform,
    /// Expiry rule for this group.
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
    /// Return visible, persistent options with a world anchor and identity transform.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Set whether the group is visible.
    #[must_use]
    pub fn visible(mut self, v: bool) -> Self {
        self.visible = v;
        self
    }

    /// Set the reference frame for the group.
    #[must_use]
    pub fn anchor(mut self, a: Anchor) -> Self {
        self.anchor = a;
        self
    }

    /// Set the group transform. Apply scale, then rotation, then translation.
    #[must_use]
    pub fn transform(mut self, t: GroupTransform) -> Self {
        self.transform = t;
        self
    }

    /// Set the group lifetime and its expiry rule.
    #[must_use]
    pub fn lifetime(mut self, l: Lifetime) -> Self {
        self.lifetime = l;
        self
    }

    /// Validate the anchor, transform, and lifetime. Return the first error.
    pub fn validate(&self) -> DebugResult<()> {
        self.anchor.validate()?;
        self.transform.validate()?;
        self.lifetime.validate()?;
        Ok(())
    }
}

/// One named retained drawing.
///
/// Items are stored in group-local space. See the coordinate-system section
/// in [`crate::debug`].
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DebugGroup {
    /// Group key. Names are 1..=64 UTF-8 bytes; slash-separated segments must be non-empty, not `.` or `..`, and contain no whitespace or control characters.
    pub name: String,
    /// Whether the renderer draws this group.
    pub visible: bool,
    /// Reference frame used after the group transform.
    pub anchor: Anchor,
    /// Group-local scale, rotation, and translation.
    pub transform: GroupTransform,
    /// Expiry rule for this group.
    pub lifetime: Lifetime,
    /// Items in submission order. The renderer may batch opaque geometry and depth-sort translucent mesh instances; submission order breaks depth ties.
    pub items: Arc<Vec<super::types::DebugItem>>,
    /// Monotonic group revision. Accepted changes increase this value.
    pub revision: u64,
    /// Revision that changes only when item geometry changes.
    pub geometry_revision: u64,
    /// Absolute sender simulation tick deadline, when the group has a tick lifetime.
    pub expires_tick: Option<u64>,
    /// Absolute Unix time deadline in milliseconds, when the group has a seconds lifetime.
    pub expires_at_ms: Option<u64>,
}

impl DebugGroup {
    /// Build and validate a named group at revision 0.
    ///
    /// `name` must pass [`validate_name`]. The list and options are validated
    /// before construction. Publication assigns the real revision.
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

    /// Validate the name, options, item count, and each item.
    ///
    /// Resource references are checked when the group is submitted to a renderer.
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

    /// Estimate retained bytes. This estimate is for budget checks, not exact allocation size.
    pub fn estimated_bytes(&self) -> usize {
        self.items
            .iter()
            .fold(self.name.len().saturating_add(160), |bytes, item| {
                bytes.saturating_add(item.estimated_bytes())
            })
    }
}

// ---------------------------------------------------------------------------
// Immutable snapshot
// ---------------------------------------------------------------------------

/// A retained-scene snapshot with named groups and reusable resources.
///
/// The renderer and receiver publish snapshots through [`Arc`]. Later scene
/// updates do not change an earlier snapshot. Unchanged groups and resources
/// share storage between snapshots. Public fields also let custom clients
/// construct scenes directly.
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct DebugScene {
    /// Groups keyed by their names. Each value is an immutable shared group.
    pub groups: BTreeMap<String, Arc<DebugGroup>>,
    /// Reusable meshes keyed by asset id.
    pub meshes: BTreeMap<String, Arc<MeshResource>>,
    /// Reusable textures keyed by asset id.
    pub textures: BTreeMap<String, Arc<TextureResource>>,
    /// Global revision. Each accepted scene publication increases this value.
    pub revision: u64,
}

impl DebugScene {
    /// Return an empty scene at revision `0`.
    #[must_use]
    pub fn empty() -> Self {
        Self::default()
    }

    /// Return `true` if all group and resource tables are empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.groups.is_empty() && self.meshes.is_empty() && self.textures.is_empty()
    }

    /// Return the number of named groups.
    #[must_use]
    pub fn group_count(&self) -> usize {
        self.groups.len()
    }

    /// Return a borrowed group for `name`, or `None` if it does not exist.
    #[must_use]
    pub fn get_group(&self, name: &str) -> Option<&DebugGroup> {
        self.groups.get(name).map(Arc::as_ref)
    }

    /// Return a borrowed mesh for `id`, or `None` if it does not exist.
    #[must_use]
    pub fn get_mesh(&self, id: &str) -> Option<&MeshResource> {
        self.meshes.get(id).map(Arc::as_ref)
    }

    /// Return a borrowed texture for `id`, or `None` if it does not exist.
    #[must_use]
    pub fn get_texture(&self, id: &str) -> Option<&TextureResource> {
        self.textures.get(id).map(Arc::as_ref)
    }

    /// Return the total item count across all groups.
    #[must_use]
    pub fn total_items(&self) -> usize {
        self.groups.values().map(|g| g.items.len()).sum()
    }

    /// Estimate retained bytes with saturating arithmetic.
    ///
    /// This is a budget estimate, not an exact allocation size.
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

    /// Wrap this snapshot in an `Arc` for shared read-only publication.
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
