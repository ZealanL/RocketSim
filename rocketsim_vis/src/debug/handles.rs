use std::sync::Arc;

use super::{
    renderer::DebugRenderer,
    resources::{MeshResource, TextureResource},
    scene::GroupOptions,
    store::validate_item_resources,
    types::{
        Anchor, DebugError, DebugItem, DebugResult, DrawList, GroupTransform, MAX_DEBUG_BYTES,
        validate_name,
    },
};

// ---------------------------------------------------------------------------
// Handles
// ---------------------------------------------------------------------------

/// Cloneable handle for a named group.
///
/// The handle owns a clone of its renderer, so it remains usable while that
/// renderer state exists. Full replacement restarts the group's lifetime;
/// visibility, transform, and anchor updates preserve it.
#[derive(Clone, Debug)]
pub struct GroupHandle {
    pub(super) renderer: DebugRenderer,
    pub(super) name: String,
}

impl GroupHandle {
    /// Replace the group and restart its lifetime from `opts`.
    ///
    /// Return the new revision. Return [`DebugError`] if validation or a scene
    /// limit rejects the replacement. Referenced assets must already exist.
    pub fn replace(&self, list: DrawList, opts: GroupOptions) -> DebugResult<u64> {
        self.renderer.replace_group(&self.name, list, opts)
    }

    /// Remove the group. Return `true` when a group existed.
    ///
    /// Return `false` when the group was already absent.
    pub fn clear(&self) -> DebugResult<bool> {
        self.renderer.clear_group(&self.name)
    }

    /// Set visibility without restarting the lifetime.
    ///
    /// Return [`DebugError::GroupNotFound`] when the group does not exist.
    pub fn set_visible(&self, visible: bool) -> DebugResult<()> {
        self.renderer.set_group_visible(&self.name, visible)
    }

    /// Set the group transform without restarting the lifetime.
    ///
    /// Invalid transform values or a missing group return [`DebugError`].
    pub fn set_transform(&self, t: GroupTransform) -> DebugResult<()> {
        self.renderer.set_group_transform(&self.name, t)
    }

    /// Set the anchor without restarting the lifetime.
    ///
    /// An invalid anchor or a missing group returns [`DebugError`].
    pub fn set_anchor(&self, anchor: Anchor) -> DebugResult<()> {
        self.renderer.set_group_anchor(&self.name, anchor)
    }

    /// Return this handle's group name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }
}

/// Cloneable handle for one reusable mesh id.
///
/// The handle owns a renderer clone. Resource removal fails while a group
/// uses the mesh.
#[derive(Clone, Debug)]
pub struct MeshHandle {
    pub(super) renderer: DebugRenderer,
    pub(super) id: String,
}

impl MeshHandle {
    /// Insert or replace this mesh resource.
    ///
    /// The resource id must match [`Self::id`]. Return its new revision, or a
    /// validation or scene-limit error.
    pub fn insert(&self, res: MeshResource) -> DebugResult<u64> {
        if res.id != self.id {
            return Err(DebugError::ResourceIdMismatch {
                expected: self.id.clone(),
                actual: res.id,
            });
        }
        self.renderer.insert_mesh(res)
    }

    /// Remove the mesh. Return `true` when removed and `false` when absent.
    ///
    /// Return [`DebugError::ResourceInUse`] if a group still refers to it.
    pub fn remove(&self) -> DebugResult<bool> {
        self.renderer.remove_mesh(&self.id)
    }

    /// Return this handle's mesh id.
    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }
}

/// Cloneable handle for one reusable texture id.
///
/// The handle owns a renderer clone. Resource removal fails while a group
/// uses the texture.
#[derive(Clone, Debug)]
pub struct TextureHandle {
    pub(super) renderer: DebugRenderer,
    pub(super) id: String,
}

impl TextureHandle {
    /// Insert or replace this texture resource.
    ///
    /// The resource id must match [`Self::id`]. Return its new revision, or a
    /// validation or scene-limit error.
    pub fn insert(&self, res: TextureResource) -> DebugResult<u64> {
        if res.id != self.id {
            return Err(DebugError::ResourceIdMismatch {
                expected: self.id.clone(),
                actual: res.id,
            });
        }
        self.renderer.insert_texture(res)
    }

    /// Remove the texture. Return `true` when removed and `false` when absent.
    ///
    /// Return [`DebugError::ResourceInUse`] if a group still refers to it.
    pub fn remove(&self) -> DebugResult<bool> {
        self.renderer.remove_texture(&self.id)
    }

    /// Return this handle's texture id.
    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }
}

/// Handle for one item at a captured group geometry revision.
///
/// The handle owns a renderer clone and preserves the group lifetime when it
/// changes an item. Any geometry edit invalidates older handles. Request a
/// new handle after a replacement or item change.
#[derive(Clone, Debug)]
pub struct ItemHandle {
    pub(super) renderer: DebugRenderer,
    pub(super) group: String,
    pub(super) index: usize,
    pub(super) revision: Option<u64>,
}

impl ItemHandle {
    /// Replace this item without restarting the group lifetime.
    ///
    /// Return [`DebugError::StaleHandle`] if group geometry changed after this
    /// handle was created. Invalid items or missing resources also return an
    /// error.
    pub fn set(&self, item: DebugItem) -> DebugResult<()> {
        item.validate()?;
        let renderer = self.renderer.clone();
        let group = self.group.clone();
        let index = self.index;
        validate_name(&group)?;
        let Ok(mut inner) = renderer.inner.lock() else {
            return Err(DebugError::InvalidNumber("debug store poisoned"));
        };
        let Some(existing) = inner.groups.get(&group).cloned() else {
            return Err(DebugError::GroupNotFound(group));
        };
        if self.revision != Some(existing.group.geometry_revision) {
            return Err(DebugError::StaleHandle);
        }
        let mut entry = (*existing).clone();
        if index >= entry.group.items.len() {
            return Err(DebugError::StaleHandle);
        }
        validate_item_resources(&item, &inner.meshes, &inner.textures)?;
        let mut group_data = (*entry.group).clone();
        Arc::make_mut(&mut group_data.items)[index] = item;
        group_data.revision = inner.alloc_rev();
        group_data.geometry_revision = group_data.revision;
        group_data.validate()?;
        let bytes = group_data.estimated_bytes()
            + inner
                .groups
                .iter()
                .filter(|(name, _)| *name != &group)
                .map(|(_, value)| value.group.estimated_bytes())
                .sum::<usize>()
            + inner
                .meshes
                .values()
                .map(|value| value.estimated_bytes())
                .sum::<usize>()
            + inner
                .textures
                .values()
                .map(|value| value.estimated_bytes())
                .sum::<usize>();
        if bytes > MAX_DEBUG_BYTES {
            return Err(DebugError::PayloadTooLarge {
                bytes,
                max: MAX_DEBUG_BYTES,
            });
        }
        entry.group = Arc::new(group_data);
        inner.groups.insert(group, Arc::new(entry));
        Ok(())
    }

    /// Remove this item without restarting the group lifetime.
    ///
    /// Return [`DebugError::StaleHandle`] if group geometry changed after this
    /// handle was created. Any geometry edit invalidates older item handles.
    pub fn remove(&self) -> DebugResult<()> {
        let renderer = self.renderer.clone();
        let group = self.group.clone();
        let index = self.index;
        validate_name(&group)?;
        let Ok(mut inner) = renderer.inner.lock() else {
            return Err(DebugError::InvalidNumber("debug store poisoned"));
        };
        let Some(existing) = inner.groups.get(&group).cloned() else {
            return Err(DebugError::GroupNotFound(group));
        };
        if self.revision != Some(existing.group.geometry_revision) {
            return Err(DebugError::StaleHandle);
        }
        let mut entry = (*existing).clone();
        if index >= entry.group.items.len() {
            return Err(DebugError::StaleHandle);
        }
        let mut group_data = (*entry.group).clone();
        Arc::make_mut(&mut group_data.items).remove(index);
        group_data.revision = inner.alloc_rev();
        group_data.geometry_revision = group_data.revision;
        entry.group = Arc::new(group_data);
        inner.groups.insert(group, Arc::new(entry));
        Ok(())
    }
}
