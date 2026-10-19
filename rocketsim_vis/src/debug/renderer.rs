//! Submit and update shared retained drawings through [`DebugRenderer`].
//!
//! Submissions validate data and update bounded scene storage under a lock.
//! They do not wait for network delivery. The sender serializes accepted
//! changes on a separate thread. Updates remain available while the viewer
//! is disconnected or the simulation is paused.
//!
//! Only group replacement restarts a group's lifetime. Visibility, transform,
//! anchor, and item changes preserve its deadline. [`Lifetime::Frame`] remains
//! through the next simulation update and expires at the following update.
//! A detached renderer advances only through [`DebugRenderer::advance_tick`].
//!
//! # Update a persistent group
//!
//! ```
//! use rocketsim_vis::debug::{DebugRenderer, GroupTransform};
//!
//! let renderer = DebugRenderer::new_detached();
//! let group = renderer.group("marker");
//! let first_revision = group
//!     .draw(|draw| {
//!         draw.point([0.0, 0.0, 0.0]);
//!     })
//!     .unwrap();
//! group
//!     .set_transform(GroupTransform::from_position([5.0, 0.0, 0.0]))
//!     .unwrap();
//! assert!(renderer.snapshot().groups["marker"].revision > first_revision);
//! ```
//!
//! Item handles capture a geometry revision. Group replacement and item
//! replacement or removal make older item handles stale. Transform, visibility,
//! and anchor updates do not. Request a new handle after a geometry change.
//!
//! ```
//! use rocketsim_vis::debug::{DebugError, DebugRenderer};
//!
//! let renderer = DebugRenderer::new_detached();
//! let group = renderer.group("items");
//! group
//!     .draw(|draw| {
//!         draw.point([0.0, 0.0, 0.0]);
//!     })
//!     .unwrap();
//! let old_item = renderer.item("items", 0);
//! group
//!     .draw(|draw| {
//!         draw.point([1.0, 0.0, 0.0]);
//!     })
//!     .unwrap();
//! assert_eq!(old_item.remove(), Err(DebugError::StaleHandle));
//! ```

use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicU64, Ordering},
    },
    time::Instant,
};

use super::{
    handles::{GroupHandle, ItemHandle, MeshHandle, TextureHandle},
    resources::{MeshResource, TextureResource},
    scene::{DebugGroup, DebugScene, GroupOptions},
    store::{GroupEntry, SharedInner, prune_locked, unix_time_ms, validate_item_resources},
    types::{
        Anchor, DebugError, DebugItem, DebugResult, DrawList, GroupTransform, Lifetime,
        MAX_DEBUG_BYTES, MAX_GROUPS, MAX_MESHES, MAX_TEXTURES, MAX_TOTAL_ITEMS, validate_name,
    },
};

/// Shared handle for submitting and updating a retained debug scene.
///
/// Obtain it from [`crate::ArenaVisExt::debug_renderer`] after attachment.
/// Use [`Self::new_detached`] for local drawing and tests without a viewer.
/// Clones share groups, resources, revisions, and the update clock.
///
/// Fallible operations return [`DebugError`] for invalid input or scene limits.
/// Repeated updates can combine before network delivery. A viewer can receive
/// only the newest version of a group, not every intermediate change.
#[derive(Clone, Debug)]
pub struct DebugRenderer {
    pub(super) inner: Arc<Mutex<SharedInner>>,
    pub(super) tick: Arc<AtomicU64>,
}

impl DebugRenderer {
    /// Create a detached renderer for local use without a viewer or network.
    ///
    /// The tick starts at `0` and advances only through
    /// [`Self::advance_tick`].
    #[must_use]
    pub fn new_detached() -> Self {
        Self {
            inner: Arc::new(Mutex::new(SharedInner::default())),
            tick: Arc::new(AtomicU64::new(0)),
        }
    }

    pub(crate) fn new_shared(tick: Arc<AtomicU64>) -> Self {
        Self {
            inner: Arc::new(Mutex::new(SharedInner::default())),
            tick,
        }
    }

    /// Return the debug update count used for tick lifetimes.
    ///
    /// An attached renderer counts visualizer updates since attachment. This
    /// can differ from the arena's tick count. A detached renderer uses the
    /// count advanced by [`Self::advance_tick`].
    #[must_use]
    pub fn current_tick(&self) -> u64 {
        self.tick.load(Ordering::Acquire)
    }

    /// Advance this renderer's tick by one.
    ///
    /// Use this with a detached renderer to expire tick-based groups.
    pub fn advance_tick(&self) {
        self.tick.fetch_add(1, Ordering::AcqRel);
    }

    /// Remove expired groups now and return the number removed.
    ///
    /// Group replacement and snapshot operations also remove expired groups.
    pub fn prune_expired(&self) -> usize {
        let now_tick = self.current_tick();
        let Ok(mut inner) = self.inner.lock() else {
            return 0;
        };
        prune_locked(&mut inner, now_tick)
    }

    /// Atomically replace or create a named group.
    ///
    /// Return the new group revision. A successful replacement restarts the
    /// lifetime in `opts`. Validate the name, options, items, and resource
    /// references before the change. Invalid input, a missing resource, a
    /// scene limit, or a poisoned store returns [`DebugError`]. On error, the
    /// requested group is not replaced.
    pub fn replace_group(
        &self,
        name: &str,
        list: DrawList,
        opts: GroupOptions,
    ) -> DebugResult<u64> {
        validate_name(name)?;
        opts.validate()?;
        list.validate(name)?;
        let now_tick = self.current_tick();
        let now = Instant::now();
        let Ok(mut inner) = self.inner.lock() else {
            return Err(DebugError::InvalidNumber("debug store poisoned"));
        };
        prune_locked(&mut inner, now_tick);
        for item in &list.items {
            validate_item_resources(item, &inner.meshes, &inner.textures)?;
        }
        if !inner.groups.contains_key(name) && inner.groups.len() >= MAX_GROUPS {
            return Err(DebugError::TooManyGroups {
                count: inner.groups.len() + 1,
                max: MAX_GROUPS,
            });
        }
        // Budget check: total items + estimated bytes.
        let mut total_items = list.items.len();
        for (n, e) in &inner.groups {
            if n != name {
                total_items = total_items.saturating_add(e.group.items.len());
            }
        }
        if total_items > MAX_TOTAL_ITEMS {
            return Err(DebugError::TooManyTotalItems {
                count: total_items,
                max: MAX_TOTAL_ITEMS,
            });
        }
        let mut group = DebugGroup::new(name, list, opts)?;
        group.expires_tick = match group.lifetime {
            Lifetime::Frame => Some(now_tick.saturating_add(2)),
            Lifetime::Ticks(n) => Some(now_tick.saturating_add(u64::from(n)).saturating_add(1)),
            _ => None,
        };
        group.expires_at_ms = match group.lifetime {
            Lifetime::Seconds(s) => Some(unix_time_ms().saturating_add((s * 1000.0).ceil() as u64)),
            _ => None,
        };
        let bytes = group.estimated_bytes();
        let mut total_bytes = bytes;
        for (n, e) in &inner.groups {
            if n != name {
                total_bytes = total_bytes.saturating_add(e.group.estimated_bytes());
            }
        }
        for m in inner.meshes.values() {
            total_bytes = total_bytes.saturating_add(m.estimated_bytes());
        }
        for t in inner.textures.values() {
            total_bytes = total_bytes.saturating_add(t.estimated_bytes());
        }
        if total_bytes > MAX_DEBUG_BYTES {
            return Err(DebugError::PayloadTooLarge {
                bytes: total_bytes,
                max: MAX_DEBUG_BYTES,
            });
        }
        let rev = inner.alloc_rev();
        group.revision = rev;
        group.geometry_revision = rev;
        if let Some(previous) = inner.groups.remove(name) {
            inner.remove_group_deadlines(&previous.group);
        }
        inner.track_group_deadlines(&group);
        inner.groups.insert(
            name.to_string(),
            Arc::new(GroupEntry {
                group: Arc::new(group),
                created_tick: now_tick,
                created_at: now,
            }),
        );
        Ok(rev)
    }

    /// Remove one group. `Ok(true)` when something was removed,
    /// `Ok(false)` when already absent (idempotent).
    pub fn clear_group(&self, name: &str) -> DebugResult<bool> {
        validate_name(name)?;
        let Ok(mut inner) = self.inner.lock() else {
            return Err(DebugError::InvalidNumber("debug store poisoned"));
        };
        if let Some(entry) = inner.groups.remove(name) {
            inner.remove_group_deadlines(&entry.group);
            inner.bump_scene_rev();
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Remove all groups, meshes, and textures.
    ///
    /// This operation has no validation error result.
    pub fn clear_all(&self) {
        if let Ok(mut inner) = self.inner.lock() {
            if inner.groups.is_empty() && inner.meshes.is_empty() && inner.textures.is_empty() {
                return;
            }
            inner.groups.clear();
            inner.meshes.clear();
            inner.textures.clear();
            inner.clear_group_deadlines();
            inner.bump_scene_rev();
        }
    }

    /// Remove all groups and keep reusable meshes and textures.
    pub fn clear_groups(&self) {
        if let Ok(mut inner) = self.inner.lock()
            && !inner.groups.is_empty()
        {
            inner.groups.clear();
            inner.clear_group_deadlines();
            inner.bump_scene_rev();
        }
    }

    /// Set visibility without restarting the lifetime.
    pub fn set_group_visible(&self, name: &str, visible: bool) -> DebugResult<()> {
        self.mutate_group(name, |entry, inner| {
            if entry.group.visible == visible {
                return;
            }
            let mut group = (*entry.group).clone();
            group.revision = inner.alloc_rev();
            group.visible = visible;
            *entry = GroupEntry {
                group: Arc::new(group),
                created_tick: entry.created_tick,
                created_at: entry.created_at,
            };
        })
    }

    /// Replace the group transform without restarting the lifetime.
    pub fn set_group_transform(&self, name: &str, t: GroupTransform) -> DebugResult<()> {
        t.validate()?;
        self.mutate_group(name, |entry, inner| {
            let mut group = (*entry.group).clone();
            group.revision = inner.alloc_rev();
            group.transform = t.clone();
            *entry = GroupEntry {
                group: Arc::new(group),
                created_tick: entry.created_tick,
                created_at: entry.created_at,
            };
        })
    }

    /// Replace the group anchor without restarting the lifetime.
    pub fn set_group_anchor(&self, name: &str, anchor: Anchor) -> DebugResult<()> {
        anchor.validate()?;
        self.mutate_group(name, |entry, inner| {
            let mut group = (*entry.group).clone();
            group.revision = inner.alloc_rev();
            group.anchor = anchor.clone();
            *entry = GroupEntry {
                group: Arc::new(group),
                created_tick: entry.created_tick,
                created_at: entry.created_at,
            };
        })
    }

    fn mutate_group(
        &self,
        name: &str,
        f: impl FnOnce(&mut GroupEntry, &mut SharedInner),
    ) -> DebugResult<()> {
        validate_name(name)?;
        let Ok(mut inner) = self.inner.lock() else {
            return Err(DebugError::InvalidNumber("debug store poisoned"));
        };
        let Some(existing) = inner.groups.get(name).cloned() else {
            return Err(DebugError::GroupNotFound(name.to_string()));
        };
        let mut entry = (*existing).clone();
        f(&mut entry, &mut inner);
        entry.group.validate()?;
        inner.groups.insert(name.to_string(), Arc::new(entry));
        Ok(())
    }

    /// Insert or replace a reusable mesh (upload once, instance many times).
    pub fn insert_mesh(&self, mut res: MeshResource) -> DebugResult<u64> {
        res.validate()?;
        let Ok(mut inner) = self.inner.lock() else {
            return Err(DebugError::InvalidNumber("debug store poisoned"));
        };
        if !inner.meshes.contains_key(&res.id) && inner.meshes.len() >= MAX_MESHES {
            return Err(DebugError::TooManyMeshes {
                count: inner.meshes.len() + 1,
                max: MAX_MESHES,
            });
        }
        let current = inner
            .groups
            .values()
            .map(|e| e.group.estimated_bytes())
            .sum::<usize>()
            + inner
                .meshes
                .iter()
                .filter(|(id, _)| *id != &res.id)
                .map(|(_, v)| v.estimated_bytes())
                .sum::<usize>()
            + inner
                .textures
                .values()
                .map(|v| v.estimated_bytes())
                .sum::<usize>();
        if current.saturating_add(res.estimated_bytes()) > MAX_DEBUG_BYTES {
            return Err(DebugError::PayloadTooLarge {
                bytes: current.saturating_add(res.estimated_bytes()),
                max: MAX_DEBUG_BYTES,
            });
        }
        let rev = inner.alloc_rev();
        res.revision = rev;
        inner.meshes.insert(res.id.clone(), Arc::new(res));
        Ok(rev)
    }

    /// Remove a reusable mesh if no retained group refers to it.
    ///
    /// Return `true` when removed, or `false` if it was absent. Return
    /// [`DebugError::ResourceInUse`] if a group still contains an instance
    /// of this mesh. Clear or replace those items before removing the mesh.
    pub fn remove_mesh(&self, id: &str) -> DebugResult<bool> {
        validate_name(id)?;
        let Ok(mut inner) = self.inner.lock() else {
            return Err(DebugError::InvalidNumber("debug store poisoned"));
        };
        if inner.groups.values().any(|entry| {
            entry.group.items.iter().any(|item| match item {
                DebugItem::Instance(instance) => instance.mesh == id,
                _ => false,
            })
        }) {
            return Err(DebugError::ResourceInUse(id.to_string()));
        }
        if inner.meshes.remove(id).is_some() {
            inner.bump_scene_rev();
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Insert or replace a reusable RGBA texture.
    pub fn insert_texture(&self, mut res: TextureResource) -> DebugResult<u64> {
        res.validate()?;
        let Ok(mut inner) = self.inner.lock() else {
            return Err(DebugError::InvalidNumber("debug store poisoned"));
        };
        if !inner.textures.contains_key(&res.id) && inner.textures.len() >= MAX_TEXTURES {
            return Err(DebugError::TooManyTextures {
                count: inner.textures.len() + 1,
                max: MAX_TEXTURES,
            });
        }
        let current = inner
            .groups
            .values()
            .map(|e| e.group.estimated_bytes())
            .sum::<usize>()
            + inner
                .meshes
                .values()
                .map(|v| v.estimated_bytes())
                .sum::<usize>()
            + inner
                .textures
                .iter()
                .filter(|(id, _)| *id != &res.id)
                .map(|(_, v)| v.estimated_bytes())
                .sum::<usize>();
        if current.saturating_add(res.estimated_bytes()) > MAX_DEBUG_BYTES {
            return Err(DebugError::PayloadTooLarge {
                bytes: current.saturating_add(res.estimated_bytes()),
                max: MAX_DEBUG_BYTES,
            });
        }
        let rev = inner.alloc_rev();
        res.revision = rev;
        inner.textures.insert(res.id.clone(), Arc::new(res));
        Ok(rev)
    }

    /// Remove a reusable texture if no retained item refers to it.
    ///
    /// Return `true` when removed, or `false` if it was absent. Return
    /// [`DebugError::ResourceInUse`] while a mesh, instance, or rectangle
    /// item still uses the texture. Invalid identifiers also return an error.
    pub fn remove_texture(&self, id: &str) -> DebugResult<bool> {
        validate_name(id)?;
        let Ok(mut inner) = self.inner.lock() else {
            return Err(DebugError::InvalidNumber("debug store poisoned"));
        };
        if inner.groups.values().any(|entry| {
            entry.group.items.iter().any(|item| match item {
                DebugItem::Mesh(mesh) => mesh.texture.as_deref() == Some(id),
                DebugItem::Instance(instance) => instance.texture.as_deref() == Some(id),
                DebugItem::Rect(rect) => rect.texture.as_deref() == Some(id),
                _ => false,
            })
        }) {
            return Err(DebugError::ResourceInUse(id.to_string()));
        }
        if inner.textures.remove(id).is_some() {
            inner.bump_scene_rev();
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Return an immutable snapshot after pruning expired groups.
    ///
    /// If the internal lock is poisoned, return an empty snapshot.
    pub fn snapshot(&self) -> Arc<DebugScene> {
        let now_tick = self.current_tick();
        let Ok(mut inner) = self.inner.lock() else {
            return Arc::new(DebugScene::empty());
        };
        prune_locked(&mut inner, now_tick);
        if let Some(scene) = &inner.cached_scene {
            return scene.clone();
        }

        let scene = Arc::new(DebugScene {
            groups: inner
                .groups
                .iter()
                .map(|(name, entry)| (name.clone(), entry.group.clone()))
                .collect(),
            meshes: inner
                .meshes
                .iter()
                .map(|(id, mesh)| (id.clone(), mesh.clone()))
                .collect(),
            textures: inner
                .textures
                .iter()
                .map(|(id, texture)| (id.clone(), texture.clone()))
                .collect(),
            revision: inner.scene_rev,
        });
        inner.cached_scene = Some(scene.clone());
        scene
    }

    /// Return the group count after pruning expired groups.
    pub fn group_count(&self) -> usize {
        self.prune_expired();
        self.inner.lock().map(|i| i.groups.len()).unwrap_or(0)
    }

    /// Live handle for one group name.
    #[must_use]
    pub fn group(&self, name: impl Into<String>) -> GroupHandle {
        GroupHandle {
            renderer: self.clone(),
            name: name.into(),
        }
    }

    /// Live handle for one mesh id.
    #[must_use]
    pub fn mesh(&self, id: impl Into<String>) -> MeshHandle {
        MeshHandle {
            renderer: self.clone(),
            id: id.into(),
        }
    }

    /// Live handle for one texture id.
    #[must_use]
    pub fn texture(&self, id: impl Into<String>) -> TextureHandle {
        TextureHandle {
            renderer: self.clone(),
            id: id.into(),
        }
    }

    /// Live handle for one item index inside a group.
    #[must_use]
    pub fn item(&self, group: impl Into<String>, index: usize) -> ItemHandle {
        let group = group.into();
        let revision = self.inner.lock().ok().and_then(|inner| {
            inner
                .groups
                .get(&group)
                .map(|entry| entry.group.geometry_revision)
        });
        ItemHandle {
            renderer: self.clone(),
            group,
            index,
            revision,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::debug::{
        BuiltinMesh, BuiltinTexture,
        types::{Color, DepthMode, Width},
    };

    fn line_list() -> DrawList {
        let mut list = DrawList::new();
        list.line(
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            Color::RED,
            Width::pixels(2.0),
            DepthMode::Test,
        );
        list
    }

    #[test]
    fn validation_rejects_bad_names_and_bounds() {
        let r = DebugRenderer::new_detached();
        assert!(matches!(
            r.replace_group("", line_list(), GroupOptions::default()),
            Err(DebugError::InvalidName(_))
        ));
        assert!(matches!(
            r.replace_group("bad name!", line_list(), GroupOptions::default()),
            Err(DebugError::InvalidName(_))
        ));
        let mut too_many = DrawList::new();
        for _ in 0..(super::super::types::MAX_ITEMS_PER_GROUP + 1) {
            too_many.point(
                [0.0, 0.0, 0.0],
                Color::WHITE,
                Width::pixels(1.0),
                DepthMode::Test,
            );
        }
        assert!(matches!(
            r.replace_group("big", too_many, GroupOptions::default()),
            Err(DebugError::TooManyItems { .. })
        ));
    }

    #[test]
    fn cached_snapshot_tracks_frame_tick_and_seconds_expiry() {
        let r = DebugRenderer::new_detached();
        r.replace_group(
            "frame",
            line_list(),
            GroupOptions::default().lifetime(Lifetime::Frame),
        )
        .unwrap();
        r.replace_group(
            "ticks",
            line_list(),
            GroupOptions::default().lifetime(Lifetime::Ticks(3)),
        )
        .unwrap();
        r.replace_group(
            "seconds",
            line_list(),
            GroupOptions::default().lifetime(Lifetime::Seconds(0.05)),
        )
        .unwrap();

        let initial = r.snapshot();
        let frame_deadline = initial.groups["frame"].expires_tick;
        let tick_deadline = initial.groups["ticks"].expires_tick;
        let seconds_deadline = initial.groups["seconds"].expires_at_ms;
        assert!(Arc::ptr_eq(&initial, &r.snapshot()));

        r.set_group_transform("seconds", GroupTransform::from_position([1.0, 0.0, 0.0]))
            .unwrap();
        let updated = r.snapshot();
        assert!(!Arc::ptr_eq(&initial, &updated));
        assert_eq!(updated.groups["frame"].expires_tick, frame_deadline);
        assert_eq!(updated.groups["ticks"].expires_tick, tick_deadline);
        assert_eq!(updated.groups["seconds"].expires_at_ms, seconds_deadline);
        assert!(Arc::ptr_eq(&updated, &r.snapshot()));

        std::thread::sleep(std::time::Duration::from_millis(275));
        let after_wall_time = r.snapshot();
        assert!(!Arc::ptr_eq(&updated, &after_wall_time));
        assert!(after_wall_time.groups.contains_key("frame"));
        assert!(after_wall_time.groups.contains_key("ticks"));
        assert!(!after_wall_time.groups.contains_key("seconds"));
        assert!(Arc::ptr_eq(&after_wall_time, &r.snapshot()));

        r.advance_tick();
        assert!(Arc::ptr_eq(&after_wall_time, &r.snapshot()));
        r.advance_tick();
        let after_frame = r.snapshot();
        assert!(!Arc::ptr_eq(&after_wall_time, &after_frame));
        assert!(!after_frame.groups.contains_key("frame"));
        assert!(after_frame.groups.contains_key("ticks"));

        r.advance_tick();
        r.advance_tick();
        let after_ticks = r.snapshot();
        assert!(!Arc::ptr_eq(&after_frame, &after_ticks));
        assert!(!after_ticks.groups.contains_key("ticks"));
    }

    #[test]
    fn stale_item_handles_cannot_mutate_shifted_items() {
        let r = DebugRenderer::new_detached();
        r.replace_group("g", line_list(), GroupOptions::default())
            .unwrap();
        let old = r.item("g", 0);
        r.item("g", 0).remove().unwrap();
        assert_eq!(old.remove(), Err(DebugError::StaleHandle));
    }

    #[test]
    fn cached_snapshots_reuse_allocations_and_invalidate_on_scene_changes() {
        let r = DebugRenderer::new_detached();
        r.insert_mesh(
            MeshResource::new(
                "tri",
                vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                vec![Color::WHITE; 3],
                None,
                None,
                vec![0, 1, 2],
            )
            .unwrap(),
        )
        .unwrap();
        r.replace_group("g", line_list(), GroupOptions::default())
            .unwrap();

        let first = r.snapshot();
        for _ in 0..16 {
            assert!(Arc::ptr_eq(&first, &r.snapshot()));
        }
        let repeated = r.snapshot();
        assert!(Arc::ptr_eq(&first.meshes["tri"], &repeated.meshes["tri"]));
        assert!(Arc::ptr_eq(&first.groups["g"], &repeated.groups["g"]));
        let inner = r.inner.lock().unwrap();
        assert_eq!(inner.deadline_scans, 0);
        drop(inner);

        let first_group = first.groups.get("g").unwrap();
        r.set_group_transform("g", GroupTransform::from_position([1.0, 0.0, 0.0]))
            .unwrap();
        let transformed = r.snapshot();
        assert!(!Arc::ptr_eq(&first, &transformed));
        assert!(!Arc::ptr_eq(first_group, &transformed.groups["g"]));
        assert!(Arc::ptr_eq(
            &first.meshes["tri"],
            &transformed.meshes["tri"]
        ));
        assert!(Arc::ptr_eq(
            &first_group.items,
            &transformed.groups["g"].items
        ));
        assert!(Arc::ptr_eq(&transformed, &r.snapshot()));

        r.clear_groups();
        let cleared = r.snapshot();
        assert!(!Arc::ptr_eq(&transformed, &cleared));
        assert!(cleared.groups.is_empty());
        assert!(Arc::ptr_eq(&cleared, &r.snapshot()));

        assert!(r.remove_mesh("tri").unwrap());
        let without_mesh = r.snapshot();
        assert!(!Arc::ptr_eq(&cleared, &without_mesh));
        assert!(without_mesh.meshes.is_empty());
        assert!(Arc::ptr_eq(&without_mesh, &r.snapshot()));
        r.clear_all();
        assert!(Arc::ptr_eq(&without_mesh, &r.snapshot()));
    }

    #[test]
    fn mesh_once_instance_transform_only() {
        let r = DebugRenderer::new_detached();
        let positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let colors = vec![Color::WHITE; 3];
        r.insert_mesh(
            MeshResource::new("tri", positions, colors, None, None, vec![0, 1, 2]).unwrap(),
        )
        .unwrap();
        let mut list = DrawList::new();
        list.instance(
            "tri",
            [0.0, 0.0, 0.0],
            GroupTransform::identity().rotation,
            [1.0; 3],
            Color::WHITE,
            None,
            DepthMode::Test,
        );
        r.replace_group("inst", list, GroupOptions::default())
            .unwrap();
        // Missing mesh is a typed error, not a silent drop.
        let mut bad = DrawList::new();
        bad.instance(
            "missing",
            [0.0, 0.0, 0.0],
            GroupTransform::identity().rotation,
            [1.0; 3],
            Color::WHITE,
            None,
            DepthMode::Test,
        );
        assert!(matches!(
            r.replace_group("bad", bad, GroupOptions::default()),
            Err(DebugError::MeshNotFound(_))
        ));

        let mut builtin = DrawList::new();
        builtin.instance(
            BuiltinMesh::Ball.id(),
            [0.0; 3],
            GroupTransform::identity().rotation,
            [1.0; 3],
            Color::WHITE,
            Some(BuiltinTexture::Ball.id().into()),
            DepthMode::Test,
        );
        r.replace_group("builtin", builtin, GroupOptions::default())
            .unwrap();
        assert!(!r.snapshot().meshes.contains_key(BuiltinMesh::Ball.id()));

        let reserved_mesh = MeshResource::new(
            BuiltinMesh::Ball.id(),
            vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            vec![Color::WHITE; 3],
            None,
            None,
            vec![0, 1, 2],
        );
        assert!(matches!(
            reserved_mesh,
            Err(DebugError::ReservedResourceId(_))
        ));
        let reserved_texture =
            TextureResource::new(BuiltinTexture::Ball.id(), 1, 1, vec![255, 255, 255, 255]);
        assert!(matches!(
            reserved_texture,
            Err(DebugError::ReservedResourceId(_))
        ));
    }
}
