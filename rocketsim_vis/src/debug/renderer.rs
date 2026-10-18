//! Cloneable [`DebugRenderer`]: retained-scene submissions accepted even
//! while disconnected or while the sim is paused.
//!
//! Short API locks (a single `Mutex` around the pending tables) versus
//! nonblocking network: submissions copy small validated records under the
//! lock; the sender thread serializes and chunks off the sim tick, and
//! [`Vis::update`](rocketsim::Vis) never touches this state (it only bumps
//! an atomic tick and overwrites the latest-only physics slot).
//!
//! TTL rule: only a full `replace_group` restarts a group's deadline.
//! `set_group_visible` / `set_group_transform` / `set_group_anchor` and
//! single-item updates preserve `created_tick` / `created_at`.

use std::{
    collections::BTreeMap,
    sync::{
        Arc, Mutex,
        atomic::{AtomicU64, Ordering},
    },
    time::Instant,
};

use super::{
    scene::{DebugGroup, DebugScene, GroupOptions, MeshResource, TextureResource},
    types::{
        Anchor, DebugError, DebugItem, DebugResult, DrawList, GroupTransform, Lifetime,
        MAX_DEBUG_BYTES, MAX_GROUPS, MAX_MESHES, MAX_TEXTURES, MAX_TOTAL_ITEMS, validate_name,
    },
};

// ---------------------------------------------------------------------------
// Shared pending tables (sender + renderer share these Arcs)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub(crate) struct GroupEntry {
    pub(crate) group: DebugGroup,
    pub(crate) created_tick: u64,
    pub(crate) created_at: Instant,
}

#[derive(Debug, Default)]
pub(crate) struct SharedInner {
    pub(crate) groups: BTreeMap<String, Arc<GroupEntry>>,
    pub(crate) meshes: BTreeMap<String, Arc<MeshResource>>,
    pub(crate) textures: BTreeMap<String, Arc<TextureResource>>,
    pub(crate) next_rev: u64,
    pub(crate) scene_rev: u64,
}

impl SharedInner {
    fn alloc_rev(&mut self) -> u64 {
        self.next_rev = self.next_rev.wrapping_add(1).max(1);
        self.scene_rev = self.scene_rev.wrapping_add(1).max(1);
        self.next_rev
    }
}

/// Cheap cloneable snapshot of pending tables (Arcs, no deep copy).
#[derive(Clone, Debug, Default)]
pub(crate) struct PendingSnapshot {
    pub groups: BTreeMap<String, Arc<GroupEntry>>,
    pub meshes: BTreeMap<String, Arc<MeshResource>>,
    pub textures: BTreeMap<String, Arc<TextureResource>>,
    pub scene_rev: u64,
}

// ---------------------------------------------------------------------------
// Renderer handle
// ---------------------------------------------------------------------------

/// Cloneable retained-scene handle obtained from
/// [`ArenaVisExt::debug_renderer`](crate::ArenaVisExt::debug_renderer)
/// after attach.
///
/// All submissions validate and return [`DebugError`] on rejection
/// (never silently dropped, never panics). Accepted changes coalesce in
/// bounded pending tables: repeated replaces collapse to the newest value,
/// and removals derive from set differences (no unbounded queues, no
/// tombstone history).
#[derive(Clone, Debug)]
pub struct DebugRenderer {
    inner: Arc<Mutex<SharedInner>>,
    tick: Arc<AtomicU64>,
}

impl DebugRenderer {
    /// Create a detached renderer (tests / local preview without `NetVis`).
    /// The tick starts at `0` and only advances via [`Self::advance_tick`].
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

    /// Current physics tick (bumped by `Vis::update`, or manually in tests).
    #[must_use]
    pub fn current_tick(&self) -> u64 {
        self.tick.load(Ordering::Acquire)
    }

    /// Advance the tick by one (tests; `NetVis` advances automatically).
    pub fn advance_tick(&self) {
        self.tick.fetch_add(1, Ordering::AcqRel);
    }

    /// Remove expired groups now (also runs lazily on submit/snapshot).
    /// Returns the number removed.
    pub fn prune_expired(&self) -> usize {
        let now_tick = self.current_tick();
        let now = Instant::now();
        let Ok(mut inner) = self.inner.lock() else {
            return 0;
        };
        prune_locked(&mut inner, now_tick, now)
    }

    /// Atomically replace (or create) a named group.
    ///
    /// Returns the group's new revision. Referenced mesh/texture ids must
    /// already exist (insert assets first); otherwise [`DebugError`] is
    /// returned and nothing changes.
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
        prune_locked(&mut inner, now_tick, now);
        // Reference check (where feasible): instances/textures must exist.
        for item in &list.items {
            match item {
                DebugItem::Instance(v) => {
                    if !inner.meshes.contains_key(&v.mesh) {
                        return Err(DebugError::MeshNotFound(v.mesh.clone()));
                    }
                    if let Some(t) = &v.texture
                        && !inner.textures.contains_key(t)
                    {
                        return Err(DebugError::TextureNotFound(t.clone()));
                    }
                }
                DebugItem::Mesh(v) => {
                    if let Some(t) = &v.texture
                        && !inner.textures.contains_key(t)
                    {
                        return Err(DebugError::TextureNotFound(t.clone()));
                    }
                }
                DebugItem::Rect(v) => {
                    if let Some(t) = &v.texture
                        && !inner.textures.contains_key(t)
                    {
                        return Err(DebugError::TextureNotFound(t.clone()));
                    }
                }
                _ => {}
            }
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
        let rev = inner.alloc_rev();
        let mut group = DebugGroup::new(name, list, opts)?;
        group.revision = rev;
        group.geometry_revision = rev;
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
        inner.groups.insert(
            name.to_string(),
            Arc::new(GroupEntry {
                group,
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
        if inner.groups.remove(name).is_some() {
            inner.scene_rev = inner.scene_rev.wrapping_add(1).max(1);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Remove all groups, meshes, and textures.
    pub fn clear_all(&self) {
        if let Ok(mut inner) = self.inner.lock() {
            inner.groups.clear();
            inner.meshes.clear();
            inner.textures.clear();
            inner.scene_rev = inner.scene_rev.wrapping_add(1).max(1);
        }
    }

    /// Remove all groups but keep reusable assets.
    pub fn clear_groups(&self) {
        if let Ok(mut inner) = self.inner.lock()
            && !inner.groups.is_empty()
        {
            inner.groups.clear();
            inner.scene_rev = inner.scene_rev.wrapping_add(1).max(1);
        }
    }

    /// Toggle visibility without restarting TTL.
    pub fn set_group_visible(&self, name: &str, visible: bool) -> DebugResult<()> {
        self.mutate_group(name, |entry, inner| {
            if entry.group.visible == visible {
                return;
            }
            let mut group = entry.group.clone();
            group.revision = inner.alloc_rev();
            group.visible = visible;
            *entry = GroupEntry {
                group,
                created_tick: entry.created_tick,
                created_at: entry.created_at,
            };
        })
    }

    /// Replace the group transform without restarting TTL.
    pub fn set_group_transform(&self, name: &str, t: GroupTransform) -> DebugResult<()> {
        t.validate()?;
        self.mutate_group(name, |entry, inner| {
            let mut group = entry.group.clone();
            group.revision = inner.alloc_rev();
            group.transform = t.clone();
            *entry = GroupEntry {
                group,
                created_tick: entry.created_tick,
                created_at: entry.created_at,
            };
        })
    }

    /// Replace the group anchor without restarting TTL.
    pub fn set_group_anchor(&self, name: &str, anchor: Anchor) -> DebugResult<()> {
        anchor.validate()?;
        self.mutate_group(name, |entry, inner| {
            let mut group = entry.group.clone();
            group.revision = inner.alloc_rev();
            group.anchor = anchor.clone();
            *entry = GroupEntry {
                group,
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

    /// Remove a reusable mesh. Instances referencing it are skipped by renderers.
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
            inner.scene_rev = inner.scene_rev.wrapping_add(1).max(1);
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

    /// Remove a reusable texture.
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
            inner.scene_rev = inner.scene_rev.wrapping_add(1).max(1);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Atomic immutable snapshot (prunes expired first).
    pub fn snapshot(&self) -> Arc<DebugScene> {
        let now_tick = self.current_tick();
        let now = Instant::now();
        let (groups, meshes, textures, scene_rev) = match self.inner.lock() {
            Ok(mut inner) => {
                prune_locked(&mut inner, now_tick, now);
                let groups = inner
                    .groups
                    .iter()
                    .map(|(k, e)| (k.clone(), Arc::new(e.group.clone())))
                    .collect();
                let meshes = inner
                    .meshes
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone()))
                    .collect();
                let textures = inner
                    .textures
                    .iter()
                    .map(|(k, v)| (k.clone(), v.clone()))
                    .collect();
                (groups, meshes, textures, inner.scene_rev)
            }
            Err(_) => (BTreeMap::new(), BTreeMap::new(), BTreeMap::new(), 0),
        };
        Arc::new(DebugScene {
            groups,
            meshes,
            textures,
            revision: scene_rev,
        })
    }

    /// Cheap pending-table clone for the sender thread (prunes expired).
    pub(crate) fn clone_pending(&self) -> PendingSnapshot {
        let now_tick = self.current_tick();
        let now = Instant::now();
        match self.inner.lock() {
            Ok(mut inner) => {
                prune_locked(&mut inner, now_tick, now);
                PendingSnapshot {
                    groups: inner.groups.clone(),
                    meshes: inner.meshes.clone(),
                    textures: inner.textures.clone(),
                    scene_rev: inner.scene_rev,
                }
            }
            Err(_) => PendingSnapshot::default(),
        }
    }

    /// Number of groups (prunes expired first).
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

pub(crate) fn prune_locked(inner: &mut SharedInner, now_tick: u64, now: Instant) -> usize {
    let before = inner.groups.len();
    inner.groups.retain(|_, e| !is_expired(e, now_tick, now));
    let removed = before.saturating_sub(inner.groups.len());
    if removed > 0 {
        inner.scene_rev = inner.scene_rev.wrapping_add(1).max(1);
    }
    removed
}

pub(crate) fn is_expired(entry: &GroupEntry, now_tick: u64, _now: Instant) -> bool {
    entry
        .group
        .expires_tick
        .is_some_and(|deadline| now_tick >= deadline)
        || entry
            .group
            .expires_at_ms
            .is_some_and(|deadline| unix_time_ms() >= deadline)
}

fn unix_time_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
        .min(u128::from(u64::MAX)) as u64
}

// ---------------------------------------------------------------------------
// Handles
// ---------------------------------------------------------------------------

/// Live handle for one named group (all ops preserve TTL except `replace`).
#[derive(Clone, Debug)]
pub struct GroupHandle {
    renderer: DebugRenderer,
    name: String,
}

impl GroupHandle {
    /// Atomic replace (restarts TTL with `opts.lifetime`).
    pub fn replace(&self, list: DrawList, opts: GroupOptions) -> DebugResult<u64> {
        self.renderer.replace_group(&self.name, list, opts)
    }

    /// Remove the group (idempotent).
    pub fn clear(&self) -> DebugResult<bool> {
        self.renderer.clear_group(&self.name)
    }

    /// Visibility update (preserves TTL).
    pub fn set_visible(&self, visible: bool) -> DebugResult<()> {
        self.renderer.set_group_visible(&self.name, visible)
    }

    /// Transform update (preserves TTL).
    pub fn set_transform(&self, t: GroupTransform) -> DebugResult<()> {
        self.renderer.set_group_transform(&self.name, t)
    }

    /// Anchor update (preserves TTL).
    pub fn set_anchor(&self, anchor: Anchor) -> DebugResult<()> {
        self.renderer.set_group_anchor(&self.name, anchor)
    }

    /// Group name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }
}

/// Live handle for one reusable mesh id.
#[derive(Clone, Debug)]
pub struct MeshHandle {
    renderer: DebugRenderer,
    id: String,
}

impl MeshHandle {
    /// Insert or replace the mesh resource.
    pub fn insert(&self, res: MeshResource) -> DebugResult<u64> {
        if res.id != self.id {
            return Err(DebugError::ResourceIdMismatch {
                expected: self.id.clone(),
                actual: res.id,
            });
        }
        self.renderer.insert_mesh(res)
    }

    /// Remove the mesh (idempotent).
    pub fn remove(&self) -> DebugResult<bool> {
        self.renderer.remove_mesh(&self.id)
    }

    /// Mesh id.
    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }
}

/// Live handle for one reusable texture id.
#[derive(Clone, Debug)]
pub struct TextureHandle {
    renderer: DebugRenderer,
    id: String,
}

impl TextureHandle {
    /// Insert or replace the texture resource.
    pub fn insert(&self, res: TextureResource) -> DebugResult<u64> {
        if res.id != self.id {
            return Err(DebugError::ResourceIdMismatch {
                expected: self.id.clone(),
                actual: res.id,
            });
        }
        self.renderer.insert_texture(res)
    }

    /// Remove the texture (idempotent).
    pub fn remove(&self) -> DebugResult<bool> {
        self.renderer.remove_texture(&self.id)
    }

    /// Texture id.
    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }
}

/// Live handle for one item index inside a group (preserves group TTL).
#[derive(Clone, Debug)]
pub struct ItemHandle {
    renderer: DebugRenderer,
    group: String,
    index: usize,
    revision: Option<u64>,
}

impl ItemHandle {
    /// Replace one item in place (preserves group TTL).
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
        // Reference check for the replacement item.
        match &item {
            DebugItem::Instance(v) => {
                if !inner.meshes.contains_key(&v.mesh) {
                    return Err(DebugError::MeshNotFound(v.mesh.clone()));
                }
            }
            DebugItem::Mesh(v) => {
                if let Some(t) = &v.texture
                    && !inner.textures.contains_key(t)
                {
                    return Err(DebugError::TextureNotFound(t.clone()));
                }
            }
            DebugItem::Rect(v) => {
                if let Some(t) = &v.texture
                    && !inner.textures.contains_key(t)
                {
                    return Err(DebugError::TextureNotFound(t.clone()));
                }
            }
            _ => {}
        }
        Arc::make_mut(&mut entry.group.items)[index] = item;
        entry.group.revision = inner.alloc_rev();
        entry.group.geometry_revision = entry.group.revision;
        entry.group.validate()?;
        let bytes = entry.group.estimated_bytes()
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
        inner.groups.insert(group, Arc::new(entry));
        Ok(())
    }

    /// Remove one item in place (preserves group TTL).
    /// Handles are conservatively invalidated by any geometry edit.
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
        Arc::make_mut(&mut entry.group.items).remove(index);
        entry.group.revision = inner.alloc_rev();
        entry.group.geometry_revision = entry.group.revision;
        inner.groups.insert(group, Arc::new(entry));
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::debug::types::{Color, DepthMode, Width};

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
    fn transform_update_preserves_ttl() {
        let r = DebugRenderer::new_detached();
        let rev0 = r
            .replace_group(
                "g",
                line_list(),
                GroupOptions::default().lifetime(Lifetime::Ticks(100)),
            )
            .unwrap();
        r.advance_tick();
        r.set_group_transform("t-missing", GroupTransform::identity())
            .unwrap_err();
        r.set_group_transform("g", GroupTransform::from_position([1.0, 2.0, 3.0]))
            .unwrap();
        let snap = r.snapshot();
        let g = snap.get_group("g").unwrap();
        assert!(g.revision != rev0);
        assert_eq!(g.lifetime, Lifetime::Ticks(100));
        // Only one tick advanced; Ticks(100) must survive transform updates.
        assert_eq!(r.group_count(), 1);
        for _ in 0..200 {
            r.advance_tick();
        }
        assert_eq!(r.prune_expired(), 1);
        assert_eq!(r.group_count(), 0);
    }

    #[test]
    fn frame_lifetime_waits_for_following_sim_tick_not_wall_time() {
        let r = DebugRenderer::new_detached();
        r.replace_group(
            "frame",
            line_list(),
            GroupOptions::default().lifetime(Lifetime::Frame),
        )
        .unwrap();
        r.advance_tick();
        std::thread::sleep(std::time::Duration::from_millis(275));
        assert_eq!(r.group_count(), 1);
        r.advance_tick();
        assert_eq!(r.group_count(), 0);
    }

    #[test]
    fn tick_lifetime_uses_absolute_deadline_across_skipped_ticks() {
        let r = DebugRenderer::new_detached();
        r.replace_group(
            "ticks",
            line_list(),
            GroupOptions::default().lifetime(Lifetime::Ticks(3)),
        )
        .unwrap();
        for _ in 0..4 {
            r.advance_tick();
        }
        assert_eq!(r.prune_expired(), 1);
    }

    #[test]
    fn seconds_deadline_survives_transform_updates() {
        let r = DebugRenderer::new_detached();
        r.replace_group(
            "seconds",
            line_list(),
            GroupOptions::default().lifetime(Lifetime::Seconds(10.0)),
        )
        .unwrap();
        let before = r.snapshot().groups["seconds"].expires_at_ms;
        r.set_group_transform("seconds", GroupTransform::from_position([1.0, 0.0, 0.0]))
            .unwrap();
        assert_eq!(r.snapshot().groups["seconds"].expires_at_ms, before);
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
    fn snapshots_share_resource_and_group_allocations() {
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
        let second = r.snapshot();
        assert!(Arc::ptr_eq(&first.meshes["tri"], &second.meshes["tri"]));

        assert!(Arc::ptr_eq(
            &first.groups["g"].items,
            &second.groups["g"].items
        ));
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
    }
}
