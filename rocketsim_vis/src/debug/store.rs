use std::{collections::BTreeMap, sync::Arc, time::Instant};

use super::{
    resources::{MeshResource, TextureResource, is_builtin_mesh_id, is_builtin_texture_id},
    scene::{DebugGroup, DebugScene},
    types::{DebugError, DebugItem, DebugResult},
};

#[derive(Clone, Debug)]
pub(super) struct GroupEntry {
    pub(super) group: Arc<DebugGroup>,
    pub(super) created_tick: u64,
    pub(super) created_at: Instant,
}

#[derive(Debug, Default)]
pub(super) struct SharedInner {
    pub(super) groups: BTreeMap<String, Arc<GroupEntry>>,
    pub(super) meshes: BTreeMap<String, Arc<MeshResource>>,
    pub(super) textures: BTreeMap<String, Arc<TextureResource>>,
    pub(super) next_rev: u64,
    pub(super) scene_rev: u64,
    pub(super) cached_scene: Option<Arc<DebugScene>>,
    tick_deadlines: BTreeMap<u64, usize>,
    time_deadlines_ms: BTreeMap<u64, usize>,
    #[cfg(test)]
    pub(super) deadline_scans: usize,
}

impl SharedInner {
    pub(super) fn alloc_rev(&mut self) -> u64 {
        self.next_rev = self.next_rev.wrapping_add(1).max(1);
        self.bump_scene_rev();
        self.next_rev
    }

    pub(super) fn bump_scene_rev(&mut self) {
        self.scene_rev = self.scene_rev.wrapping_add(1).max(1);
        self.cached_scene = None;
    }

    pub(super) fn track_group_deadlines(&mut self, group: &DebugGroup) {
        add_deadline(&mut self.tick_deadlines, group.expires_tick);
        add_deadline(&mut self.time_deadlines_ms, group.expires_at_ms);
    }

    pub(super) fn remove_group_deadlines(&mut self, group: &DebugGroup) {
        remove_deadline(&mut self.tick_deadlines, group.expires_tick);
        remove_deadline(&mut self.time_deadlines_ms, group.expires_at_ms);
    }

    pub(super) fn clear_group_deadlines(&mut self) {
        self.tick_deadlines.clear();
        self.time_deadlines_ms.clear();
    }

    fn earliest_tick_deadline(&self) -> Option<u64> {
        self.tick_deadlines
            .first_key_value()
            .map(|(&deadline, _)| deadline)
    }

    fn earliest_time_deadline_ms(&self) -> Option<u64> {
        self.time_deadlines_ms
            .first_key_value()
            .map(|(&deadline, _)| deadline)
    }
}

fn add_deadline(deadlines: &mut BTreeMap<u64, usize>, deadline: Option<u64>) {
    if let Some(deadline) = deadline {
        *deadlines.entry(deadline).or_default() += 1;
    }
}

fn remove_deadline(deadlines: &mut BTreeMap<u64, usize>, deadline: Option<u64>) {
    let Some(deadline) = deadline else {
        return;
    };
    if deadlines.get(&deadline).is_some_and(|count| *count <= 1) {
        deadlines.remove(&deadline);
    } else if let Some(count) = deadlines.get_mut(&deadline) {
        *count -= 1;
    }
}

pub(super) fn validate_item_resources(
    item: &DebugItem,
    meshes: &BTreeMap<String, Arc<MeshResource>>,
    textures: &BTreeMap<String, Arc<TextureResource>>,
) -> DebugResult<()> {
    let check_texture = |id: &str| {
        if is_builtin_texture_id(id) || textures.contains_key(id) {
            Ok(())
        } else {
            Err(DebugError::TextureNotFound(id.to_owned()))
        }
    };
    match item {
        DebugItem::Instance(instance) => {
            if !is_builtin_mesh_id(&instance.mesh) && !meshes.contains_key(&instance.mesh) {
                return Err(DebugError::MeshNotFound(instance.mesh.clone()));
            }
            if let Some(texture) = &instance.texture {
                check_texture(texture)?;
            }
        }
        DebugItem::Mesh(mesh) => {
            if let Some(texture) = &mesh.texture {
                check_texture(texture)?;
            }
        }
        DebugItem::Rect(rect) => {
            if let Some(texture) = &rect.texture {
                check_texture(texture)?;
            }
        }
        _ => {}
    }
    Ok(())
}

pub(super) fn prune_locked(inner: &mut SharedInner, now_tick: u64) -> usize {
    let now_ms = inner.earliest_time_deadline_ms().map(|_| unix_time_ms());
    let tick_due = inner
        .earliest_tick_deadline()
        .is_some_and(|deadline| now_tick >= deadline);
    let time_due = inner
        .earliest_time_deadline_ms()
        .zip(now_ms)
        .is_some_and(|(deadline, now)| now >= deadline);
    if !tick_due && !time_due {
        return 0;
    }

    #[cfg(test)]
    {
        inner.deadline_scans += 1;
    }

    let expired = inner
        .groups
        .iter()
        .filter(|(_, entry)| is_expired(entry, now_tick, now_ms))
        .map(|(name, _)| name.clone())
        .collect::<Vec<_>>();
    for name in &expired {
        if let Some(entry) = inner.groups.remove(name) {
            inner.remove_group_deadlines(&entry.group);
        }
    }
    if !expired.is_empty() {
        inner.bump_scene_rev();
    }
    expired.len()
}

fn is_expired(entry: &GroupEntry, now_tick: u64, now_ms: Option<u64>) -> bool {
    entry
        .group
        .expires_tick
        .is_some_and(|deadline| now_tick >= deadline)
        || entry
            .group
            .expires_at_ms
            .zip(now_ms)
            .is_some_and(|(deadline, now)| now >= deadline)
}

pub(super) fn unix_time_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
        .min(u128::from(u64::MAX)) as u64
}
