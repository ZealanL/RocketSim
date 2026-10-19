//! Produce, apply, and assemble retained-scene updates.
//!
//! This module contains no socket code. Publishers send a full scene after a
//! connection, then reliable deltas. Receivers assemble chunks before they
//! apply a transfer. Physics frames can use the same stream between chunks.

use std::{collections::BTreeMap, fmt, sync::Arc};

use crate::{
    debug::{DebugGroup, DebugScene},
    proto::{
        DebugChunk, DebugDelta, DebugFull, DebugGroupPatch, DebugSyncMsg, MAX_DEBUG_SYNC_BYTES,
        ProtocolError, decode_debug_sync,
    },
};

/// Errors from retained-scene assembly or application.
#[derive(Debug)]
pub enum SyncError {
    /// A chunk has invalid transfer framing or exceeds the transfer limit.
    InvalidChunk(&'static str),
    /// A delta refers to a group that does not exist.
    MissingGroup(String),
    /// The resulting scene does not pass validation.
    InvalidScene,
    /// The scene lock was poisoned.
    Poisoned,
    /// A completed transfer has invalid protocol data.
    Protocol(ProtocolError),
}

impl fmt::Display for SyncError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidChunk(reason) => write!(f, "invalid retained-scene chunks: {reason}"),
            Self::MissingGroup(name) => {
                write!(f, "retained-scene patch refers to missing group {name}")
            }
            Self::InvalidScene => f.write_str("retained-scene data failed validation"),
            Self::Poisoned => f.write_str("retained-scene lock was poisoned"),
            Self::Protocol(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for SyncError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Protocol(error) => Some(error),
            _ => None,
        }
    }
}

impl From<ProtocolError> for SyncError {
    fn from(error: ProtocolError) -> Self {
        Self::Protocol(error)
    }
}

/// Per-connection revisions used to produce retained-scene updates.
///
/// Reset this value on reconnect. The first non-empty publication is a full
/// scene. Later publications contain only changes and removals.
#[derive(Debug, Default)]
pub(crate) struct SentDebug {
    groups: BTreeMap<String, (u64, u64)>,
    meshes: BTreeMap<String, u64>,
    textures: BTreeMap<String, u64>,
    pub(crate) full_sent: bool,
    pub(crate) transfer_id: u64,
}

/// Build the next retained-scene publication for the pending tables.
pub(crate) fn next_debug_sync(pending: &DebugScene, sent: &SentDebug) -> Option<DebugSyncMsg> {
    if !sent.full_sent {
        if pending.groups.is_empty() && pending.meshes.is_empty() && pending.textures.is_empty() {
            return None;
        }
        return Some(DebugSyncMsg::Full(DebugFull {
            groups: pending
                .groups
                .values()
                .map(|group| (**group).clone())
                .collect(),
            meshes: pending
                .meshes
                .values()
                .map(|value| (**value).clone())
                .collect(),
            textures: pending
                .textures
                .values()
                .map(|value| (**value).clone())
                .collect(),
            rev: pending.revision,
        }));
    }

    let mut delta = DebugDelta::default();
    for (name, entry) in &pending.groups {
        match sent.groups.get(name) {
            None => delta.upsert_groups.push((**entry).clone()),
            Some((_, geometry_revision)) if *geometry_revision != entry.geometry_revision => {
                delta.upsert_groups.push((**entry).clone());
            }
            Some((revision, _)) if *revision != entry.revision => {
                delta.group_patches.push(DebugGroupPatch {
                    name: name.clone(),
                    visible: entry.visible,
                    anchor: entry.anchor.clone(),
                    transform: entry.transform.clone(),
                    lifetime: entry.lifetime,
                    expires_tick: entry.expires_tick,
                    expires_at_ms: entry.expires_at_ms,
                    revision: entry.revision,
                });
            }
            _ => {}
        }
    }
    for name in sent.groups.keys() {
        if !pending.groups.contains_key(name) {
            delta.removed_groups.push(name.clone());
        }
    }
    for (id, resource) in &pending.meshes {
        if sent.meshes.get(id) != Some(&resource.revision) {
            delta.upsert_meshes.push((**resource).clone());
        }
    }
    for id in sent.meshes.keys() {
        if !pending.meshes.contains_key(id) {
            delta.removed_meshes.push(id.clone());
        }
    }
    for (id, resource) in &pending.textures {
        if sent.textures.get(id) != Some(&resource.revision) {
            delta.upsert_textures.push((**resource).clone());
        }
    }
    for id in sent.textures.keys() {
        if !pending.textures.contains_key(id) {
            delta.removed_textures.push(id.clone());
        }
    }
    if delta.is_empty() {
        None
    } else {
        delta.rev = pending.revision;
        Some(DebugSyncMsg::Delta(delta))
    }
}

/// Record pending tables after their publication is queued.
pub(crate) fn mark_debug_sent(sent: &mut SentDebug, pending: &DebugScene) {
    sent.groups.clear();
    sent.meshes.clear();
    sent.textures.clear();
    for (name, entry) in &pending.groups {
        sent.groups
            .insert(name.clone(), (entry.revision, entry.geometry_revision));
    }
    for (id, resource) in &pending.meshes {
        sent.meshes.insert(id.clone(), resource.revision);
    }
    for (id, resource) in &pending.textures {
        sent.textures.insert(id.clone(), resource.revision);
    }
    sent.full_sent = true;
    sent.transfer_id = sent.transfer_id.wrapping_add(1).max(1);
}

/// Assemble one retained-scene transfer from multiplexed socket chunks.
#[derive(Default)]
pub(crate) struct ChunkAssembler {
    current: Option<Transfer>,
}

struct Transfer {
    transfer_id: u64,
    total: usize,
    parts: Vec<Option<Vec<u8>>>,
    received: usize,
    byte_len: usize,
}

impl ChunkAssembler {
    /// Add one chunk. Return a complete, validated transfer when all parts exist.
    pub(crate) fn push(&mut self, chunk: DebugChunk) -> Result<Option<DebugSyncMsg>, SyncError> {
        chunk.validate_framing()?;
        let total = chunk.total as usize;
        let index = chunk.index as usize;
        let new_transfer = self
            .current
            .as_ref()
            .is_none_or(|transfer| transfer.transfer_id != chunk.transfer_id);
        if new_transfer {
            self.current = Some(Transfer {
                transfer_id: chunk.transfer_id,
                total,
                parts: (0..total).map(|_| None).collect(),
                received: 0,
                byte_len: 0,
            });
        }
        let transfer = self.current.as_mut().expect("transfer was initialized");
        if transfer.total != total || index >= transfer.total {
            return Err(SyncError::InvalidChunk("chunk count or index changed"));
        }
        if let Some(existing) = &transfer.parts[index] {
            if existing != &chunk.bytes {
                return Err(SyncError::InvalidChunk(
                    "duplicate chunk has different bytes",
                ));
            }
        } else {
            transfer.byte_len = transfer.byte_len.saturating_add(chunk.bytes.len());
            if transfer.byte_len > MAX_DEBUG_SYNC_BYTES {
                return Err(SyncError::InvalidChunk("transfer exceeds byte limit"));
            }
            transfer.parts[index] = Some(chunk.bytes);
            transfer.received += 1;
        }
        if transfer.received != transfer.total {
            return Ok(None);
        }
        let transfer = self.current.take().expect("completed transfer exists");
        let mut bytes = Vec::with_capacity(transfer.byte_len);
        for part in transfer.parts {
            bytes.extend_from_slice(&part.expect("all transfer parts exist"));
        }
        decode_debug_sync(&bytes).map(Some).map_err(SyncError::from)
    }
}

/// Apply a full scene or delta to a canonical scene.
pub(crate) fn apply_sync(
    current: &DebugScene,
    message: DebugSyncMsg,
) -> Result<DebugScene, SyncError> {
    let next = match message {
        DebugSyncMsg::Full(full) => apply_full(full)?,
        DebugSyncMsg::Delta(delta) => apply_delta(current, delta)?,
    };
    next.validate().map_err(|_| SyncError::InvalidScene)?;
    Ok(next)
}

fn apply_full(full: DebugFull) -> Result<DebugScene, SyncError> {
    let scene = DebugScene {
        groups: full
            .groups
            .into_iter()
            .map(|group| (group.name.clone(), Arc::new(group)))
            .collect(),
        meshes: full
            .meshes
            .into_iter()
            .map(|mesh| (mesh.id.clone(), Arc::new(mesh)))
            .collect(),
        textures: full
            .textures
            .into_iter()
            .map(|texture| (texture.id.clone(), Arc::new(texture)))
            .collect(),
        revision: full.rev.max(1),
    };
    scene.validate().map_err(|_| SyncError::InvalidScene)?;
    Ok(scene)
}

fn apply_delta(current: &DebugScene, delta: DebugDelta) -> Result<DebugScene, SyncError> {
    let mut next = current.clone();
    for group in delta.upsert_groups {
        next.groups.insert(group.name.clone(), Arc::new(group));
    }
    for patch in delta.group_patches {
        let Some(existing) = next.groups.get(&patch.name) else {
            return Err(SyncError::MissingGroup(patch.name));
        };
        let mut group: DebugGroup = (**existing).clone();
        apply_group_patch(&mut group, patch);
        next.groups.insert(group.name.clone(), Arc::new(group));
    }
    for name in delta.removed_groups {
        next.groups.remove(&name);
    }
    for mesh in delta.upsert_meshes {
        next.meshes.insert(mesh.id.clone(), Arc::new(mesh));
    }
    for id in delta.removed_meshes {
        next.meshes.remove(&id);
    }
    for texture in delta.upsert_textures {
        next.textures.insert(texture.id.clone(), Arc::new(texture));
    }
    for id in delta.removed_textures {
        next.textures.remove(&id);
    }
    next.revision = delta.rev.max(1);
    Ok(next)
}

fn apply_group_patch(group: &mut DebugGroup, patch: DebugGroupPatch) {
    group.visible = patch.visible;
    group.anchor = patch.anchor;
    group.transform = patch.transform;
    group.lifetime = patch.lifetime;
    group.expires_tick = patch.expires_tick;
    group.expires_at_ms = patch.expires_at_ms;
    group.revision = patch.revision;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        debug::{Color, DebugGroup, DepthMode, DrawList, GroupOptions, Lifetime, Width},
        proto::SimMsg,
    };

    fn group(name: &str) -> DebugGroup {
        let mut list = DrawList::new();
        list.line(
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            Color::WHITE,
            Width::pixels(1.0),
            DepthMode::Test,
        );
        DebugGroup::new(name, list, GroupOptions::default()).unwrap()
    }

    #[test]
    fn full_and_delta_apply_atomically_with_removals() {
        let base = DebugScene::empty();
        let full = DebugSyncMsg::Full(DebugFull {
            groups: vec![group("keep"), group("remove")],
            rev: 1,
            ..Default::default()
        });
        let first = apply_sync(&base, full).unwrap();
        let delta = DebugSyncMsg::Delta(DebugDelta {
            upsert_groups: vec![group("new")],
            removed_groups: vec!["remove".into()],
            rev: 2,
            ..Default::default()
        });
        let next = apply_sync(&first, delta).unwrap();
        assert!(next.groups.contains_key("keep"));
        assert!(next.groups.contains_key("new"));
        assert!(!next.groups.contains_key("remove"));
        assert!(first.groups.contains_key("remove"));
    }

    #[test]
    fn chunk_assembly_is_bounded_and_decodes_only_after_completion() {
        let message = DebugSyncMsg::Full(DebugFull {
            groups: vec![group("chunked")],
            rev: 3,
            ..Default::default()
        });
        let encoded = crate::proto::encode_debug_sync(&message).unwrap();
        let sim_chunks = crate::proto::split_debug_sync(&encoded, 9);
        let mut assembler = ChunkAssembler::default();
        let mut result = None;
        for sim_chunk in sim_chunks {
            let SimMsg::DebugChunk(chunk) = sim_chunk else {
                unreachable!()
            };
            result = assembler.push(chunk).unwrap().or(result);
        }
        assert!(matches!(result, Some(DebugSyncMsg::Full(_))));
    }

    #[test]
    fn delta_patch_to_missing_group_is_rejected() {
        let delta = DebugSyncMsg::Delta(DebugDelta {
            group_patches: vec![DebugGroupPatch {
                name: "missing".into(),
                visible: true,
                anchor: crate::debug::Anchor::World,
                transform: crate::debug::GroupTransform::identity(),
                lifetime: Lifetime::Persistent,
                expires_tick: None,
                expires_at_ms: None,
                revision: 2,
            }],
            rev: 2,
            ..Default::default()
        });
        assert!(matches!(
            apply_sync(&DebugScene::empty(), delta),
            Err(SyncError::MissingGroup(_))
        ));
    }
}
