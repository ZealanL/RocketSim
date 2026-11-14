//! Packed shared static AABB bounds for single-cell ray queries.
//!
//! `ArenaMemWeightMode::Light` uses one broadphase cell. That cell holds every
//! static proxy. A wheel packet tests all static proxies on each tick.
//!
//! This module stores four static boxes per block in structure-of-arrays form.
//! A ray query compares all four boxes against the query AABB with `Vec4`
//! operations. It reads the full proxy only when a lane hits.
//!
//! The block order is the cell `static_handles` insertion order. A hit
//! processes in ascending lane order. This order matches the unpacked scan.
//! The table holds no mutable arena data. Arenas with the same static handles
//! and the same AABB bits share one immutable table.

use std::sync::{Arc, Mutex, OnceLock};

use glam::Vec4;

use super::BroadphaseProxy;
use crate::shared::Aabb;

/// Boxes in one packed block.
const LANES: usize = 4;

/// Maximum number of shared tables to retain.
const MAX_CACHED_TABLES: usize = 8;

/// Four static boxes in structure-of-arrays form.
#[derive(Clone, Copy)]
struct StaticBlock {
    min_x: Vec4,
    max_x: Vec4,
    min_y: Vec4,
    max_y: Vec4,
    min_z: Vec4,
    max_z: Vec4,
    /// Logical broadphase handle for each lane.
    handles: [u32; LANES],
    /// Bit mask of the lanes that hold a real handle.
    valid_lanes: u32,
}

/// Exact identity of one cell static set.
///
/// The key records the handle order and the six AABB component bit patterns.
/// It compares named fields only. It does not compare struct padding.
#[derive(Clone, PartialEq, Eq)]
struct StaticKey {
    entries: Vec<KeyEntry>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
struct KeyEntry {
    handle: u32,
    min: [u32; 3],
    max: [u32; 3],
}

impl StaticKey {
    fn from_cell(static_handles: &[usize], proxies: &[BroadphaseProxy]) -> Self {
        let entries = static_handles
            .iter()
            .map(|&handle| {
                let aabb = proxies[handle].aabb;
                KeyEntry {
                    handle: u32::try_from(handle).expect("broadphase handle fits in u32"),
                    min: [
                        aabb.min.x.to_bits(),
                        aabb.min.y.to_bits(),
                        aabb.min.z.to_bits(),
                    ],
                    max: [
                        aabb.max.x.to_bits(),
                        aabb.max.y.to_bits(),
                        aabb.max.z.to_bits(),
                    ],
                }
            })
            .collect();

        Self { entries }
    }
}

/// Immutable packed static bounds for one cell.
pub(super) struct PackedStatic {
    key: StaticKey,
    blocks: Box<[StaticBlock]>,
}

impl PackedStatic {
    /// Build the table, or reuse an identical shared table.
    ///
    /// The cache is bounded. It holds geometry only. It never holds an arena,
    /// a proxy, or a mutable state block.
    pub(super) fn build_shared(static_handles: &[usize], proxies: &[BroadphaseProxy]) -> Arc<Self> {
        let key = StaticKey::from_cell(static_handles, proxies);
        let mut cache = table_cache()
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);

        if let Some(existing) = cache.iter().find(|table| table.key == key) {
            return Arc::clone(existing);
        }

        let table = Arc::new(Self {
            blocks: build_blocks(static_handles, proxies),
            key,
        });

        if cache.len() >= MAX_CACHED_TABLES {
            cache.remove(0);
        }
        cache.push(Arc::clone(&table));

        table
    }

    /// Call `on_hit` for each static handle that overlaps `query`.
    ///
    /// The call order is the cell insertion order. The inclusive AABB test is
    /// the same test as [`Aabb::intersects`].
    #[inline]
    pub(super) fn for_each_hit(&self, query: &Aabb, mut on_hit: impl FnMut(u32)) {
        let query_min_x = Vec4::splat(query.min.x);
        let query_min_y = Vec4::splat(query.min.y);
        let query_min_z = Vec4::splat(query.min.z);
        let query_max_x = Vec4::splat(query.max.x);
        let query_max_y = Vec4::splat(query.max.y);
        let query_max_z = Vec4::splat(query.max.z);

        for block in self.blocks.iter() {
            let hit = block.min_x.cmple(query_max_x)
                & block.max_x.cmpge(query_min_x)
                & block.min_y.cmple(query_max_y)
                & block.max_y.cmpge(query_min_y)
                & block.min_z.cmple(query_max_z)
                & block.max_z.cmpge(query_min_z);

            let mut lanes = hit.bitmask() & block.valid_lanes;
            while lanes != 0 {
                let lane = lanes.trailing_zeros() as usize;
                lanes &= lanes - 1;
                on_hit(block.handles[lane]);
            }
        }
    }
}

fn build_blocks(static_handles: &[usize], proxies: &[BroadphaseProxy]) -> Box<[StaticBlock]> {
    let mut blocks = Vec::with_capacity(static_handles.len().div_ceil(LANES));

    for chunk in static_handles.chunks(LANES) {
        let mut min_x = [0.0_f32; LANES];
        let mut max_x = [0.0_f32; LANES];
        let mut min_y = [0.0_f32; LANES];
        let mut max_y = [0.0_f32; LANES];
        let mut min_z = [0.0_f32; LANES];
        let mut max_z = [0.0_f32; LANES];
        let mut handles = [0_u32; LANES];

        for (lane, &handle) in chunk.iter().enumerate() {
            let aabb = proxies[handle].aabb;
            min_x[lane] = aabb.min.x;
            max_x[lane] = aabb.max.x;
            min_y[lane] = aabb.min.y;
            max_y[lane] = aabb.max.y;
            min_z[lane] = aabb.min.z;
            max_z[lane] = aabb.max.z;
            handles[lane] = u32::try_from(handle).expect("broadphase handle fits in u32");
        }

        blocks.push(StaticBlock {
            min_x: Vec4::from_array(min_x),
            max_x: Vec4::from_array(max_x),
            min_y: Vec4::from_array(min_y),
            max_y: Vec4::from_array(max_y),
            min_z: Vec4::from_array(min_z),
            max_z: Vec4::from_array(max_z),
            handles,
            valid_lanes: (1_u32 << chunk.len()) - 1,
        });
    }

    blocks.into_boxed_slice()
}

fn table_cache() -> &'static Mutex<Vec<Arc<PackedStatic>>> {
    static CACHE: OnceLock<Mutex<Vec<Arc<PackedStatic>>>> = OnceLock::new();
    CACHE.get_or_init(|| Mutex::new(Vec::new()))
}
