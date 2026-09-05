use std::mem;

use super::{BroadphasePair, BroadphaseProxy};
use crate::bullet::{
    collision::{
        dispatch::collision_dispatcher::CollisionDispatcher,
        narrowphase::persistent_manifold::ContactAddedCallback,
    },
    dynamics::rigid_body::RigidBody,
};

/// Pair dedup keyed by sorted proxy IDs.
///
/// Proxy `unique_id`s are assigned once at insertion with no removal path,
/// so IDs stay valid for the cache lifetime. Generation `0` marks empty
/// cells; live ticks run from `1`.
pub struct OverlappingPairCache {
    overlapping_pair_array: Vec<BroadphasePair>,
    seen_gens: Vec<u32>,
    cur_gen: u32,
}

impl Default for OverlappingPairCache {
    fn default() -> Self {
        Self {
            overlapping_pair_array: Vec::with_capacity(32),
            seen_gens: Vec::new(),
            cur_gen: 1,
        }
    }
}

impl OverlappingPairCache {
    pub fn add_overlapping_pair(
        &mut self,
        proxy0: &BroadphaseProxy,
        proxy0_idx: usize,
        proxy1: &BroadphaseProxy,
        proxy1_idx: usize,
    ) {
        if !Self::needs_broadphase_collision(proxy0, proxy1) {
            return;
        }

        let (mut proxy0_id, mut proxy0_idx) = (proxy0.unique_id, proxy0_idx);
        let (mut proxy1_id, mut proxy1_idx) = (proxy1.unique_id, proxy1_idx);
        if proxy0_id > proxy1_id {
            mem::swap(&mut proxy0_id, &mut proxy1_id);
            mem::swap(&mut proxy0_idx, &mut proxy1_idx);
        }

        let (lo, hi) = (proxy0_id as usize, proxy1_id as usize);
        debug_assert!(lo <= hi);
        // Triangular index is stable under growth: extending the table for
        // a new max ID never moves existing pairs.
        let table_idx = hi * (hi + 1) / 2 + lo;
        if table_idx >= self.seen_gens.len() {
            // Cold path: runs only when a new max ID appears.
            let needed = (hi + 1).checked_mul(hi + 2).expect("pair table overflow") / 2;
            self.seen_gens.resize(needed, 0);
        }
        debug_assert!(table_idx < self.seen_gens.len());
        if self.seen_gens[table_idx] == self.cur_gen {
            return;
        }
        self.seen_gens[table_idx] = self.cur_gen;
        self.overlapping_pair_array.push(BroadphasePair {
            proxy0: proxy0_idx,
            proxy1: proxy1_idx,
        });
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.overlapping_pair_array.is_empty()
    }

    #[inline]
    pub const fn needs_broadphase_collision(
        proxy0: &BroadphaseProxy,
        proxy1: &BroadphaseProxy,
    ) -> bool {
        (proxy0.collision_filter_group & proxy1.collision_filter_mask) != 0
            && (proxy1.collision_filter_group & proxy0.collision_filter_mask) != 0
    }

    pub fn process_all_overlapping_pairs<T: ContactAddedCallback>(
        &mut self,
        collision_objs: &[RigidBody],
        dispatcher: &mut CollisionDispatcher,
        handles: &[BroadphaseProxy],
        contact_added_callback: &mut T,
    ) {
        for pair in &self.overlapping_pair_array {
            dispatcher.near_callback(
                collision_objs,
                &handles[pair.proxy0],
                &handles[pair.proxy1],
                contact_added_callback,
            );
        }

        self.overlapping_pair_array.clear();
        // Next tick uses a fresh generation; no table clear needed.
        self.cur_gen = self.cur_gen.wrapping_add(1);
        if self.cur_gen == 0 {
            // `u32` wrap after ~4B ticks: reset all marks and restart.
            self.seen_gens.fill(0);
            self.cur_gen = 1;
        }
    }
}
