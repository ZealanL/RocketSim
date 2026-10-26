use super::{BroadphasePair, BroadphaseProxy};
use crate::bullet::{
    collision::{
        dispatch::collision_dispatcher::CollisionDispatcher,
        narrowphase::persistent_manifold::ContactAddedCallback,
    },
    dynamics::rigid_body::RigidBody,
};

/// Dense retained pair cache keyed by ordered proxy unique IDs.
///
/// `pair_indices` maps a triangular proxy-ID key to the pair-array index plus one.
/// Zero means that no pair is retained.
#[derive(Clone)]
pub struct OverlappingPairCache {
    overlapping_pair_array: Vec<BroadphasePair>,
    pair_indices: Vec<u32>,
}

impl Default for OverlappingPairCache {
    fn default() -> Self {
        Self {
            overlapping_pair_array: Vec::with_capacity(32),
            pair_indices: Vec::new(),
        }
    }
}

impl OverlappingPairCache {
    /// Filter, deduplicate, append at the end.
    ///
    /// Returns true when a new pair was appended.
    pub fn add_overlapping_pair(
        &mut self,
        proxy0: &BroadphaseProxy,
        proxy0_idx: usize,
        proxy1: &BroadphaseProxy,
        proxy1_idx: usize,
    ) -> bool {
        if !Self::needs_broadphase_collision(proxy0, proxy1) {
            return false;
        }

        let (proxy0_id, proxy0_idx, proxy1_id, proxy1_idx) = if proxy0.unique_id <= proxy1.unique_id
        {
            (proxy0.unique_id, proxy0_idx, proxy1.unique_id, proxy1_idx)
        } else {
            (proxy1.unique_id, proxy1_idx, proxy0.unique_id, proxy0_idx)
        };

        let table_idx = Self::table_index(proxy0_id, proxy1_id);
        if table_idx >= self.pair_indices.len() {
            self.pair_indices.resize(table_idx + 1, 0);
        }
        if self.pair_indices[table_idx] != 0 {
            return false;
        }

        self.overlapping_pair_array.push(BroadphasePair {
            proxy0: proxy0_idx,
            proxy1: proxy1_idx,
        });
        self.pair_indices[table_idx] =
            u32::try_from(self.overlapping_pair_array.len()).expect("pair index overflow");
        true
    }

    /// Swap-with-last and fix the moved entry.
    ///
    /// Returns the removed pair. The caller must release its manifold at
    /// this removal site, before any later edge event or dispatch.
    pub fn remove_overlapping_pair(
        &mut self,
        handles: &[BroadphaseProxy],
        proxy0_id: u32,
        proxy1_id: u32,
    ) -> Option<BroadphasePair> {
        let table_idx = Self::table_index(proxy0_id, proxy1_id);
        if table_idx >= self.pair_indices.len() {
            return None;
        }

        let entry = self.pair_indices[table_idx];
        if entry == 0 {
            return None;
        }

        let index = entry as usize - 1;
        let last_index = self.overlapping_pair_array.len() - 1;
        let removed = self.overlapping_pair_array[index];

        self.pair_indices[table_idx] = 0;
        if index != last_index {
            self.overlapping_pair_array.swap(index, last_index);
            let moved = self.overlapping_pair_array[index];
            let moved_key = Self::table_index(
                handles[moved.proxy0].unique_id,
                handles[moved.proxy1].unique_id,
            );
            self.pair_indices[moved_key] = u32::try_from(index + 1).expect("pair index overflow");
        }
        self.overlapping_pair_array.pop();

        Some(removed)
    }

    #[inline]
    pub(super) fn retained_pairs(&self) -> &[BroadphasePair] {
        &self.overlapping_pair_array
    }

    #[inline]
    pub const fn needs_broadphase_collision(
        proxy0: &BroadphaseProxy,
        proxy1: &BroadphaseProxy,
    ) -> bool {
        (proxy0.collision_filter_group & proxy1.collision_filter_mask) != 0
            && (proxy1.collision_filter_group & proxy0.collision_filter_mask) != 0
    }

    fn table_index(proxy0_id: u32, proxy1_id: u32) -> usize {
        let lo = proxy0_id.min(proxy1_id) as usize;
        let hi = proxy0_id.max(proxy1_id) as usize;
        hi * (hi + 1) / 2 + lo
    }

    pub(super) fn remove_handle(&mut self, handles: &[BroadphaseProxy], handle: usize) {
        self.overlapping_pair_array
            .retain(|pair| pair.proxy0 != handle && pair.proxy1 != handle);
        self.pair_indices.fill(0);
        for (index, pair) in self.overlapping_pair_array.iter().enumerate() {
            let key = Self::table_index(
                handles[pair.proxy0].unique_id,
                handles[pair.proxy1].unique_id,
            );
            self.pair_indices[key] = u32::try_from(index + 1).expect("pair index overflow");
        }
    }

    /// Dispatch the retained pairs in array order.
    ///
    /// Pair add and removal already happened during `setAabb`. There is no
    /// stale sweep here.
    pub fn process_all_overlapping_pairs<T: ContactAddedCallback>(
        &mut self,
        collision_objs: &[RigidBody],
        dispatcher: &mut CollisionDispatcher,
        handles: &[BroadphaseProxy],
        contact_added_callback: &mut T,
    ) {
        dispatcher.prepare_dispatch_bounds(collision_objs.len());
        for pair in &self.overlapping_pair_array {
            dispatcher.near_callback_with_cached_bounds(
                collision_objs,
                &handles[pair.proxy0],
                &handles[pair.proxy1],
                contact_added_callback,
            );
        }
    }
}
