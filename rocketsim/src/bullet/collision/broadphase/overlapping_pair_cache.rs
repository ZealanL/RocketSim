use std::{
    collections::{HashMap, hash_map::Entry},
    mem,
};

use rustc_hash::{FxBuildHasher, FxHashMap};

use super::{BroadphasePair, BroadphaseProxy};
use crate::bullet::{
    collision::{
        dispatch::collision_dispatcher::CollisionDispatcher,
        narrowphase::persistent_manifold::ContactAddedCallback,
    },
    dynamics::rigid_body::RigidBody,
};

pub struct HashedOverlappingPairCache {
    overlapping_pair_array: Vec<BroadphasePair>,
    hash_table: FxHashMap<u64, usize>,
}

impl Default for HashedOverlappingPairCache {
    fn default() -> Self {
        let overlapping_pair_array = Vec::with_capacity(32);
        let new_capacity = overlapping_pair_array.capacity();

        Self {
            overlapping_pair_array,
            hash_table: HashMap::with_capacity_and_hasher(new_capacity, FxBuildHasher),
        }
    }
}

impl HashedOverlappingPairCache {
    pub(crate) fn add_overlapping_pair(
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

        let pair_key = (u64::from(proxy0_id) << 32) | u64::from(proxy1_id);
        let pair_idx = self.overlapping_pair_array.len();
        let Entry::Vacant(entry) = self.hash_table.entry(pair_key) else {
            return;
        };
        entry.insert(pair_idx);
        self.overlapping_pair_array.push(BroadphasePair {
            proxy0: proxy0_idx,
            proxy1: proxy1_idx,
        });
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.hash_table.is_empty()
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
        self.hash_table.clear();
    }
}
