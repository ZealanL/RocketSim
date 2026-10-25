use arrayvec::ArrayVec;

use crate::bullet::{
    collision::narrowphase::persistent_manifold::PersistentManifold,
    dynamics::rigid_body::{ActivationState, RigidBody},
};

/// This limit applies only to the sort cache. Physics inputs remain unbounded.
const SORT_CACHE_CAPACITY: usize = 64;

#[derive(Clone, Default)]
pub(super) struct SimulationIslandManager {
    body_to_dynamic: Vec<usize>,
    parents: Vec<usize>,
    island_tags: Vec<isize>,
    island_has_active_body: Vec<bool>,
    sort_keys: Vec<isize>,
    cached_input: ArrayVec<usize, SORT_CACHE_CAPACITY>,
    cached_output: ArrayVec<usize, SORT_CACHE_CAPACITY>,
}

impl SimulationIslandManager {
    pub(super) fn sort_manifolds(
        &mut self,
        bodies: &mut [RigidBody],
        dynamic_body_indices: &[usize],
        overlapping_dynamic_pairs: &[(usize, usize)],
        manifolds: &[PersistentManifold],
        active_manifold_indices: &mut [usize],
    ) {
        self.update_body_mapping(bodies.len(), dynamic_body_indices);
        for (dynamic_index, parent) in self.parents.iter_mut().enumerate() {
            *parent = dynamic_index;
        }

        let body_to_dynamic = &self.body_to_dynamic;
        union_dynamic_pairs(
            &mut self.parents,
            overlapping_dynamic_pairs
                .iter()
                .filter_map(|&(body0, body1)| {
                    let a = body_to_dynamic[body0];
                    let b = body_to_dynamic[body1];
                    if a != usize::MAX
                        && b != usize::MAX
                        && bodies[body0].has_contact_response()
                        && bodies[body1].has_contact_response()
                    {
                        Some((a, b))
                    } else {
                        None
                    }
                }),
        );

        for dynamic_index in 0..self.island_tags.len() {
            self.island_tags[dynamic_index] = self.find(dynamic_index) as isize;
        }

        self.island_has_active_body
            .resize(self.island_tags.len(), false);
        self.island_has_active_body.fill(false);
        for (dynamic_index, &body_index) in dynamic_body_indices.iter().enumerate() {
            let island = self.island_tags[dynamic_index] as usize;
            if bodies[body_index].activation_state() == ActivationState::Active {
                self.island_has_active_body[island] = true;
            }
        }
        for (dynamic_index, &body_index) in dynamic_body_indices.iter().enumerate() {
            let island = self.island_tags[dynamic_index] as usize;
            let state = bodies[body_index].activation_state();
            if !self.island_has_active_body[island] && state == ActivationState::WantsDeactivation {
                bodies[body_index].sleep();
            }
        }

        if active_manifold_indices.len() <= 1 {
            return;
        }

        let body_to_dynamic = &self.body_to_dynamic;
        let island_tags = &self.island_tags;
        sort_active_manifolds(
            active_manifold_indices,
            &mut self.sort_keys,
            &mut self.cached_input,
            &mut self.cached_output,
            manifolds.len(),
            |manifold_index| {
                let manifold = &manifolds[manifold_index];
                let tag0 = body_tag(body_to_dynamic, island_tags, manifold.body0_idx);
                let tag1 = body_tag(body_to_dynamic, island_tags, manifold.body1_idx);
                manifold_island_key(tag0, tag1)
            },
        );
    }

    fn update_body_mapping(&mut self, body_count: usize, dynamic_body_indices: &[usize]) {
        self.body_to_dynamic.resize(body_count, usize::MAX);

        let mapping_is_current = self.parents.len() == dynamic_body_indices.len()
            && dynamic_body_indices
                .iter()
                .enumerate()
                .all(|(dynamic_index, &body_index)| {
                    self.body_to_dynamic.get(body_index) == Some(&dynamic_index)
                });
        if !mapping_is_current {
            self.body_to_dynamic.fill(usize::MAX);
            for (dynamic_index, &body_index) in dynamic_body_indices.iter().enumerate() {
                self.body_to_dynamic[body_index] = dynamic_index;
            }
        }

        self.parents.resize(dynamic_body_indices.len(), 0);
        self.island_tags.resize(dynamic_body_indices.len(), -1);
    }

    fn find(&mut self, index: usize) -> usize {
        let mut root = index;
        while self.parents[root] != root {
            root = self.parents[root];
        }
        let mut current = index;
        while self.parents[current] != current {
            let parent = self.parents[current];
            self.parents[current] = root;
            current = parent;
        }
        root
    }
}

fn body_tag(body_to_dynamic: &[usize], island_tags: &[isize], body_index: usize) -> isize {
    let dynamic_index = body_to_dynamic[body_index];
    if dynamic_index == usize::MAX {
        -1
    } else {
        island_tags[dynamic_index]
    }
}

fn manifold_island_key(tag0: isize, tag1: isize) -> isize {
    if tag0 >= 0 { tag0 } else { tag1 }
}

fn sort_active_manifolds(
    indices: &mut [usize],
    sort_keys: &mut Vec<isize>,
    cached_input: &mut ArrayVec<usize, SORT_CACHE_CAPACITY>,
    cached_output: &mut ArrayVec<usize, SORT_CACHE_CAPACITY>,
    manifold_count: usize,
    mut key_for: impl FnMut(usize) -> isize,
) -> bool {
    let key_count_matches = sort_keys.len() == manifold_count;
    sort_keys.resize(manifold_count, -1);

    if indices.len() > SORT_CACHE_CAPACITY {
        cached_input.clear();
        cached_output.clear();
        indices.sort_unstable();
        for &index in indices.iter() {
            sort_keys[index] = key_for(index);
        }
        hoare_sort(indices, sort_keys);
        return false;
    }

    let mut cache_hit = key_count_matches
        && cached_input.as_slice() == indices
        && cached_output.len() == indices.len();
    for &index in indices.iter() {
        let key = key_for(index);
        if sort_keys[index] != key {
            cache_hit = false;
        }
        sort_keys[index] = key;
    }

    if cache_hit {
        indices.copy_from_slice(cached_output);
        return true;
    }

    cached_input.clear();
    cached_input.try_extend_from_slice(indices).unwrap();
    indices.sort_unstable();
    hoare_sort(indices, sort_keys);
    cached_output.clear();
    cached_output.try_extend_from_slice(indices).unwrap();
    false
}

fn union_dynamic_pairs(parents: &mut [usize], pairs: impl IntoIterator<Item = (usize, usize)>) {
    fn find(parents: &mut [usize], index: usize) -> usize {
        if parents[index] != index {
            parents[index] = find(parents, parents[index]);
        }
        parents[index]
    }

    for (body0, body1) in pairs {
        let root0 = find(parents, body0);
        let root1 = find(parents, body1);
        if root0 != root1 {
            parents[root0] = root1;
        }
    }
}

fn hoare_sort(indices: &mut [usize], keys: &[isize]) {
    fn sort(indices: &mut [usize], keys: &[isize], lo: usize, hi: usize) {
        let mut i = lo;
        let mut j = hi;
        let pivot = keys[indices[(lo + hi) / 2]];
        loop {
            while keys[indices[i]] < pivot {
                i += 1;
            }
            while pivot < keys[indices[j]] {
                j -= 1;
            }
            if i > j {
                break;
            }
            indices.swap(i, j);
            i += 1;
            if j == 0 {
                break;
            }
            j -= 1;
            if i > j {
                break;
            }
        }
        if lo < j {
            sort(indices, keys, lo, j);
        }
        if i < hi {
            sort(indices, keys, i, hi);
        }
    }

    if indices.len() > 1 {
        sort(indices, keys, 0, indices.len() - 1);
    }
}
