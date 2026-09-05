use arrayvec::ArrayVec;
use glam::{Vec3A, Vec4};

use super::manifold_point::ManifoldPoint;
use crate::bullet::{
    dynamics::rigid_body::{CollisionFlags, RigidBody},
    linear_math::{AffineExt, plane_space_1},
};

pub trait ContactAddedCallback {
    fn callback(
        &mut self,
        contact_point: &mut ManifoldPoint,
        body_a: &RigidBody,
        body_b: &RigidBody,
        idx: Option<usize>,
    );
}

pub const CONTACT_BREAKING_THRESHOLD: f32 = 0.02;
pub const MANIFOLD_CACHE_SIZE: usize = 4;

/// Order-independent lookup key for a body pair.
/// Packs `min` in the high bits and `max` in the low bits so the
/// dispatcher matches pairs with one `u64` compare instead of two
/// `min`/`max` calls per cached manifold.
#[inline]
pub fn pair_key(body_a_idx: usize, body_b_idx: usize) -> u64 {
    let pair_min = body_a_idx.min(body_b_idx) as u64;
    let pair_max = body_a_idx.max(body_b_idx) as u64;
    (pair_min << 32) | pair_max
}

#[derive(Clone)]
pub struct PersistentManifold {
    pub point_cache: ArrayVec<ManifoldPoint, MANIFOLD_CACHE_SIZE>,
    pub(crate) most_recently_evicted_point: Option<ManifoldPoint>,
    pub body0_idx: usize,
    pub body1_idx: usize,
    pub pair_key: u64,
    pub contact_breaking_threshold: f32,
    pub contact_processing_threshold: f32,
}

impl PersistentManifold {
    pub fn new(body0: &RigidBody, body1: &RigidBody) -> Self {
        debug_assert_ne!(body0.world_array_idx, body1.world_array_idx);

        // Thresholds are cached per body: shapes never change after
        // construction, so this is two loads and two `min` calls.
        let contact_breaking_threshold = body0
            .get_contact_breaking_threshold()
            .min(body1.get_contact_breaking_threshold());
        let contact_processing_threshold = body0
            .contact_processing_threshold
            .min(body1.contact_processing_threshold);

        Self {
            body0_idx: body0.world_array_idx,
            body1_idx: body1.world_array_idx,
            pair_key: pair_key(body0.world_array_idx, body1.world_array_idx),
            contact_breaking_threshold,
            contact_processing_threshold,
            point_cache: ArrayVec::new(),
            most_recently_evicted_point: None,
        }
    }

    fn calculate_combined_friction(body0: &RigidBody, body1: &RigidBody) -> f32 {
        if body0.is_static_obj() || body1.is_static_obj() {
            body0.friction.min(body1.friction)
        } else {
            body0.friction * body1.friction
        }
    }

    fn calculate_combined_restitution(body0: &RigidBody, body1: &RigidBody) -> f32 {
        if body0.is_static_obj() || body1.is_static_obj() {
            body0.restitution.max(body1.restitution)
        } else {
            body0.restitution * body1.restitution
        }
    }

    #[inline]
    fn get_res(new_contact_local: Vec3A, point1: Vec3A, point2: Vec3A, point3: Vec3A) -> f32 {
        (new_contact_local - point1)
            .cross(point2 - point3)
            .length_squared()
    }

    #[inline]
    fn get_res_0(&self, new_contact_local: Vec3A) -> f32 {
        Self::get_res(
            new_contact_local,
            self.point_cache[1].local_point_a,
            self.point_cache[3].local_point_a,
            self.point_cache[2].local_point_a,
        )
    }

    #[inline]
    fn get_res_1(&self, new_contact_local: Vec3A) -> f32 {
        Self::get_res(
            new_contact_local,
            self.point_cache[0].local_point_a,
            self.point_cache[3].local_point_a,
            self.point_cache[2].local_point_a,
        )
    }

    #[inline]
    fn get_res_2(&self, new_contact_local: Vec3A) -> f32 {
        Self::get_res(
            new_contact_local,
            self.point_cache[0].local_point_a,
            self.point_cache[3].local_point_a,
            self.point_cache[1].local_point_a,
        )
    }

    #[inline]
    fn get_res_3(&self, new_contact_local: Vec3A) -> f32 {
        Self::get_res(
            new_contact_local,
            self.point_cache[0].local_point_a,
            self.point_cache[2].local_point_a,
            self.point_cache[1].local_point_a,
        )
    }

    fn sort_cached_points(&self, new_contact: &ManifoldPoint) -> usize {
        let mut max_penetration_idx = MANIFOLD_CACHE_SIZE;
        let mut max_penetration = new_contact.distance_1;
        for (i, contact) in self.point_cache.iter().enumerate() {
            if contact.distance_1 < max_penetration {
                max_penetration_idx = i;
                max_penetration = contact.distance_1;
            }
        }

        let res = match max_penetration_idx {
            0 => Vec4::new(
                0.,
                self.get_res_1(new_contact.local_point_a),
                self.get_res_2(new_contact.local_point_a),
                self.get_res_3(new_contact.local_point_a),
            ),
            1 => Vec4::new(
                self.get_res_0(new_contact.local_point_a),
                0.,
                self.get_res_2(new_contact.local_point_a),
                self.get_res_3(new_contact.local_point_a),
            ),
            2 => Vec4::new(
                self.get_res_0(new_contact.local_point_a),
                self.get_res_1(new_contact.local_point_a),
                0.,
                self.get_res_3(new_contact.local_point_a),
            ),
            3 => Vec4::new(
                self.get_res_0(new_contact.local_point_a),
                self.get_res_1(new_contact.local_point_a),
                self.get_res_2(new_contact.local_point_a),
                0.,
            ),
            _ => Vec4::new(
                self.get_res_0(new_contact.local_point_a),
                self.get_res_1(new_contact.local_point_a),
                self.get_res_2(new_contact.local_point_a),
                self.get_res_3(new_contact.local_point_a),
            ),
        };

        // Bullet's closestAxis4: first index of the maximum component wins ties.
        let mut biggest_area = 0;
        let mut max_area = res.x;
        if res.y > max_area {
            biggest_area = 1;
            max_area = res.y;
        }
        if res.z > max_area {
            biggest_area = 2;
            max_area = res.z;
        }
        if res.w > max_area {
            biggest_area = 3;
        }

        biggest_area
    }

    fn get_cache_entry(&self, new_contact: &ManifoldPoint) -> Option<usize> {
        let threshold_sq = self.contact_breaking_threshold * self.contact_breaking_threshold;
        let mut shortest_dist = threshold_sq;
        let mut nearest_point: Option<usize> = None;
        for (index, contact) in self.point_cache.iter().enumerate() {
            let distance_sq = (contact.local_point_a - new_contact.local_point_a).length_squared();
            if distance_sq < shortest_dist {
                shortest_dist = distance_sq;
                nearest_point = Some(index);
            }
        }
        nearest_point
    }

    fn replace_contact_point(&mut self, index: usize, mut contact: ManifoldPoint) {
        let old_contact = self.point_cache[index];
        contact.applied_impulse = old_contact.applied_impulse;
        self.point_cache[index] = contact;
    }

    fn add_manifold_point(&mut self, contact: ManifoldPoint) -> usize {
        let num_points = self.point_cache.len();
        if num_points == MANIFOLD_CACHE_SIZE {
            let idx = self.sort_cached_points(&contact);
            self.most_recently_evicted_point = Some(self.point_cache[idx]);
            self.point_cache[idx] = contact;
            idx
        } else {
            self.point_cache.push(contact);
            num_points
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn add_contact_point<T: ContactAddedCallback>(
        &mut self,
        body0: &RigidBody,
        body1: &RigidBody,
        normal_on_b_in_world: Vec3A,
        point_in_world: Vec3A,
        depth: f32,
        idx_1: Option<usize>,
        contact_added_callback: &mut T,
    ) {
        if depth > self.contact_breaking_threshold {
            return;
        }

        let point_a = point_in_world + normal_on_b_in_world * depth;
        let (local_a, local_b) = (
            body0.get_world_trans().inv_x_form(point_a),
            body1.get_world_trans().inv_x_form(point_in_world),
        );

        let mut new_pt = ManifoldPoint::new(local_a, local_b, normal_on_b_in_world, depth);
        new_pt.pos_world_on_a = point_a;
        new_pt.pos_world_on_b = point_in_world;

        new_pt.combined_friction = Self::calculate_combined_friction(body0, body1);
        new_pt.combined_restitution = Self::calculate_combined_restitution(body0, body1);

        new_pt.lateral_friction_dir_1 = plane_space_1(new_pt.normal_world_on_b);

        let insert_idx = self.add_contact_without_callback(new_pt);

        if (body0.collision_flags & CollisionFlags::CustomMaterialCallback) != 0
            || (body1.collision_flags & CollisionFlags::CustomMaterialCallback) != 0
        {
            contact_added_callback.callback(&mut self.point_cache[insert_idx], body0, body1, idx_1);
        }
    }

    fn add_contact_without_callback(&mut self, contact: ManifoldPoint) -> usize {
        if let Some(insert_idx) = self.get_cache_entry(&contact) {
            self.replace_contact_point(insert_idx, contact);
            insert_idx
        } else {
            self.add_manifold_point(contact)
        }
    }

    pub(crate) fn merge_contact_points(&mut self, other: &Self) {
        // Borrow the fresh manifold: points are `Copy`, so iterating by
        // reference moves the same values in the same order without
        // memmoving the whole donor struct (`ArrayVec` + evicted point).
        for contact in other.point_cache.iter() {
            self.add_contact_without_callback(*contact);
        }
    }

    /// Re-project carried points and cull separated ones.
    /// Refreshing an empty cache is a no-op (returns early with no
    /// observable change), so hot-path callers skip the call when
    /// `point_cache` is empty.
    pub fn refresh_contact_points(&mut self, body0: &RigidBody, body1: &RigidBody) {
        if self.point_cache.is_empty() {
            return;
        }

        let tr_a = body0.get_world_trans();
        let tr_b = body1.get_world_trans();

        for manifold_point in &mut self.point_cache {
            manifold_point.pos_world_on_a = tr_a.transform_point3a(manifold_point.local_point_a);
            manifold_point.pos_world_on_b = tr_b.transform_point3a(manifold_point.local_point_b);
            manifold_point.distance_1 = (manifold_point.pos_world_on_a
                - manifold_point.pos_world_on_b)
                .dot(manifold_point.normal_world_on_b);
        }

        let contact_breaking_threshold_sq =
            self.contact_breaking_threshold * self.contact_breaking_threshold;

        for i in (0..self.point_cache.len()).rev() {
            let point = self.point_cache[i];
            if point.distance_1 > self.contact_breaking_threshold {
                // contact becomes invalid when signed distance exceeds margin (projected on contact normal direction)
                self.point_cache.swap_remove(i);
                continue;
            }

            let projected_point = point.pos_world_on_a - point.normal_world_on_b * point.distance_1;
            let projected_difference = point.pos_world_on_b - projected_point;
            let distance_2d = projected_difference.dot(projected_difference);
            if distance_2d > contact_breaking_threshold_sq {
                // contact also becomes invalid when relative movement orthogonal to normal exceeds margin
                self.point_cache.swap_remove(i);
            }
        }
    }
}
