use std::mem;

use arrayvec::ArrayVec;
use glam::{Vec3A, Vec4};

use super::manifold_point::ManifoldPoint;
use crate::bullet::dynamics::rigid_body::{CollisionFlags, RigidBody};
use crate::bullet::linear_math::{AffineExt, plane_space_2};

pub trait ContactAddedCallback {
    fn callback(
        &mut self,
        contact_point: &mut ManifoldPoint,
        body_a: &RigidBody,
        body_b: &RigidBody,
    );
}

pub const CONTACT_BREAKING_THRESHOLD: f32 = 0.02;
pub const MANIFOLD_CACHE_SIZE: usize = 4;

pub struct PersistentManifold {
    pub point_cache: ArrayVec<ManifoldPoint, MANIFOLD_CACHE_SIZE>,
    pub body0_idx: usize,
    pub body1_idx: usize,
    pub contact_breaking_threshold: f32,
    pub contact_processing_threshold: f32,
    pub is_swapped: bool,
}

impl PersistentManifold {
    pub fn new(body0: &RigidBody, body1: &RigidBody, is_swapped: bool) -> Self {
        debug_assert_ne!(body0.world_array_idx, body1.world_array_idx);

        let body0_cbt = body0
            .get_collision_shape()
            .get_contact_breaking_threshold(CONTACT_BREAKING_THRESHOLD);
        let body1_cbt = body1
            .get_collision_shape()
            .get_contact_breaking_threshold(CONTACT_BREAKING_THRESHOLD);
        let contact_breaking_threshold = body0_cbt.min(body1_cbt);
        let contact_processing_threshold = body0
            .contact_processing_threshold
            .min(body1.contact_processing_threshold);

        Self {
            body0_idx: body0.world_array_idx,
            body1_idx: body1.world_array_idx,
            contact_breaking_threshold,
            contact_processing_threshold,
            point_cache: ArrayVec::new(),
            is_swapped,
        }
    }

    fn calculate_combined_friction(body0: &RigidBody, body1: &RigidBody) -> f32 {
        if body0.is_static_object() || body1.is_static_object() {
            body0.friction.min(body1.friction)
        } else {
            body0.friction * body1.friction
        }
    }

    fn calculate_combined_restitution(body0: &RigidBody, body1: &RigidBody) -> f32 {
        if body0.is_static_object() || body1.is_static_object() {
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

        res.max_position()
    }

    fn add_manifold_point(&mut self, contact: ManifoldPoint) -> usize {
        let num_points = self.point_cache.len();
        if num_points == MANIFOLD_CACHE_SIZE {
            let index = self.sort_cached_points(&contact);
            self.point_cache[index] = contact;
            index
        } else {
            self.point_cache.push(contact);
            num_points
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn add_contact_point<'a, T: ContactAddedCallback>(
        &mut self,
        mut body0: &'a RigidBody,
        mut body1: &'a RigidBody,
        normal_on_b_in_world: Vec3A,
        point_in_world: Vec3A,
        depth: f32,
        index_0: i32,
        index_1: i32,
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
        new_pt.position_world_on_a = point_a;
        new_pt.position_world_on_b = point_in_world;

        new_pt.combined_friction = Self::calculate_combined_friction(body0, body1);
        new_pt.combined_restitution = Self::calculate_combined_restitution(body0, body1);

        (new_pt.lateral_friction_dir_1, new_pt.lateral_friction_dir_2) =
            plane_space_2(new_pt.normal_world_on_b);

        if self.is_swapped {
            new_pt.index_0 = index_1;
            new_pt.index_1 = index_0;
        } else {
            new_pt.index_0 = index_0;
            new_pt.index_1 = index_1;
        }

        let insert_idx = self.add_manifold_point(new_pt);

        if self.is_swapped {
            mem::swap(&mut body0, &mut body1);
        }

        if body0.collision_flags & CollisionFlags::CustomMaterialCallback as u8 != 0
            || body1.collision_flags & CollisionFlags::CustomMaterialCallback as u8 != 0
        {
            contact_added_callback.callback(&mut self.point_cache[insert_idx], body0, body1);
        }
    }

    pub fn refresh_contact_points(&mut self, body0: &RigidBody, body1: &RigidBody) {
        if self.point_cache.is_empty() {
            return;
        }

        let tr_a = body0.get_world_trans();
        let tr_b = body1.get_world_trans();

        for manifold_point in &mut self.point_cache {
            manifold_point.position_world_on_a =
                tr_a.transform_point3a(manifold_point.local_point_a);
            manifold_point.position_world_on_b =
                tr_b.transform_point3a(manifold_point.local_point_b);
            manifold_point.distance_1 = (manifold_point.position_world_on_a
                - manifold_point.position_world_on_b)
                .dot(manifold_point.normal_world_on_b);
        }

        let contact_breaking_threshold_sq =
            self.contact_breaking_threshold * self.contact_breaking_threshold;

        for i in (0..self.point_cache.len()).rev() {
            let point = &self.point_cache[i];
            if point.distance_1 > self.contact_breaking_threshold {
                // contact becomes invalid when signed distance exceeds margin (projected on contact normal direction)
                self.point_cache.remove(i);
                continue;
            }

            let projected_point =
                point.position_world_on_a - point.normal_world_on_b * point.distance_1;
            let projected_difference = point.position_world_on_b - projected_point;
            let distance_2d = projected_difference.dot(projected_difference);
            if distance_2d > contact_breaking_threshold_sq {
                // contact also becomes invalid when relative movement orthogonal to normal exceeds margin
                self.point_cache.remove(i);
            }
        }
    }
}
