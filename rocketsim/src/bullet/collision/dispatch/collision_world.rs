use glam::{Mat3A, Vec3A};

use super::{
    collision_dispatcher::CollisionDispatcher,
    quad_ray_callbacks::{BridgeTriQuadRayCallback, QuadRayResultCallback},
};
use crate::{
    bullet::{
        collision::{
            broadphase::GridBroadphase,
            narrowphase::persistent_manifold::{CONTACT_BREAKING_THRESHOLD, ContactAddedCallback},
        },
        dynamics::rigid_body::RigidBody,
        linear_math::AffineExt,
    },
    shared::QuadRayInfo,
};

pub struct CollisionWorld {
    pub collision_objs: Vec<RigidBody>,
    pub dispatcher1: CollisionDispatcher,
    pub broadphase_pair_cache: GridBroadphase,
    num_skippable_statics: usize,
}

impl CollisionWorld {
    pub fn new(pair_cache: GridBroadphase) -> Self {
        Self {
            collision_objs: Vec::new(),
            dispatcher1: CollisionDispatcher::default(),
            broadphase_pair_cache: pair_cache,
            num_skippable_statics: 0,
        }
    }

    pub fn add_collision_obj(
        &mut self,
        mut obj: RigidBody,
        filter_group: u8,
        filter_mask: u8,
    ) -> usize {
        let idx = self.collision_objs.len();
        obj.world_array_idx = idx;

        let trans = obj.get_world_trans();
        let aabb = obj.get_collision_shape().get_aabb(trans);
        let proxy = self
            .broadphase_pair_cache
            .create_proxy(aabb, &obj, filter_group, filter_mask);

        obj.set_broadphase_handle(proxy);
        self.collision_objs.push(obj);

        idx
    }

    fn update_aabbs(&mut self) {
        const CBT: Vec3A = Vec3A::splat(CONTACT_BREAKING_THRESHOLD);

        let mut prev_is_static = true;
        for (i, col_obj) in self
            .collision_objs
            .iter()
            .enumerate()
            .skip(self.num_skippable_statics)
        {
            debug_assert_eq!(col_obj.world_array_idx, i);

            if prev_is_static && col_obj.is_static_obj() {
                // static objects only need their aabbs set the first time
                self.num_skippable_statics += 1;
            } else {
                prev_is_static = false;
            }

            let mut aabb = col_obj
                .get_collision_shape()
                .get_aabb(col_obj.get_world_trans());

            aabb.min -= CBT;
            aabb.max += CBT;

            if !col_obj.is_static_obj() {
                let mut aabb2 = col_obj
                    .get_collision_shape()
                    .get_aabb(&col_obj.interp_world_trans);
                aabb2.min -= CBT;
                aabb2.max += CBT;
                aabb += aabb2;
            }

            debug_assert!(
                col_obj.is_static_obj() || (aabb.max - aabb.min).length_squared() < 1e12,
                "object #{i} {:?} has invalid aabb: {:?}",
                col_obj.user_idx,
                aabb
            );
            self.broadphase_pair_cache
                .set_aabb(col_obj, col_obj.get_broadphase_handle(), aabb);
        }
    }

    pub fn perform_discrete_collision_detection<T: ContactAddedCallback>(
        &mut self,
        contact_added_callback: &mut T,
    ) {
        self.update_aabbs();

        self.broadphase_pair_cache.calculate_overlapping_pairs();
        self.dispatcher1.dispatch_all_collision_pairs(
            &self.collision_objs,
            &mut self.broadphase_pair_cache,
            contact_added_callback,
        );
    }

    pub(crate) fn quad_ray_test<T: QuadRayResultCallback>(
        ray_from: &[Vec3A; 4],
        ray_to: &[Vec3A; 4],
        co: &RigidBody,
        obj_idx: usize,
        result_callback: &mut T,
    ) {
        // Triangle meshes use local vertices. Goal components carry a
        // body translation, so move world rays into mesh-local space.
        // Other shapes use the same inverse-transform path.
        // Fast path for translation-only bodies: local = world - translation.
        // Bit-identical to the transpose path for identity rotation.
        let co_trans = co.get_world_trans();
        let (ray_from_local, ray_to_local) = if co_trans.matrix3 == Mat3A::IDENTITY {
            let t = co_trans.translation;
            (
                [
                    ray_from[0] - t,
                    ray_from[1] - t,
                    ray_from[2] - t,
                    ray_from[3] - t,
                ],
                [ray_to[0] - t, ray_to[1] - t, ray_to[2] - t, ray_to[3] - t],
            )
        } else {
            let world_to_co = co_trans.transpose();
            (
                [
                    world_to_co.transform_point3a(ray_from[0]),
                    world_to_co.transform_point3a(ray_from[1]),
                    world_to_co.transform_point3a(ray_from[2]),
                    world_to_co.transform_point3a(ray_from[3]),
                ],
                [
                    world_to_co.transform_point3a(ray_to[0]),
                    world_to_co.transform_point3a(ray_to[1]),
                    world_to_co.transform_point3a(ray_to[2]),
                    world_to_co.transform_point3a(ray_to[3]),
                ],
            )
        };

        let mut rcb = BridgeTriQuadRayCallback {
            from: &ray_from_local,
            to: &ray_to_local,
            hit_fraction: result_callback.get_base().closest_hit_fraction,
            collision_obj: co,
            collision_obj_idx: obj_idx,
            result_callback,
        };

        let mut ray_info = QuadRayInfo::new(&ray_from_local, &ray_to_local);
        ray_info.lambda_max = rcb.hit_fraction;

        co.get_collision_shape()
            .perform_quad_raycast(&mut rcb, &mut ray_info);
    }
}
