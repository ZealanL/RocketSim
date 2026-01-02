use glam::Vec3A;

use super::{collision_dispatcher::CollisionDispatcher, collision_object::CollisionObject};
use crate::bullet::{
    collision::{
        broadphase::GridBroadphase,
        dispatch::ray_callbacks::{
            BridgeTriangleRaycastPacketCallback, QuadRayCallback, RayResultCallback,
        },
        narrowphase::persistent_manifold::{CONTACT_BREAKING_THRESHOLD, ContactAddedCallback},
    },
    dynamics::rigid_body::RigidBody,
    linear_math::{AffineExt, ray_packet::RayInfo},
};

pub struct CollisionWorld {
    pub collision_objects: Vec<RigidBody>,
    pub dispatcher1: CollisionDispatcher,
    pub(crate) broadphase_pair_cache: GridBroadphase,
    num_skippable_statics: usize,
}

impl CollisionWorld {
    pub const fn new(dispatcher: CollisionDispatcher, pair_cache: GridBroadphase) -> Self {
        Self {
            collision_objects: Vec::new(),
            dispatcher1: dispatcher,
            broadphase_pair_cache: pair_cache,
            num_skippable_statics: 0,
        }
    }

    pub fn add_collision_object(
        &mut self,
        mut object: RigidBody,
        filter_group: u8,
        filter_mask: u8,
    ) -> usize {
        {
            let obj = &mut object.co;
            obj.world_array_idx = self.collision_objects.len();

            let trans = obj.get_world_trans();
            let aabb = obj.get_collision_shape().get_aabb(trans);

            let proxy =
                self.broadphase_pair_cache
                    .create_proxy(aabb, obj, filter_group, filter_mask);

            obj.set_broadphase_handle(proxy);
        }

        let index = self.collision_objects.len();
        self.collision_objects.push(object);

        index
    }

    fn update_aabbs(&mut self) {
        const CBT: Vec3A = Vec3A::splat(CONTACT_BREAKING_THRESHOLD);

        let mut prev_is_static = true;
        for (i, rb) in self
            .collision_objects
            .iter()
            .enumerate()
            .skip(self.num_skippable_statics)
        {
            let col_obj = &rb.co;
            debug_assert_eq!(col_obj.world_array_idx, i);

            if prev_is_static && col_obj.is_static_object() {
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

            if !col_obj.is_static_object() {
                let mut aabb2 = col_obj
                    .get_collision_shape()
                    .get_aabb(&col_obj.interp_world_trans);
                aabb2.min -= CBT;
                aabb2.max += CBT;
                aabb += aabb2;
            }

            debug_assert!(
                col_obj.is_static_object() || (aabb.max - aabb.min).length_squared() < 1e12
            );
            self.broadphase_pair_cache.set_aabb(
                col_obj,
                col_obj.get_broadphase_handle().unwrap(),
                aabb,
            );
        }
    }

    pub fn perform_discrete_collision_detection<T: ContactAddedCallback>(
        &mut self,
        contact_added_callback: &mut T,
    ) {
        self.update_aabbs();

        self.broadphase_pair_cache.calculate_overlapping_pairs();
        self.dispatcher1.dispatch_all_collision_pairs(
            &self.collision_objects,
            &mut self.broadphase_pair_cache,
            contact_added_callback,
        );
    }

    pub(crate) fn quad_ray_test<T: RayResultCallback>(
        ray_from: &[Vec3A; 4],
        ray_to: &[Vec3A; 4],
        co: &CollisionObject,
        object_idx: usize,
        result_callback: &mut T,
    ) {
        let world_to_co = co.get_world_trans().transpose();

        let ray_from_local = [
            world_to_co.transform_point3a(ray_from[0]),
            world_to_co.transform_point3a(ray_from[1]),
            world_to_co.transform_point3a(ray_from[2]),
            world_to_co.transform_point3a(ray_from[3]),
        ];

        let ray_to_local = [
            world_to_co.transform_point3a(ray_to[0]),
            world_to_co.transform_point3a(ray_to[1]),
            world_to_co.transform_point3a(ray_to[2]),
            world_to_co.transform_point3a(ray_to[3]),
        ];

        let mut rcb = BridgeTriangleRaycastPacketCallback {
            from: &ray_from_local,
            to: &ray_to_local,
            hit_fraction: result_callback.get_base().closest_hit_fraction,
            collision_object: co,
            collision_object_idx: object_idx,
            result_callback,
        };

        let mut ray_info = RayInfo::new(&ray_from_local, &ray_to_local);
        ray_info.lambda_max = rcb.hit_fraction;

        co.get_collision_shape()
            .perform_raycast(&mut rcb, &mut ray_info);
    }

    pub fn ray_test<T: RayResultCallback>(
        &self,
        ray_from_world: &[Vec3A; 4],
        ray_to_world: &[Vec3A; 4],
        result_callback: &mut T,
    ) {
        let mut ray_cb = QuadRayCallback::new(ray_from_world, ray_to_world, self, result_callback);
        self.broadphase_pair_cache
            .ray_test(ray_from_world, ray_to_world, &mut ray_cb);
    }
}
