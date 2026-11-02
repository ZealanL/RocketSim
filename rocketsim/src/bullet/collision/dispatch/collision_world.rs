use glam::Vec3A;

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
        dynamics::rigid_body::{CollisionFlags, RigidBody, RigidBodyUpdateInfo},
        linear_math::AffineExt,
    },
    shared::QuadRayInfo,
};

#[derive(Clone)]
pub struct CollisionWorld {
    pub collision_objs: Vec<RigidBody>,
    pub dispatcher1: CollisionDispatcher,
    pub broadphase_pair_cache: GridBroadphase,
}

impl CollisionWorld {
    pub fn new(pair_cache: GridBroadphase) -> Self {
        Self {
            collision_objs: Vec::new(),
            dispatcher1: CollisionDispatcher::default(),
            broadphase_pair_cache: pair_cache,
        }
    }

    /// Insert a body and create its broadphase proxy from the raw shape AABB.
    ///
    /// A static body must set its transform before this call. After this call
    /// the proxy AABB is final. The `RigidBody` transform setters assert this
    /// invariant.
    pub fn add_collision_obj(
        &mut self,
        mut obj: RigidBody,
        filter_group: u8,
        filter_mask: u8,
    ) -> usize {
        let Self {
            collision_objs,
            dispatcher1,
            broadphase_pair_cache,
            ..
        } = self;

        let idx = collision_objs.len();
        obj.world_array_idx = idx;

        let aabb = obj.get_collision_shape().get_aabb(obj.get_world_trans());
        let proxy =
            broadphase_pair_cache.create_proxy(aabb, &obj, filter_group, filter_mask, dispatcher1);

        obj.set_broadphase_handle(proxy);
        collision_objs.push(obj);

        idx
    }

    pub(crate) fn update_body_properties(
        &mut self,
        body_idx: usize,
        properties: RigidBodyUpdateInfo,
        dynamic_body_idcs: &[usize],
    ) {
        debug_assert!(
            !self.collision_objs[body_idx].is_static_obj(),
            "cannot update properties of static collision object {body_idx}: its broadphase AABB is final"
        );

        let invalidate_contacts = properties.invalidate_contacts;
        self.collision_objs[body_idx].update_physical_properties(properties);
        if invalidate_contacts {
            self.dispatcher1.clear_persistent_manifolds();
        }

        self.update_aabbs(dynamic_body_idcs);
    }

    /// Update the proxy AABB for each body in `dynamic_body_idcs`.
    pub fn update_aabbs(&mut self, dynamic_body_idcs: &[usize]) {
        const CBT: Vec3A = Vec3A::splat(CONTACT_BREAKING_THRESHOLD);

        let Self {
            collision_objs,
            dispatcher1,
            broadphase_pair_cache,
        } = self;

        debug_assert!(
            dynamic_body_idcs.windows(2).all(|pair| pair[0] < pair[1]),
            "dynamic body indices must be strictly increasing"
        );

        for &i in dynamic_body_idcs {
            let col_obj = &collision_objs[i];
            debug_assert_eq!(col_obj.world_array_idx, i);
            debug_assert!(
                !col_obj.is_static_obj(),
                "dynamic body index {i} points at a static body"
            );

            let mut aabb = col_obj
                .get_collision_shape()
                .get_aabb(col_obj.get_world_trans());

            aabb.min -= CBT;
            aabb.max += CBT;

            let mut aabb2 = col_obj
                .get_collision_shape()
                .get_aabb(&col_obj.interp_world_trans);
            aabb2.min -= CBT;
            aabb2.max += CBT;
            aabb += aabb2;

            debug_assert!(
                (aabb.max - aabb.min).length_squared() < 1e12,
                "object #{i} {:?} has invalid aabb: {:?}",
                col_obj.user_idx,
                aabb
            );
            broadphase_pair_cache.set_aabb(
                col_obj,
                col_obj.get_broadphase_handle(),
                aabb,
                dispatcher1,
            );
        }
    }

    pub fn perform_discrete_collision_detection<T: ContactAddedCallback>(
        &mut self,
        contact_added_callback: &mut T,
        dynamic_body_idcs: &[usize],
    ) {
        self.update_aabbs(dynamic_body_idcs);

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
        if co.collision_flags & CollisionFlags::IdentityTransform != 0 {
            Self::quad_ray_test_local(ray_from, ray_to, co, obj_idx, result_callback);
            return;
        }

        // Fast path for translation-only bodies: local = world - translation.
        // Bit-identical to the transpose path for identity rotation.
        let co_trans = co.get_world_trans();
        let (ray_from_local, ray_to_local) =
            if co.collision_flags & CollisionFlags::NoAngularMotion != 0 {
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

        Self::quad_ray_test_local(&ray_from_local, &ray_to_local, co, obj_idx, result_callback);
    }

    fn quad_ray_test_local<T: QuadRayResultCallback>(
        ray_from_local: &[Vec3A; 4],
        ray_to_local: &[Vec3A; 4],
        co: &RigidBody,
        obj_idx: usize,
        result_callback: &mut T,
    ) {
        let mut rcb = BridgeTriQuadRayCallback {
            from: ray_from_local,
            to: ray_to_local,
            hit_fraction: result_callback.get_base().closest_hit_fraction,
            collision_obj: co,
            collision_obj_idx: obj_idx,
            result_callback,
        };

        let mut ray_info = QuadRayInfo::new(ray_from_local, ray_to_local);
        ray_info.lambda_max = rcb.hit_fraction;

        co.get_collision_shape()
            .perform_quad_raycast(&mut rcb, &mut ray_info);
    }
}
