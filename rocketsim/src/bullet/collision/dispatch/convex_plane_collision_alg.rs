use glam::Affine3A;

use crate::bullet::collision::{
    broadphase::CollisionAlgorithm,
    dispatch::collision_obj_wrapper::RigidBodyWrapper,
    narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
    shapes::static_plane_shape::StaticPlaneShape,
};
use crate::bullet::dynamics::rigid_body::RigidBody;

pub struct ConvexPlaneCollisionAlgorithm<'a, T: ContactAddedCallback> {
    is_swapped: bool,
    convex_obj: RigidBodyWrapper<'a>,
    plane_obj: &'a RigidBody,
    plane_shape: &'a StaticPlaneShape,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ConvexPlaneCollisionAlgorithm<'a, T> {
    pub const fn new(
        convex_obj: RigidBodyWrapper<'a>,
        plane_obj: &'a RigidBody,
        plane_shape: &'a StaticPlaneShape,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            is_swapped,
            convex_obj,
            plane_obj,
            plane_shape,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for ConvexPlaneCollisionAlgorithm<'_, T> {
    fn process_collision(self) -> Option<PersistentManifold> {
        let col_shape = self.convex_obj.obj.get_collision_shape();
        let convex_aabb = col_shape.get_aabb(&self.convex_obj.world_trans);
        if !convex_aabb.intersects(&self.plane_shape.aabb_cache) {
            return None;
        }

        let plane_normal = self.plane_shape.get_plane_normal();

        let plane_trans = self.plane_obj.get_world_trans();
        let plane_in_convex = self.convex_obj.world_trans.matrix3.transpose() * plane_trans.matrix3;
        let convex_in_plane_trans = Affine3A {
            matrix3: plane_trans.matrix3.transpose() * self.convex_obj.world_trans.matrix3,
            translation: plane_trans.matrix3 * self.convex_obj.world_trans.translation
                - plane_trans.translation,
        };

        let vtx = col_shape.local_get_supporting_vertex(plane_in_convex * -plane_normal);
        let vtx_in_plane = convex_in_plane_trans.transform_point3a(vtx);
        let distance = plane_normal.dot(vtx_in_plane);

        let mut manifold =
            PersistentManifold::new(self.convex_obj.obj, self.plane_obj, self.is_swapped);
        if distance >= manifold.contact_breaking_threshold {
            return None;
        }

        let vtx_in_plane_projected = vtx_in_plane - distance * plane_normal;
        let vtx_in_plane_world = self
            .plane_obj
            .get_world_trans()
            .transform_point3a(vtx_in_plane_projected);
        let normal_on_surface_b = self.plane_obj.get_world_trans().matrix3 * plane_normal;

        manifold.add_contact_point(
            self.convex_obj.obj,
            self.plane_obj,
            normal_on_surface_b,
            vtx_in_plane_world,
            distance,
            -1,
            -1,
            self.contact_added_callback,
        );

        manifold.refresh_contact_points(self.convex_obj.obj, self.plane_obj);
        Some(manifold)
    }
}
