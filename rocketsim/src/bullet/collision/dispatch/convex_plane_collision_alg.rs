use glam::Affine3A;

use super::collision_obj_wrapper::RigidBodyWrapper;
use crate::bullet::{
    collision::{
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::static_plane_shape::StaticPlaneShape,
    },
    dynamics::rigid_body::RigidBody,
};

pub fn process_collision<T: ContactAddedCallback>(
    convex_obj: &RigidBodyWrapper,
    plane_obj: &RigidBody,
    plane_shape: &StaticPlaneShape,
    contact_added_callback: &mut T,
    out: &mut Option<PersistentManifold>,
) {
    debug_assert!(out.is_none());
    let convex_aabb = convex_obj.get_aabb();
    if !convex_aabb.intersects(&plane_shape.aabb_cache) {
        return;
    }

    let plane_normal = plane_shape.get_plane_normal();

    let plane_trans = plane_obj.get_world_trans();
    let plane_in_convex = convex_obj.world_trans.matrix3.transpose() * plane_trans.matrix3;
    let convex_in_plane_trans = Affine3A {
        matrix3: plane_trans.matrix3.transpose() * convex_obj.world_trans.matrix3,
        translation: plane_trans.matrix3 * convex_obj.world_trans.translation
            - plane_trans.translation,
    };

    let vtx = convex_obj.local_get_supporting_vertex(plane_in_convex * -plane_normal);
    let vtx_in_plane = convex_in_plane_trans.transform_point3a(vtx);
    let distance = plane_normal.dot(vtx_in_plane);

    let mut manifold = PersistentManifold::new(convex_obj.obj, plane_obj);
    if distance >= manifold.contact_breaking_threshold {
        return;
    }

    let vtx_in_plane_projected = vtx_in_plane - distance * plane_normal;
    let vtx_in_plane_world = plane_obj
        .get_world_trans()
        .transform_point3a(vtx_in_plane_projected);
    let normal_on_surface_b = plane_obj.get_world_trans().matrix3 * plane_normal;

    manifold.add_contact_point(
        convex_obj.obj,
        plane_obj,
        normal_on_surface_b,
        vtx_in_plane_world,
        distance,
        None,
        contact_added_callback,
    );

    manifold.refresh_contact_points(convex_obj.obj, plane_obj);
    *out = Some(manifold);
}
