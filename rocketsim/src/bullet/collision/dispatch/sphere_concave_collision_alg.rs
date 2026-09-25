use glam::{Mat3A, Vec3A};

use crate::bullet::{
    collision::{
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{
            bvh_triangle_mesh_shape::BvhTriangleMeshShape, sphere_shape::SphereShape,
            triangle_callback::ProcessTriangle, triangle_shape::TriangleShape,
        },
    },
    dynamics::rigid_body::RigidBody,
};

struct SphereTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: &'a mut PersistentManifold,
    pub sphere_obj: &'a RigidBody,
    pub tri_obj: &'a RigidBody,
    sphere_center: Vec3A,
    sphere_radius: f32,
    radius_with_threshold: f32,
    radius_with_threshold_sqr: f32,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> SphereTriangleCallback<'a, T> {
    pub fn new(
        manifold: &'a mut PersistentManifold,
        sphere_obj: &'a RigidBody,
        tri_obj: &'a RigidBody,
        sphere_center: Vec3A,
        sphere_radius: f32,
        contact_breaking_threshold: f32,
        contact_added_callback: &'a mut T,
    ) -> Self {
        let radius_with_threshold = sphere_radius + contact_breaking_threshold;
        let radius_with_threshold_sqr = radius_with_threshold * radius_with_threshold;

        Self {
            manifold,
            sphere_obj,
            tri_obj,
            sphere_center,
            sphere_radius,
            radius_with_threshold,
            radius_with_threshold_sqr,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> ProcessTriangle for SphereTriangleCallback<'_, T> {
    fn process_triangle(&mut self, triangle: &TriangleShape, triangle_idx: usize) {
        let Some(contact_info) = triangle.intersect_sphere_front_precomputed(
            self.sphere_center,
            self.sphere_radius,
            self.radius_with_threshold,
            self.radius_with_threshold_sqr,
        ) else {
            return;
        };

        let tri_world = self.tri_obj.get_world_trans();
        debug_assert_eq!(tri_world.matrix3, Mat3A::IDENTITY);

        let normal_on_b = contact_info.result_normal;
        let point_in_world = contact_info.contact_point + tri_world.translation;

        self.manifold.add_contact_point(
            self.sphere_obj,
            self.tri_obj,
            normal_on_b,
            point_in_world,
            contact_info.depth,
            Some(triangle_idx),
            self.contact_added_callback,
        )
    }
}

pub(crate) fn process_collision_into<T: ContactAddedCallback>(
    convex_obj: &RigidBody,
    sphere_shape: &SphereShape,
    concave_obj: &RigidBody,
    tri_mesh: &BvhTriangleMeshShape,
    manifold: &mut PersistentManifold,
    contact_added_callback: &mut T,
) -> bool {
    let xform1 = convex_obj.get_world_trans();
    let mesh_trans = *concave_obj.get_world_trans();

    debug_assert_eq!(mesh_trans.matrix3, Mat3A::IDENTITY);
    let convex_in_triangle_space = xform1.translation - mesh_trans.translation;

    let contact_breaking_threshold = manifold.contact_breaking_threshold;
    {
        let mut convex_triangle_callback = SphereTriangleCallback::new(
            manifold,
            convex_obj,
            concave_obj,
            convex_in_triangle_space,
            sphere_shape.get_radius(),
            contact_breaking_threshold,
            contact_added_callback,
        );

        let aabb = sphere_shape.get_aabb(convex_in_triangle_space);
        tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb);
    }

    if !manifold.point_cache.is_empty() {
        manifold.refresh_contact_points(convex_obj, concave_obj);
    }

    !manifold.point_cache.is_empty()
}
