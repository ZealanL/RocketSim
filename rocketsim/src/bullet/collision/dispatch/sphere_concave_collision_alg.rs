use glam::{Affine3A, Vec3A};

use crate::{
    bullet::{
        collision::{
            narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
            shapes::{
                bvh_triangle_mesh_shape::BvhTriangleMeshShape, sphere_shape::SphereShape,
                triangle_callback::ProcessTriangle, triangle_shape::TriangleShape,
            },
        },
        dynamics::rigid_body::RigidBody,
        linear_math::AffineExt,
    },
    shared::Aabb,
};

struct SphereTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: PersistentManifold,
    pub convex_obj: &'a RigidBody,
    pub tri_obj: &'a RigidBody,
    contact_added_callback: &'a mut T,
    sphere_center: Vec3A,
    sphere_radius: f32,
}

impl<'a, T: ContactAddedCallback> SphereTriangleCallback<'a, T> {
    pub fn new(
        convex_obj: &'a RigidBody,
        tri_obj: &'a RigidBody,
        sphere_center: Vec3A,
        sphere_radius: f32,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            manifold: PersistentManifold::new(convex_obj, tri_obj, is_swapped),
            convex_obj,
            tri_obj,
            sphere_center,
            sphere_radius,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> ProcessTriangle for SphereTriangleCallback<'_, T> {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        _tri_aabb: &Aabb,
        triangle_idx: usize,
    ) {
        let Some(contact_info) = triangle.intersect_sphere(
            self.sphere_center,
            self.sphere_radius,
            self.manifold.contact_breaking_threshold,
        ) else {
            return;
        };

        let normal_on_b = self
            .tri_obj
            .get_world_trans()
            .transform_vector3a(contact_info.result_normal);
        let point_in_world = self
            .tri_obj
            .get_world_trans()
            .transform_point3a(contact_info.contact_point);

        self.manifold.add_contact_point(
            self.convex_obj,
            self.tri_obj,
            normal_on_b,
            point_in_world,
            contact_info.depth,
            -1,
            triangle_idx as i32,
            self.contact_added_callback,
        );
    }
}

pub fn process_collision<T: ContactAddedCallback>(
    convex_obj: &RigidBody,
    sphere_shape: &SphereShape,
    concave_obj: &RigidBody,
    tri_mesh: &BvhTriangleMeshShape,
    is_swapped: bool,
    contact_added_callback: &mut T,
) -> Option<PersistentManifold> {
    let xform1 = convex_obj.get_world_trans();
    let xform2 = concave_obj.get_world_trans().transpose();
    let convex_in_triangle_space = Affine3A {
        matrix3: xform2.matrix3 * xform1.matrix3,
        translation: xform2.transform_point3a(xform1.translation),
    };

    let mut convex_triangle_callback = SphereTriangleCallback::new(
        convex_obj,
        concave_obj,
        convex_in_triangle_space.translation,
        sphere_shape.get_radius(),
        is_swapped,
        contact_added_callback,
    );

    let aabb = sphere_shape.get_aabb(&convex_in_triangle_space);
    tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb);

    if convex_triangle_callback.manifold.point_cache.is_empty() {
        None
    } else {
        convex_triangle_callback
            .manifold
            .refresh_contact_points(convex_obj, concave_obj);
        Some(convex_triangle_callback.manifold)
    }
}
