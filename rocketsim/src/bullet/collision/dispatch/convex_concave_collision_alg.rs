use glam::{Affine3A, Vec3A};

use crate::bullet::{
    collision::{
        narrowphase::{
            gjk::{ClosestPointInput, GjkPairDetector, GjkResult},
            persistent_manifold::{ContactAddedCallback, PersistentManifold},
        },
        shapes::{
            bvh_triangle_mesh_shape::BvhTriangleMeshShape, collision_shape::CollisionShapes,
            triangle_callback::ProcessTriangle, triangle_shape::TriangleShape,
        },
    },
    dynamics::rigid_body::RigidBody,
};

struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: PersistentManifold,
    pub convex_obj: &'a RigidBody,
    pub tri_obj: &'a RigidBody,
    contact_added_callback: &'a mut T,
    current_triangle_index: usize,
}

impl<'a, T: ContactAddedCallback> ConvexTriangleCallback<'a, T> {
    pub fn new(
        convex_obj: &'a RigidBody,
        tri_obj: &'a RigidBody,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            manifold: PersistentManifold::new(convex_obj, tri_obj),
            convex_obj,
            tri_obj,
            contact_added_callback,
            current_triangle_index: 0,
        }
    }

    /// Check if this is fully on one side of the triangle
    fn is_triangle_separated(&self, triangle_normal: Vec3A, triangle_point: Vec3A) -> bool {
        let convex = self.convex_obj.get_collision_shape();
        let convex_trans = self.convex_obj.get_world_trans();

        let local_pt = convex
            .local_get_supporting_vertex(convex_trans.matrix3.mul_transpose_vec3a(triangle_normal));
        let world_pt = convex_trans.transform_point3a(local_pt);

        let proj_dist_pt = triangle_normal.dot(world_pt);
        let proj_dist_tr = triangle_normal.dot(triangle_point);

        let dist = proj_dist_tr - proj_dist_pt;
        dist > self.manifold.contact_breaking_threshold
    }
}

impl<'a, T: ContactAddedCallback> GjkResult for ConvexTriangleCallback<'a, T> {
    fn add_contact_point(&mut self, normal_on_b: Vec3A, point_on_b_world: Vec3A, depth: f32) {
        self.manifold.add_contact_point(
            self.convex_obj,
            self.tri_obj,
            normal_on_b,
            point_on_b_world,
            depth,
            Some(self.current_triangle_index),
            self.contact_added_callback,
        );
    }
}

impl<T: ContactAddedCallback> ProcessTriangle for ConvexTriangleCallback<'_, T> {
    fn process_triangle(&mut self, triangle: &TriangleShape, triangle_idx: usize) {
        if self.is_triangle_separated(triangle.normal, triangle.points[0])
            || self.is_triangle_separated(-triangle.normal, triangle.points[0])
        {
            return;
        }

        self.current_triangle_index = triangle_idx;

        let margin_a = self.convex_obj.get_collision_shape().get_margin();

        let input = ClosestPointInput::new(
            self.convex_obj.get_world_trans(),
            &Affine3A::IDENTITY,
            margin_a + self.manifold.contact_breaking_threshold,
        );

        let detector = GjkPairDetector::new(margin_a, 0.0);
        detector.get_closest_points(
            &input,
            self.convex_obj.get_collision_shape(),
            &CollisionShapes::Triangle(*triangle),
            self,
        );
    }
}

pub fn process_collision<T: ContactAddedCallback>(
    convex_obj: &RigidBody,
    concave_obj: &RigidBody,
    tri_mesh: &BvhTriangleMeshShape,
    contact_added_callback: &mut T,
) -> Option<PersistentManifold> {
    let mut convex_triangle_callback =
        ConvexTriangleCallback::new(convex_obj, concave_obj, contact_added_callback);

    let aabb = convex_obj
        .get_collision_shape()
        .get_aabb(convex_obj.get_world_trans());
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
