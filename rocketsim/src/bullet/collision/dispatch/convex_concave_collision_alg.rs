use glam::Vec3A;

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
    linear_math::AffineExt,
};

struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: &'a mut PersistentManifold,
    pub convex_obj: &'a RigidBody,
    pub tri_obj: &'a RigidBody,
    contact_added_callback: &'a mut T,
    current_triangle_index: usize,
}

impl<'a, T: ContactAddedCallback> ConvexTriangleCallback<'a, T> {
    pub fn new(
        manifold: &'a mut PersistentManifold,
        convex_obj: &'a RigidBody,
        tri_obj: &'a RigidBody,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            manifold,
            convex_obj,
            tri_obj,
            contact_added_callback,
            current_triangle_index: 0,
        }
    }

    /// Check if this is fully on one side of the triangle.
    /// Triangles are component-local. Move the face to world with the
    /// mesh body transform before the support test.
    fn is_triangle_separated(&self, triangle_normal: Vec3A, triangle_point: Vec3A) -> bool {
        let tri_trans = self.tri_obj.get_world_trans();
        let world_normal = tri_trans.transform_vector3a(triangle_normal);
        let world_point = tri_trans.transform_point3a(triangle_point);
        let convex = self.convex_obj.get_collision_shape();
        let convex_trans = self.convex_obj.get_world_trans();

        let local_pt = convex
            .local_get_supporting_vertex(convex_trans.matrix3.mul_transpose_vec3a(world_normal));
        let world_pt = convex_trans.transform_point3a(local_pt);

        let proj_dist_pt = world_normal.dot(world_pt);
        let proj_dist_tr = world_normal.dot(world_point);

        let dist = proj_dist_tr - proj_dist_pt;
        dist > self.manifold.contact_breaking_threshold
    }
}

impl<T: ContactAddedCallback> GjkResult for ConvexTriangleCallback<'_, T> {
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
            self.tri_obj.get_world_trans(),
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

pub fn process_collision_into<T: ContactAddedCallback>(
    convex_obj: &RigidBody,
    concave_obj: &RigidBody,
    tri_mesh: &BvhTriangleMeshShape,
    manifold: &mut PersistentManifold,
    contact_added_callback: &mut T,
) -> bool {
    {
        let mut convex_triangle_callback =
            ConvexTriangleCallback::new(manifold, convex_obj, concave_obj, contact_added_callback);

        // BVH holds local triangles. Move the convex AABB into mesh-local
        // space, mirroring the compound and sphere paths.
        let world_to_local = concave_obj.get_world_trans().transpose();
        let convex_aabb_world = convex_obj
            .get_collision_shape()
            .get_aabb(convex_obj.get_world_trans());
        let aabb = convex_aabb_world.transform(&world_to_local, 0.0);
        tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb);
    }

    // Skip the no-op empty refresh (see `refresh_contact_points`).
    if !manifold.point_cache.is_empty() {
        manifold.refresh_contact_points(convex_obj, concave_obj);
    }
    !manifold.point_cache.is_empty()
}
