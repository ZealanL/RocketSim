use glam::{Affine3A, Vec3A};

use super::{
    concave_shape::ConcaveShape, triangle_callback::ProcessTriangle, triangle_mesh::TriangleMesh,
    triangle_shape::TriangleShape,
};
use crate::shared::Aabb;

pub struct TriangleMeshShape {
    pub concave_shape: ConcaveShape,
    pub local_aabb: Aabb,
}

impl TriangleMeshShape {
    pub fn new(mesh_interface: &TriangleMesh) -> Self {
        let concave_shape = ConcaveShape::default();

        Self {
            local_aabb: Self::calc_local_aabb(mesh_interface, concave_shape.collision_margin),
            concave_shape,
        }
    }

    pub fn get_aabb(&self, trans: &Affine3A) -> Aabb {
        let mut local_half_extents = 0.5 * (self.local_aabb.max - self.local_aabb.min);
        local_half_extents += Vec3A::splat(self.concave_shape.collision_margin);
        let local_center = 0.5 * (self.local_aabb.max + self.local_aabb.min);

        let abs_b = trans.matrix3.abs();
        let center = trans.transform_point3a(local_center);

        let extent = abs_b * local_half_extents;

        Aabb {
            min: center - extent,
            max: center + extent,
        }
    }

    pub fn process_all_triangles<T: ProcessTriangle>(
        mesh_interface: &TriangleMesh,
        callback: &mut T,
    ) {
        mesh_interface.internal_process_all_triangles(callback);
    }

    fn local_get_support_vertex(mesh_interface: &TriangleMesh, vec: Vec3A) -> Vec3A {
        let mut support_callback = SupportVertexCallback::new(vec, Affine3A::IDENTITY);

        Self::process_all_triangles(mesh_interface, &mut support_callback);

        support_callback.get_support_vertex_local()
    }

    fn calc_local_aabb(mesh_interface: &TriangleMesh, collision_margin: f32) -> Aabb {
        let mut min = Vec3A::ZERO;
        let mut max = Vec3A::ZERO;

        for i in 0..3 {
            let mut vec = Vec3A::ZERO;

            vec[i] = 1.0;
            let mut tmp = Self::local_get_support_vertex(mesh_interface, vec);
            max[i] = tmp[i] + collision_margin;

            vec[i] = -1.0;
            tmp = Self::local_get_support_vertex(mesh_interface, vec);
            min[i] = tmp[i] - collision_margin;
        }

        Aabb { min, max }
    }
}

struct SupportVertexCallback {
    support_vertex_local: Vec3A,
    max_dot: f32,
    support_vec_local: Vec3A,
}

impl ProcessTriangle for SupportVertexCallback {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        _tri_aabb: &Aabb,
        _triangle_idx: usize,
    ) {
        for vert in triangle.points {
            let dot = self.support_vec_local.dot(vert);
            if dot > self.max_dot {
                self.max_dot = dot;
                self.support_vertex_local = vert;
            }
        }
    }
}

impl SupportVertexCallback {
    pub fn new(support_vec_world: Vec3A, world_trans: Affine3A) -> Self {
        Self {
            support_vertex_local: Vec3A::ZERO,
            support_vec_local: world_trans.matrix3 * support_vec_world,
            max_dot: f32::MIN,
        }
    }

    pub const fn get_support_vertex_local(&self) -> Vec3A {
        self.support_vertex_local
    }
}
