use glam::Vec3A;

use super::{
    triangle_callback::ProcessTriangle, triangle_mesh::TriangleMesh, triangle_shape::TriangleShape,
};
use crate::shared::Aabb;

pub struct TriangleMeshShape {
    pub local_aabb: Aabb,
}

impl TriangleMeshShape {
    pub fn new(mesh_interface: &TriangleMesh) -> Self {
        Self {
            local_aabb: Self::calc_local_aabb(mesh_interface),
        }
    }

    pub fn get_ident_aabb(&self) -> Aabb {
        let local_half_extents = 0.5 * (self.local_aabb.max - self.local_aabb.min);
        let local_center = 0.5 * (self.local_aabb.max + self.local_aabb.min);

        Aabb {
            min: local_center - local_half_extents,
            max: local_center + local_half_extents,
        }
    }

    pub fn process_all_triangles<T: ProcessTriangle>(
        mesh_interface: &TriangleMesh,
        callback: &mut T,
    ) {
        mesh_interface.internal_process_all_triangles(callback);
    }

    fn local_get_support_vertex(mesh_interface: &TriangleMesh, vec: Vec3A) -> Vec3A {
        let mut support_callback = SupportVertexCallback::new(vec);

        Self::process_all_triangles(mesh_interface, &mut support_callback);

        support_callback.get_support_vertex_local()
    }

    fn calc_local_aabb(mesh_interface: &TriangleMesh) -> Aabb {
        let mut min = Vec3A::ZERO;
        let mut max = Vec3A::ZERO;

        for i in 0..3 {
            let mut vec = Vec3A::ZERO;

            vec[i] = 1.0;
            max[i] = Self::local_get_support_vertex(mesh_interface, vec)[i];

            vec[i] = -1.0;
            min[i] = Self::local_get_support_vertex(mesh_interface, vec)[i];
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
    fn process_triangle(&mut self, triangle: &TriangleShape, _triangle_idx: usize) {
        let dots = Vec3A::new(
            self.support_vec_local.dot(triangle.points[0]),
            self.support_vec_local.dot(triangle.points[1]),
            self.support_vec_local.dot(triangle.points[2]),
        );

        let max_dot = dots.max_element();
        if max_dot > self.max_dot {
            self.max_dot = max_dot;
            self.support_vertex_local = triangle.points[dots.max_position()];
        }
    }
}

impl SupportVertexCallback {
    pub fn new(support_vec_local: Vec3A) -> Self {
        Self {
            support_vertex_local: Vec3A::ZERO,
            support_vec_local,
            max_dot: f32::MIN,
        }
    }

    pub const fn get_support_vertex_local(&self) -> Vec3A {
        self.support_vertex_local
    }
}
