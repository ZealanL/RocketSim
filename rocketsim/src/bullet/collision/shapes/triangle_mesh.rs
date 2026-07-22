use glam::Vec3A;

use super::{triangle_callback::ProcessTriangle, triangle_shape::TriangleShape};

#[derive(Debug)]
pub struct TriangleMesh {
    triangles: Box<[TriangleShape]>,
}

impl TriangleMesh {
    pub fn new(verts: &[Vec3A], ids: &[usize]) -> Self {
        debug_assert_eq!(ids.len() % 3, 0);

        Self {
            triangles: ids
                .as_chunks::<3>()
                .0
                .iter()
                .map(|ids| TriangleShape::from_points_iter(ids.iter().map(|&j| verts[j])))
                .collect(),
        }
    }

    pub fn internal_process_all_triangles<T: ProcessTriangle>(&self, callback: &mut T) {
        for (i, triangle) in self.triangles.iter().enumerate() {
            callback.process_triangle(triangle, i);
        }
    }

    pub fn get_triangle(&self, idx: usize) -> &TriangleShape {
        &self.triangles[idx]
    }

    pub fn get_total_num_faces(&self) -> usize {
        self.triangles.len()
    }

    pub fn get_tris(&self) -> &[TriangleShape] {
        &self.triangles
    }
}
