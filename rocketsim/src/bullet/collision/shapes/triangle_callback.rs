use glam::Vec4;

use super::triangle_shape::TriangleShape;

pub trait ProcessTriangle {
    fn process_triangle(&mut self, triangle: &TriangleShape, triangle_idx: usize);
}

pub trait ProcessQuadRayTriangle {
    fn process_node(&mut self, triangle: &TriangleShape, active_mask: u8, lambda_max: &mut Vec4);
}
