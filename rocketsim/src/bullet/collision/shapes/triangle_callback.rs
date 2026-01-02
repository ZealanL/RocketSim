use glam::Vec4;

use super::triangle_shape::TriangleShape;
use crate::shared::Aabb;

pub trait ProcessTriangle {
    fn process_triangle(&mut self, triangle: &TriangleShape, tri_aabb: &Aabb, triangle_idx: usize);
}

pub trait ProcessRayTriangle {
    fn process_node(&mut self, triangle: &TriangleShape, active_mask: u8, lambda_max: &mut Vec4);
}
