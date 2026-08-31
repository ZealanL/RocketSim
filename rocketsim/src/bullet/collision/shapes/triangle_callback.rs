use glam::Vec4;

use super::triangle_shape::TriangleShape;

pub trait ProcessTriangle {
    fn process_triangle(&mut self, triangle: &TriangleShape, triangle_idx: usize);

    /// Diagnostic hook used by parity benchmarks to isolate callback-order
    /// effects.  Compound-car callbacks may opt into stable triangle-index
    /// ordering without changing sphere/mesh traversal.
    fn stable_triangle_order(&self) -> bool {
        false
    }
}

pub trait ProcessQuadRayTriangle {
    fn process_node(&mut self, triangle: &TriangleShape, active_mask: u8, lambda_max: &mut Vec4);
}
