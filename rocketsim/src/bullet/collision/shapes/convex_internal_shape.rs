use glam::Vec3A;

use super::collision_margin::CONVEX_DISTANCE_MARGIN;

pub struct ConvexInternalShape {
    pub implicit_dim: Vec3A,
    pub margin: f32,
}

impl Default for ConvexInternalShape {
    fn default() -> Self {
        Self {
            implicit_dim: Vec3A::ZERO,
            margin: CONVEX_DISTANCE_MARGIN,
        }
    }
}
