use glam::{Affine3A, Vec3A};

use crate::bullet::{collision::shapes::collision_shape::CollisionShapes, linear_math::AffineExt};

/// Minkowski support wrapper for GJK/EPA.
/// Computes support points in world space for two convex shapes.
pub struct MinkowskiDiff<'a> {
    pub shape_a: &'a CollisionShapes,
    pub shape_b: &'a CollisionShapes,
    /// Transform from shape-B local coordinates into shape-A local space.
    /// Bullet's btGjkEpaSolver2 performs its EPA in this relative frame,
    /// avoiding large world-space translations and preserving its float path.
    pub b_to_a: Affine3A,
    pub trans_a: Affine3A,
}

impl<'a> MinkowskiDiff<'a> {
    pub fn new(
        shape_a: &'a CollisionShapes,
        trans_a: Affine3A,
        shape_b: &'a CollisionShapes,
        trans_b: Affine3A,
    ) -> Self {
        Self {
            shape_a,
            shape_b,
            b_to_a: trans_a.transpose() * trans_b,
            trans_a,
        }
    }

    pub fn support0<const ENABLE_MAGIN: bool>(&self, dir_a: Vec3A) -> Vec3A {
        if ENABLE_MAGIN {
            self.shape_a.local_get_supporting_vertex(dir_a)
        } else {
            self.shape_a.local_get_support_vertex_without_margin(dir_a)
        }
    }

    pub fn support1<const ENABLE_MAGIN: bool>(&self, dir_a: Vec3A) -> Vec3A {
        let dir_b = self.b_to_a.matrix3.transpose() * dir_a;
        let local_support = if ENABLE_MAGIN {
            self.shape_b.local_get_supporting_vertex(dir_b)
        } else {
            self.shape_b.local_get_support_vertex_without_margin(dir_b)
        };
        self.b_to_a.transform_point3a(local_support)
    }

    /// Minkowski support point for the difference A - B.
    pub fn support<const ENABLE_MAGIN: bool>(&self, dir_world: Vec3A) -> Vec3A {
        self.support0::<ENABLE_MAGIN>(dir_world) - self.support1::<ENABLE_MAGIN>(-dir_world)
    }
}
