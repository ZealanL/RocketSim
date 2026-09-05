use glam::{Affine3A, Vec3A};

use crate::bullet::collision::shapes::collision_shape::CollisionShapes;

/// Minkowski support wrapper for GJK/EPA.
/// Computes support points in world space for two convex shapes.
pub struct MinkowskiDiff<'a> {
    pub shape_a: &'a CollisionShapes,
    pub shape_b: &'a CollisionShapes,
    pub trans_a: Affine3A,
    pub trans_b: Affine3A,
}

impl<'a> MinkowskiDiff<'a> {
    pub const fn new(
        shape_a: &'a CollisionShapes,
        trans_a: Affine3A,
        shape_b: &'a CollisionShapes,
        trans_b: Affine3A,
    ) -> Self {
        Self {
            shape_a,
            shape_b,
            trans_a,
            trans_b,
        }
    }

    pub fn support0<const ENABLE_MAGIN: bool>(&self, dir_world: Vec3A) -> Vec3A {
        let dir_local = self.trans_a.matrix3.mul_transpose_vec3a(dir_world);
        let local_support = if ENABLE_MAGIN {
            // Bullet `btGjkEpa2` margined path uses `NonVirtual` (sphere
            // expansion for boxes), not the virtual per-axis padding.
            self.shape_a
                .local_get_supporting_vertex_nonvirtual(dir_local)
        } else {
            self.shape_a
                .local_get_support_vertex_without_margin(dir_local)
        };

        self.trans_a.transform_point3a(local_support)
    }

    pub fn support1<const ENABLE_MAGIN: bool>(&self, dir_world: Vec3A) -> Vec3A {
        let dir_local = self.trans_b.matrix3.mul_transpose_vec3a(dir_world);
        let local_support = if ENABLE_MAGIN {
            self.shape_b
                .local_get_supporting_vertex_nonvirtual(dir_local)
        } else {
            self.shape_b
                .local_get_support_vertex_without_margin(dir_local)
        };

        self.trans_b.transform_point3a(local_support)
    }

    /// Minkowski support point for the difference A - B.
    pub fn support<const ENABLE_MAGIN: bool>(&self, dir_world: Vec3A) -> Vec3A {
        self.support0::<ENABLE_MAGIN>(dir_world) - self.support1::<ENABLE_MAGIN>(-dir_world)
    }
}
