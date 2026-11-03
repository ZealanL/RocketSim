use glam::{Affine3A, Mat3A, Vec3A};

use crate::bullet::collision::shapes::collision_shape::CollisionShapes;

pub struct MinkowskiDiff<'a> {
    pub shape_a: &'a CollisionShapes,
    pub shape_b: &'a CollisionShapes,
    toshape1: Mat3A,
    toshape0: Affine3A,
}

impl<'a> MinkowskiDiff<'a> {
    pub fn new(
        shape_a: &'a CollisionShapes,
        trans_a: Affine3A,
        shape_b: &'a CollisionShapes,
        trans_b: Affine3A,
    ) -> Self {
        let trans_a_basis_t = trans_a.matrix3.transpose();
        let toshape1 = trans_b.matrix3.transpose() * trans_a.matrix3;
        let toshape0 = Affine3A {
            matrix3: trans_a_basis_t * trans_b.matrix3,
            translation: trans_a_basis_t * (trans_b.translation - trans_a.translation),
        };

        Self {
            shape_a,
            shape_b,
            toshape1,
            toshape0,
        }
    }

    pub const fn margin_a(&self) -> f32 {
        self.shape_a.get_margin()
    }

    pub fn support0_with<const ENABLE_MARGIN: bool>(&self, d: Vec3A) -> Vec3A {
        local_support::<ENABLE_MARGIN>(self.shape_a, d)
    }

    pub fn support1_with<const ENABLE_MARGIN: bool>(&self, d: Vec3A) -> Vec3A {
        let d_b = self.toshape1 * d;
        let local = local_support::<ENABLE_MARGIN>(self.shape_b, d_b);
        self.toshape0.transform_point3a(local)
    }

    pub fn support_with<const ENABLE_MARGIN: bool>(&self, d: Vec3A) -> Vec3A {
        self.support0_with::<ENABLE_MARGIN>(d) - self.support1_with::<ENABLE_MARGIN>(-d)
    }

    pub fn support0_without_margin(&self, d: Vec3A) -> Vec3A {
        self.support0_with::<false>(d)
    }

    pub fn support1_without_margin(&self, d: Vec3A) -> Vec3A {
        self.support1_with::<false>(d)
    }
}

fn local_support<const ENABLE_MARGIN: bool>(shape: &CollisionShapes, d: Vec3A) -> Vec3A {
    if !ENABLE_MARGIN {
        return shape.local_get_support_vertex_without_margin(d);
    }

    let mut vecnorm = d;
    if vecnorm.length_squared() < f32::EPSILON * f32::EPSILON {
        vecnorm = Vec3A::new(-1.0, -1.0, -1.0);
    }
    vecnorm = vecnorm * (1.0 / vecnorm.length());
    shape.local_get_support_vertex_without_margin(vecnorm) + shape.get_margin() * vecnorm
}
