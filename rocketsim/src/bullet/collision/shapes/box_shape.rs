use glam::{Affine3A, Vec3A, Vec3Swizzles};

use super::{collision_margin::CONVEX_DISTANCE_MARGIN, convex_internal_shape::ConvexInternalShape};
use crate::shared::Aabb;

pub struct BoxShape {
    internal_shape: ConvexInternalShape,
}

impl BoxShape {
    pub fn new(box_half_extents: Vec3A) -> Self {
        let safe_margin = 0.1 * box_half_extents.min_element();
        let margin = safe_margin.min(CONVEX_DISTANCE_MARGIN);
        Self {
            internal_shape: ConvexInternalShape {
                implicit_dim: box_half_extents - margin,
                margin,
            },
        }
    }

    #[inline]
    pub const fn get_half_extents(&self) -> Vec3A {
        self.internal_shape.implicit_dim
    }

    pub const fn get_margin(&self) -> f32 {
        self.internal_shape.margin
    }

    pub fn get_aabb(&self, t: &Affine3A) -> Aabb {
        Aabb::from_half_extents_transform(
            self.internal_shape.implicit_dim,
            self.internal_shape.margin,
            t,
        )
    }

    pub fn calculate_local_intertia(&self, mass: f32) -> Vec3A {
        let l = 2.0 * (self.get_half_extents() + self.get_margin());
        let yxx = l.yxx();
        let zzy = l.zzy();

        mass / 12.0 * (yxx * yxx + zzy * zzy)
    }

    pub fn local_get_supporting_vertex_without_margin(&self, vec: Vec3A) -> Vec3A {
        let support_sign = Vec3A::select(vec.cmplt(Vec3A::ZERO), Vec3A::NEG_ONE, Vec3A::ONE);
        self.get_half_extents() * support_sign
    }

    pub fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        let support = self.local_get_supporting_vertex_without_margin(vec);
        support + self.get_margin() * support.signum()
    }
}
