use glam::{Affine3A, Vec3A, Vec3Swizzles};

use super::{
    collision_margin::CONVEX_DISTANCE_MARGIN, convex_internal_shape::ConvexInternalShape,
    polyhedral_convex_shape::PolyhedralConvexShape,
};
use crate::bullet::linear_math::aabb_util_2::transform_aabb;
use crate::shared::Aabb;

pub struct BoxShape {
    polyhedral_convex_shape: PolyhedralConvexShape,
}

impl BoxShape {
    pub fn new(box_half_extents: Vec3A) -> Self {
        Self {
            polyhedral_convex_shape: PolyhedralConvexShape {
                convex_internal_shape: ConvexInternalShape {
                    implicit_shape_dimensions: box_half_extents - CONVEX_DISTANCE_MARGIN,
                    collision_margin: {
                        let safe_margin = 0.1 * box_half_extents.min_element();
                        safe_margin.min(CONVEX_DISTANCE_MARGIN)
                    },
                },
            },
        }
    }

    #[inline]
    pub fn get_half_extents(&self) -> Vec3A {
        self.polyhedral_convex_shape
            .convex_internal_shape
            .implicit_shape_dimensions
    }

    pub fn get_margin(&self) -> f32 {
        self.polyhedral_convex_shape
            .convex_internal_shape
            .collision_margin
    }

    pub fn get_aabb(&self, t: &Affine3A) -> Aabb {
        transform_aabb(
            self.polyhedral_convex_shape
                .convex_internal_shape
                .implicit_shape_dimensions,
            self.polyhedral_convex_shape
                .convex_internal_shape
                .collision_margin,
            t,
        )
    }

    pub fn calculate_local_intertia(&self, mass: f32) -> Vec3A {
        let l = 2.0 * self.get_half_extents();
        let yxx = l.yxx();
        let zzy = l.zzy();

        mass / 12.0 * (yxx * yxx + zzy * zzy)
    }

    pub fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        self.get_half_extents() * vec.signum()
    }
}
