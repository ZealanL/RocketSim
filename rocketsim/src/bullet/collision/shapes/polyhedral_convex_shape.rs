use glam::{Affine3A, Vec3A};

use crate::{
    bullet::{collision::shapes::convex_internal_shape::ConvexInternalShape, linear_math::max_dot},
    shared::Aabb,
};

fn calc_local_aabb(points: &[Vec3A], collision_margin: f32) -> Aabb {
    const DIRECTIONS: [Vec3A; 6] = [
        Vec3A::X,
        Vec3A::Y,
        Vec3A::Z,
        Vec3A::NEG_X,
        Vec3A::NEG_Y,
        Vec3A::NEG_Z,
    ];

    let mut supporting = [Vec3A::ZERO; DIRECTIONS.len()];
    for (support_vertex, direction) in supporting.iter_mut().zip(DIRECTIONS) {
        *support_vertex = max_dot(points, direction);
    }

    let mut local_aabb_max = Vec3A::ZERO;
    let mut local_aabb_min = Vec3A::ZERO;

    for i in 0..3 {
        local_aabb_max[i] = supporting[i][i];
        local_aabb_min[i] = supporting[i + 3][i];
    }

    Aabb {
        max: local_aabb_max + collision_margin,
        min: local_aabb_min - collision_margin,
    }
}

pub struct PolyhedralConvexShape {
    convex_internal_shape: ConvexInternalShape,
    local_aabb: Aabb,
}

impl PolyhedralConvexShape {
    pub fn new(points: &[Vec3A]) -> Self {
        let convex_internal_shape = ConvexInternalShape::default();
        let local_aabb = calc_local_aabb(points, convex_internal_shape.margin);

        Self {
            convex_internal_shape,
            local_aabb,
        }
    }

    #[inline]
    pub fn get_margin(&self) -> f32 {
        self.convex_internal_shape.margin
    }

    #[inline]
    pub fn get_ident_aabb(&self) -> &Aabb {
        &self.local_aabb
    }

    #[inline]
    pub fn get_aabb(&self, trans: &Affine3A) -> Aabb {
        self.local_aabb.transform(trans, self.get_margin())
    }
}
