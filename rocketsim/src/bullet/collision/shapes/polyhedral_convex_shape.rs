use glam::{Affine3A, Vec3A};

use crate::{
    bullet::collision::shapes::{
        convex_internal_shape::ConvexInternalShape, convex_polyhedron::ConvexPolyhedron,
    },
    shared::Aabb,
};

fn unit_vector_get_supporting_vertex_without_margin(
    directions: &[Vec3A],
    support_vertices_out: &mut [Vec3A],
    points: &[Vec3A],
) {
    for (&direction, support_vertex) in directions.iter().zip(support_vertices_out) {
        let mut max_dot = f32::NEG_INFINITY;
        for &point in points {
            let dot = direction.dot(point);
            if dot > max_dot {
                *support_vertex = point;
                max_dot = dot;
            }
        }
    }
}

struct PolyhedralConvexShape {
    convex_internal_shape: ConvexInternalShape,
    polyhedron: Option<ConvexPolyhedron>,
}

impl PolyhedralConvexShape {
    fn new() -> Self {
        Self {
            convex_internal_shape: ConvexInternalShape::default(),
            polyhedron: None,
        }
    }

    fn calc_local_aabb(&self, points: &[Vec3A]) -> Aabb {
        const DIRECTIONS: [Vec3A; 6] = [
            Vec3A::X,
            Vec3A::Y,
            Vec3A::Z,
            Vec3A::NEG_X,
            Vec3A::NEG_Y,
            Vec3A::NEG_Z,
        ];

        let mut supporting = [Vec3A::ZERO; DIRECTIONS.len()];

        unit_vector_get_supporting_vertex_without_margin(&DIRECTIONS, &mut supporting, points);

        let mut local_aabb_max = Vec3A::ZERO;
        let mut local_aabb_min = Vec3A::ZERO;

        for i in 0..3 {
            local_aabb_max[i] = supporting[i][i];
            local_aabb_min[i] = supporting[i + 3][i];
        }

        Aabb {
            max: local_aabb_max + self.convex_internal_shape.margin,
            min: local_aabb_min - self.convex_internal_shape.margin,
        }
    }
}

pub struct PolyhedralConvexAabbCachingShape {
    polyhedral_convex_shape: PolyhedralConvexShape,
    local_aabb: Aabb,
}

impl PolyhedralConvexAabbCachingShape {
    pub fn new(points: &[Vec3A]) -> Self {
        let polyhedral_convex_shape = PolyhedralConvexShape::new();
        let local_aabb = polyhedral_convex_shape.calc_local_aabb(points);

        Self {
            polyhedral_convex_shape,
            local_aabb,
        }
    }

    #[inline]
    pub fn get_margin(&self) -> f32 {
        self.polyhedral_convex_shape.convex_internal_shape.margin
    }

    #[inline]
    pub fn get_aabb_ident(&self) -> &Aabb {
        &self.local_aabb
    }

    #[inline]
    pub fn get_aabb(&self, trans: &Affine3A) -> Aabb {
        self.local_aabb.transform(trans, self.get_margin())
    }
}
