use glam::{Affine3A, Vec3A};

use crate::{
    bullet::collision::shapes::polyhedral_convex_shape::PolyhedralConvexAabbCachingShape,
    shared::Aabb,
};

pub struct ConvexHullShape {
    polyhedral_convex_aabb_caching_shape: PolyhedralConvexAabbCachingShape,
    unscaled_points: Vec<Vec3A>,
}

impl ConvexHullShape {
    pub fn new(points: Vec<Vec3A>) -> Self {
        Self {
            polyhedral_convex_aabb_caching_shape: PolyhedralConvexAabbCachingShape::new(&points),
            unscaled_points: points,
        }
    }

    fn local_get_supporting_vertex_without_margin(&self, vec: Vec3A) -> Vec3A {
        let mut sup_vec = Vec3A::ZERO;
        let mut max_dot = -f32::INFINITY;

        for &point in &self.unscaled_points {
            let dot = vec.dot(point);
            if dot > max_dot {
                sup_vec = point;
                max_dot = dot;
            }
        }

        sup_vec
    }

    fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        let mut sup_vertex = self.local_get_supporting_vertex_without_margin(vec);

        debug_assert_ne!(self.polyhedral_convex_aabb_caching_shape.get_margin(), 0.0);
        let vec_norm = vec.normalize_or(Vec3A::NEG_ONE);
        sup_vertex += self.polyhedral_convex_aabb_caching_shape.get_margin() * vec_norm;

        sup_vertex
    }

    fn get_aabb(&self, trans: &Affine3A) -> Aabb {
        let margin = self.polyhedral_convex_aabb_caching_shape.get_margin();

        let mut aabb = Aabb::ZERO;

        for i in 0..3 {
            let mut vec = Vec3A::ZERO;

            vec[i] = 1.0;
            let sv = self.local_get_supporting_vertex(trans.matrix3 * vec);
            let tmp = trans.transform_point3a(sv);
            aabb.max[i] = tmp[i] + margin;

            vec[i] = -1.0;
            let sv = self.local_get_supporting_vertex(trans.matrix3 * vec);
            let tmp = trans.transform_point3a(sv);
            aabb.min[i] = tmp[i] - margin;
        }

        aabb
    }

    pub fn calculate_local_intertia(&self, mass: f32) -> Vec3A {
        let margin = self.polyhedral_convex_aabb_caching_shape.get_margin();

        let aabb = self.get_aabb(&Affine3A::IDENTITY);
        let half_extents = (aabb.max - aabb.min) * 0.5;

        let l = 2.0 * (half_extents + margin);
        let l2 = l * l;
        let scaled_mass = mass * 0.08333333;

        scaled_mass * Vec3A::new(l2.y + l2.z, l2.x + l2.z, l2.x + l2.y)
    }
}
