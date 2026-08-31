use glam::{Affine3A, Vec3A, Vec4};

use super::polyhedral_convex_shape::PolyhedralConvexShape;
use crate::{
    bullet::{
        collision::{
            dispatch::quad_ray_callbacks::{BridgeTriQuadRayCallback, QuadRayResultCallback},
            narrowphase::gjk::calc_time_of_impact,
            shapes::collision_margin::CONVEX_DISTANCE_MARGIN,
        },
        linear_math::max_dot,
    },
    shared::{Aabb, QuadRayInfo},
};

pub struct ConvexHullShape {
    polyhedral_convex_shape: PolyhedralConvexShape,
    unscaled_points: Box<[Vec3A]>,
    simd_unscaled_points: Box<[[Vec4; 3]]>,
    /// When true this eight-point hull is a btBoxShape proxy. Bullet's box
    /// support uses per-component sign selection (including + for zero), while
    /// the generic Bullet margin path expands the corner along the normalized
    /// query direction.
    box_support: bool,
}

impl ConvexHullShape {
    pub fn new(unscaled_points: Box<[Vec3A]>) -> Self {
        Self::new_with_margin(unscaled_points, CONVEX_DISTANCE_MARGIN)
    }

    pub fn new_with_margin(unscaled_points: Box<[Vec3A]>, collision_margin: f32) -> Self {
        let simd_unscaled_points: Box<_> = unscaled_points
            .as_chunks::<4>()
            .0
            .iter()
            .map(|chunk| {
                [
                    Vec4::new(chunk[0].x, chunk[1].x, chunk[2].x, chunk[3].x),
                    Vec4::new(chunk[0].y, chunk[1].y, chunk[2].y, chunk[3].y),
                    Vec4::new(chunk[0].z, chunk[1].z, chunk[2].z, chunk[3].z),
                ]
            })
            .collect();

        Self {
            polyhedral_convex_shape: PolyhedralConvexShape::new_with_margin(
                &simd_unscaled_points,
                &unscaled_points,
                collision_margin,
            ),
            unscaled_points,
            simd_unscaled_points,
            box_support: false,
        }
    }

    pub fn new_box_with_margin(half_extents: Vec3A, collision_margin: f32) -> Self {
        let points = [
            Vec3A::new(-half_extents.x, -half_extents.y, -half_extents.z),
            Vec3A::new(half_extents.x, -half_extents.y, -half_extents.z),
            Vec3A::new(-half_extents.x, half_extents.y, -half_extents.z),
            Vec3A::new(half_extents.x, half_extents.y, -half_extents.z),
            Vec3A::new(-half_extents.x, -half_extents.y, half_extents.z),
            Vec3A::new(half_extents.x, -half_extents.y, half_extents.z),
            Vec3A::new(-half_extents.x, half_extents.y, half_extents.z),
            Vec3A::new(half_extents.x, half_extents.y, half_extents.z),
        ];
        let mut shape = Self::new_with_margin(points.into(), collision_margin);
        shape.box_support = true;
        shape
    }

    #[inline]
    pub fn local_get_supporting_vertex_without_margin(&self, vec: Vec3A) -> Vec3A {
        if self.box_support {
            let h =
                self.polyhedral_convex_shape.get_ident_aabb().max - Vec3A::splat(self.get_margin());
            return Vec3A::new(
                if vec.x >= 0.0 { h.x } else { -h.x },
                if vec.y >= 0.0 { h.y } else { -h.y },
                if vec.z >= 0.0 { h.z } else { -h.z },
            );
        }
        max_dot(&self.simd_unscaled_points, &self.unscaled_points, vec)
    }

    pub fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        if self.box_support {
            // btConvexShape::localGetSupportVertexNonVirtual (used by
            // btGjkEpa2 with margins) selects the margin-free box corner,
            // then expands it by margin along the normalized query vector.
            // It does not use btBoxShape::localGetSupportingVertex's
            // component-wise full-extents corner.
            let h =
                self.polyhedral_convex_shape.get_ident_aabb().max - Vec3A::splat(self.get_margin());
            let vec_norm = if vec.length_squared() < f32::EPSILON * f32::EPSILON {
                crate::bullet::linear_math::bullet_normalize(Vec3A::NEG_ONE)
            } else {
                crate::bullet::linear_math::bullet_normalize(vec)
            };
            let corner = Vec3A::new(
                if vec_norm.x >= 0.0 { h.x } else { -h.x },
                if vec_norm.y >= 0.0 { h.y } else { -h.y },
                if vec_norm.z >= 0.0 { h.z } else { -h.z },
            );
            return corner + self.get_margin() * vec_norm;
        }
        let mut sup_vertex = self.local_get_supporting_vertex_without_margin(vec);

        debug_assert_ne!(self.polyhedral_convex_shape.get_margin(), 0.0);
        let vec_norm = vec.normalize_or(Vec3A::NEG_ONE);
        sup_vertex += self.polyhedral_convex_shape.get_margin() * vec_norm;

        sup_vertex
    }

    #[inline]
    pub const fn get_margin(&self) -> f32 {
        self.polyhedral_convex_shape.get_margin()
    }

    fn get_ident_aabb_slow(&self) -> Aabb {
        let margin = self.get_margin();

        let mut aabb = Aabb::ZERO;
        for i in 0..3 {
            let mut vec = Vec3A::ZERO;

            vec[i] = 1.0;
            let sv = self.local_get_supporting_vertex(vec);
            aabb.max[i] = sv[i] + margin;

            vec[i] = -1.0;
            let sv = self.local_get_supporting_vertex(vec);
            aabb.min[i] = sv[i] - margin;
        }

        aabb
    }

    #[inline]
    pub const fn get_ident_aabb(&self) -> &Aabb {
        self.polyhedral_convex_shape.get_ident_aabb()
    }

    #[inline]
    pub fn get_aabb(&self, trans: &Affine3A) -> Aabb {
        self.polyhedral_convex_shape.get_aabb(trans)
    }

    pub fn calculate_local_intertia(&self, mass: f32) -> Vec3A {
        let margin = self.polyhedral_convex_shape.get_margin();

        let aabb = self.get_ident_aabb_slow();
        let half_extents = (aabb.max - aabb.min) * 0.5;

        let l = 2.0 * (half_extents + margin);
        let l2 = l * l;
        let scaled_mass = mass * 0.083_333_33;

        scaled_mass * Vec3A::new(l2.y + l2.z, l2.x + l2.z, l2.x + l2.y)
    }

    pub fn perform_quad_raycast<T: QuadRayResultCallback>(
        &self,
        result_callback: &mut BridgeTriQuadRayCallback<'_, T>,
        ray_info: &QuadRayInfo<'_>,
    ) {
        let hull_aabb = self.get_ident_aabb();
        if !ray_info.aabb.intersects(hull_aabb) {
            return;
        }

        let (origins, inv_dirs) = ray_info.calc_pos_dir();
        let mask = QuadRayInfo::intersect_quad_ray_aabb(
            &origins,
            &inv_dirs,
            hull_aabb,
            result_callback.hit_fraction,
        );

        for i in 0..4 {
            if (mask & (1 << i)) == 0 {
                continue;
            }

            self.internal_perform_raycast(
                result_callback,
                ray_info.ray_sources[i],
                ray_info.ray_targets[i],
                i,
            );
        }
    }

    fn internal_perform_raycast<T: QuadRayResultCallback>(
        &self,
        result_callback: &mut BridgeTriQuadRayCallback<'_, T>,
        ray_source: Vec3A,
        ray_target: Vec3A,
        ray_idx: usize,
    ) {
        if let Some(cast_result) = calc_time_of_impact(self, ray_source, ray_target) {
            result_callback.report_hit(cast_result.normal, cast_result.fraction, ray_idx);
        }
    }
}
