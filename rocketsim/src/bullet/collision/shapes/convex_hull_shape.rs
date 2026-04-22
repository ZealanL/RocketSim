use glam::{Affine3A, Vec3A};

use super::polyhedral_convex_shape::PolyhedralConvexShape;
use crate::{
    bullet::{
        collision::{
            dispatch::ray_packet_callbacks::{
                BridgeTriangleRaycastPacketCallback, RayResultCallback,
            },
            narrowphase::gjk::calc_time_of_impact,
        },
        linear_math::max_dot,
    },
    shared::{Aabb, RayPacketInfo},
};

pub struct ConvexHullShape {
    polyhedral_convex_shape: PolyhedralConvexShape,
    unscaled_points: Box<[Vec3A]>,
}

impl ConvexHullShape {
    pub fn new(points: Box<[Vec3A]>) -> Self {
        Self {
            polyhedral_convex_shape: PolyhedralConvexShape::new(&points),
            unscaled_points: points,
        }
    }

    #[inline]
    pub fn local_get_supporting_vertex_without_margin(&self, vec: Vec3A) -> Vec3A {
        max_dot(&self.unscaled_points, vec)
    }

    pub fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        let mut sup_vertex = self.local_get_supporting_vertex_without_margin(vec);

        debug_assert_ne!(self.polyhedral_convex_shape.get_margin(), 0.0);
        let vec_norm = vec.normalize_or(Vec3A::NEG_ONE);
        sup_vertex += self.polyhedral_convex_shape.get_margin() * vec_norm;

        sup_vertex
    }

    #[inline]
    pub fn get_margin(&self) -> f32 {
        self.polyhedral_convex_shape.get_margin()
    }

    fn get_aabb_slow(&self, trans: &Affine3A) -> Aabb {
        let margin = self.polyhedral_convex_shape.get_margin();

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

    #[inline]
    pub fn get_ident_aabb(&self) -> &Aabb {
        self.polyhedral_convex_shape.get_ident_aabb()
    }

    #[inline]
    pub fn get_aabb(&self, trans: &Affine3A) -> Aabb {
        self.polyhedral_convex_shape.get_aabb(trans)
    }

    pub fn calculate_local_intertia(&self, mass: f32) -> Vec3A {
        let margin = self.polyhedral_convex_shape.get_margin();

        let aabb = self.get_aabb_slow(&Affine3A::IDENTITY);
        let half_extents = (aabb.max - aabb.min) * 0.5;

        let l = 2.0 * (half_extents + margin);
        let l2 = l * l;
        let scaled_mass = mass * 0.08333333;

        scaled_mass * Vec3A::new(l2.y + l2.z, l2.x + l2.z, l2.x + l2.y)
    }

    pub fn perform_raycast<T: RayResultCallback>(
        &self,
        result_callback: &mut BridgeTriangleRaycastPacketCallback<'_, T>,
        ray_info: &mut RayPacketInfo<'_>,
    ) {
        let hull_aabb = self.get_ident_aabb();
        if !ray_info.aabb.intersects(hull_aabb) {
            return;
        }

        let (origins, inv_dirs) = ray_info.calc_pos_dir();
        let mask = RayPacketInfo::intersect_ray_aabb_packet(
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

    fn internal_perform_raycast<T: RayResultCallback>(
        &self,
        result_callback: &mut BridgeTriangleRaycastPacketCallback<'_, T>,
        ray_source: Vec3A,
        ray_target: Vec3A,
        ray_idx: usize,
    ) {
        if let Some(cast_result) = calc_time_of_impact(self, ray_source, ray_target) {
            result_callback.report_hit(cast_result.normal, cast_result.fraction, ray_idx);
        }
    }
}
