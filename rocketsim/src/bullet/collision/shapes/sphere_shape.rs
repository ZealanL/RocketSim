use core::cmp::Ordering;

use glam::{Vec3A, Vec4};

use super::convex_internal_shape::ConvexInternalShape;
use crate::{
    bullet::collision::{
        dispatch::quad_ray_callbacks::{BridgeTriQuadRayCallback, QuadRayResultCallback},
        narrowphase::gjk::calc_point_time_of_impact,
    },
    shared::{Aabb, QuadRayInfo},
};

pub const SPHERE_RADIUS_MARGIN: f32 = 0.08;

#[derive(Clone, Copy)]
pub struct SphereShape {
    pub convex_internal_shape: ConvexInternalShape,
}

impl SphereShape {
    #[inline]
    pub const fn new(radius: f32) -> Self {
        Self {
            convex_internal_shape: ConvexInternalShape {
                implicit_dim: Vec3A::new(radius, 0.0, 0.0),
                margin: radius,
            },
        }
    }

    #[inline]
    pub const fn get_radius(&self) -> f32 {
        self.convex_internal_shape.margin
    }

    #[inline]
    pub const fn get_margin(&self) -> f32 {
        self.get_radius()
    }

    pub fn get_aabb(&self, center: Vec3A) -> Aabb {
        let margin = self.get_margin() + SPHERE_RADIUS_MARGIN;
        let extent = Vec3A::splat(margin);

        Aabb {
            min: center - extent,
            max: center + extent,
        }
    }

    pub fn calculate_local_inertia(&self, mass: f32) -> Vec3A {
        Vec3A::splat(0.4 * mass * self.get_margin() * self.get_margin())
    }

    pub fn local_get_supporting_vertex(&self, vec: Vec3A) -> Vec3A {
        self.get_margin() * vec.try_normalize().unwrap()
    }

    /// Distance from a shape-space point to the sphere surface and the outward normal.
    /// The sphere centre is the shape origin.
    pub(crate) fn closest_point(&self, point: Vec3A) -> Option<(f32, Vec3A)> {
        let len_sq = point.length_squared();
        let len = len_sq.sqrt();
        if len_sq.partial_cmp(&(f32::EPSILON * f32::EPSILON)) != Some(Ordering::Greater) {
            return None;
        }

        let rlen = 1.0 / len;
        Some((1.0 / rlen - self.get_radius(), point * rlen))
    }

    pub fn perform_quad_raycast<T: QuadRayResultCallback>(
        &self,
        result_callback: &mut BridgeTriQuadRayCallback<T>,
        ray_info: &QuadRayInfo,
    ) {
        let sources = ray_info.ray_sources;
        let targets = ray_info.ray_targets;

        let source_x = Vec4::new(sources[0].x, sources[1].x, sources[2].x, sources[3].x);
        let source_y = Vec4::new(sources[0].y, sources[1].y, sources[2].y, sources[3].y);
        let source_z = Vec4::new(sources[0].z, sources[1].z, sources[2].z, sources[3].z);

        let target_x = Vec4::new(targets[0].x, targets[1].x, targets[2].x, targets[3].x);
        let target_y = Vec4::new(targets[0].y, targets[1].y, targets[2].y, targets[3].y);
        let target_z = Vec4::new(targets[0].z, targets[1].z, targets[2].z, targets[3].z);

        let ray_aabb_min_x = source_x.min(target_x);
        let ray_aabb_min_y = source_y.min(target_y);
        let ray_aabb_min_z = source_z.min(target_z);

        let ray_aabb_max_x = source_x.max(target_x);
        let ray_aabb_max_y = source_y.max(target_y);
        let ray_aabb_max_z = source_z.max(target_z);

        let radius = self.get_radius();
        let radius_sq = Vec4::splat(radius * radius);

        let closest_x = Vec4::ZERO.clamp(ray_aabb_min_x, ray_aabb_max_x);
        let closest_y = Vec4::ZERO.clamp(ray_aabb_min_y, ray_aabb_max_y);
        let closest_z = Vec4::ZERO.clamp(ray_aabb_min_z, ray_aabb_max_z);

        let closest_len_sq = closest_x * closest_x + closest_y * closest_y + closest_z * closest_z;
        let within_sphere_mask = closest_len_sq.cmple(radius_sq);
        if !within_sphere_mask.any() {
            return;
        }

        let within_sphere: [bool; 4] = within_sphere_mask.into();
        for i in 0..4 {
            if !within_sphere[i] {
                continue;
            }

            let Some(result) = calc_point_time_of_impact(sources[i], targets[i], |point| {
                self.closest_point(point)
            }) else {
                continue;
            };

            result_callback.report_hit(result.normal, result.fraction, i);
        }
    }
}
