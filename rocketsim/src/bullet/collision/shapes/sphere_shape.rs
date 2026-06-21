use glam::{Affine3A, Vec3A, Vec4};

use super::convex_internal_shape::ConvexInternalShape;
use crate::{
    bullet::collision::dispatch::ray_packet_callbacks::{
        BridgeTriRayPacketCallback, QuadRayResultCallback,
    },
    shared::{Aabb, RayPacketInfo},
};

pub const SPHERE_RADIUS_MARGIN: f32 = 0.08;

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

    pub fn get_aabb(&self, t: &Affine3A) -> Aabb {
        let center = t.translation;
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

    pub fn perform_quad_raycast<T: QuadRayResultCallback>(
        &self,
        result_callback: &mut BridgeTriRayPacketCallback<T>,
        ray_info: &RayPacketInfo,
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

        let delta_x = target_x - source_x;
        let delta_y = target_y - source_y;
        let delta_z = target_z - source_z;

        let dist = (delta_x * delta_x + delta_y * delta_y + delta_z * delta_z).sqrt();

        let dir_x = delta_x / dist;
        let dir_y = delta_y / dist;
        let dir_z = delta_z / dist;

        let b = 2.0 * (source_x * dir_x + source_y * dir_y + source_z * dir_z);
        let c = source_x * source_x + source_y * source_y + source_z * source_z - radius_sq;

        let discriminant = b * b - 4.0 * c;
        let valid_mask = discriminant.cmpge(Vec4::ZERO) & within_sphere_mask;
        if !valid_mask.any() {
            return;
        }

        let sqrt_disc = discriminant.sqrt();
        let mut t0 = (-b - sqrt_disc) / 2.0;
        let mut t1 = (-b + sqrt_disc) / 2.0;

        let swap_mask = t0.cmpgt(t1);
        t0 = Vec4::select(swap_mask, t1, t0);
        t1 = Vec4::select(swap_mask, t0, t1);

        let t0_nearest_mask = t0.cmpge(Vec4::ZERO) & t0.cmple(dist);
        let t1_nearest_mask = t1.cmpge(Vec4::ZERO) & t1.cmple(dist) & !t0_nearest_mask;
        let nearest_mask = valid_mask & (t0_nearest_mask | t1_nearest_mask);
        if !nearest_mask.any() {
            return;
        }

        let t = Vec4::select(t0_nearest_mask, t0, t1);

        let hit_point_x = source_x + dir_x * t;
        let hit_point_y = source_y + dir_y * t;
        let hit_point_z = source_z + dir_z * t;

        let normal_x = (hit_point_x / radius).to_array();
        let normal_y = (hit_point_y / radius).to_array();
        let normal_z = (hit_point_z / radius).to_array();

        let hit_mask = nearest_mask.bitmask();
        let hit_fraction = t / dist;
        for (i, hit_fraction) in hit_fraction.to_array().into_iter().enumerate() {
            if (hit_mask & (1 << i)) == 0 {
                continue;
            }

            result_callback.report_hit(
                Vec3A::new(normal_x[i], normal_y[i], normal_z[i]),
                hit_fraction,
                i,
            );
        }
    }
}
