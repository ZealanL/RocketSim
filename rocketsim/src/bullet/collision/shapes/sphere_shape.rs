use std::mem;

use glam::{Affine3A, Vec3A};

use super::convex_internal_shape::ConvexInternalShape;
use crate::bullet::collision::dispatch::ray_packet_callbacks::{
    BridgeTriangleRaycastPacketCallback, RayResultCallback,
};
use crate::shared::{Aabb, RayPacketInfo};

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

    pub fn get_radius(&self) -> f32 {
        self.convex_internal_shape.implicit_dim.x
    }

    #[inline]

    pub fn get_margin(&self) -> f32 {
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

    pub fn perform_raycast<T: RayResultCallback>(
        &self,
        result_callback: &mut BridgeTriangleRaycastPacketCallback<T>,
        ray_info: &RayPacketInfo,
    ) {
        for i in 0..4 {
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
        result_callback: &mut BridgeTriangleRaycastPacketCallback<T>,
        ray_source: Vec3A,
        ray_target: Vec3A,
        ray_idx: usize,
    ) {
        let ray_aabb_min = ray_source.min(ray_target);
        let ray_aabb_max = ray_source.max(ray_target);

        let radius = self.get_radius();
        let radius_sq = radius * radius;

        let closest = Vec3A::ZERO.clamp(ray_aabb_min, ray_aabb_max);
        if closest.length_squared() > radius_sq {
            return;
        }

        let delta = ray_target - ray_source;
        let dist = delta.length();
        let dir = delta / dist;

        let b = 2.0 * ray_source.dot(dir);
        let c = ray_source.length_squared() - radius_sq;

        let discriminant = b * b - 4.0 * c;
        if discriminant < 0.0 {
            return;
        }

        let sqrt_disc = discriminant.sqrt();
        let mut t0 = (-b - sqrt_disc) / 2.0;
        let mut t1 = (-b + sqrt_disc) / 2.0;

        if t0 > t1 {
            mem::swap(&mut t0, &mut t1);
        }

        // pick nearest valid hit
        let t = if t0 >= 0.0 && t0 <= dist {
            t0
        } else if t1 >= 0.0 && t1 <= dist {
            t1
        } else {
            return;
        };

        let hit_point = ray_source + dir * t;
        let normal = hit_point / radius;
        let hit_fraction = t / dist;

        result_callback.report_hit(normal, hit_fraction, ray_idx);
    }
}
