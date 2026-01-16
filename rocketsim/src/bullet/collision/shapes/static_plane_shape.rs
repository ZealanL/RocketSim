use glam::{Affine3A, Vec3A};

use crate::bullet::{
    collision::dispatch::ray_packet_callbacks::{BridgeTriangleRaycastPacketCallback, RayResultCallback},
    linear_math::LARGE_FLOAT,
};
use crate::shared::{Aabb, RayPacketInfo};

pub struct StaticPlaneShape {
    plane_normal: Vec3A,
    is_single_axis: bool,
    single_axis_idx: usize,
    single_axis_backwards: bool,
    pub aabb_ident_cache: Aabb,
    pub aabb_cache: Aabb,
    #[cfg(debug_assertions)]
    pub aabb_cache_trans: Affine3A,
}

impl StaticPlaneShape {
    pub fn new(world_trans: Affine3A, plane_normal: Vec3A) -> Self {
        debug_assert!(plane_normal.is_normalized());

        let [x, y, z]: [bool; 3] = plane_normal.abs().cmpge(Vec3A::splat(f32::EPSILON)).into();

        let (is_single_axis, single_axis_idx, single_axis_backwards) =
            if u8::from(x) + u8::from(y) + u8::from(z) == 1 {
                let axis = plane_normal.abs().max_position();

                (true, axis, plane_normal[axis].is_sign_negative())
            } else {
                (false, 0, false)
            };

        let mut plane = Self {
            plane_normal,
            is_single_axis,
            single_axis_idx,
            single_axis_backwards,
            aabb_ident_cache: Aabb::ZERO,
            aabb_cache: Aabb::ZERO,
            #[cfg(debug_assertions)]
            aabb_cache_trans: world_trans,
        };

        plane.aabb_ident_cache = plane.get_aabb(&Affine3A::IDENTITY);
        plane.aabb_cache = plane.get_aabb(&world_trans);

        plane
    }

    pub fn get_aabb(&self, t: &Affine3A) -> Aabb {
        let mut min = Vec3A::splat(-LARGE_FLOAT);
        let mut max = Vec3A::splat(LARGE_FLOAT);

        if self.is_single_axis {
            const PLANE_CONSTANT_OFFSET: f32 = 0.2;

            min[self.single_axis_idx] = t.translation[self.single_axis_idx] - PLANE_CONSTANT_OFFSET;
            max[self.single_axis_idx] = t.translation[self.single_axis_idx] + PLANE_CONSTANT_OFFSET;

            (if self.single_axis_backwards {
                &mut max
            } else {
                &mut min
            })[self.single_axis_idx] = if self.single_axis_backwards {
                LARGE_FLOAT
            } else {
                -LARGE_FLOAT
            };
        }

        Aabb { min, max }
    }

    pub const fn get_plane_normal(&self) -> Vec3A {
        self.plane_normal
    }

    pub fn perform_raycast<T: RayResultCallback>(
        &self,
        result_callback: &mut BridgeTriangleRaycastPacketCallback<T>,
        ray_info: &RayPacketInfo,
    ) {
        let plane = &self.aabb_ident_cache;
        if !ray_info.aabb.intersects(plane) {
            return;
        }

        let (origins, inv_dirs) = ray_info.calc_pos_dir();
        let mask =
            RayPacketInfo::intersect_ray_aabb_packet(&origins, &inv_dirs, plane, result_callback.hit_fraction);

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
        result_callback: &mut BridgeTriangleRaycastPacketCallback<T>,
        ray_source: Vec3A,
        ray_target: Vec3A,
        ray_idx: usize,
    ) {
        let delta = ray_target - ray_source;
        let dist = delta.length();
        let ray_direction = delta / dist;

        let dir_align = self.plane_normal.dot(ray_direction);
        if dir_align.abs() < f32::EPSILON {
            return;
        }

        let normal_start = self.plane_normal.dot(ray_source);
        let t = -normal_start / dir_align;
        if !(0.0..1.0).contains(&t) {
            return;
        }

        let hit_fraction = t / dist;
        result_callback.report_hit(self.plane_normal, hit_fraction, ray_idx);
    }
}
