use glam::{Affine3A, Vec3A, Vec4};

use crate::{
    bullet::collision::dispatch::quad_ray_callbacks::{
        BridgeTriQuadRayCallback, QuadRayResultCallback,
    },
    shared::{Aabb, QuadRayInfo},
};

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
        let mut min = Vec3A::MIN;
        let mut max = Vec3A::MAX;

        if self.is_single_axis {
            const PLANE_CONSTANT_OFFSET: f32 = 0.2;

            min[self.single_axis_idx] = t.translation[self.single_axis_idx] - PLANE_CONSTANT_OFFSET;
            max[self.single_axis_idx] = t.translation[self.single_axis_idx] + PLANE_CONSTANT_OFFSET;

            (if self.single_axis_backwards {
                &mut max
            } else {
                &mut min
            })[self.single_axis_idx] = if self.single_axis_backwards {
                f32::MAX
            } else {
                f32::MIN
            };
        }

        Aabb { min, max }
    }

    pub const fn get_plane_normal(&self) -> Vec3A {
        self.plane_normal
    }

    pub fn perform_quad_raycast<T: QuadRayResultCallback>(
        &self,
        result_callback: &mut BridgeTriQuadRayCallback<T>,
        ray_info: &QuadRayInfo,
    ) {
        let plane = &self.aabb_ident_cache;
        if !ray_info.aabb.intersects(plane) {
            return;
        }

        let sources = ray_info.ray_sources;
        let targets = ray_info.ray_targets;

        let source_x = Vec4::new(sources[0].x, sources[1].x, sources[2].x, sources[3].x);
        let source_y = Vec4::new(sources[0].y, sources[1].y, sources[2].y, sources[3].y);
        let source_z = Vec4::new(sources[0].z, sources[1].z, sources[2].z, sources[3].z);

        let target_x = Vec4::new(targets[0].x, targets[1].x, targets[2].x, targets[3].x);
        let target_y = Vec4::new(targets[0].y, targets[1].y, targets[2].y, targets[3].y);
        let target_z = Vec4::new(targets[0].z, targets[1].z, targets[2].z, targets[3].z);

        let delta_x = target_x - source_x;
        let delta_y = target_y - source_y;
        let delta_z = target_z - source_z;

        let inv_delta_x = Vec4::ONE / delta_x;
        let inv_delta_y = Vec4::ONE / delta_y;
        let inv_delta_z = Vec4::ONE / delta_z;

        let ray_mask = QuadRayInfo::intersect_quad_ray_aabb(
            &[source_x, source_y, source_z],
            &[inv_delta_x, inv_delta_y, inv_delta_z],
            plane,
            result_callback.hit_fraction,
        );
        if ray_mask == 0 {
            return;
        }

        let dist = (delta_x * delta_x + delta_y * delta_y + delta_z * delta_z).sqrt();
        let inv_dist = Vec4::ONE / dist;

        let dir_x = delta_x * inv_dist;
        let dir_y = delta_y * inv_dist;
        let dir_z = delta_z * inv_dist;

        let dir_align =
            dir_x * self.plane_normal.x + dir_y * self.plane_normal.y + dir_z * self.plane_normal.z;
        let dir_align_mask = dir_align.abs().cmpge(Vec4::splat(f32::EPSILON));
        if !dir_align_mask.any() {
            return;
        }

        let normal_start = source_x * self.plane_normal.x
            + source_y * self.plane_normal.y
            + source_z * self.plane_normal.z;

        let t = -normal_start / dir_align;

        let t_mask = t.cmpge(Vec4::ZERO) & t.cmplt(dist);
        let hit_mask = ray_mask & (dir_align_mask & t_mask).bitmask() as u8;
        if hit_mask == 0 {
            return;
        }

        let hit_fraction = t / dist;
        for (i, hit_fraction) in hit_fraction.to_array().into_iter().enumerate() {
            if (hit_mask & (1 << i)) == 0 {
                continue;
            }

            result_callback.report_hit(self.plane_normal, hit_fraction, i);
        }
    }
}
