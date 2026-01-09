use glam::{Affine3A, Vec3A};

use super::box_shape::BoxShape;
use crate::bullet::{
    collision::dispatch::ray_callbacks::{BridgeTriangleRaycastPacketCallback, RayResultCallback},
};
use crate::shared::{Aabb, RayPacketInfo};

pub struct CompoundShape {
    pub child_shape: BoxShape,
    pub child_trans: Affine3A,
    local_aabb: Aabb,
}

impl CompoundShape {
    pub fn new(child_shape: BoxShape, child_trans: Affine3A) -> Self {
        let local_aabb = child_shape.get_aabb(&child_trans);

        Self {
            child_shape,
            child_trans,
            local_aabb,
        }
    }

    pub fn get_aabb(&self, trans: &Affine3A) -> Aabb {
        let local_half_extents = 0.5 * (self.local_aabb.max - self.local_aabb.min);
        let local_center = 0.5 * (self.local_aabb.max + self.local_aabb.min);

        let abs_b = trans.matrix3.abs();
        let center = trans.transform_point3a(local_center);
        let extent = abs_b * local_half_extents;

        Aabb {
            min: center - extent,
            max: center + extent,
        }
    }

    #[inline]
    pub const fn get_ident_aabb(&self) -> &Aabb {
        &self.local_aabb
    }

    pub fn perform_raycast<T: RayResultCallback>(
        &self,
        result_callback: &mut BridgeTriangleRaycastPacketCallback<T>,
        ray_info: &RayPacketInfo,
    ) {
        let box_aabb = self.get_ident_aabb();
        if !ray_info.aabb.intersects(box_aabb) {
            return;
        }

        let (origins, inv_dirs) = ray_info.calc_pos_dir();
        let mask =
            RayPacketInfo::intersect_ray_aabb_packet(&origins, &inv_dirs, box_aabb, result_callback.hit_fraction);

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
        let dir = delta / dist;

        // implementation of the slab method to handle `dir` potentially having elements that are `0`
        let mut tenter = 0f32;
        let mut texit = dist;
        let mut hit_axis = 0usize;

        let inv = 1.0 / dir;
        let t1 = (self.local_aabb.min - ray_source) * inv;
        let t2 = (self.local_aabb.max - ray_source) * inv;

        let tmin = t1.min(t2);
        let tmax = t1.max(t2);

        let is_neg = tmax.is_negative_bitmask();
        let is_finite: [bool; 3] = inv.is_finite_mask().into();
        for axis in 0..3 {
            if !is_finite[axis] {
                let origin = ray_source[axis];
                let min = self.local_aabb.min[axis];
                let max = self.local_aabb.max[axis];

                // parallel - if the origin not within slab, no hit
                if min > origin || origin > max {
                    return;
                }

                // Axis does not clip the interval
                continue;
            }

            if is_neg & (1 << axis) != 0 {
                return;
            }

            texit = texit.min(tmax[axis]);
            if tmin[axis] > tenter {
                tenter = tmin[axis];
                hit_axis = axis;
            }

            if tenter > texit {
                return;
            }
        }

        if tenter > dist {
            return;
        }

        let mut hit_normal = Vec3A::ZERO;
        hit_normal[hit_axis] = -dir[hit_axis].signum();

        let hit_fraction = tenter.max(0.0) / dist;
        result_callback.report_hit(hit_normal, hit_fraction, ray_idx);
    }
}
