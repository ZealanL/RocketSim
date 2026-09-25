use glam::{Affine3A, Mat3A, Vec3A};

use super::box_shape::BoxShape;
use crate::{
    bullet::collision::dispatch::quad_ray_callbacks::{
        BridgeTriQuadRayCallback, QuadRayResultCallback,
    },
    shared::{Aabb, QuadRayInfo},
};

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

    #[inline]
    pub const fn get_margin(&self) -> f32 {
        self.child_shape.get_margin()
    }

    pub fn perform_quad_raycast<T: QuadRayResultCallback>(
        &self,
        result_callback: &mut BridgeTriQuadRayCallback<T>,
        ray_info: &QuadRayInfo,
    ) {
        let box_aabb = self.get_ident_aabb();
        if !ray_info.aabb.intersects(box_aabb) {
            return;
        }

        let (origins, inv_dirs) = ray_info.calc_pos_dir();
        let mask = QuadRayInfo::intersect_quad_ray_aabb(
            &origins,
            &inv_dirs,
            box_aabb,
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
        result_callback: &mut BridgeTriQuadRayCallback<T>,
        ray_source: Vec3A,
        ray_target: Vec3A,
        ray_idx: usize,
    ) {
        debug_assert_eq!(self.child_trans.matrix3, Mat3A::IDENTITY);
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

        if 0.0 >= tenter || tenter > dist {
            return;
        }

        let inner_half = self.child_shape.get_half_extents();
        let margin = self.child_shape.get_margin();

        // Pure-face fast path for translation-only child transform.
        // Keep the original slab normal and fraction bit-identical.
        {
            let entry = ray_source + dir * tenter;
            let local = entry - self.child_trans.translation;
            let mut inside_face = true;
            for axis in 0..3 {
                if axis == hit_axis {
                    continue;
                }

                if local[axis].abs() > inner_half[axis] {
                    inside_face = false;
                    break;
                }
            }

            if inside_face {
                let mut hit_normal = Vec3A::ZERO;
                hit_normal[hit_axis] = -dir[hit_axis].signum();

                if hit_normal.length_squared() > 0.0 {
                    let hit_fraction = tenter.max(0.0) / dist;
                    result_callback.report_hit(hit_normal, hit_fraction, ray_idx);
                }

                return;
            }
        }

        // Edge or corner entry: exact rounded box solve.
        let Some((t_hit, normal_compound)) =
            self.analytic_rounded_hit(ray_source, dir, dist, inner_half, margin)
        else {
            return;
        };

        if 0.0 >= t_hit || t_hit > dist {
            return;
        }

        let hit_fraction = t_hit / dist;
        result_callback.report_hit(normal_compound, hit_fraction, ray_idx);
    }

    fn analytic_rounded_hit(
        &self,
        ray_source: Vec3A,
        dir: Vec3A,
        dist: f32,
        inner_half: Vec3A,
        margin: f32,
    ) -> Option<(f32, Vec3A)> {
        debug_assert_eq!(self.child_trans.matrix3, Mat3A::IDENTITY);
        let source_l = ray_source - self.child_trans.translation;
        let dir_l = dir;

        let (t_hit, normal_c) = solve_rounded_box(source_l, dir_l, dist, inner_half, margin)?;

        // Entering hit must oppose the ray. Allow exact tangency (dot == 0).
        let normal_c = normal_c / normal_c.length();
        if normal_c.dot(dir) > 0.0 {
            return None;
        }

        Some((t_hit, normal_c))
    }
}

fn solve_rounded_box(
    source_l: Vec3A,
    dir_l: Vec3A,
    dist: f32,
    inner_half: Vec3A,
    margin: f32,
) -> Option<(f32, Vec3A)> {
    // Fixed-size stack endpoints from the six inner planes plus [0, dist].
    let mut points = [0.0f32; 8];
    points[0] = 0.0;
    points[1] = dist;
    let mut count = 2usize;

    for axis in 0..3 {
        let d = dir_l[axis];
        if d == 0.0 {
            continue;
        }

        let s = source_l[axis];
        let h = inner_half[axis];
        for bound in [-h, h] {
            let t = (bound - s) / d;
            if t > 0.0 && t < dist && count < points.len() {
                points[count] = t;
                count += 1;
            }
        }
    }

    // Deterministic insertion sort, no allocation.
    for i in 1..count {
        let key = points[i];
        let mut j = i;
        while j > 0 && points[j - 1] > key {
            points[j] = points[j - 1];
            j -= 1;
        }

        points[j] = key;
    }

    let margin_sq = margin * margin;

    for i in 0..(count - 1) {
        let t0 = points[i];
        let t1 = points[i + 1];
        if t1 <= t0 {
            continue;
        }

        let mid = t0 + (t1 - t0) * 0.5;
        let p_mid = source_l + dir_l * mid;

        let mut a = 0.0f32;
        let mut b = 0.0f32;
        let mut c = 0.0f32;
        let mut outside = 0u32;
        for axis in 0..3 {
            let pm = p_mid[axis];
            let h = inner_half[axis];
            let bound = if pm < -h {
                -h
            } else if pm > h {
                h
            } else {
                continue;
            };

            outside += 1;

            let o = source_l[axis] - bound;
            let d = dir_l[axis];

            a += d * d;
            b += 2.0 * d * o;
            c += o * o;
        }

        if outside == 0 {
            continue;
        }

        let cc = c - margin_sq;
        let disc = b * b - 4.0 * a * cc;
        let disc = if disc < 0.0 {
            let scale = b * b + (4.0 * a * cc).abs();
            if disc > -f32::EPSILON * scale {
                0.0
            } else {
                continue;
            }
        } else {
            disc
        };

        let sqrt_d = disc.sqrt();
        let t_hit = if disc == 0.0 {
            -b / (2.0 * a)
        } else {
            let q = -0.5 * if b >= 0.0 { b + sqrt_d } else { b - sqrt_d };

            if q == 0.0 {
                -b / (2.0 * a)
            } else {
                let r0 = q / a;
                let r1 = cc / q;
                if r0 <= r1 { r0 } else { r1 }
            }
        };

        if t_hit <= 0.0 || t_hit > dist || t_hit < t0 || t_hit > t1 {
            continue;
        }

        let hit = source_l + dir_l * t_hit;
        let clamped = hit.clamp(-inner_half, inner_half);
        let n = hit - clamped;
        let len_sq = n.length_squared();
        if len_sq <= f32::EPSILON * f32::EPSILON {
            continue;
        }

        // Entering hit must oppose the ray; keep tangency (dot == 0).
        let n_norm = n / len_sq.sqrt();
        if n_norm.dot(dir_l) > 0.0 {
            continue;
        }

        return Some((t_hit, n_norm));
    }

    None
}
