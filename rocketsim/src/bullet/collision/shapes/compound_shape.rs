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
        if !dist.is_finite() || dist <= 0.0 {
            return;
        }
        let dir = delta / dist;
        if !dir.is_finite() {
            return;
        }

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

        if tenter <= 0.0 {
            return;
        }

        if !tenter.is_finite() || !texit.is_finite() {
            return;
        }

        if !self.child_trans.is_finite() {
            return;
        }

        let inner_half = self.child_shape.get_half_extents();
        let margin = self.child_shape.get_margin();
        if !inner_half.is_finite() || !margin.is_finite() {
            return;
        }
        if margin < 0.0 {
            return;
        }
        if inner_half.x < 0.0 || inner_half.y < 0.0 || inner_half.z < 0.0 {
            return;
        }

        // Pure-face fast path for translation-only child transform.
        // Keep the original slab normal and fraction bit-identical.
        {
            let entry = ray_source + dir * tenter;
            if entry.is_finite() {
                let local = entry - self.child_trans.translation;
                if local.is_finite() {
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
                        if hit_normal.is_finite() && hit_normal.length_squared() > 0.0 {
                            let hit_fraction = tenter.max(0.0) / dist;
                            result_callback.report_hit(hit_normal, hit_fraction, ray_idx);
                        }
                        return;
                    }
                }
            }
        }

        // Edge or corner entry: exact rounded box solve.
        let Some((t_hit, normal_compound)) =
            self.analytic_rounded_hit(ray_source, dir, dist, inner_half, margin)
        else {
            return;
        };

        if !t_hit.is_finite() || t_hit <= 0.0 || t_hit > dist {
            return;
        }
        if !normal_compound.is_finite() || normal_compound.length_squared() <= 0.0 {
            return;
        }
        let hit_fraction = t_hit / dist;
        if !hit_fraction.is_finite() {
            return;
        }
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
        if !ray_source.is_finite() || !dir.is_finite() {
            return None;
        }
        let source_l = ray_source - self.child_trans.translation;
        let dir_l = dir;

        if !source_l.is_finite() || !dir_l.is_finite() {
            return None;
        }

        let (t_hit, normal_l) = solve_rounded_box(source_l, dir_l, dist, inner_half, margin)?;
        let normal_c = normal_l;

        if !normal_c.is_finite() || normal_c.length_squared() <= 0.0 {
            return None;
        }
        let normal_c = normal_c / normal_c.length();
        if !normal_c.is_finite() {
            return None;
        }
        // Entering hit must oppose the ray. Allow exact tangency (dot == 0).
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
    if !dist.is_finite() || dist <= 0.0 {
        return None;
    }
    if !source_l.is_finite() || !dir_l.is_finite() {
        return None;
    }
    if !inner_half.is_finite() || !margin.is_finite() || margin < 0.0 {
        return None;
    }

    // Fixed-size stack endpoints from the six inner planes plus [0, dist].
    let mut points = [0.0f32; 8];
    points[0] = 0.0;
    points[1] = dist;
    let mut count = 2usize;

    for axis in 0..3 {
        let d = dir_l[axis];
        if d == 0.0 || !d.is_finite() {
            continue;
        }
        let s = source_l[axis];
        let h = inner_half[axis];
        for bound in [-h, h] {
            if !bound.is_finite() {
                continue;
            }
            let t = (bound - s) / d;
            if !t.is_finite() {
                continue;
            }
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

    for i in 0..count.saturating_sub(1) {
        let t0 = points[i];
        let t1 = points[i + 1];
        if t1 <= t0 {
            continue;
        }
        if !t0.is_finite() || !t1.is_finite() {
            continue;
        }
        let mid = t0 + (t1 - t0) * 0.5;
        if !mid.is_finite() {
            continue;
        }
        let p_mid = source_l + dir_l * mid;
        if !p_mid.is_finite() {
            continue;
        }

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
        if a <= 0.0 || !a.is_finite() || !b.is_finite() || !c.is_finite() {
            continue;
        }

        let cc = c - margin_sq;
        if !cc.is_finite() {
            continue;
        }

        let disc = b * b - 4.0 * a * cc;
        if !disc.is_finite() {
            continue;
        }
        let disc = if disc < 0.0 {
            let scale = b * b + (4.0 * a * cc).abs();
            if !scale.is_finite() {
                continue;
            }
            if disc > -f32::EPSILON * scale {
                0.0
            } else {
                continue;
            }
        } else {
            disc
        };

        let sqrt_d = disc.sqrt();
        if !sqrt_d.is_finite() {
            continue;
        }

        let t_hit = if disc == 0.0 {
            -b / (2.0 * a)
        } else {
            let q = if b >= 0.0 {
                -0.5 * (b + sqrt_d)
            } else {
                -0.5 * (b - sqrt_d)
            };
            if q == 0.0 || !q.is_finite() {
                -b / (2.0 * a)
            } else {
                let r0 = q / a;
                let r1 = cc / q;
                if !r0.is_finite() || !r1.is_finite() {
                    continue;
                }
                if r0 <= r1 { r0 } else { r1 }
            }
        };

        if !t_hit.is_finite() || t_hit <= 0.0 || t_hit > dist {
            continue;
        }
        if t_hit < t0 || t_hit > t1 {
            continue;
        }

        let hit = source_l + dir_l * t_hit;
        if !hit.is_finite() {
            continue;
        }
        let clamped = hit.clamp(-inner_half, inner_half);
        let n = hit - clamped;
        let len_sq = n.length_squared();
        if !len_sq.is_finite() || len_sq <= 0.0 {
            continue;
        }
        let len = len_sq.sqrt();
        if !len.is_finite() || len <= 0.0 {
            continue;
        }
        let n_norm = n / len;
        if !n_norm.is_finite() {
            continue;
        }
        // Entering hit must oppose the ray; keep tangency (dot == 0).
        if n_norm.dot(dir_l) > 0.0 {
            continue;
        }

        return Some((t_hit, n_norm));
    }

    None
}

#[cfg(test)]
mod tests {
    use std::f32::consts::{FRAC_1_SQRT_2, FRAC_PI_2};

    use glam::{Affine3A, Mat3A, Vec3A, Vec4};

    use super::CompoundShape;
    use crate::{
        bullet::{
            collision::{
                dispatch::quad_ray_callbacks::{
                    BridgeTriQuadRayCallback, ClosestQuadRayResultCallback, QuadRayResultCallback,
                },
                shapes::{
                    box_shape::BoxShape, collision_shape::CollisionShapes,
                    sphere_shape::SphereShape,
                },
            },
            dynamics::rigid_body::{RigidBody, RigidBodyConstructionInfo},
        },
        shared::QuadRayInfo,
    };

    fn unit_box() -> BoxShape {
        BoxShape::new(Vec3A::new(1.0, 1.0, 1.0))
    }

    fn ident_compound(shape: BoxShape, translation: Vec3A) -> CompoundShape {
        CompoundShape::new(
            shape,
            Affine3A {
                matrix3: Mat3A::IDENTITY,
                translation,
            },
        )
    }

    fn cast_one(compound: &CompoundShape, from: Vec3A, to: Vec3A) -> Option<(f32, Vec3A)> {
        let froms = [from; 4];
        let tos = [to; 4];
        let dummy = RigidBody::new(RigidBodyConstructionInfo::new(
            0.0,
            CollisionShapes::Sphere(SphereShape::new(1.0)),
        ));
        let mut cb = ClosestQuadRayResultCallback::new(&froms, &tos, None);
        {
            let mut bridge = BridgeTriQuadRayCallback {
                from: &froms,
                to: &tos,
                hit_fraction: Vec4::ONE,
                collision_obj: &dummy,
                collision_obj_idx: 0,
                result_callback: &mut cb,
            };
            let info = QuadRayInfo::new(&froms, &tos);
            compound.perform_quad_raycast(&mut bridge, &info);
        }
        if cb.has_hit(0) {
            Some((cb.base.closest_hit_fraction[0], cb.hit_normal_world[0]))
        } else {
            None
        }
    }

    #[test]
    fn pure_face_returns_exact_slab_fraction_and_normal() {
        let compound = ident_compound(unit_box(), Vec3A::ZERO);
        let from = Vec3A::new(0.0, 0.0, 5.0);
        let to = Vec3A::new(0.0, 0.0, -5.0);
        let (fraction, normal) = cast_one(&compound, from, to).expect("face must hit");
        assert_eq!(fraction, 4.0f32 / 10.0);
        assert_eq!(normal, Vec3A::Z);
    }

    #[test]
    fn edge_cylinder_hit_matches_analytic() {
        let compound = ident_compound(unit_box(), Vec3A::ZERO);
        let from = Vec3A::new(0.98, 0.0, 5.0);
        let to = Vec3A::new(0.98, 0.0, -5.0);
        let (fraction, normal) = cast_one(&compound, from, to).expect("edge must hit");
        let dz = 0.0012f32.sqrt();
        let hit_z = 0.96 + dz;
        let expected_fraction = (5.0 - hit_z) / 10.0;
        let expected_normal = Vec3A::new(0.5, 0.0, 0.8660254).normalize();
        assert!(
            (fraction - expected_fraction).abs() < 1e-4,
            "fraction {fraction} vs {expected_fraction}"
        );
        assert!(
            normal.dot(expected_normal) > 0.999,
            "normal {normal:?} vs {expected_normal:?}"
        );
        let dir = (to - from).normalize();
        assert!(normal.dot(dir) < 0.0);
    }

    #[test]
    fn corner_sphere_hit_matches_analytic() {
        let compound = ident_compound(unit_box(), Vec3A::ZERO);
        let from = Vec3A::new(0.98, 0.98, 5.0);
        let to = Vec3A::new(0.98, 0.98, -5.0);
        let (fraction, normal) = cast_one(&compound, from, to).expect("corner must hit");
        let dz = 0.0008f32.sqrt();
        let hit_z = 0.96 + dz;
        let expected_fraction = (5.0 - hit_z) / 10.0;
        let expected_normal = Vec3A::new(0.5, 0.5, FRAC_1_SQRT_2).normalize();
        assert!(
            (fraction - expected_fraction).abs() < 1e-4,
            "fraction {fraction} vs {expected_fraction}"
        );
        assert!(
            normal.dot(expected_normal) > 0.999,
            "normal {normal:?} vs {expected_normal:?}"
        );
    }

    #[test]
    fn clear_miss_reports_none() {
        let compound = ident_compound(unit_box(), Vec3A::ZERO);
        assert!(
            cast_one(
                &compound,
                Vec3A::new(2.0, 0.0, 5.0),
                Vec3A::new(2.0, 0.0, -5.0)
            )
            .is_none()
        );
    }

    #[test]
    fn parallel_hit_and_parallel_miss() {
        let compound = ident_compound(unit_box(), Vec3A::ZERO);
        let (fraction, normal) = cast_one(
            &compound,
            Vec3A::new(-5.0, 0.0, 0.0),
            Vec3A::new(5.0, 0.0, 0.0),
        )
        .expect("parallel inside must hit");
        assert_eq!(fraction, 4.0f32 / 10.0);
        assert_eq!(normal, Vec3A::NEG_X);
        assert!(
            cast_one(
                &compound,
                Vec3A::new(-5.0, 2.0, 0.0),
                Vec3A::new(5.0, 2.0, 0.0)
            )
            .is_none(),
            "parallel outside must miss"
        );
    }

    #[test]
    fn exact_tangency_reports_hit() {
        // Tangent to the vertical edge (0.96, 0.96) with dx = dy = margin / sqrt(2).
        // Discriminant is zero; packet AABB still passes since x/y stay inside outer.
        let compound = ident_compound(unit_box(), Vec3A::ZERO);
        let offset = 0.04f32 / 2.0f32.sqrt();
        let x = 0.96 + offset;
        let from = Vec3A::new(x, x, 5.0);
        let to = Vec3A::new(x, x, -5.0);
        let (fraction, normal) = cast_one(&compound, from, to).expect("tangency must hit");
        assert!((fraction - 0.404).abs() < 1e-4, "fraction {fraction}");
        let expected = Vec3A::new(FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0).normalize();
        assert!(normal.dot(expected) > 0.999, "normal {normal:?}");
    }

    #[test]
    fn starts_inside_and_on_surface_preserve_miss() {
        let compound = ident_compound(unit_box(), Vec3A::ZERO);
        assert!(
            cast_one(&compound, Vec3A::ZERO, Vec3A::new(0.0, 0.0, 5.0)).is_none(),
            "starts inside must miss"
        );
        assert!(
            cast_one(
                &compound,
                Vec3A::new(0.0, 0.0, 1.0),
                Vec3A::new(0.0, 0.0, -5.0)
            )
            .is_none(),
            "on-surface inward must miss"
        );
        assert!(
            cast_one(
                &compound,
                Vec3A::new(0.0, 0.0, 1.0),
                Vec3A::new(0.0, 0.0, 5.0)
            )
            .is_none(),
            "on-surface outward must miss"
        );
    }

    #[test]
    fn zero_length_and_nonfinite_safety() {
        let compound = ident_compound(unit_box(), Vec3A::ZERO);
        assert!(
            cast_one(
                &compound,
                Vec3A::new(1.0, 1.0, 1.0),
                Vec3A::new(1.0, 1.0, 1.0)
            )
            .is_none()
        );
        assert!(
            cast_one(
                &compound,
                Vec3A::new(f32::NAN, 0.0, 0.0),
                Vec3A::new(0.0, 0.0, 5.0)
            )
            .is_none()
        );
        assert!(
            cast_one(
                &compound,
                Vec3A::new(f32::INFINITY, 0.0, 0.0),
                Vec3A::new(0.0, 0.0, 5.0)
            )
            .is_none()
        );
        assert!(cast_one(&compound, Vec3A::ZERO, Vec3A::new(f32::NAN, 0.0, 0.0)).is_none());
        assert!(cast_one(&compound, Vec3A::ZERO, Vec3A::new(f32::INFINITY, 0.0, 0.0)).is_none());
    }

    #[test]
    fn translated_child_recenters_hit() {
        let compound = ident_compound(unit_box(), Vec3A::new(1.0, 0.0, 0.0));
        let (fraction, normal) = cast_one(
            &compound,
            Vec3A::new(1.0, 0.0, 5.0),
            Vec3A::new(1.0, 0.0, -5.0),
        )
        .expect("translated center must hit");
        assert_eq!(fraction, 4.0f32 / 10.0);
        assert_eq!(normal, Vec3A::Z);
    }

    #[test]
    fn valid_constructors_use_identity_child_rotation() {
        let car_child = Affine3A {
            matrix3: Mat3A::IDENTITY,
            translation: Vec3A::new(0.277514, 0.0, 0.4151),
        };
        let car = CompoundShape::new(unit_box(), car_child);
        assert_eq!(car.child_trans.matrix3, Mat3A::IDENTITY);
        let wheel = CompoundShape::new(unit_box(), Affine3A::IDENTITY);
        assert_eq!(wheel.child_trans.matrix3, Mat3A::IDENTITY);
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic]
    fn rotated_child_triggers_debug_assert() {
        let shape = BoxShape::new(Vec3A::new(2.0, 0.5, 0.5));
        let child = Affine3A {
            matrix3: Mat3A::from_rotation_z(FRAC_PI_2),
            translation: Vec3A::ZERO,
        };
        let compound = CompoundShape::new(shape, child);
        let _ = cast_one(
            &compound,
            Vec3A::new(0.0, 1.0, 5.0),
            Vec3A::new(0.0, 1.0, -5.0),
        );
    }

    #[test]
    fn mirror_symmetry_and_deterministic_tie() {
        let compound = ident_compound(unit_box(), Vec3A::ZERO);
        let from_a = Vec3A::new(0.98, 0.0, 5.0);
        let to_a = Vec3A::new(0.98, 0.0, -5.0);
        let from_b = Vec3A::new(-0.98, 0.0, 5.0);
        let to_b = Vec3A::new(-0.98, 0.0, -5.0);
        let (fa, na) = cast_one(&compound, from_a, to_a).expect("mirror A must hit");
        let (fb, nb) = cast_one(&compound, from_b, to_b).expect("mirror B must hit");
        assert!((fa - fb).abs() < 1e-6, "fractions {fa} vs {fb}");
        assert!((na.x + nb.x).abs() < 1e-6, "normals {na:?} vs {nb:?}");
        assert!((na.y - nb.y).abs() < 1e-6);
        assert!((na.z - nb.z).abs() < 1e-6);
        let repeat = cast_one(&compound, from_a, to_a).expect("repeat must hit");
        assert_eq!(fa, repeat.0);
        assert_eq!(na, repeat.1);
        let diag_from = Vec3A::new(2.0, 2.0, 0.0);
        let diag_to = Vec3A::new(-2.0, -2.0, 0.0);
        let first = cast_one(&compound, diag_from, diag_to).expect("diagonal must hit");
        let second = cast_one(&compound, diag_from, diag_to).expect("repeat must hit");
        assert_eq!(first.0, second.0);
        assert_eq!(first.1, second.1);
    }

    #[test]
    fn wisp_9704_recorded_geometry_regression() {
        // Recorded-geometry regression data only, copied as test numbers.
        // Live staging did not reproduce the exact contact; this guards the analytic solve.
        let shape = BoxShape::new(Vec3A::new(1.20507, 0.866994, 0.386591));
        let child = Affine3A {
            matrix3: Mat3A::IDENTITY,
            translation: Vec3A::new(0.277514, 0.0, 0.4151),
        };
        let compound = CompoundShape::new(shape, child);
        let from_a = Vec3A::new(1.085316, 0.145283, 1.635977);
        let to_a = Vec3A::new(1.507248, 0.29966, 0.714581);
        let (fraction, normal) = cast_one(&compound, from_a, to_a).expect("9704 ray must hit");
        let target_n = Vec3A::new(0.76937, 0.000005, 0.638803).normalize();
        assert!(
            normal.dot(target_n) > 0.999,
            "normal {normal:?} vs target {target_n:?}"
        );
        let ray_len = (to_a - from_a).length();
        let trace = fraction * ray_len;
        assert!(
            (trace - 0.943611).abs() < 0.01,
            "trace {trace} fraction {fraction}"
        );
    }
}
