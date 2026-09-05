use std::{
    f32::consts::{PI, TAU},
    mem,
};

use arrayvec::ArrayVec;
use glam::{Affine3A, FloatExt, Mat3A, Vec3A, Vec4};

use crate::bullet::{
    collision::{
        narrowphase::persistent_manifold::{
            ContactAddedCallback, MANIFOLD_CACHE_SIZE, PersistentManifold,
        },
        shapes::box_shape::BoxShape,
    },
    dynamics::rigid_body::RigidBody,
    linear_math::Obb,
};

fn line_closest_approach(pa: Vec3A, ua: Vec3A, pb: Vec3A, ub: Vec3A) -> f32 {
    let p = pb - pa;
    let uaub = ua.dot(ub);
    let q1 = ua.dot(p);
    let q2 = -ub.dot(p);

    let d = 1.0 - uaub * uaub;
    if d <= 0.0001 {
        0.0
    } else {
        (uaub * q1 + q2) / d
    }
}

/// Clip a quad (4 points) against the axis-aligned rectangle defined by ±h[0], ±h[1].
///
/// `h` is (half_width, half_height) and `poly` is a reference to an array of 4 (x, y) f32 points.
/// Returns an `ArrayVec<(f32, f32), 16>` containing the clipped polygon vertices.
pub fn intersect_rect_quad2(h: [f32; 2], poly: &[[f32; 2]; 4]) -> ArrayVec<[f32; 2], 16> {
    let mut q: ArrayVec<[f32; 2], 16> = ArrayVec::new();
    q.try_extend_from_slice(poly).unwrap();

    for dir in 0..2 {
        for sign in [-1.0_f32, 1.0_f32] {
            let mut r: ArrayVec<[f32; 2], 16> = ArrayVec::new();
            let clip_val = sign * h[dir];

            // Traverse current polygon edges in order
            for i in 0..q.len() {
                let cur = q[i];
                let next = q[(i + 1) % q.len()];

                let cur_val = cur[dir];
                let next_val = next[dir];

                let inside_cur = sign * cur_val <= h[dir];

                // If current point is inside, keep it
                if inside_cur {
                    r.push(cur);
                }

                let inside_next = sign * next_val <= h[dir];

                // If the edge crosses the boundary, add intersection
                if inside_cur ^ inside_next {
                    let denom = next_val - cur_val;
                    let t = if denom.abs() < f32::EPSILON {
                        0.0
                    } else {
                        (clip_val - cur_val) / denom
                    };

                    let mut p1 = cur[1 - dir].lerp(next[1 - dir], t);
                    let mut p2 = clip_val;
                    if dir == 0 {
                        mem::swap(&mut p1, &mut p2);
                    }

                    r.push([p1, p2]);
                }
            }

            q = r;
        }
    }

    q
}

/// Given up to 8 points in the plane, select `m` points that best represent the set.
fn cull_points2(p: &[[f32; 2]], i0: usize, m: usize) -> ArrayVec<usize, 8> {
    let n = p.len();

    // Compute centroid
    let (cx, cy) = match n {
        0 => unreachable!(),
        1 => (p[0][0], p[0][1]),
        2 => ((p[0][0] + p[1][0]) * 0.5, (p[0][1] + p[1][1]) * 0.5),
        _ => {
            let mut a_sum = 0.0f32;
            let mut cx_sum = 0.0f32;
            let mut cy_sum = 0.0f32;

            for i in 0..(n - 1) {
                let (x0, y0) = (p[i][0], p[i][1]);
                let (x1, y1) = (p[i + 1][0], p[i + 1][1]);
                let q = x0 * y1 - x1 * y0;

                a_sum += q;
                cx_sum += q * (x0 + x1);
                cy_sum += q * (y0 + y1);
            }

            let (x_last, y_last) = (p[n - 1][0], p[n - 1][1]);
            let (x0, y0) = (p[0][0], p[0][1]);
            let q_last = x_last * y0 - x0 * y_last;
            let area = a_sum + q_last;

            let inv = if area.abs() > f32::EPSILON {
                1.0 / (3.0 * area)
            } else {
                f32::INFINITY
            };

            (
                inv * (cx_sum + q_last * (x_last + x0)),
                inv * (cy_sum + q_last * (y_last + y0)),
            )
        }
    };

    // Compute angles
    let mut angles: ArrayVec<f32, 8> = ArrayVec::new();
    for c in p {
        let dx = c[0] - cx;
        let dy = c[1] - cy;
        angles.push(dy.atan2(dx));
    }

    // Select points with closest angles
    let mut result: ArrayVec<usize, 8> = ArrayVec::new();
    let mut avail = [false; 8];
    for a in &mut avail[1..n] {
        *a = true;
    }

    result.push(i0);

    for j in 1..m {
        let mut target = (j as f32) * (TAU / m as f32) + angles[i0];
        if target > PI {
            target -= TAU;
        }

        let mut best_idx = i0;
        let mut best_diff = f32::MAX;

        for i in 0..n {
            if avail[i] {
                let mut diff = (angles[i] - target).abs();
                if diff > PI {
                    diff = TAU - diff;
                }

                if diff < best_diff {
                    best_diff = diff;
                    best_idx = i;
                }
            }
        }

        avail[best_idx] = false;
        result.push(best_idx);
    }

    result
}

#[derive(Debug)]
struct Hit {
    depth: f32,
    normal: Vec3A,
    axis_idx: usize,
}

pub struct BoxBoxDetector<'a, T: ContactAddedCallback> {
    pub box1: &'a BoxShape,
    pub col1: &'a RigidBody,
    pub box2: &'a BoxShape,
    pub col2: &'a RigidBody,
    pub contact_added_callback: &'a mut T,
}

impl<T: ContactAddedCallback> BoxBoxDetector<'_, T> {
    pub fn get_closest_points(
        &mut self,
        transform_a: Affine3A,
        transform_b: Affine3A,
        out: &mut Option<PersistentManifold>,
    ) {
        debug_assert!(out.is_none());
        let axis_a = transform_a.matrix3;
        let axis_b = transform_b.matrix3;
        let axis_a_inv = axis_a.transpose();

        let side1 = self.box1.get_half_extents() + self.box1.get_margin();
        let side2 = self.box2.get_half_extents() + self.box2.get_margin();

        let obb1 = Obb::new(transform_a.translation, axis_a, side1);
        let obb2 = Obb::new(transform_b.translation, axis_b, side2);

        let Some(hit) = box_box_sat(&obb1, &axis_a_inv, &obb2) else {
            return;
        };

        let mut manifold = PersistentManifold::new(self.col1, self.col2);

        self.compute_contact_points(&obb1, &axis_a, &obb2, &axis_b, &hit, &mut manifold);

        if manifold.point_cache.is_empty() {
            return;
        }

        manifold.refresh_contact_points(self.col1, self.col2);
        *out = Some(manifold);
    }

    fn compute_contact_points<'b>(
        &mut self,
        mut obb1: &'b Obb,
        mut r1t: &'b Mat3A,
        mut obb2: &'b Obb,
        mut r2t: &'b Mat3A,
        hit: &Hit,
        manifold: &mut PersistentManifold,
    ) {
        let mut r1t_axes = [r1t.x_axis, r1t.y_axis, r1t.z_axis];
        let mut r2t_axes = [r2t.x_axis, r2t.y_axis, r2t.z_axis];

        if hit.axis_idx > 6 {
            // an edge from box 1 touches an edge from box 2
            // find a point pa on the intersecting edge of box 1
            let mut pa = obb1.center;

            for (extent, axis) in obb1.extent.to_array().into_iter().zip(r1t_axes) {
                let sign = 2.0 * f32::from(hit.normal.dot(axis) > 0.0) - 1.0;
                pa += sign * extent * axis;
            }

            // find a point pb on the intersecting edge of box 2
            let mut pb = obb2.center;

            for (extent, axis) in obb2.extent.to_array().into_iter().zip(r2t_axes) {
                let sign = 1.0 - 2.0 * f32::from(hit.normal.dot(axis) > 0.0);
                pb += sign * extent * axis;
            }

            let ua = r1t_axes[(hit.axis_idx - 7) / 3];
            let ub = r2t_axes[(hit.axis_idx - 7) % 3];

            let beta = line_closest_approach(pa, ua, pb, ub);
            manifold.add_contact_point(
                self.col1,
                self.col2,
                -hit.normal,
                pb + ub * beta,
                -hit.depth,
                None,
                self.contact_added_callback,
            );

            return;
        }

        // okay, we have a face-something intersection (because the separating
        // axis is perpendicular to a face). define face 'a' to be the reference
        // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
        // the incident face (the closest face of the other box).
        if hit.axis_idx > 3 {
            mem::swap(&mut obb1, &mut obb2);
            mem::swap(&mut r1t, &mut r2t);
            mem::swap(&mut r1t_axes, &mut r2t_axes);
        }

        // nr = normal vector of reference face dotted with axes of incident box
        // anr = absolute values of nr
        let normal_2 = if hit.axis_idx <= 3 {
            hit.normal
        } else {
            -hit.normal
        };

        let nr = r2t.mul_transpose_vec3a(normal_2);
        let anr = nr.abs();

        // find the largest compontent of anr: this corresponds to the normal
        // for the indident face. the other axis numbers of the indicent face
        // are stored in a1,a2.
        let (lanr, a1, a2) = if anr.y > anr.x {
            if anr.y > anr.z { (1, 0, 2) } else { (2, 0, 1) }
        } else if anr.x > anr.z {
            (0, 1, 2)
        } else {
            (2, 0, 1)
        };

        // compute center point of incident face, in reference-face coordinates
        // btVector3 center;
        let center = obb2.center - obb1.center
            + if nr[lanr] < 0.0 {
                obb2.extent[lanr]
            } else {
                -obb2.extent[lanr]
            } * r2t_axes[lanr];

        // find the normal and non-normal axis numbers of the reference box
        let code_n = if hit.axis_idx <= 3 {
            hit.axis_idx - 1
        } else {
            hit.axis_idx - 4
        };

        let (code1, code2) = match code_n {
            0 => (1, 2),
            1 => (0, 2),
            _ => (0, 1),
        };

        // find the four corners of the incident face, in reference-face coordinates
        let c1 = center.dot(r1t_axes[code1]);
        let c2 = center.dot(r1t_axes[code2]);

        let mut m = Vec4::new(
            r1t_axes[code1].dot(r2t_axes[a1]),
            r1t_axes[code1].dot(r2t_axes[a2]),
            r1t_axes[code2].dot(r2t_axes[a1]),
            r1t_axes[code2].dot(r2t_axes[a2]),
        );

        let k = m * Vec4::new(
            obb2.extent[a1],
            obb2.extent[a1],
            obb2.extent[a2],
            obb2.extent[a2],
        );

        let quad = [
            [c1 - k.x - k.z, c2 - k.y - k.w],
            [c1 - k.x + k.z, c2 - k.y + k.w],
            [c1 + k.x + k.z, c2 + k.y + k.w],
            [c1 + k.x - k.z, c2 + k.y - k.w],
        ];

        // find the size of the reference face
        let rect = [obb1.extent[code1], obb1.extent[code2]];

        // intersect the incident and reference faces
        let mut ret = intersect_rect_quad2(rect, &quad);
        if ret.is_empty() {
            return;
        }

        // convert the intersection points into reference-face coordinates,
        // and compute the contact position and depth for each point. only keep
        // those points that have a positive (penetrating) depth. delete points in
        // the 'ret' array as necessary so that 'point' and 'ret' correspond.
        let det1 = 1.0 / (m.x * m.w - m.y * m.z);
        m *= det1;

        let mut cnum = 0;
        let mut point = [Vec3A::ZERO; 8];
        let mut dep = [0f32; 8];
        for j in 0..ret.len() {
            let k1 = m.w * (ret[j][0] - c1) - m.y * (ret[j][1] - c2);
            let k2 = -m.z * (ret[j][0] - c1) + m.x * (ret[j][1] - c2);

            point[cnum] = center + k1 * r2t_axes[a1] + k2 * r2t_axes[a2];
            dep[cnum] = obb1.extent[code_n] - normal_2.dot(point[cnum]);

            if dep[cnum] >= 0.0 {
                ret[cnum] = ret[j];
                cnum += 1;
            }
        }
        if cnum == 0 {
            return;
        }

        // we can't generate more contacts than we actually have
        let maxc = MANIFOLD_CACHE_SIZE.clamp(1, cnum);

        if cnum <= maxc {
            // use all the contact points we have
            if hit.axis_idx < 4 {
                for (depth, point) in dep.into_iter().zip(point).take(cnum) {
                    manifold.add_contact_point(
                        self.col1,
                        self.col2,
                        -hit.normal,
                        point + obb1.center,
                        -depth,
                        None,
                        self.contact_added_callback,
                    );
                }
            } else {
                for (depth, point) in dep.into_iter().zip(point).take(cnum) {
                    manifold.add_contact_point(
                        self.col1,
                        self.col2,
                        -hit.normal,
                        point + obb1.center - hit.normal * depth,
                        -depth,
                        None,
                        self.contact_added_callback,
                    );
                }
            }

            return;
        }

        // we have more contacts than are wanted, some of them must be culled.
        // find the deepest point, it is always the first contact.
        let (i1, _) = dep[..cnum]
            .iter()
            .copied()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();

        let iret = cull_points2(&ret[..cnum], i1, maxc);

        for idx in iret.into_iter().take(maxc) {
            let pos_in_world = point[idx] + obb1.center;

            if hit.axis_idx < 4 {
                manifold.add_contact_point(
                    self.col1,
                    self.col2,
                    -hit.normal,
                    pos_in_world,
                    -dep[idx],
                    None,
                    self.contact_added_callback,
                );
            } else {
                manifold.add_contact_point(
                    self.col1,
                    self.col2,
                    -hit.normal,
                    pos_in_world - hit.normal * dep[idx],
                    -dep[idx],
                    None,
                    self.contact_added_callback,
                );
            }
        }
    }
}

fn box_box_sat(obb1: &Obb, r1t: &Mat3A, obb2: &Obb) -> Option<Hit> {
    const FUDGE_FACTOR: f32 = 1.05;
    const FUDGE_2: Vec3A = Vec3A::splat(1e-5);

    let p = obb2.center - obb1.center;
    let pp = r1t * p;

    // Rij is R1'*R2, i.e. the relative rotation between R1 and R2.
    // `Mat3A` is column-major, so `rij_cols[j][i]` is Bullet's R(i + 1, j + 1).
    let rij = r1t * obb2.axis;
    let rij_cols = [rij.x_axis, rij.y_axis, rij.z_axis];

    let q_cols_mat = rij.abs();
    let mut q_cols = [q_cols_mat.x_axis, q_cols_mat.y_axis, q_cols_mat.z_axis];

    let q_rows_mat = q_cols_mat.transpose();
    let q_rows = [q_rows_mat.x_axis, q_rows_mat.y_axis, q_rows_mat.z_axis];

    // for all 15 possible separating axes:
    //   * see if the axis separates the boxes. if so, return 0.
    //   * find the depth of the penetration along the separating axis (s2)
    //   * if this is the largest depth so far, record it.
    // the normal vector will be set to the separating axis with the smallest
    // depth. note: normalR is set to point to a column of R1 or R2 if that is
    // the smallest depth normal so far. otherwise normalR is 0 and normalC is
    // set to a vector relative to body 1. invert_normal is 1 if the sign of
    // the normal should be flipped.

    let mut s = f32::NEG_INFINITY;
    let mut s2 = 0.0;
    let mut normal_r: Option<Vec3A> = None;
    let mut normal_c = Vec3A::ZERO;
    let mut invert_normal = false;
    let mut code = 0;

    let mut tst = |expr1: f32, expr2: f32, n: Vec3A, cc: usize| {
        s2 = expr1.abs() - expr2;
        if s2 > 0.0 {
            return false;
        }

        if s2 > s {
            s = s2;
            normal_r = Some(n);
            invert_normal = expr1 < 0.0;
            code = cc;
        }

        true
    };

    // separating axis = u1,u2,u3
    for (i, normal) in [obb1.axis.x_axis, obb1.axis.y_axis, obb1.axis.z_axis]
        .into_iter()
        .enumerate()
    {
        if !tst(
            pp[i],
            obb1.extent[i] + obb2.extent.dot(q_rows[i]),
            normal,
            i + 1,
        ) {
            return None;
        }
    }

    // separating axis = v1,v2,v3
    for (i, normal) in [obb2.axis.x_axis, obb2.axis.y_axis, obb2.axis.z_axis]
        .into_iter()
        .enumerate()
    {
        if !tst(
            normal.dot(p),
            obb1.extent.dot(q_cols[i]) + obb2.extent[i],
            normal,
            i + 4,
        ) {
            return None;
        }
    }

    // note: cross product axes need to be scaled when s is computed.
    // normal (n1,n2,n3) is relative to box 1.
    let mut tst = |expr1: f32, expr2: f32, n: Vec3A, cc: usize| {
        s2 = expr1.abs() - expr2;
        if s2 > f32::EPSILON {
            return false;
        }

        let l = n.length();
        if l > f32::EPSILON {
            s2 /= l;
            if s2 * FUDGE_FACTOR > s {
                s = s2;
                normal_r = None;
                normal_c = n / l;
                invert_normal = expr1 < 0.0;
                code = cc;
            }
        }

        true
    };

    for col in &mut q_cols {
        *col += FUDGE_2;
    }
    let q_rows_mat = Mat3A::from_cols(q_cols[0], q_cols[1], q_cols[2]).transpose();
    let q_rows = [q_rows_mat.x_axis, q_rows_mat.y_axis, q_rows_mat.z_axis];

    // separating axis = u_i x v_j
    for (i, axis) in [Vec3A::X, Vec3A::Y, Vec3A::Z].into_iter().enumerate() {
        let i1 = (i + 1) % 3;
        let i2 = (i + 2) % 3;

        for (j, r_col) in rij_cols.into_iter().enumerate() {
            let j1 = (j + 1) % 3;
            let j2 = (j + 2) % 3;
            let n = axis.cross(r_col);
            let expr2 = obb1.extent[i1] * q_cols[j][i2]
                + obb1.extent[i2] * q_cols[j][i1]
                + obb2.extent[j1] * q_rows[i][j2]
                + obb2.extent[j2] * q_rows[i][j1];

            if !tst(pp.dot(n), expr2, n, 7 + i * 3 + j) {
                return None;
            }
        }
    }

    if code == 0 {
        return None;
    }

    // if we get to this point, the boxes interpenetrate. compute the normal
    // in global coordinates.
    let mut normal = normal_r.unwrap_or_else(|| obb1.axis * normal_c);
    if invert_normal {
        normal = -normal;
    }

    let depth = -s;

    Some(Hit {
        depth,
        normal,
        axis_idx: code,
    })
}

#[cfg(test)]
mod tests {
    use glam::{Mat3A, Vec3A};

    use super::{Obb, box_box_sat};
    use crate::{CarBodyConfig, consts::UU_TO_BT};

    fn octane_obb(pos_uu: Vec3A, rot_mat: Mat3A) -> Obb {
        let config = CarBodyConfig::OCTANE;
        Obb::new(
            (pos_uu + rot_mat * config.hitbox_pos_offset) * UU_TO_BT,
            rot_mat,
            config.hitbox_size * UU_TO_BT * 0.5,
        )
    }

    #[test]
    fn rotated_offset_octane_hitboxes_overlap() {
        let car_a_rot = Mat3A::from_cols(
            Vec3A::new(-0.197_008_49, -0.710_327_74, -0.675_738_2),
            Vec3A::new(-0.328_847_1, -0.592_099, 0.735_586_46),
            Vec3A::new(0.924_127_6, -0.364_699_04, 0.113_941_91),
        );
        let car_b_rot = Mat3A::from_cols(
            Vec3A::new(0.031_831_503, 0.093_713_61, 0.995_090_25),
            Vec3A::new(0.937_990_5, -0.346_187_98, 0.003_024_293_8),
            Vec3A::new(0.344_967_37, 0.933_385_7, -0.098_937_63),
        );

        let car_a = octane_obb(Vec3A::new(161.537_67, -40.835_064, 366.293_03), car_a_rot);
        let car_b = octane_obb(Vec3A::new(148.926_6, -160.566_18, 400.462_95), car_b_rot);

        let hit = box_box_sat(&car_a, &car_a.axis.transpose(), &car_b);
        assert!(hit.is_some());
    }
}
