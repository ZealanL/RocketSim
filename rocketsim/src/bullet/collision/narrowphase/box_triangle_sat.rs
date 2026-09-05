use glam::{Affine3A, Vec3A};

/// Tie window in box-local distance units, shared by deepest-feature
/// selection and flat-patch detection. It covers float noise across the
/// different closest-point paths (positions reach ~100 units, so f32 noise
/// alone is ~1e-5) while staying tight against real feature gaps.
const TIE_EPS: f32 = 1e-4;

/// Single box-vs-triangle contact. The normal is unit length and points from
/// the triangle toward the box; the witness lies on the (unmargined) triangle;
/// `distance` is the unmargined signed separation minus the box margin
/// (negative for penetration, positive for separation).
#[derive(Clone, Copy, Debug)]
pub struct BoxTriangleSatContact {
    pub normal_on_b_world: Vec3A,
    pub point_on_b_world: Vec3A,
    pub distance: f32,
}

/// Isolated box-vs-triangle SAT/closest-feature kernel.
///
/// Inputs mirror the per-triangle compound leaf in `compound_collision_alg.rs`:
/// `box_trans` is the world transform of the (unmargined) box center,
/// `half_unmargined` is the implicit box half extents (full half minus
/// margin), `margin` is the box margin, the triangle `q` is given in the
/// box-local frame (via the precomputed mesh-to-box transform, so goal
/// component translations are already applied), and `maximum_distance` is
/// `margin + contact_breaking_threshold` (the `ClosestPointInput` limit).
/// `box_trans` is used only to convert the final witness and normal back
/// to world space.
///
/// The kernel uses only SAT sweeps and exact closest-feature queries.
/// Separated contacts take the normal from the maximum-gap SAT axis and the
/// distance/witness from the strict first-minimum exact closest-feature
/// pair (a margin-penetrating flat tie instead reports the tie-interval
/// point nearest the triangle centroid). Penetrating
/// contacts take the normal from the minimum-overlap SAT axis with a
/// witness from the winning axis family (box clamping, point-triangle,
/// segment-segment features, support vertices, or the centroid-snapped tie
/// interval, never arbitrary constants).
pub fn box_triangle_sat(
    box_trans: &Affine3A,
    half_unmargined: Vec3A,
    margin: f32,
    q: &[Vec3A; 3],
    maximum_distance: f32,
) -> Option<BoxTriangleSatContact> {
    // Cheap exact box-face pre-screen in sweep order (X, then Y, then Z).
    // For an axis-aligned unit axis, `dot(axis) == component` and the radius
    // is exactly the matching half extent, so each check below is bit-exact
    // with the sweep's first three axes: it returns `None` only when the full
    // sweep would reject on the same axis. This skips edge, face-normal
    // (including `sqrt`), and centroid work for the majority of rejected
    // triangles; the full sweep below re-evaluates these axes for witness
    // selection, so accepted contacts are unchanged.
    let tri_min = q[0].min(q[1]).min(q[2]);
    let tri_max = q[0].max(q[1]).max(q[2]);
    if tri_min.x > half_unmargined.x {
        if tri_min.x - half_unmargined.x - margin >= maximum_distance {
            return None;
        }
    } else if tri_max.x < -half_unmargined.x
        && -half_unmargined.x - tri_max.x - margin >= maximum_distance
    {
        return None;
    }
    if tri_min.y > half_unmargined.y {
        if tri_min.y - half_unmargined.y - margin >= maximum_distance {
            return None;
        }
    } else if tri_max.y < -half_unmargined.y
        && -half_unmargined.y - tri_max.y - margin >= maximum_distance
    {
        return None;
    }
    if tri_min.z > half_unmargined.z {
        if tri_min.z - half_unmargined.z - margin >= maximum_distance {
            return None;
        }
    } else if tri_max.z < -half_unmargined.z
        && -half_unmargined.z - tri_max.z - margin >= maximum_distance
    {
        return None;
    }

    let e0 = q[1] - q[0];
    let e1 = q[2] - q[1];
    let e2 = q[0] - q[2];

    // Triangle face axis (box-local). Skip when degenerate.
    let face_cross = e0.cross(-e2);
    let face_len2 = face_cross.length_squared();
    let has_face_axis = face_len2 > 1e-20;
    let face_axis = if has_face_axis {
        face_cross / face_len2.sqrt()
    } else {
        Vec3A::ZERO
    };

    let centroid = (q[0] + q[1] + q[2]) * (1.0 / 3.0);
    let edges = [e0, e1, e2];

    let sweep = sat_sweep(
        half_unmargined,
        q,
        tri_min,
        tri_max,
        &edges,
        face_axis,
        has_face_axis,
        centroid,
        margin,
        maximum_distance,
    )?;
    let has_sep = sweep.has_sep;
    let best_gap = sweep.best_gap;
    let best_sep_axis = sweep.best_sep_axis;
    let best_overlap = sweep.best_overlap;
    let best_pen_axis = sweep.best_pen_axis;

    if has_sep {
        // Separated: normal from the maximum-gap SAT axis; distance/witness from
        // the strict first-minimum exact pair (no tie averaging). A
        // margin-penetrating flat tie instead reports the tie-interval point nearest
        // the centroid; a degenerate (touching) pair uses the support vertex below.
        // The sweep exits early on any axis proving rejection, so an accepted
        // sweep always admits here.
        debug_assert!(best_gap - margin < maximum_distance);
        let n_world = (box_trans.matrix3 * best_sep_axis).normalize_or_zero();
        if n_world.length_squared() < 0.5 {
            return None;
        }
        // Closest-feature candidates are enumerated once and shared by the
        // strict first-minimum witness and the flat-tie analysis below.
        let pairs = enum_aabb_triangle_pairs(half_unmargined, q);
        if let Some((pa_first, pb_first)) =
            first_min_of_pairs(&pairs).filter(|(pa, pb)| (*pa - *pb).length_squared() > 1e-24)
        {
            let mut pb_local = pb_first;
            let mut snapped = false;
            if (pa_first - pb_first).length() - margin < 0.0
                && let Some((e0, e1)) = flat_tie_segment(half_unmargined, &pairs)
            {
                pb_local = closest_point_on_segment(centroid, e0, e1);
                snapped = true;
            }
            // Exact pair depth, or the box-clamped depth for the snapped interval point
            // (consistent with its witness).
            let distance = if snapped {
                (clamp_point_to_aabb(pb_local, half_unmargined) - pb_local).length() - margin
            } else {
                (pa_first - pb_first).length() - margin
            };
            if distance >= maximum_distance {
                return None;
            }
            return Some(BoxTriangleSatContact {
                normal_on_b_world: n_world,
                point_on_b_world: box_point_to_world(box_trans, pb_local),
                distance,
            });
        }
        // Degenerate touching pair: SAT axis + first-max support vertex.
        let n_local = best_sep_axis;
        let pb_local = tri_support_first_max(q, n_local);
        let distance = best_gap - margin;
        return Some(BoxTriangleSatContact {
            normal_on_b_world: n_world,
            point_on_b_world: box_point_to_world(box_trans, pb_local),
            distance,
        });
    }

    // Penetrating (or exactly touching): SAT minimum-overlap axis.
    if best_overlap == f32::MAX {
        return None;
    }
    let n_local = if best_pen_axis.length_squared() > 0.5 {
        best_pen_axis
    } else if has_face_axis {
        let s = face_axis.dot(-centroid);
        if s < 0.0 { -face_axis } else { face_axis }
    } else {
        return None;
    };
    let distance = -best_overlap - margin;
    if distance >= maximum_distance {
        return None;
    }
    // Witness by winning axis family; falls back to the overlap centroid, then support.
    // Depth and normal stay on the min-overlap axis.
    let pb_local = pen_witness_by_kind(half_unmargined, q, sweep.pen_kind, n_local)
        .or_else(|| clipped_tri_box_centroid(half_unmargined, q))
        .unwrap_or_else(|| tri_support_first_max(q, n_local));
    let n_world = (box_trans.matrix3 * n_local).normalize_or_zero();
    if n_world.length_squared() < 0.5 {
        return None;
    }
    Some(BoxTriangleSatContact {
        normal_on_b_world: n_world,
        point_on_b_world: box_point_to_world(box_trans, pb_local),
        distance,
    })
}

#[cfg(test)]
#[inline]
fn box_point_to_local(box_trans: &Affine3A, p_world: Vec3A) -> Vec3A {
    box_trans
        .matrix3
        .mul_transpose_vec3a(p_world - box_trans.translation)
}

#[inline]
fn box_point_to_world(box_trans: &Affine3A, p_local: Vec3A) -> Vec3A {
    box_trans.transform_point3a(p_local)
}

/// Winning SAT axis family of a sweep direction, in box-local space.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AxisKind {
    TriFace,
    BoxFace(usize),
    EdgeEdge { box_axis: usize, tri_edge: usize },
}

/// Outcome of the SAT axis sweep over one box-local triangle.
#[derive(Clone, Copy, Debug)]
struct SweepResult {
    has_sep: bool,
    best_gap: f32,
    best_sep_axis: Vec3A,
    best_overlap: f32,
    best_pen_axis: Vec3A,
    pen_kind: Option<AxisKind>,
}

/// Penetrating-axis orientation toward the box center (origin), from the
/// triangle toward the box. Hoisted out of the per-axis sweep so every axis
/// shares one exact `-centroid` value.
#[inline]
fn orient_pen_axis(a: Vec3A, to_box: Vec3A, face_axis: Vec3A, has_face_axis: bool) -> Vec3A {
    let s = a.dot(to_box);
    if s < 0.0 {
        -a
    } else if s > 1e-12 {
        a
    } else if has_face_axis && a.dot(face_axis) > 0.0 {
        // Exact center tie (plane through the box middle): keep
        // continuity with the approaching side by opposing the
        // triangle face instead of defaulting to +axis.
        -a
    } else {
        a
    }
}

/// Generic sweep axis for the triangle-face and edge-edge axes (the box faces
/// above use the cached triangle bounds directly). Plain function instead of
/// a closure so no environment is threaded through the hot per-axis calls.
#[inline]
#[allow(clippy::too_many_arguments)]
fn consider_axis(
    out: &mut SweepResult,
    a: Vec3A,
    kind: AxisKind,
    q: &[Vec3A; 3],
    half: Vec3A,
    to_box: Vec3A,
    face_axis: Vec3A,
    has_face_axis: bool,
    margin: f32,
    maximum_distance: f32,
) -> bool {
    let (gap, overlap, side) = axis_gap_overlap(a, q, half);
    if gap > 0.0 {
        if gap - margin >= maximum_distance {
            return true;
        }
        // Strict `>` keeps the first axis on exact ties.
        if !out.has_sep || gap > out.best_gap {
            out.has_sep = true;
            out.best_gap = gap;
            out.best_sep_axis = if side > 0.0 { -a } else { a };
        }
    } else if !out.has_sep && overlap < out.best_overlap {
        // Strict `<` keeps the first axis on exact ties.
        out.best_overlap = overlap;
        out.pen_kind = Some(kind);
        // Orient toward the box center (origin): from triangle to box.
        out.best_pen_axis = orient_pen_axis(a, to_box, face_axis, has_face_axis);
    }
    false
}

/// SAT sweep in fixed axis order (box X/Y/Z, triangle face, then box-axis x
/// triangle-edge), so exact ties resolve deterministically. Returns `None` as
/// soon as one axis proves `gap - margin >= maximum_distance`, which matches
/// the admission check on the full-sweep maximum gap.
#[inline]
#[allow(clippy::too_many_arguments)]
fn sat_sweep(
    half: Vec3A,
    q: &[Vec3A; 3],
    tri_min: Vec3A,
    tri_max: Vec3A,
    edges: &[Vec3A; 3],
    face_axis: Vec3A,
    has_face_axis: bool,
    centroid: Vec3A,
    margin: f32,
    maximum_distance: f32,
) -> Option<SweepResult> {
    let mut out = SweepResult {
        has_sep: false,
        best_gap: 0.0,
        best_sep_axis: Vec3A::ZERO,
        best_overlap: f32::MAX,
        best_pen_axis: Vec3A::ZERO,
        pen_kind: None,
    };
    let to_box = -centroid;
    // Box faces in original order via the cached triangle bounds. For a unit
    // axis `dot(axis) == component` and the radius is exactly the matching
    // half extent, so every gap/overlap below is bit-exact with the former
    // generic evaluation; strict tie rules and orientation match.
    if tri_min.x > half.x {
        let gap = tri_min.x - half.x;
        if gap - margin >= maximum_distance {
            return None;
        }
        // Strict `>` keeps the first axis on exact ties.
        if !out.has_sep || gap > out.best_gap {
            out.has_sep = true;
            out.best_gap = gap;
            out.best_sep_axis = -Vec3A::X;
        }
    } else if tri_max.x < -half.x {
        let gap = -half.x - tri_max.x;
        if gap - margin >= maximum_distance {
            return None;
        }
        // Strict `>` keeps the first axis on exact ties.
        if !out.has_sep || gap > out.best_gap {
            out.has_sep = true;
            out.best_gap = gap;
            out.best_sep_axis = Vec3A::X;
        }
    } else if !out.has_sep {
        let overlap = (tri_max.x + half.x).min(half.x - tri_min.x).max(0.0);
        // Strict `<` keeps the first axis on exact ties.
        if overlap < out.best_overlap {
            out.best_overlap = overlap;
            out.pen_kind = Some(AxisKind::BoxFace(0));
            out.best_pen_axis = orient_pen_axis(Vec3A::X, to_box, face_axis, has_face_axis);
        }
    }
    if tri_min.y > half.y {
        let gap = tri_min.y - half.y;
        if gap - margin >= maximum_distance {
            return None;
        }
        // Strict `>` keeps the first axis on exact ties.
        if !out.has_sep || gap > out.best_gap {
            out.has_sep = true;
            out.best_gap = gap;
            out.best_sep_axis = -Vec3A::Y;
        }
    } else if tri_max.y < -half.y {
        let gap = -half.y - tri_max.y;
        if gap - margin >= maximum_distance {
            return None;
        }
        // Strict `>` keeps the first axis on exact ties.
        if !out.has_sep || gap > out.best_gap {
            out.has_sep = true;
            out.best_gap = gap;
            out.best_sep_axis = Vec3A::Y;
        }
    } else if !out.has_sep {
        let overlap = (tri_max.y + half.y).min(half.y - tri_min.y).max(0.0);
        // Strict `<` keeps the first axis on exact ties.
        if overlap < out.best_overlap {
            out.best_overlap = overlap;
            out.pen_kind = Some(AxisKind::BoxFace(1));
            out.best_pen_axis = orient_pen_axis(Vec3A::Y, to_box, face_axis, has_face_axis);
        }
    }
    if tri_min.z > half.z {
        let gap = tri_min.z - half.z;
        if gap - margin >= maximum_distance {
            return None;
        }
        // Strict `>` keeps the first axis on exact ties.
        if !out.has_sep || gap > out.best_gap {
            out.has_sep = true;
            out.best_gap = gap;
            out.best_sep_axis = -Vec3A::Z;
        }
    } else if tri_max.z < -half.z {
        let gap = -half.z - tri_max.z;
        if gap - margin >= maximum_distance {
            return None;
        }
        // Strict `>` keeps the first axis on exact ties.
        if !out.has_sep || gap > out.best_gap {
            out.has_sep = true;
            out.best_gap = gap;
            out.best_sep_axis = Vec3A::Z;
        }
    } else if !out.has_sep {
        let overlap = (tri_max.z + half.z).min(half.z - tri_min.z).max(0.0);
        // Strict `<` keeps the first axis on exact ties.
        if overlap < out.best_overlap {
            out.best_overlap = overlap;
            out.pen_kind = Some(AxisKind::BoxFace(2));
            out.best_pen_axis = orient_pen_axis(Vec3A::Z, to_box, face_axis, has_face_axis);
        }
    }
    if has_face_axis
        && consider_axis(
            &mut out,
            face_axis,
            AxisKind::TriFace,
            q,
            half,
            to_box,
            face_axis,
            has_face_axis,
            margin,
            maximum_distance,
        )
    {
        return None;
    }
    for bi in 0..3 {
        let b_axis = match bi {
            0 => Vec3A::X,
            1 => Vec3A::Y,
            _ => Vec3A::Z,
        };
        for (ei, e) in edges.iter().enumerate() {
            let c = b_axis.cross(*e);
            let l2 = c.length_squared();
            if l2 < 1e-18 {
                continue;
            }
            if consider_axis(
                &mut out,
                c / l2.sqrt(),
                AxisKind::EdgeEdge {
                    box_axis: bi,
                    tri_edge: ei,
                },
                q,
                half,
                to_box,
                face_axis,
                has_face_axis,
                margin,
                maximum_distance,
            ) {
                return None;
            }
        }
    }
    Some(out)
}

/// `(gap, overlap, side)` for a unit axis `a`: `gap > 0` when disjoint,
/// `side > 0` when the triangle is on the + side, `overlap >= 0` otherwise.
#[inline]
fn axis_gap_overlap(a: Vec3A, q: &[Vec3A; 3], half: Vec3A) -> (f32, f32, f32) {
    let r = half.x * a.x.abs() + half.y * a.y.abs() + half.z * a.z.abs();
    let d0 = q[0].dot(a);
    let d1 = q[1].dot(a);
    let d2 = q[2].dot(a);
    let min_t = d0.min(d1).min(d2);
    let max_t = d0.max(d1).max(d2);
    if min_t > r {
        (min_t - r, 0.0, 1.0)
    } else if max_t < -r {
        (-r - max_t, 0.0, -1.0)
    } else {
        // Penetration depth: `min` over the two exit directions, not the intersection
        // length (a zero-thickness plane through the box center needs exit travel `r`).
        let exit_pos = max_t + r;
        let exit_neg = r - min_t;
        (0.0, exit_pos.min(exit_neg).max(0.0), 0.0)
    }
}

#[inline]
fn tri_support_first_max(q: &[Vec3A; 3], dir: Vec3A) -> Vec3A {
    let mut best = q[0];
    let mut best_d = q[0].dot(dir);
    for v in q.iter().skip(1) {
        let d = v.dot(dir);
        if d > best_d {
            best_d = d;
            best = *v;
        }
    }
    best
}

fn clipped_tri_box_centroid(half: Vec3A, q: &[Vec3A; 3]) -> Option<Vec3A> {
    let mut poly = [Vec3A::ZERO; 9];
    poly[0] = q[0];
    poly[1] = q[1];
    poly[2] = q[2];
    let mut len = 3usize;
    let mut scratch = [Vec3A::ZERO; 9];
    for axis in 0..3 {
        let h = half[axis];
        len = clip_halfspace(&poly[..len], &mut scratch, axis, -h, true);
        if len == 0 {
            return None;
        }
        len = clip_halfspace(&scratch[..len], &mut poly, axis, h, false);
        if len == 0 {
            return None;
        }
    }
    let mut sum = Vec3A::ZERO;
    for v in poly[..len].iter() {
        sum += *v;
    }
    Some(sum / (len as f32))
}

fn clip_halfspace(
    input: &[Vec3A],
    output: &mut [Vec3A],
    axis: usize,
    bound: f32,
    keep_above: bool,
) -> usize {
    if input.is_empty() {
        return 0;
    }
    let dist = |v: Vec3A| {
        let c = v[axis];
        if keep_above { c - bound } else { bound - c }
    };
    let mut out = 0usize;
    let mut prev = input[input.len() - 1];
    let mut prev_d = dist(prev);
    for v in input.iter() {
        let d = dist(*v);
        if d >= 0.0 {
            if prev_d < 0.0 {
                let t = prev_d / (prev_d - d);
                output[out] = prev + (*v - prev) * t;
                out += 1;
            }
            output[out] = *v;
            out += 1;
        } else if prev_d >= 0.0 {
            let t = prev_d / (prev_d - d);
            output[out] = prev + (*v - prev) * t;
            out += 1;
        }
        prev = *v;
        prev_d = d;
    }
    out
}

#[inline]
fn clamp_point_to_aabb(p: Vec3A, half: Vec3A) -> Vec3A {
    Vec3A::new(
        p.x.clamp(-half.x, half.x),
        p.y.clamp(-half.y, half.y),
        p.z.clamp(-half.z, half.z),
    )
}

/// Closest point on triangle `q` to `p` (box-local), with a segment fallback
/// for degenerate triangles.
fn closest_point_on_triangle(p: Vec3A, q: &[Vec3A; 3]) -> Vec3A {
    let ab = q[1] - q[0];
    let ac = q[2] - q[0];
    let ap = p - q[0];
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return q[0];
    }
    let bp = p - q[1];
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return q[1];
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return q[0] + ab * v;
    }
    let cp = p - q[2];
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return q[2];
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return q[0] + ac * w;
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return q[1] + (q[2] - q[1]) * w;
    }
    // Inside face region; the denom can vanish for degenerate triangles.
    let denom = va + vb + vc;
    if denom.abs() < 1e-24 {
        return closest_point_on_degenerate_triangle(p, q);
    }
    let v = vb / denom;
    let w = vc / denom;
    q[0] + ab * v + ac * w
}

fn closest_point_on_segment(p: Vec3A, a: Vec3A, b: Vec3A) -> Vec3A {
    let ab = b - a;
    let denom = ab.dot(ab);
    if denom < 1e-30 {
        return a;
    }
    let t = ((p - a).dot(ab) / denom).clamp(0.0, 1.0);
    a + ab * t
}

fn closest_point_on_degenerate_triangle(p: Vec3A, q: &[Vec3A; 3]) -> Vec3A {
    let c0 = closest_point_on_segment(p, q[0], q[1]);
    let c1 = closest_point_on_segment(p, q[1], q[2]);
    let c2 = closest_point_on_segment(p, q[2], q[0]);
    let d0 = (p - c0).length_squared();
    let d1 = (p - c1).length_squared();
    let d2 = (p - c2).length_squared();
    // First-min wins ties deterministically.
    if d0 <= d1 && d0 <= d2 {
        c0
    } else if d1 <= d2 {
        c1
    } else {
        c2
    }
}

/// Closest points between segments `(p1,q1)` and `(p2,q2)` (Ericson 5.1.9);
/// returns `(c1, c2)` with `c1` on the first segment.
fn segment_segment_closest(p1: Vec3A, q1: Vec3A, p2: Vec3A, q2: Vec3A) -> (Vec3A, Vec3A) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);
    let eps = 1e-30;
    let (mut s, mut t);
    if a <= eps && e <= eps {
        return (p1, p2);
    }
    if a <= eps {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= eps {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            // Nearly parallel overlapping segments share a tie segment: use the overlap
            // midpoint (~1.8-degree threshold).
            let para = b.abs() / (a * e).sqrt();
            if para > 0.9995 {
                let t0 = (p2 - p1).dot(d1) / a;
                let t1 = (q2 - p1).dot(d1) / a;
                let lo = 0.0f32.max(t0.min(t1));
                let hi = 1.0f32.min(t0.max(t1));
                if lo <= hi {
                    let sm = (lo + hi) * 0.5;
                    let c1 = p1 + d1 * sm;
                    let tm = ((c1 - p2).dot(d2) / e).clamp(0.0, 1.0);
                    return (c1, p2 + d2 * tm);
                }
            }
            let denom = a * e - b * b;
            // Remaining parallel cases do not overlap, so `s = 0` applies.
            s = if denom > eps {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
        }
    }
    (p1 + d1 * s, p2 + d2 * t)
}

/// Box support corner without margin, mirroring
/// `BoxShape::local_get_supporting_vertex_without_margin` (componentwise
/// sign with ties toward +1).
fn box_support_corner(half: Vec3A, dir: Vec3A) -> Vec3A {
    Vec3A::new(
        if dir.x < 0.0 { -half.x } else { half.x },
        if dir.y < 0.0 { -half.y } else { half.y },
        if dir.z < 0.0 { -half.z } else { half.z },
    )
}

/// Edge-function containment of `p` in triangle `q` with plane normal `n`.
fn point_in_triangle_eps(p: Vec3A, q: &[Vec3A; 3], n: Vec3A) -> bool {
    const EPS: f32 = 1e-6;
    let mut pos = false;
    let mut neg = false;
    for i in 0..3 {
        let s = (q[(i + 1) % 3] - q[i]).cross(p - q[i]).dot(n);
        if s > EPS {
            pos = true;
        } else if s < -EPS {
            neg = true;
        }
        if pos && neg {
            return false;
        }
    }
    true
}

/// Clip segment AB to the centered box `[-half, half]`.
fn clip_segment_to_box(a: Vec3A, b: Vec3A, half: Vec3A) -> Option<(Vec3A, Vec3A)> {
    let mut t0 = 0.0f32;
    let mut t1 = 1.0f32;
    let d = b - a;
    for axis in 0..3 {
        let h = half[axis];
        for is_min in [true, false] {
            let p = if is_min { d[axis] } else { -d[axis] };
            let qv = if is_min { a[axis] + h } else { h - a[axis] };
            if p.abs() < f32::EPSILON {
                if qv < 0.0 {
                    return None;
                }
            } else {
                let r = qv / p;
                if p < 0.0 {
                    if r > t1 {
                        return None;
                    }
                    if r > t0 {
                        t0 = r;
                    }
                } else {
                    if r < t0 {
                        return None;
                    }
                    if r < t1 {
                        t1 = r;
                    }
                }
            }
        }
    }
    if t0 > t1 {
        return None;
    }
    Some((a + d * t0, a + d * t1))
}

/// Penetrating witness by winning axis family (box-local, unmargined).
/// `n_local` points from the triangle toward the box. All outputs lie on the
/// triangle; returns `None` when the family rule has no witness (e.g. face
/// projection outside the triangle), and the caller falls back to the overlap
/// centroid.
fn pen_witness_by_kind(
    half: Vec3A,
    q: &[Vec3A; 3],
    kind: Option<AxisKind>,
    n_local: Vec3A,
) -> Option<Vec3A> {
    match kind? {
        AxisKind::TriFace => {
            let plane_dist = n_local.dot(q[0]);
            // `n_local` points from the triangle toward the box, so the deepest corner
            // is the support in `-n_local`; projecting the far-side support instead
            // lands a full box diagonal away, usually outside the triangle.
            let corner = box_support_corner(half, -n_local);
            let p = corner - n_local * (n_local.dot(corner) - plane_dist);
            point_in_triangle_eps(p, q, n_local).then_some(p)
        }
        AxisKind::BoxFace(_) => {
            // Deepest triangle vertices toward the box along the axis.
            let mut deepest = f32::NEG_INFINITY;
            for v in q.iter() {
                deepest = deepest.max(n_local.dot(*v));
            }
            let mut tied = [Vec3A::ZERO; 3];
            let mut n_tied = 0usize;
            for v in q.iter() {
                if (n_local.dot(*v) - deepest).abs() <= TIE_EPS {
                    tied[n_tied] = *v;
                    n_tied += 1;
                }
            }
            match n_tied {
                0 => None,
                1 => Some(tied[0]),
                2 => {
                    // Midpoint of the tied edge clipped to the box.
                    clip_segment_to_box(tied[0], tied[1], half).map(|(a, b)| (a + b) * 0.5)
                }
                _ => {
                    // Whole-triangle tie: centroid of the overlap patch.
                    clipped_tri_box_centroid(half, q)
                }
            }
        }
        AxisKind::EdgeEdge { box_axis, tri_edge } => {
            // Triangle-side closest point to the winning box edge.
            let s = (-n_local).signum();
            let mut start = half * s;
            let mut end = half * s;
            start[box_axis] = half[box_axis];
            end[box_axis] = -half[box_axis];
            let (_, tri_pt) =
                segment_segment_closest(start, end, q[tri_edge], q[(tri_edge + 1) % 3]);
            Some(tri_pt)
        }
    }
}

/// Every feature-pair candidate between the centered AABB `[-half, half]` and
/// the triangle `q` (both box-local) in deterministic enumeration order: 3
/// clamped triangle vertices, 8 box corners vs triangle, then 36 triangle-edge
/// vs box-edge segment pairs. Returns `(point_on_box, point_on_tri)` pairs.
fn enum_aabb_triangle_pairs(half: Vec3A, q: &[Vec3A; 3]) -> [(Vec3A, Vec3A); 47] {
    let mut pairs = [(Vec3A::ZERO, Vec3A::ZERO); 47];
    let mut n = 0usize;
    // Triangle vertices vs box (face interiors via clamping).
    for v in q.iter() {
        pairs[n] = (clamp_point_to_aabb(*v, half), *v);
        n += 1;
    }
    // Box corners vs triangle (covers box-vertex vs tri-face).
    for sx in [-1.0f32, 1.0] {
        for sy in [-1.0f32, 1.0] {
            for sz in [-1.0f32, 1.0] {
                let corner = Vec3A::new(sx * half.x, sy * half.y, sz * half.z);
                pairs[n] = (corner, closest_point_on_triangle(corner, q));
                n += 1;
            }
        }
    }
    // Edge-edge interiors: 3 triangle edges vs 12 box edges.
    let c = [
        Vec3A::new(-half.x, -half.y, -half.z),
        Vec3A::new(half.x, -half.y, -half.z),
        Vec3A::new(half.x, half.y, -half.z),
        Vec3A::new(-half.x, half.y, -half.z),
        Vec3A::new(-half.x, -half.y, half.z),
        Vec3A::new(half.x, -half.y, half.z),
        Vec3A::new(half.x, half.y, half.z),
        Vec3A::new(-half.x, half.y, half.z),
    ];
    const BOX_EDGES: [(usize, usize); 12] = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];
    const TRI_EDGES: [(usize, usize); 3] = [(0, 1), (1, 2), (2, 0)];
    for (ti0, ti1) in TRI_EDGES {
        for (bi0, bi1) in BOX_EDGES {
            let (ctri, cbox) = segment_segment_closest(q[ti0], q[ti1], c[bi0], c[bi1]);
            pairs[n] = (cbox, ctri);
            n += 1;
        }
    }
    debug_assert_eq!(n, 47);
    pairs
}

/// Strict first minimum over the exact closest-feature candidates: the first
/// pair in enumeration order achieving the minimum distance wins ties, so
/// repeated queries return the same witness without blending.
fn first_min_of_pairs(pairs: &[(Vec3A, Vec3A); 47]) -> Option<(Vec3A, Vec3A)> {
    let mut best = pairs[0];
    let mut best_d2 = (best.0 - best.1).length_squared();
    for (pa, pb) in pairs.iter().skip(1) {
        let d2 = (*pa - *pb).length_squared();
        // Strict `<` keeps the first candidate on exact ties.
        if d2 < best_d2 {
            best_d2 = d2;
            best = (*pa, *pb);
        }
    }
    Some(best)
}

/// Tie-interval endpoints of a flat closest-feature patch (box-local tri-side
/// points of the diameter pair). Collects pairs within `TIE_EPS` of the minimum,
/// extends along flat valley directions (a far candidate counts when the depth
/// slope toward it stays level and the segment middle is itself at minimum
/// depth), and returns the diameter endpoints when they span more than the link
/// length. Returns `None` for a unique minimum.
fn flat_tie_segment(half: Vec3A, pairs: &[(Vec3A, Vec3A); 47]) -> Option<(Vec3A, Vec3A)> {
    // Shared `TIE_EPS` tie window (see top of file).
    // Flatness is judged by slope, not by widening the distance window, so distinct
    // minima (steep slopes) still resolve unique. The slope gate is the tie window
    // over the link length, which scales with the box (a near-parallel edge over a
    // face measures ~7e-5).
    let flat_link: f32 = 0.5 * half.max_element();
    // A zero-size box has no patch extent, so no interval can form; this also
    // guards the slope division below against a zero link.
    if flat_link <= 0.0 || !flat_link.is_finite() {
        return None;
    }
    let flat_slope: f32 = TIE_EPS / flat_link;

    let mut best_d2 = f32::MAX;
    for (pa, pb) in pairs.iter() {
        let d2 = (*pa - *pb).length_squared();
        if d2 < best_d2 {
            best_d2 = d2;
        }
    }
    let best_d = best_d2.sqrt();

    // Tied pairs (within the tie window) and their mean, which anchors the
    // flat-valley search below.
    let mut tied = [(Vec3A::ZERO, Vec3A::ZERO); 47];
    let mut n_tied = 0usize;
    for (pa, pb) in pairs.iter() {
        if (*pa - *pb).length() <= best_d + TIE_EPS {
            tied[n_tied] = (*pa, *pb);
            n_tied += 1;
        }
    }
    if n_tied == 0 {
        return None;
    }
    let mut mean_a = Vec3A::ZERO;
    let mut mean_b = Vec3A::ZERO;
    for (pa, pb) in tied[..n_tied].iter() {
        mean_a += *pa;
        mean_b += *pb;
    }
    let inv = 1.0 / (n_tied as f32);
    mean_a *= inv;
    mean_b *= inv;
    let d0 = (mean_a - mean_b).length();

    // Flat extension past the tied set: both shapes are convex, so the middle of a
    // true tie segment is itself at minimum depth while distinct minima bulge away.
    let mut flat: Option<(Vec3A, Vec3A)> = None;
    let mut best_slope = flat_slope;
    for (pa, pb) in pairs.iter() {
        let sep = (*pb - mean_b).length();
        if sep <= flat_link {
            continue;
        }
        let slope = ((pa - pb).length() - d0) / sep;
        if slope <= best_slope {
            let mid_a = (mean_a + *pa) * 0.5;
            let mid_b = (mean_b + *pb) * 0.5;
            if (mid_a - mid_b).length() <= d0 + TIE_EPS {
                best_slope = slope;
                flat = Some((*pa, *pb));
            }
        }
    }

    // Diameter endpoints over the tied set plus the verified flat partner; a
    // small diameter means a unique minimum (possibly with float-dust
    // neighbours on the same feature).
    let mut end0 = tied[0];
    let mut end1 = tied[0];
    let mut span = 0.0f32;
    let mut consider = |cand: (Vec3A, Vec3A)| {
        for (pa, pb) in tied[..n_tied].iter().chain(flat.iter()) {
            let s = (cand.1 - *pb).length();
            if s > span {
                span = s;
                end0 = cand;
                end1 = (*pa, *pb);
            }
        }
    };
    for t in tied[..n_tied].iter() {
        consider(*t);
    }
    if let Some(f) = flat {
        consider(f);
    }
    if span <= flat_link {
        return None;
    }
    // Final guard: the diameter middle of two distinct minima bulges away
    // (e.g. near-equal coincidences at opposite box ends).
    let mid_a = (end0.0 + end1.0) * 0.5;
    let mid_b = (end0.1 + end1.1) * 0.5;
    if (mid_a - mid_b).length() > best_d + TIE_EPS {
        return None;
    }
    Some((end0.1, end1.1))
}

#[cfg(test)]
mod oracle_tests {
    use glam::{Mat3A, Quat, Vec3};

    use super::*;
    use crate::bullet::collision::{
        narrowphase::gjk::{ClosestPointInput, GjkPairDetector, GjkResult},
        shapes::{
            box_shape::BoxShape, collision_shape::CollisionShapes, compound_shape::CompoundShape,
            triangle_shape::TriangleShape,
        },
    };

    struct Collect {
        pub normal: Option<Vec3A>,
        pub point: Option<Vec3A>,
        pub depth: Option<f32>,
    }

    impl GjkResult for Collect {
        fn add_contact_point(&mut self, normal_on_b: Vec3A, point_on_b_world: Vec3A, depth: f32) {
            self.normal = Some(normal_on_b);
            self.point = Some(point_on_b_world);
            self.depth = Some(depth);
        }
    }

    struct OracleCase {
        pub name: &'static str,
        pub box_trans: Affine3A,
        pub half_full: Vec3A,
        pub tri_world: [Vec3A; 3],
    }

    fn gjk_oracle(
        box_trans: &Affine3A,
        half_full: Vec3A,
        tri_world: &[Vec3A; 3],
        max_dist: f32,
    ) -> (Option<(Vec3A, Vec3A, f32)>, f32, bool) {
        let box_shape = BoxShape::new(half_full);
        let compound = CompoundShape::new(box_shape, Affine3A::IDENTITY);
        let shape_a = CollisionShapes::Compound(compound);
        let shape_a_ref = &shape_a;
        let margin_a = shape_a_ref.get_margin();
        let triangle = TriangleShape::new(*tri_world);
        let shape_b = CollisionShapes::Triangle(triangle);
        let tri_ident = Affine3A::IDENTITY;
        let input = ClosestPointInput::new(box_trans, &tri_ident, max_dist);
        let detector = GjkPairDetector::new(margin_a, 0.0);
        let mut out = Collect {
            normal: None,
            point: None,
            depth: None,
        };
        let ran = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            detector.get_closest_points(&input, shape_a_ref, &shape_b, &mut out);
        }));
        if ran.is_err() {
            return (None, margin_a, true);
        }
        let res = match (out.normal, out.point, out.depth) {
            (Some(n), Some(p), Some(d)) => Some((n, p, d)),
            _ => None,
        };
        (res, margin_a, false)
    }

    fn sat_call(
        box_trans: &Affine3A,
        half_full: Vec3A,
        tri_world: &[Vec3A; 3],
        max_dist: f32,
    ) -> (Option<BoxTriangleSatContact>, f32) {
        let probe = BoxShape::new(half_full);
        let half_unmargined = probe.get_half_extents();
        let margin = probe.get_margin();
        let to_local = box_trans.inverse();
        let q = [
            to_local.transform_point3a(tri_world[0]),
            to_local.transform_point3a(tri_world[1]),
            to_local.transform_point3a(tri_world[2]),
        ];
        let res = box_triangle_sat(box_trans, half_unmargined, margin, &q, max_dist);
        (res, margin)
    }

    /// Mismatch categories for one oracle pair.
    #[derive(Default, Debug)]
    struct Tallies {
        pub total: usize,
        pub both_none: usize,
        pub admit_mismatch: usize,
        pub admit_gjk_only: usize,
        pub admit_sat_only: usize,
        pub hemisphere: usize,
        pub distance: usize,
        pub point_gap: usize,
        pub point_normal_gap: usize,
        pub point_tangent_only: usize,
        pub point_off_tri: usize,
        pub degenerate: usize,
        pub oracle_crash: usize,
        pub admit_logs: usize,
        pub hemi_logs: usize,
    }

    fn point_triangle_distance(p: Vec3A, tri: &[Vec3A; 3]) -> f32 {
        (p - closest_point_on_triangle(p, tri)).length()
    }

    #[allow(clippy::too_many_arguments)]
    fn compare_pair(
        name: &str,
        gjk: &Option<(Vec3A, Vec3A, f32)>,
        sat: &Option<BoxTriangleSatContact>,
        tri_world: &[Vec3A; 3],
        t: &mut Tallies,
        verbose: bool,
    ) {
        t.total += 1;
        match (gjk, sat) {
            (None, None) => {
                t.both_none += 1;
            }
            (Some(_), None) | (None, Some(_)) => {
                t.admit_mismatch += 1;
                if gjk.is_some() {
                    t.admit_gjk_only += 1;
                } else {
                    t.admit_sat_only += 1;
                }
                if verbose || t.admit_logs < 8 {
                    t.admit_logs += 1;
                    let gd = gjk.as_ref().map(|v| v.2);
                    let sd = sat.as_ref().map(|v| v.distance);
                    eprintln!("admit mismatch {name}: gjk_dist={gd:?} sat_dist={sd:?}");
                }
            }
            (Some((gn, gp, gd)), Some(s)) => {
                let deg = TriangleShape::new(*tri_world).normal_length < 1e-6
                    || gn.length_squared() < 1e-8
                    || s.normal_on_b_world.length_squared() < 1e-8;
                if deg {
                    t.degenerate += 1;
                }
                if gn.dot(s.normal_on_b_world) < 0.0 {
                    t.hemisphere += 1;
                    if verbose || t.hemi_logs < 8 {
                        t.hemi_logs += 1;
                        eprintln!(
                            "hemisphere {name}: gjk_n={gn:?} sat_n={:?} dot={} gjk_d={} sat_d={}",
                            s.normal_on_b_world,
                            gn.dot(s.normal_on_b_world),
                            gd,
                            s.distance,
                        );
                    }
                }
                if (gd - s.distance).abs() > 5e-3 {
                    t.distance += 1;
                    if verbose {
                        eprintln!(
                            "distance {name}: gjk={gd} sat={} diff={}",
                            s.distance,
                            (gd - s.distance).abs()
                        );
                    }
                }
                if (*gp - s.point_on_b_world).length() > 5e-2 {
                    t.point_gap += 1;
                    let delta = s.point_on_b_world - *gp;
                    let n_gap = delta.dot(*gn).abs();
                    let t_gap = (delta - *gn * delta.dot(*gn)).length();
                    if n_gap > 5e-2 {
                        t.point_normal_gap += 1;
                    } else {
                        t.point_tangent_only += 1;
                    }
                    if verbose {
                        eprintln!(
                            "point gap {name}: gjk_p={gp:?} sat_p={:?} gap={} normal={n_gap:.4} tangent={t_gap:.4}",
                            s.point_on_b_world,
                            (*gp - s.point_on_b_world).length()
                        );
                    }
                }
                // Point-on-B contract: SAT witness must lie on the triangle.
                let off = point_triangle_distance(s.point_on_b_world, tri_world);
                if off > 1e-3 {
                    t.point_off_tri += 1;
                    if verbose {
                        eprintln!("point off tri {name}: off={off}");
                    }
                }
            }
        }
    }

    fn deterministic_cases() -> Vec<OracleCase> {
        let h = Vec3A::new(0.6, 0.43, 0.19);
        let ident = Affine3A::IDENTITY;
        vec![
            OracleCase {
                name: "face_top_touch",
                box_trans: ident,
                half_full: h,
                tri_world: [
                    Vec3A::new(-1.0, -1.0, 0.19),
                    Vec3A::new(1.0, -1.0, 0.19),
                    Vec3A::new(0.0, 1.0, 0.19),
                ],
            },
            OracleCase {
                name: "face_top_gap",
                box_trans: ident,
                half_full: h,
                tri_world: [
                    Vec3A::new(-1.0, -1.0, 0.30),
                    Vec3A::new(1.0, -1.0, 0.30),
                    Vec3A::new(0.0, 1.0, 0.30),
                ],
            },
            OracleCase {
                name: "face_top_pen_shallow",
                box_trans: ident,
                half_full: h,
                tri_world: [
                    Vec3A::new(-1.0, -1.0, 0.17),
                    Vec3A::new(1.0, -1.0, 0.17),
                    Vec3A::new(0.0, 1.0, 0.17),
                ],
            },
            OracleCase {
                name: "face_top_pen_deep",
                box_trans: ident,
                half_full: h,
                tri_world: [
                    Vec3A::new(-1.0, -1.0, 0.0),
                    Vec3A::new(1.0, -1.0, 0.0),
                    Vec3A::new(0.0, 1.0, 0.0),
                ],
            },
            OracleCase {
                name: "face_side_x",
                box_trans: ident,
                half_full: h,
                tri_world: [
                    Vec3A::new(0.62, -1.0, -1.0),
                    Vec3A::new(0.62, 1.0, -1.0),
                    Vec3A::new(0.62, 0.0, 1.0),
                ],
            },
            OracleCase {
                name: "edge_contact",
                box_trans: ident,
                half_full: h,
                tri_world: [
                    Vec3A::new(0.6, 0.43, 0.19),
                    Vec3A::new(1.5, 1.0, 0.8),
                    Vec3A::new(1.5, -0.5, 0.8),
                ],
            },
            OracleCase {
                name: "corner_graze",
                box_trans: Affine3A::from_translation(Vec3A::new(0.0, 119.0, 4.66).into()),
                half_full: Vec3A::new(1.0, 2.0, 0.5),
                tri_world: [
                    Vec3A::new(-5.12, 118.9, 5.14),
                    Vec3A::new(5.12, 118.9, 5.14),
                    Vec3A::new(0.0, 120.0, 5.1),
                ],
            },
            OracleCase {
                name: "far_separated_none",
                box_trans: ident,
                half_full: h,
                tri_world: [
                    Vec3A::new(5.0, 5.0, 5.0),
                    Vec3A::new(6.0, 5.0, 5.0),
                    Vec3A::new(5.0, 6.0, 5.0),
                ],
            },
            OracleCase {
                name: "yaw_45_face",
                box_trans: Affine3A::from_rotation_translation(
                    Quat::from_axis_angle(Vec3::Y, std::f32::consts::FRAC_PI_4),
                    Vec3::ZERO,
                ),
                half_full: h,
                tri_world: [
                    Vec3A::new(-1.0, -1.0, 0.25),
                    Vec3A::new(1.0, -1.0, 0.25),
                    Vec3A::new(0.0, 1.0, 0.25),
                ],
            },
            OracleCase {
                name: "pitch_30_edge",
                box_trans: Affine3A::from_rotation_translation(
                    Quat::from_axis_angle(Vec3::X, std::f32::consts::FRAC_PI_6),
                    Vec3::ZERO,
                ),
                half_full: h,
                tri_world: [
                    Vec3A::new(-0.8, 0.0, 0.4),
                    Vec3A::new(0.8, 0.0, 0.4),
                    Vec3A::new(0.0, 0.0, -0.6),
                ],
            },
            OracleCase {
                name: "degenerate_collinear",
                box_trans: ident,
                half_full: h,
                tri_world: [
                    Vec3A::new(0.0, 0.0, 1.0),
                    Vec3A::new(1.0, 0.0, 1.0),
                    Vec3A::new(2.0, 0.0, 1.0),
                ],
            },
            OracleCase {
                name: "degenerate_point",
                box_trans: ident,
                half_full: h,
                tri_world: [
                    Vec3A::new(0.1, 0.1, 0.5),
                    Vec3A::new(0.1, 0.1, 0.5),
                    Vec3A::new(0.1, 0.1, 0.5),
                ],
            },
            OracleCase {
                name: "tie_parallel_face",
                box_trans: ident,
                half_full: h,
                tri_world: [
                    Vec3A::new(-0.3, -0.2, 0.19),
                    Vec3A::new(0.3, -0.2, 0.19),
                    Vec3A::new(0.0, 0.2, 0.19),
                ],
            },
            OracleCase {
                name: "needle_through_box",
                box_trans: ident,
                half_full: h,
                tri_world: [
                    Vec3A::new(-2.0, 0.0, 0.0),
                    Vec3A::new(2.0, 0.0, 0.0),
                    Vec3A::new(0.0, 0.0, 0.001),
                ],
            },
            OracleCase {
                name: "small_sliver",
                box_trans: ident,
                half_full: h,
                tri_world: [
                    Vec3A::new(0.61, 0.0, 0.0),
                    Vec3A::new(0.62, 0.01, 0.0),
                    Vec3A::new(0.61, 0.0, 0.01),
                ],
            },
            OracleCase {
                name: "turtle_tri116",
                box_trans: Affine3A {
                    matrix3: Mat3A::from_cols(
                        Vec3A::new(-0.59333324, -0.01082119, 0.8048841),
                        Vec3A::new(-0.0034576724, 0.9999347, 0.010894649),
                        Vec3A::new(-0.8049494, 0.0036811323, -0.5933318),
                    ),
                    translation: Vec3A::new(-80.52899, 7.1077247, 2.4730747),
                },
                half_full: Vec3A::new(1.20507, 0.866994, 0.386591),
                tri_world: [
                    Vec3A::new(-81.81588, 10.240001, 4.0888753),
                    Vec3A::new(-81.51728, 5.096394e-7, 3.1281445),
                    Vec3A::new(-81.51729, 10.24, 3.1281428),
                ],
            },
        ]
    }

    #[test]
    fn sat_oracle_deterministic_report() {
        let mut t = Tallies::default();
        for case in deterministic_cases() {
            // Admission limit mirrors the compound leaf: margin + CBT floor.
            // Use the true manifold-scale CBT proxy (0.05) to include near
            // contacts while keeping far cases rejected.
            let probe = BoxShape::new(case.half_full);
            let max_dist = probe.get_margin() + 0.05;
            // Degenerate triangles (zero area / duplicate points) can abort
            // the EPA fallback inside the GJK oracle via debug_assert, so
            // they are checked for graceful SAT output only, not parity.
            if TriangleShape::new(case.tri_world).normal_length < 1e-9 {
                let (sat, _) = sat_call(&case.box_trans, case.half_full, &case.tri_world, max_dist);
                t.total += 1;
                t.degenerate += 1;
                if let Some(s) = sat {
                    let off = point_triangle_distance(s.point_on_b_world, &case.tri_world);
                    if off > 1e-3 {
                        t.point_off_tri += 1;
                    }
                    eprintln!(
                        "degenerate {}: sat emitted dist={} off_tri={off}",
                        case.name, s.distance
                    );
                } else {
                    t.both_none += 1;
                    eprintln!("degenerate {}: sat emitted nothing", case.name);
                }
                continue;
            }
            let (gjk, _, crashed) =
                gjk_oracle(&case.box_trans, case.half_full, &case.tri_world, max_dist);
            if crashed {
                t.total += 1;
                t.oracle_crash += 1;
                eprintln!("oracle crash {}", case.name);
                continue;
            }
            let (sat, _) = sat_call(&case.box_trans, case.half_full, &case.tri_world, max_dist);
            compare_pair(case.name, &gjk, &sat, &case.tri_world, &mut t, true);
        }
        eprintln!("deterministic oracle tallies: {t:?}");
        // Regression tripwires: admission, hemisphere, distance, and the
        // point-on-triangle contract must hold exactly on every
        // deterministic case. Witness POSITION within tie flats is allowed
        // to differ (tangential-only, measured but not gated).
        assert_eq!(t.admit_mismatch, 0, "deterministic admission mismatch");
        assert_eq!(t.hemisphere, 0, "deterministic hemisphere flip");
        assert_eq!(t.distance, 0, "deterministic distance drift");
        assert_eq!(t.point_normal_gap, 0, "deterministic normal witness gap");
        assert_eq!(t.point_off_tri, 0, "deterministic witness off triangle");
    }

    // Simple deterministic RNG (xorshift64*) so the randomized oracle run is
    // reproducible without extra dependencies.
    struct Rng(u64);

    impl Rng {
        fn next_u64(&mut self) -> u64 {
            let mut x = self.0;
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            self.0 = x;
            x.wrapping_mul(0x2545F4914F6CDD1D)
        }

        fn next_f32(&mut self) -> f32 {
            ((self.next_u64() >> 32) as f32) / (u32::MAX as f32)
        }

        fn range(&mut self, lo: f32, hi: f32) -> f32 {
            lo + (hi - lo) * self.next_f32()
        }
    }

    fn random_unit(rng: &mut Rng) -> Vec3A {
        let z = rng.range(-1.0, 1.0);
        let a = rng.range(0.0, std::f32::consts::TAU);
        let r = (1.0 - z * z).max(0.0).sqrt();
        Vec3A::new(r * a.cos(), r * a.sin(), z).normalize_or_zero()
    }

    fn random_quat_trans(rng: &mut Rng, pos_range: f32) -> Affine3A {
        let axis = random_unit(rng);
        let angle = rng.range(0.0, std::f32::consts::TAU);
        let rot = Mat3A::from_quat(Quat::from_axis_angle(Vec3::from(axis), angle));
        let pos = Vec3A::new(
            rng.range(-pos_range, pos_range),
            rng.range(-pos_range, pos_range),
            rng.range(-pos_range, pos_range),
        );
        Affine3A {
            matrix3: rot,
            translation: pos,
        }
    }

    #[test]
    fn sat_oracle_randomized_report() {
        let half_choices = [
            Vec3A::new(0.6, 0.43, 0.19),
            Vec3A::new(1.0, 2.0, 0.5),
            Vec3A::new(0.5, 0.5, 0.5),
        ];
        let mut rng = Rng(0x1234_5678_9ABC_DEF1);
        let mut t = Tallies::default();
        let n = 4000;
        for i in 0..n {
            let half_full = half_choices[i % half_choices.len()];
            let box_trans = random_quat_trans(&mut rng, 3.0);
            // Triangle near the box: centroid within ~2 units of box center.
            let center = box_trans.translation + random_unit(&mut rng) * rng.range(0.0, 2.0);
            let u = random_unit(&mut rng);
            let v = random_unit(&mut rng);
            let s0 = rng.range(0.2, 1.5);
            let s1 = rng.range(0.2, 1.5);
            let mut tri = [
                center + u * s0,
                center + v * s1,
                center - u * s0 * 0.5 + v * s1 * 0.3,
            ];
            // Sprinkle degenerate/tie cases deterministically.
            if i % 97 == 0 {
                tri[2] = tri[0] + (tri[1] - tri[0]) * 2.0;
            } else if i % 89 == 0 {
                tri[1] = tri[0];
                tri[2] = tri[0];
            }
            if TriangleShape::new(tri).normal_length < 1e-9 {
                // Graceful-output check only; the EPA oracle can abort on
                // zero-area input.
                let probe = BoxShape::new(half_full);
                let max_dist = probe.get_margin() + 0.05;
                let (sat, _) = sat_call(&box_trans, half_full, &tri, max_dist);
                t.total += 1;
                t.degenerate += 1;
                if let Some(s) = sat {
                    if point_triangle_distance(s.point_on_b_world, &tri) > 1e-3 {
                        t.point_off_tri += 1;
                    }
                } else {
                    t.both_none += 1;
                }
                continue;
            }
            let probe = BoxShape::new(half_full);
            let max_dist = probe.get_margin() + 0.05;
            let (gjk, _, crashed) = gjk_oracle(&box_trans, half_full, &tri, max_dist);
            if crashed {
                t.total += 1;
                t.oracle_crash += 1;
                continue;
            }
            let (sat, _) = sat_call(&box_trans, half_full, &tri, max_dist);
            compare_pair("rand", &gjk, &sat, &tri, &mut t, false);
        }
        let total = t.total.max(1) as f32;
        eprintln!(
            "randomized oracle tallies (n={}): {t:?} rates admit={:.4} hemi={:.4} dist={:.4} ptgap={:.4} ptnormal={:.4} pttangent_only={:.4} ptoff={:.4} degen_frac={:.4} crash_frac={:.4}",
            t.total,
            t.admit_mismatch as f32 / total,
            t.hemisphere as f32 / total,
            t.distance as f32 / total,
            t.point_gap as f32 / total,
            t.point_normal_gap as f32 / total,
            t.point_tangent_only as f32 / total,
            t.point_off_tri as f32 / total,
            t.degenerate as f32 / total,
            t.oracle_crash as f32 / total,
        );
        // Loose regression tripwires at several times the observed rate
        // (admit 0.0018, hemi 0.0050, dist 0.0005, off-tri 0.0). These are
        // smoke tripwires, not exact parity: the oracle resolves tie-flat
        // witnesses differently by construction, so only gross drift fails.
        // The witness is built to stay on the triangle; degenerate inputs
        // stay graceful.
        assert!(
            t.admit_mismatch as f32 / total < 0.01,
            "randomized admission mismatch rate regressed"
        );
        assert!(
            t.hemisphere as f32 / total < 0.02,
            "randomized hemisphere flip rate regressed"
        );
        assert!(
            t.distance as f32 / total < 0.01,
            "randomized distance drift rate regressed"
        );
        assert_eq!(t.point_off_tri, 0, "randomized witness off triangle");
    }

    #[test]
    fn sat_turtle_tri116_contract() {
        // Production turtle contact (box edge near-parallel to a wall-triangle
        // face): the witness is free to sit anywhere on the iso-depth plane,
        // so assert the output contract, not exact GJK parity. The GJK oracle
        // witness sits ~0.28 away on the same plane with the same normal and
        // depth (a Voronoi tie artifact the kernel centers instead); both
        // triples are valid, and only the contract is pinned.
        let half_full = Vec3A::new(1.20507, 0.866994, 0.386591);
        let probe = BoxShape::new(half_full);
        let max_dist = probe.get_margin() + 0.05;
        let box_trans = Affine3A {
            matrix3: Mat3A::from_cols(
                Vec3A::new(-0.59333324, -0.01082119, 0.8048841),
                Vec3A::new(-0.0034576724, 0.9999347, 0.010894649),
                Vec3A::new(-0.8049494, 0.0036811323, -0.5933318),
            ),
            translation: Vec3A::new(-80.52899, 7.1077247, 2.4730747),
        };
        let tri = [
            Vec3A::new(-81.81588, 10.240001, 4.0888753),
            Vec3A::new(-81.51728, 5.096394e-7, 3.1281445),
            Vec3A::new(-81.51729, 10.24, 3.1281428),
        ];
        let (gjk, _, crashed) = gjk_oracle(&box_trans, half_full, &tri, max_dist);
        assert!(!crashed);
        let (gn, _, gd) = gjk.expect("turtle pair must emit a GJK contact");
        let (sat, _) = sat_call(&box_trans, half_full, &tri, max_dist);
        let s = sat.expect("turtle pair must emit a SAT contact");
        assert!(
            gn.dot(s.normal_on_b_world) > 0.0,
            "hemisphere flip: {gn:?} vs {:?}",
            s.normal_on_b_world
        );
        assert!(
            (gd - s.distance).abs() < 5e-3,
            "distance drift: {gd} vs {}",
            s.distance
        );
        assert!(
            (s.normal_on_b_world.length() - 1.0).abs() < 1e-5,
            "normal not unit: {:?}",
            s.normal_on_b_world
        );
        assert!(
            point_triangle_distance(s.point_on_b_world, &tri) < 1e-3,
            "witness off triangle: {:?}",
            s.point_on_b_world
        );
    }

    #[test]
    fn sat_kernel_unit_vectors() {
        // Box-local helpers stay consistent with world transforms.
        let bt = Affine3A::from_translation(Vec3A::new(1.0, 2.0, 3.0).into());
        let p = Vec3A::new(2.0, 3.0, 4.0);
        let l = box_point_to_local(&bt, p);
        assert!((l - Vec3A::ONE).length() < 1e-5);
        assert!((box_point_to_world(&bt, l) - p).length() < 1e-5);

        // Segment-segment parallel overlap tie returns the overlap midpoint.
        let (c1, c2) = segment_segment_closest(Vec3A::ZERO, Vec3A::X, Vec3A::ZERO, Vec3A::X);
        assert!((c1 - Vec3A::X * 0.5).length() < 1e-6);
        assert!((c2 - Vec3A::X * 0.5).length() < 1e-6);

        // Degenerate triangle fallback stays on the segment.
        let dq = [Vec3A::ZERO, Vec3A::X, Vec3A::X * 2.0];
        let c = closest_point_on_triangle(Vec3A::new(0.5, 1.0, 0.0), &dq);
        assert!((c - Vec3A::new(0.5, 0.0, 0.0)).length() < 1e-5);
    }

    #[test]
    fn sat_degenerate_box_graceful() {
        // Zero-size and flat boxes must not panic or divide by zero. The
        // kernel contract permits any half extents, so an emitted contact
        // still honors the output contract (unit normal, finite distance,
        // witness on the triangle).
        let tri = [
            Vec3A::new(-1.0, -1.0, 0.05),
            Vec3A::new(1.0, -1.0, 0.05),
            Vec3A::new(0.0, 1.0, 0.05),
        ];
        for half_unmargined in [Vec3A::ZERO, Vec3A::new(1.0, 0.0, 0.5)] {
            // Identity box: box-local triangle equals the input triangle.
            let contact = box_triangle_sat(&Affine3A::IDENTITY, half_unmargined, 0.04, &tri, 0.09);
            if let Some(c) = contact {
                assert!(
                    (c.normal_on_b_world.length() - 1.0).abs() < 1e-5,
                    "normal not unit: {:?}",
                    c.normal_on_b_world
                );
                assert!(
                    c.distance.is_finite(),
                    "distance not finite: {}",
                    c.distance
                );
                assert!(
                    point_triangle_distance(c.point_on_b_world, &tri) < 1e-3,
                    "witness off triangle: {:?}",
                    c.point_on_b_world
                );
            }
        }
    }
}
