//! Exact nearest feature pair for separated box cores.
//!
//! Work in box A local space. Box A is axis aligned at the origin. Box B has
//! `rotation` (the box B axes in box A space) and `center` (the box B center in
//! box A space). `rotation` must be orthonormal. The routine uses its transpose
//! as the inverse rotation.
//!
//! The routine tests these candidates in this order:
//!
//! 1. Each box A vertex against its clamp point on box B.
//! 2. Each box B vertex against its clamp point on box A.
//! 3. The 12 by 12 box edge pairs.
//!
//! These candidates cover every vertex, edge, and face feature pair. A strict
//! `<` comparison keeps the first candidate on a tie. Repeated queries
//! therefore return the same witnesses.

use glam::{Mat3A, Vec3A};

/// One nearest witness pair in box A local space.
pub(super) struct Nearest {
    pub(super) distance_sq: f32,
    pub(super) point_a: Vec3A,
    pub(super) point_b: Vec3A,
}

/// Start value for the strict minimum search.
const EMPTY: Nearest = Nearest {
    distance_sq: f32::MAX,
    point_a: Vec3A::ZERO,
    point_b: Vec3A::ZERO,
};

/// Return the nearest witness pair of two separated box cores.
pub(super) fn nearest(half_a: Vec3A, half_b: Vec3A, rotation: Mat3A, center: Vec3A) -> Nearest {
    let mut best = EMPTY;
    let inverse_rotation = rotation.transpose();

    // Box A vertices against box B.
    for bits in 0..8 {
        let point_a = box_vertex(half_a, bits);
        let local_b = inverse_rotation * (point_a - center);
        let point_b = rotation * clamp_to_box(local_b, half_b) + center;
        consider(&mut best, point_a, point_b);
    }

    // Box B vertices against box A.
    for bits in 0..8 {
        let point_b = rotation * box_vertex(half_b, bits) + center;
        let point_a = clamp_to_box(point_b, half_a);
        consider(&mut best, point_a, point_b);
    }

    // Box edge pairs. The edge order is fixed: axis X, Y, Z, then the four
    // sign pairs of the other two axes.
    let edges_a = box_edges(half_a, Mat3A::IDENTITY, Vec3A::ZERO);
    let edges_b = box_edges(half_b, rotation, center);
    for &[a0, a1] in &edges_a {
        for &[b0, b1] in &edges_b {
            let (point_a, point_b) = segment_closest(a0, a1, b0, b1);
            consider(&mut best, point_a, point_b);
        }
    }

    best
}

/// Record a candidate pair when it is strictly nearer than the current best.
fn consider(best: &mut Nearest, point_a: Vec3A, point_b: Vec3A) {
    let distance_sq = (point_a - point_b).length_squared();
    if distance_sq < best.distance_sq {
        *best = Nearest {
            distance_sq,
            point_a,
            point_b,
        };
    }
}

/// Return one corner of an axis aligned box. Bit 0, 1, and 2 select a positive
/// sign on the X, Y, and Z axis.
fn box_vertex(half: Vec3A, bits: usize) -> Vec3A {
    Vec3A::new(
        if (bits & 1) == 0 { -half.x } else { half.x },
        if (bits & 2) == 0 { -half.y } else { half.y },
        if (bits & 4) == 0 { -half.z } else { half.z },
    )
}

/// Clamp a point to an axis aligned box around the origin.
fn clamp_to_box(point: Vec3A, half: Vec3A) -> Vec3A {
    Vec3A::new(
        point.x.clamp(-half.x, half.x),
        point.y.clamp(-half.y, half.y),
        point.z.clamp(-half.z, half.z),
    )
}

/// Return the 12 edges of a box in a fixed order.
fn box_edges(half: Vec3A, rotation: Mat3A, center: Vec3A) -> [[Vec3A; 2]; 12] {
    let half_components = [half.x, half.y, half.z];
    let mut edges = [[Vec3A::ZERO; 2]; 12];
    let mut index = 0;

    for (axis, &half_axis) in half_components.iter().enumerate() {
        let next_axis = (axis + 1) % 3;
        let other_axis = (axis + 2) % 3;

        for bits in 0..4 {
            let mut start = Vec3A::ZERO;
            start[next_axis] = if (bits & 1) == 0 {
                -half_components[next_axis]
            } else {
                half_components[next_axis]
            };
            start[other_axis] = if (bits & 2) == 0 {
                -half_components[other_axis]
            } else {
                half_components[other_axis]
            };

            let mut end = start;
            start[axis] = -half_axis;
            end[axis] = half_axis;
            edges[index] = [center + rotation * start, center + rotation * end];
            index += 1;
        }
    }

    edges
}

/// Return the closest point pair of two segments.
///
/// This is the standard clamped least-squares solution. The first point is on
/// the segment `(p1, q1)` and the second point is on `(p2, q2)`.
fn segment_closest(p1: Vec3A, q1: Vec3A, p2: Vec3A, q2: Vec3A) -> (Vec3A, Vec3A) {
    /// Squared length below which a segment is a point.
    const MIN_SEGMENT_LEN_SQ: f32 = 1.0e-30;

    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let offset = p1 - p2;

    let len1_sq = d1.length_squared();
    let len2_sq = d2.length_squared();
    let len2_offset = d2.dot(offset);

    let s;
    let t;

    if len1_sq <= MIN_SEGMENT_LEN_SQ && len2_sq <= MIN_SEGMENT_LEN_SQ {
        return (p1, p2);
    }

    if len1_sq <= MIN_SEGMENT_LEN_SQ {
        s = 0.0;
        t = (len2_offset / len2_sq).clamp(0.0, 1.0);
    } else {
        let len1_offset = d1.dot(offset);

        if len2_sq <= MIN_SEGMENT_LEN_SQ {
            t = 0.0;
            s = (-len1_offset / len1_sq).clamp(0.0, 1.0);
        } else {
            let cross = d1.dot(d2);
            let denom = len1_sq * len2_sq - cross * cross;

            let mut raw_s = if denom > MIN_SEGMENT_LEN_SQ {
                ((cross * len2_offset - len1_offset * len2_sq) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };

            let mut raw_t = (cross * raw_s + len2_offset) / len2_sq;
            if raw_t < 0.0 {
                raw_t = 0.0;
                raw_s = (-len1_offset / len1_sq).clamp(0.0, 1.0);
            } else if raw_t > 1.0 {
                raw_t = 1.0;
                raw_s = ((cross - len1_offset) / len1_sq).clamp(0.0, 1.0);
            }

            s = raw_s;
            t = raw_t;
        }
    }

    (p1 + d1 * s, p2 + d2 * t)
}
