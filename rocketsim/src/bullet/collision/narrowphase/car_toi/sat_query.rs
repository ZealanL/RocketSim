//! Analytic SAT query for the car-to-car time-of-impact sweep.
//!
//! The query uses the box cores, which are the `BoxShape` implicit half
//! extents, and applies the sum of the shape margins. It returns the same
//! signed distance convention as the GJK path:
//!
//! - Separated cores: the exact Euclidean core distance minus the margin sum.
//! - Overlapping cores: the negative sum of the minimum translation depth and
//!   the margin sum.
//!
//! A separating-axis gap alone underestimates the Euclidean distance between
//! a corner and an edge. For separated cores, this query therefore uses the
//! exact nearest vertex, edge, and face feature pair.
//!
//! The overlap normal is the winning SAT axis. At exact core contact, this can
//! differ from the nearest-feature normal. With the default car margins, the
//! margin sum is larger than the sweep overlap budget, so the rounded distance
//! usually stops the loop. A custom margin sum below the budget can let the
//! loop advance past the contact. Do not treat the two normals as continuous.
//!
//! The box rotations must be orthonormal. The shared sweep builds the
//! transforms from normalized quaternions, so this holds. All arithmetic is
//! `f32`. The result is exact only up to rounding and the fixed tie order. It
//! is not a general exact predicate.

mod closest_features;

use glam::{Affine3A, Mat3A, Vec3A};

use super::{CarToiBoxShape, CarToiContact, MAXIMUM_DISTANCE_SQUARED};

/// Smallest squared length of a rotation column or an edge cross axis.
///
/// A cross axis below this limit means that the related edges are parallel.
/// The direction is then not stable, and the box face axes already cover it.
const MIN_AXIS_LEN_SQ: f32 = 1.0e-12;

/// Smallest squared core distance that gives a stable normal.
///
/// Below this limit the witness pair is too close to define a direction with
/// `f32` precision. The query then uses the winning SAT axis.
const NORMAL_MIN_CORE_DISTANCE_SQ: f32 = 1.0e-12;

/// One box core in world space.
struct CoreBox {
    center: Vec3A,
    rotation: Mat3A,
    /// Implicit half extents without the margin.
    half_extents: Vec3A,
}

impl CoreBox {
    /// Read the box core from one sweep shape.
    fn new(shape: &CarToiBoxShape, trans: &Affine3A) -> Self {
        let compound = shape.compound();
        let local = compound.child_trans;

        Self {
            center: trans.transform_point3a(local.translation),
            rotation: trans.matrix3 * local.matrix3,
            half_extents: compound.child_shape.get_half_extents(),
        }
    }
}

/// Result of the fixed-order 15-axis sweep.
struct AxisSweep {
    /// Largest core separation. A positive value means separated cores. When
    /// all cores overlap, this is the negative minimum overlap.
    max_gap: f32,
    /// Unit axis of `max_gap`, in box A local space.
    max_gap_axis: Vec3A,
}

/// Return the closest points of two boxes through the SAT query.
///
/// Return `None` when the normal is not valid or the distance exceeds the bound.
#[must_use]
pub(crate) fn closest_points(
    shape_a: &CarToiBoxShape,
    trans_a: &Affine3A,
    shape_b: &CarToiBoxShape,
    trans_b: &Affine3A,
) -> Option<CarToiContact> {
    let core_a = CoreBox::new(shape_a, trans_a);
    let core_b = CoreBox::new(shape_b, trans_b);

    // The GJK path centres both transforms on their midpoint. Keep this offset
    // even though the geometry is offset invariant: `f32` rounding can change
    // the result bits, and the GJK comparison needs the same frame.
    let offset = (trans_a.translation + trans_b.translation) * 0.5;
    let center_a = core_a.center - offset;
    let center_b = core_b.center - offset;

    let rotation = core_a.rotation.transpose() * core_b.rotation;
    let center_rel = core_a.rotation.transpose() * (center_b - center_a);

    let sweep = sat_sweep(
        core_a.half_extents,
        core_b.half_extents,
        rotation,
        center_rel,
    );
    let margin_sum = shape_a.margin() + shape_b.margin();

    let (distance, normal_local) = if sweep.max_gap > 0.0 {
        let nearest = closest_features::nearest(
            core_a.half_extents,
            core_b.half_extents,
            rotation,
            center_rel,
        );

        // Point from box B toward box A. A flat contact gives one normal for
        // every minimum pair, so the first minimum is stable.
        let witness_delta = nearest.point_a - nearest.point_b;
        let witness_len_sq = witness_delta.length_squared();
        let normal = if witness_len_sq > NORMAL_MIN_CORE_DISTANCE_SQ {
            witness_delta * (1.0 / witness_len_sq.sqrt())
        } else {
            normal_toward_a(sweep.max_gap_axis, center_rel)
        };

        (nearest.distance_sq.sqrt() - margin_sum, normal)
    } else {
        // `max_gap` is the negative minimum overlap. Keep both negations to
        // preserve the distance sign at zero.
        (
            -(-sweep.max_gap + margin_sum),
            normal_toward_a(sweep.max_gap_axis, center_rel),
        )
    };

    let normal_world = core_a.rotation * normal_local;
    let normal_len_sq = normal_world.length_squared();
    if normal_len_sq <= f32::MIN_POSITIVE {
        return None;
    }

    let normal = normal_world * (1.0 / normal_len_sq.sqrt());

    if distance < 0.0 || distance * distance < MAXIMUM_DISTANCE_SQUARED {
        Some(CarToiContact { distance, normal })
    } else {
        None
    }
}

/// Test the box faces and edge cross axes in a fixed order.
///
/// Box A is axis aligned in its own frame. `rotation` holds the box B axes in
/// box A space. `center` holds the box B center in box A space.
fn sat_sweep(half_a: Vec3A, half_b: Vec3A, rotation: Mat3A, center: Vec3A) -> AxisSweep {
    let mut sweep = AxisSweep {
        max_gap: f32::NEG_INFINITY,
        max_gap_axis: Vec3A::X,
    };

    let a_axes = [Vec3A::X, Vec3A::Y, Vec3A::Z];
    let b_axes = [rotation.x_axis, rotation.y_axis, rotation.z_axis];

    // Box A face axes.
    let a_half = [half_a.x, half_a.y, half_a.z];
    let a_center = [center.x, center.y, center.z];
    for ((axis, half), projection) in a_axes.into_iter().zip(a_half).zip(a_center) {
        consider_axis(
            &mut sweep,
            axis,
            projection,
            half + projected_radius_b(half_b, b_axes, axis),
        );
    }

    // Box B face axes.
    for raw_axis in b_axes {
        let Some(axis) = unit_axis(raw_axis) else {
            continue;
        };

        consider_axis(
            &mut sweep,
            axis,
            center.dot(axis),
            projected_radius_a(half_a, axis) + projected_radius_b(half_b, b_axes, axis),
        );
    }

    // Edge cross axes.
    for axis_a in a_axes {
        for axis_b in b_axes {
            let Some(axis) = unit_axis(axis_a.cross(axis_b)) else {
                continue;
            };

            consider_axis(
                &mut sweep,
                axis,
                center.dot(axis),
                projected_radius_a(half_a, axis) + projected_radius_b(half_b, b_axes, axis),
            );
        }
    }

    sweep
}

/// Record one unit axis in the sweep.
fn consider_axis(sweep: &mut AxisSweep, axis: Vec3A, projection: f32, radius: f32) {
    let gap = projection.abs() - radius;

    // A strict comparison keeps the first axis on exact ties. When all cores
    // overlap, the same axis holds the minimum overlap.
    if gap > sweep.max_gap {
        sweep.max_gap = gap;
        sweep.max_gap_axis = axis;
    }
}

/// Return the unit form of `v`, or `None` when its length is too small.
fn unit_axis(v: Vec3A) -> Option<Vec3A> {
    let len_sq = v.length_squared();
    if len_sq < MIN_AXIS_LEN_SQ {
        return None;
    }

    Some(v * (1.0 / len_sq.sqrt()))
}

/// Return the box A radius along an axis. Box A is axis aligned.
fn projected_radius_a(half: Vec3A, axis: Vec3A) -> f32 {
    half.x * axis.x.abs() + half.y * axis.y.abs() + half.z * axis.z.abs()
}

/// Return the box B radius along an axis in box A space.
fn projected_radius_b(half: Vec3A, b_axes: [Vec3A; 3], axis: Vec3A) -> f32 {
    let x = half.x * b_axes[0].dot(axis).abs();
    let y = half.y * b_axes[1].dot(axis).abs();
    let z = half.z * b_axes[2].dot(axis).abs();

    (x + y) + z
}

/// Point `axis` from box B toward box A.
///
/// `center_rel` is the box B center relative to the box A center. An exact
/// center tie keeps the input direction, so repeated queries are deterministic.
fn normal_toward_a(axis: Vec3A, center_rel: Vec3A) -> Vec3A {
    if axis.dot(center_rel) > 0.0 {
        -axis
    } else {
        axis
    }
}
