//! Fast orientation math backend.
//!
//! This backend is the default. It uses `glam` conversions and the platform
//! `f32` sin and cos functions. It keeps the integration angle bound, the
//! small-angle branch, and the input and output behavior of the native backend.
//! Its output bits can differ from the native backend.

use std::f32::consts::FRAC_PI_4;

use glam::{Mat3A, Quat, Vec3A};

/// Maximum angular displacement in radians for one integration step.
const ANGULAR_MOTION_THRESHOLD: f32 = FRAC_PI_4;

/// Convert imported basis columns to a quaternion.
///
/// The columns should form a rotation matrix. Use the direct glam conversion
/// when the columns are normalized. Otherwise, normalize a local copy and
/// replace unusable columns with world axes. Normalize a non-unit result.
/// This function does not change the caller's matrix.
#[must_use]
pub(crate) fn bullet_quat_from_basis_columns(fwd: Vec3A, right: Vec3A, up: Vec3A) -> Quat {
    quat_from_basis(fwd, right, up)
}

/// Convert current body basis columns to a quaternion.
///
/// Use the same conversion and fallback rules as
/// [`bullet_quat_from_basis_columns`]. This function does not change the
/// caller's matrix.
#[must_use]
pub(crate) fn game_get_rotation_scalar(fwd: Vec3A, right: Vec3A, up: Vec3A) -> Quat {
    quat_from_basis(fwd, right, up)
}

/// Convert basis columns to a quaternion.
///
/// Use the direct conversion when each column is normalized. This path keeps
/// the output bits of a proper rotation matrix. Otherwise, use
/// [`fallback_quat_from_basis`].
fn quat_from_basis(fwd: Vec3A, right: Vec3A, up: Vec3A) -> Quat {
    // The `from_mat3a` call asserts that each column is normalized in a debug
    // build. The guard uses the same tolerance as that assertion.
    if fwd.is_normalized() && right.is_normalized() && up.is_normalized() {
        let quat = Quat::from_mat3a(&Mat3A::from_cols(fwd, right, up));
        if quat.is_normalized() {
            return quat;
        }

        // Unit columns can still form a shear or a degenerate basis.
        return game_quat_normalize_scalar(quat, 0.0);
    }
    fallback_quat_from_basis(fwd, right, up)
}

/// Convert basis columns that a caller supplied without validation.
///
/// This helper runs only when a column fails the normalized check.
/// Normalize a local copy of each column with [`Vec3A::normalize_or_zero`].
/// Replace a zero, NaN, or infinite column with the matching world axis.
/// Normalize the result, because a shear can make the conversion return a
/// non-unit quaternion. The caller keeps its original matrix.
fn fallback_quat_from_basis(fwd: Vec3A, right: Vec3A, up: Vec3A) -> Quat {
    let axes = [Vec3A::X, Vec3A::Y, Vec3A::Z];
    let mut columns = [fwd, right, up];
    for (column, axis) in columns.iter_mut().zip(axes) {
        *column = if column.is_finite() {
            let unit = column.normalize_or_zero();
            if unit == Vec3A::ZERO { axis } else { unit }
        } else {
            axis
        };
    }

    let quat = Quat::from_mat3a(&Mat3A::from_cols(columns[0], columns[1], columns[2]));
    if !quat.is_finite() {
        // A degenerate column set can make the conversion non-finite.
        return Quat::IDENTITY;
    }

    // A shear can make the quaternion non-unit. Normalize it, so the matrix
    // conversion of this quaternion cannot assert in a debug build.
    game_quat_normalize_scalar(quat, -1.0)
}

/// Convert a unit quaternion to basis columns.
///
/// `q` must be a unit quaternion. This function does not scale or normalize
/// `q`. Normalize `q` first with [`game_quat_normalize_scalar`] when its length
/// can differ from one.
#[must_use]
pub(crate) fn game_set_rotation_scalar_columns(q: Quat) -> Mat3A {
    Mat3A::from_quat(q)
}

/// Normalize a quaternion with glam.
///
/// Return the identity when the squared length is zero, is NaN, or is not
/// greater than `min_len_sq`. The zero rule also applies to a negative
/// `min_len_sq`. Without it, the `glam` normalization asserts a nonzero length
/// in a debug build.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
#[must_use]
pub(crate) fn game_quat_normalize_scalar(q: Quat, min_len_sq: f32) -> Quat {
    let len_sq = q.length_squared();
    if len_sq == 0.0 || !(len_sq > min_len_sq) {
        return Quat::IDENTITY;
    }

    q.normalize()
}

/// Integrate the orientation of basis columns by one time step.
///
/// `ang_vel_world` is in radians per second. `dt` is in seconds.
/// Use the current columns as the source orientation when `basis_rot` is
/// `None`. Write both the new columns and the matching quaternion.
/// Bound the angle argument of the sin and cos functions to
/// [`ANGULAR_MOTION_THRESHOLD`]. The rotation vector keeps the full
/// `ang_vel_world` components, so a step above the bound does not produce a
/// rotation of only the bound angle.
pub(crate) fn game_integrate_orientation_basis_columns(
    fwd: &mut Vec3A,
    right: &mut Vec3A,
    up: &mut Vec3A,
    q_mirror: &mut Quat,
    ang_vel_world: Vec3A,
    dt: f32,
    basis_rot: Option<Quat>,
) {
    let mut angle = ang_vel_world.length_squared().sqrt();
    if angle * dt > ANGULAR_MOTION_THRESHOLD {
        angle = ANGULAR_MOTION_THRESHOLD / dt;
    }

    let (sin_half, cos_half) = ((angle * dt) * 0.5).sin_cos();
    let f = if angle < 0.001 {
        dt * 0.5 - (((dt * dt) * dt) * 0.020_833_334 * angle) * angle
    } else {
        sin_half / angle
    };

    let rotation = Quat::from_xyzw(
        ang_vel_world.x * f,
        ang_vel_world.y * f,
        ang_vel_world.z * f,
        cos_half,
    );

    let o = basis_rot.unwrap_or_else(|| game_get_rotation_scalar(*fwd, *right, *up));
    *q_mirror = game_quat_normalize_scalar(rotation * o, -1.0);

    let columns = game_set_rotation_scalar_columns(*q_mirror);
    *fwd = columns.x_axis;
    *right = columns.y_axis;
    *up = columns.z_axis;
}
