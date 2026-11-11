//! Native orientation math backend.
//!
//! Enable the `native-math` feature to use this backend. It keeps the
//! engine-specific scalar math and its exact output bits.
//! Do not reorder its operations or change its constants.

use std::f32::consts::FRAC_PI_4;

use glam::{Mat3A, Quat, Vec3A};

use crate::shared::math::{cosf, sinf};

/// Maximum angular displacement in radians for one integration step.
const ANGULAR_MOTION_THRESHOLD: f32 = FRAC_PI_4;

/// Convert imported basis columns to a quaternion with the Bullet conversion.
///
/// The columns must form a rotation matrix. This conversion keeps the scalar
/// branch order used when a rigid body imports a transform.
#[must_use]
pub(crate) fn bullet_quat_from_basis_columns(fwd: Vec3A, right: Vec3A, up: Vec3A) -> Quat {
    let m00 = fwd.x;
    let m01 = right.x;
    let m02 = up.x;
    let m10 = fwd.y;
    let m11 = right.y;
    let m12 = up.y;
    let m20 = fwd.z;
    let m21 = right.z;
    let m22 = up.z;

    let trace = m00 + m11 + m22;
    let (raw0, raw1, raw2, raw3, x);
    if trace > 0.0 {
        x = trace + 1.0;
        raw0 = m21 - m12;
        raw1 = m02 - m20;
        raw2 = m10 - m01;
        raw3 = x;
    } else if m00 < m11 {
        if m11 < m22 {
            x = m22 - m00 - m11 + 1.0;
            raw0 = m02 + m20;
            raw1 = m12 + m21;
            raw2 = x;
            raw3 = m10 - m01;
        } else {
            x = m11 - m22 - m00 + 1.0;
            raw0 = m01 + m10;
            raw1 = x;
            raw2 = m21 + m12;
            raw3 = m02 - m20;
        }
    } else if m00 < m22 {
        x = m22 - m00 - m11 + 1.0;
        raw0 = m02 + m20;
        raw1 = m12 + m21;
        raw2 = x;
        raw3 = m10 - m01;
    } else {
        x = m00 - m11 - m22 + 1.0;
        raw0 = x;
        raw1 = m10 + m01;
        raw2 = m20 + m02;
        raw3 = m21 - m12;
    }

    let s = x.sqrt();
    let scale = 0.5 / s;
    Quat::from_xyzw(raw0 * scale, raw1 * scale, raw2 * scale, raw3 * scale)
}

/// Convert current body basis columns to a quaternion.
///
/// Time integration and orientation quantization use this scalar conversion.
/// Its branch and multiplication order differ from the import conversion.
#[must_use]
pub(crate) fn game_get_rotation_scalar(fwd: Vec3A, right: Vec3A, up: Vec3A) -> Quat {
    let m00 = fwd.x;
    let m01 = right.x;
    let m02 = up.x;
    let m10 = fwd.y;
    let m11 = right.y;
    let m12 = up.y;
    let m20 = fwd.z;
    let m21 = right.z;
    let m22 = up.z;

    let trace = (m00 + m11) + m22;
    if trace > 0.0 {
        let mut s = (trace + 1.0).sqrt();
        let w = s * 0.5;
        s = 0.5 / s;
        return Quat::from_xyzw((m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s, w);
    }
    if if m00 < m11 { m11 < m22 } else { m00 < m22 } {
        let mut s = (((m22 - m00) - m11) + 1.0).sqrt();
        let z = s * 0.5;
        s = 0.5 / s;
        return Quat::from_xyzw((m02 + m20) * s, (m12 + m21) * s, z, (m10 - m01) * s);
    }
    if m00 < m11 {
        let mut s = (((m11 - m22) - m00) + 1.0).sqrt();
        let y = s * 0.5;
        s = 0.5 / s;
        return Quat::from_xyzw((m01 + m10) * s, y, (m21 + m12) * s, (m02 - m20) * s);
    }
    let mut s = (((m00 - m11) - m22) + 1.0).sqrt();
    let x = s * 0.5;
    s = 0.5 / s;
    Quat::from_xyzw(x, (m10 + m01) * s, (m20 + m02) * s, (m21 - m12) * s)
}

/// Convert a quaternion to basis columns.
///
/// Scale by the quaternion length squared before constructing the columns.
/// `q` must have a finite, nonzero length.
#[must_use]
pub(crate) fn game_set_rotation_scalar_columns(q: Quat) -> Mat3A {
    let d = ((q.x * q.x + q.y * q.y) + q.z * q.z) + q.w * q.w;
    let s = 2.0 / d;
    let xs = q.x * s;
    let ys = q.y * s;
    let zs = q.z * s;
    let wx = q.w * xs;
    let wy = q.w * ys;
    let wz = q.w * zs;
    let xx = q.x * xs;
    let xy = q.x * ys;
    let xz = q.x * zs;
    let yy = q.y * ys;
    let yz = q.y * zs;
    let zz = q.z * zs;
    Mat3A::from_cols(
        Vec3A::new(1.0 - (yy + zz), xy + wz, xz - wy),
        Vec3A::new(xy - wz, 1.0 - (xx + zz), yz + wx),
        Vec3A::new(xz + wy, yz - wx, 1.0 - (xx + yy)),
    )
}

/// Normalize a quaternion with scalar arithmetic.
///
/// Return the identity when the squared length is NaN or is not greater than
/// `min_len_sq`. Otherwise, multiply each component by the inverse length.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
#[must_use]
pub(crate) fn game_quat_normalize_scalar(q: Quat, min_len_sq: f32) -> Quat {
    let len_sq = ((q.x * q.x + q.y * q.y) + q.z * q.z) + q.w * q.w;
    if !(len_sq > min_len_sq) {
        return Quat::IDENTITY;
    }
    let s = 1.0 / len_sq.sqrt();
    Quat::from_xyzw(q.x * s, q.y * s, q.z * s, q.w * s)
}

/// Integrate the orientation of basis columns by one time step.
///
/// `ang_vel_world` is in radians per second. `dt` is in seconds.
/// Use the current columns as the source orientation when `basis_rot` is
/// `None`. Write both the new columns and the matching quaternion.
/// Construct the columns from the normalized result to keep both values
/// consistent.
pub(crate) fn game_integrate_orientation_basis_columns(
    fwd: &mut Vec3A,
    right: &mut Vec3A,
    up: &mut Vec3A,
    q_mirror: &mut Quat,
    ang_vel_world: Vec3A,
    dt: f32,
    basis_rot: Option<Quat>,
) {
    let angle_sq = (ang_vel_world.x * ang_vel_world.x + ang_vel_world.y * ang_vel_world.y)
        + ang_vel_world.z * ang_vel_world.z;
    let mut angle = angle_sq.sqrt();
    if angle * dt > ANGULAR_MOTION_THRESHOLD {
        angle = ANGULAR_MOTION_THRESHOLD / dt;
    }

    let f = if angle < 0.001 {
        dt * 0.5 - (((dt * dt) * dt) * 0.020_833_334 * angle) * angle
    } else {
        let half_angle = (angle * 0.5) * dt;
        sinf(half_angle) / angle
    };

    let ax = ang_vel_world.x * f;
    let ay = ang_vel_world.y * f;
    let az = ang_vel_world.z * f;
    let c = cosf((angle * dt) * 0.5);

    let o = basis_rot.unwrap_or_else(|| game_get_rotation_scalar(*fwd, *right, *up));
    let p = Quat::from_xyzw(
        ((ax * o.w + c * o.x) + ay * o.z) - az * o.y,
        ((c * o.y + ay * o.w) + az * o.x) - ax * o.z,
        ((c * o.z + az * o.w) + ax * o.y) - ay * o.x,
        ((c * o.w - ax * o.x) - ay * o.y) - az * o.z,
    );
    *q_mirror = game_quat_normalize_scalar(p, -1.0);

    let columns = game_set_rotation_scalar_columns(*q_mirror);
    *fwd = columns.x_axis;
    *right = columns.y_axis;
    *up = columns.z_axis;
}
