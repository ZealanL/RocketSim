//! Fast quaternion math.
//!
//! This backend is selected without `native-math`. It uses `glam` quaternion
//! interpolation and `glam` axis-angle conversion. Its output bits can differ
//! from the native backend.

use glam::{Mat3A, Quat, Vec3A};

use crate::bullet::linear_math::{game_get_rotation_scalar, game_quat_normalize_scalar};

/// Spherical linear interpolation between two quaternions.
///
/// Use the `glam` short-path interpolation.
#[must_use]
pub(crate) fn slerp(q1: Quat, q2: Quat, t: f32) -> Quat {
    q1.slerp(q2, t)
}

/// Return the constant angular velocity that turns one basis into another.
///
/// Invert the first basis with `glam`, convert the product to a quaternion,
/// and use the `glam` axis-angle conversion.
#[must_use]
pub(crate) fn angular_velocity(
    fwd0: Vec3A,
    right0: Vec3A,
    up0: Vec3A,
    fwd1: Vec3A,
    right1: Vec3A,
    up1: Vec3A,
) -> Vec3A {
    let basis0 = Mat3A::from_cols(fwd0, right0, up0);
    let basis1 = Mat3A::from_cols(fwd1, right1, up1);
    let relative = basis0.inverse() * basis1;

    let q = game_quat_normalize_scalar(
        game_get_rotation_scalar(relative.x_axis, relative.y_axis, relative.z_axis),
        -1.0,
    );
    Vec3A::from(q.to_scaled_axis())
}
