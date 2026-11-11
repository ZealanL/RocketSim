//! Steered axle direction for a front wheel.

use glam::Vec3A;

#[cfg(feature = "native-math")]
use crate::shared::math::{cosf, sinf};

/// Right column of a steered wheel, in world space.
///
/// `car_up` is the rotation axis. `car_right` is the chassis right axis in
/// world space. `steer_angle` is the steer angle in radians.
///
/// The helper builds a quaternion for `steer_angle` about `car_up`, converts
/// it to a rotation matrix, and multiplies the matrix by `car_right`. The
/// helper does not use a reciprocal square root. It uses the scalar scale
/// `2 / |q|^2` instead.
///
/// `car_up` must be finite and nonzero. Normalize its contribution when
/// constructing the quaternion. The caller supplies the chassis basis.
#[cfg(feature = "native-math")]
pub(super) fn steered_right_column(car_up: Vec3A, car_right: Vec3A, steer_angle: f32) -> Vec3A {
    let up = car_up;
    let half_angle = steer_angle * 0.5;
    // Use the half angle directly when it is zero and the axis is non-zero.
    // The division then has a zero numerator and no zero divisor.
    let sin_scale = if half_angle == 0.0 && up.length_squared() > 0.0 {
        half_angle
    } else {
        sinf(half_angle) / up.length()
    };
    let cos_half = if half_angle == 0.0 {
        1.0
    } else {
        cosf(half_angle)
    };

    let qx = up.x * sin_scale;
    let qy = up.y * sin_scale;
    let qz = up.z * sin_scale;
    let qw = cos_half;

    let s = 2.0 / (((qx * qx + qy * qy) + qz * qz) + qw * qw);
    let xs = qx * s;
    let ys = qy * s;
    let zs = qz * s;
    let wx = qw * xs;
    let wy = qw * ys;
    let wz = qw * zs;
    let xx = qx * xs;
    let xy = qx * ys;
    let xz = qx * zs;
    let yy = qy * ys;
    let yz = qy * zs;
    let zz = qz * zs;

    let col0 = Vec3A::new(1.0 - (yy + zz), xy + wz, xz - wy);
    let col1 = Vec3A::new(xy - wz, 1.0 - (xx + zz), yz + wx);
    let col2 = Vec3A::new(xz + wy, yz - wx, 1.0 - (xx + yy));

    // The reference multiplies the matrix by `-wheel_axle_ws`, and
    // `wheel_axle_ws` is `-car_right`, so the operand is `car_right`.
    let nr = car_right;
    Vec3A::new(
        col0.x * nr.x + col1.x * nr.y + col2.x * nr.z,
        col0.y * nr.x + col1.y * nr.y + col2.y * nr.z,
        col0.z * nr.x + col1.z * nr.y + col2.z * nr.z,
    )
}

/// Rotate the chassis right axis around its up axis with glam.
///
/// `car_up` must be finite and nonzero. `steer_angle` is in radians.
#[cfg(not(feature = "native-math"))]
#[inline]
pub(super) fn steered_right_column(car_up: Vec3A, car_right: Vec3A, steer_angle: f32) -> Vec3A {
    glam::Quat::from_axis_angle(car_up.normalize().into(), steer_angle) * car_right
}
