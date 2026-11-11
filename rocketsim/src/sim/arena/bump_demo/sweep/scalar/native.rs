//! Native scalar quaternion math.
//!
//! Enable the `native-math` feature to use these routines. They keep the
//! engine operation order and output bits.

use glam::{Quat, Vec3A};

use crate::{
    bullet::linear_math::{game_get_rotation_scalar, game_quat_normalize_scalar},
    shared::math::{acosf, sinf},
};

/// Spherical linear interpolation between two quaternions.
///
/// Use the short path. The result is not normalized.
#[must_use]
pub(crate) fn slerp(q1: Quat, q2: Quat, t: f32) -> Quat {
    let raw = ((q1.x * q2.x + q1.y * q2.y) + q1.z * q2.z) + q1.w * q2.w;
    let cosom = if raw >= 0.0 { raw } else { -raw };

    let (scale0, scale1);
    if cosom < 0.9999 {
        let c = if -1.0 > cosom {
            -1.0
        } else if cosom < 1.0 {
            cosom
        } else {
            1.0
        };
        let omega = acosf(c);
        let inv_sin = 1.0 / sinf(omega);
        scale0 = sinf((1.0 - t) * omega) * inv_sin;
        scale1 = sinf(omega * t) * inv_sin;
    } else {
        scale0 = 1.0 - t;
        scale1 = t;
    }

    let scale1 = if raw >= 0.0 { scale1 } else { -scale1 };
    Quat::from_xyzw(
        scale1 * q2.x + q1.x * scale0,
        scale1 * q2.y + q1.y * scale0,
        scale1 * q2.z + q1.z * scale0,
        scale1 * q2.w + q1.w * scale0,
    )
}

/// Return the constant angular velocity that turns one basis into another.
///
/// Invert the first basis, convert the product to a quaternion, and build the
/// axis-angle vector. Use the positive X axis when the quaternion vector part
/// is too small.
#[must_use]
pub(crate) fn angular_velocity(
    fwd0: Vec3A,
    right0: Vec3A,
    up0: Vec3A,
    fwd1: Vec3A,
    right1: Vec3A,
    up1: Vec3A,
) -> Vec3A {
    let m00 = fwd0.x;
    let m01 = right0.x;
    let m02 = up0.x;
    let m10 = fwd0.y;
    let m11 = right0.y;
    let m12 = up0.y;
    let m20 = fwd0.z;
    let m21 = right0.z;
    let m22 = up0.z;

    let co0 = m11 * m22 - m12 * m21;
    let co1 = m12 * m20 - m10 * m22;
    let co2 = m10 * m21 - m11 * m20;
    let s = 1.0 / ((m00 * co0 + m01 * co1) + m02 * co2);
    let i00 = co0 * s;
    let i01 = (m02 * m21 - m01 * m22) * s;
    let i02 = (m01 * m12 - m02 * m11) * s;
    let i10 = co1 * s;
    let i11 = (m00 * m22 - m02 * m20) * s;
    let i12 = (m02 * m10 - m00 * m12) * s;
    let i20 = co2 * s;
    let i21 = (m01 * m20 - m00 * m21) * s;
    let i22 = (m00 * m11 - m01 * m10) * s;

    let row = |b0: f32, b1: f32, b2: f32| {
        Vec3A::new(
            (i00 * b0 + i10 * b1) + i20 * b2,
            (i01 * b0 + i11 * b1) + i21 * b2,
            (i02 * b0 + i12 * b1) + i22 * b2,
        )
    };

    let d0 = row(fwd1.x, right1.x, up1.x);
    let d1 = row(fwd1.y, right1.y, up1.y);
    let d2 = row(fwd1.z, right1.z, up1.z);

    let q = game_quat_normalize_scalar(
        game_get_rotation_scalar(
            Vec3A::new(d0.x, d1.x, d2.x),
            Vec3A::new(d0.y, d1.y, d2.y),
            Vec3A::new(d0.z, d1.z, d2.z),
        ),
        -1.0,
    );

    let mut w = q.w;
    w = if -1.0 <= w { w } else { -1.0 };
    w = if w <= 1.0 { w } else { 1.0 };
    let half = acosf(w);
    let angle = half + half;

    let len2 = (q.x * q.x + q.y * q.y) + q.z * q.z;
    let mut axis = Vec3A::new(1.0, 0.0, 0.0);
    if len2 >= 1.421_085_5e-14 {
        let inv = 1.0 / len2.sqrt();
        axis = Vec3A::new(inv * q.x, q.y * inv, inv * q.z);
    }

    Vec3A::new(axis.x * angle, axis.y * angle, axis.z * angle)
}
