//! UE3 quaternion axes, vector rotation, and rotator conversions.

use glam::{Quat, Vec3A};

use super::{Rotator, rotator_from_axes};

/// Return the forward, right, and up columns of a unit quaternion.
///
/// Keep this arithmetic separate from the body-matrix conversion. The two
/// conversions use different addition orders.
#[inline]
#[must_use]
pub(crate) fn axes(q: Quat) -> (Vec3A, Vec3A, Vec3A) {
    let z2 = q.z + q.z;
    let y2 = q.y + q.y;
    let x2x = (q.x + q.x) * q.x;
    let x2w = (q.x + q.x) * q.w;

    let fwd = Vec3A::new(
        1.0 - (z2 * q.z + y2 * q.y),
        z2 * q.w + y2 * q.x,
        z2 * q.x - y2 * q.w,
    );
    let right = Vec3A::new(y2 * q.x - z2 * q.w, 1.0 - (z2 * q.z + x2x), x2w + z2 * q.y);
    let up = Vec3A::new(y2 * q.w + z2 * q.x, z2 * q.y - x2w, 1.0 - (y2 * q.y + x2x));
    (fwd, right, up)
}

/// Rotate `v` by the unit quaternion `q` with scalar arithmetic.
#[inline]
#[must_use]
pub(crate) fn rotate(q: Quat, v: Vec3A) -> Vec3A {
    let qq = (q.y * q.y + q.x * q.x) + q.z * q.z;
    let d = q.w * q.w - qq;
    let w2 = q.w + q.w;
    let dot = (v.x * q.x + v.y * q.y) + v.z * q.z;
    let t = dot + dot;
    let cx = v.z * q.y - v.y * q.z;
    let cy = v.x * q.z - v.z * q.x;
    let cz = v.y * q.x - v.x * q.y;
    Vec3A::new(
        (cx * w2 + d * v.x) + t * q.x,
        (cy * w2 + d * v.y) + t * q.y,
        (cz * w2 + d * v.z) + t * q.z,
    )
}

/// Convert a unit quaternion to pitch, yaw, and roll in rotator units.
///
/// Construct the basis columns before applying the sine-table rounding.
#[inline]
#[must_use]
pub(crate) fn quat_to_rotator(q: Quat) -> Rotator {
    let x2 = q.x + q.x;
    let y2 = q.y + q.y;
    let z2 = q.z + q.z;
    let xx = q.x * x2;
    let xy = q.x * y2;
    let xz = q.x * z2;
    let yy = q.y * y2;
    let yz = q.y * z2;
    let zz = q.z * z2;
    let wx = q.w * x2;
    let wy = q.w * y2;
    let wz = q.w * z2;
    let forward = Vec3A::new(1.0 - (yy + zz), xy + wz, xz - wy);
    let right = Vec3A::new(xy - wz, 1.0 - (xx + zz), yz + wx);
    let up = Vec3A::new(xz + wy, yz - wx, 1.0 - (xx + yy));
    rotator_from_axes(forward, right, up)
}
