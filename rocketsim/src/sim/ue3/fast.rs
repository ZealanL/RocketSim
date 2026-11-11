//! Default UE3 helper backend.
//!
//! This backend uses `glam` and platform math. It returns geometric results.
//! It does not use integer rotators or the sine table.
//!
//! All helpers assume a finite unit quaternion. Directions must be finite.

use glam::{Mat3A, Quat, Vec3A};

/// Return the forward, right, and up columns of a unit quaternion.
///
/// The columns are the rotated `+X`, `+Y`, and `+Z` axes, in that order.
#[inline]
#[must_use]
pub(crate) fn quat_axes(q: Quat) -> (Vec3A, Vec3A, Vec3A) {
    let m = Mat3A::from_quat(q);
    (m.x_axis, m.y_axis, m.z_axis)
}

/// Rotate `v` by the unit quaternion `q`.
#[inline]
#[must_use]
pub(crate) fn quat_rotate(q: Quat, v: Vec3A) -> Vec3A {
    q * v
}

/// Forward direction from a direction vector.
///
/// Return the normalized direction. A zero vector returns zero.
#[inline]
#[must_use]
pub(crate) fn rotator_forward(forward: Vec3A) -> Vec3A {
    forward.normalize_or_zero()
}

/// Up direction of the body from a unit quaternion.
///
/// Return the rotated `+Z` axis.
#[inline]
#[must_use]
pub(crate) fn auto_flip_up(q: Quat) -> Vec3A {
    q * Vec3A::Z
}

/// Horizontal dodge direction from a forward vector.
///
/// Clear the `z` component and normalize the result.
///
/// A zero horizontal part has no defined direction. In this case, use
/// platform `atan2` on the original components. The fallback keeps the
/// signed-zero behavior of `atan2`.
#[inline]
#[must_use]
pub(crate) fn dodge_direction(forward: Vec3A) -> Vec3A {
    let horizontal = Vec3A::new(forward.x, forward.y, 0.0).normalize_or_zero();
    if horizontal == Vec3A::ZERO {
        let (sin, cos) = forward.y.atan2(forward.x).sin_cos();
        return Vec3A::new(cos, sin, 0.0);
    }
    horizontal
}
