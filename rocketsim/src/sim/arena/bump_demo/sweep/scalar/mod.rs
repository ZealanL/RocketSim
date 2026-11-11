//! Scalar orientation and angle math for the car-to-car time-of-impact sweep.
//!
//! This module selects one backend at compile time:
//!
//! - Without `native-math`, use `glam` quaternion interpolation and `glam`
//!   axis-angle conversion.
//! - The `native-math` feature uses the engine scalar sine, cosine, and
//!   inverse cosine routines, in the engine operation order.
//!
//! Orientation integration is shared. It comes from
//! `bullet::linear_math::game_integrate_orientation_basis_columns`, which
//! already selects the matching backend.

#[cfg(not(feature = "native-math"))]
mod fast;
#[cfg(feature = "native-math")]
mod native;

#[cfg(not(feature = "native-math"))]
pub(super) use fast::{angular_velocity, slerp};
use glam::Vec3A;
#[cfg(feature = "native-math")]
pub(super) use native::{angular_velocity, slerp};

/// Dot product in the order `(x + y) + z`.
#[inline]
#[must_use]
pub(super) fn dot(a: Vec3A, b: Vec3A) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

/// Squared length in the order `(x + y) + z`.
#[inline]
#[must_use]
pub(super) fn length_sq(v: Vec3A) -> f32 {
    (v.x * v.x + v.y * v.y) + v.z * v.z
}

/// Length with one square root.
#[inline]
#[must_use]
pub(super) fn length(v: Vec3A) -> f32 {
    length_sq(v).sqrt()
}
