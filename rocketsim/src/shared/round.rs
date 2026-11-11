//! Inline rounding helpers for physics quantization.
//!
//! The helpers use the magic-constant method. They do not call a library
//! function. The result is the same as [`f32::round_ties_even`].

use glam::Vec3A;

/// The smallest `f32` whose adjacent binade has an integer unit in the last
/// place. This is `2^23`.
const MAGIC: f32 = 8_388_608.0;

/// Rounds to the nearest integer. Rounds halfway cases to the nearest even
/// integer.
///
/// Finite results match [`f32::round_ties_even`] bit for bit. Signed zeros
/// stay signed. Values at or above `2^23` in magnitude are already integral.
/// NaN and infinities stay unchanged. NaN payload bits can differ from the
/// standard function.
///
/// The add and subtract use the active floating-point rounding mode. The
/// default mode is round to nearest even. The target must not use extended
/// precision for `f32` values.
#[inline]
#[must_use]
pub(crate) fn round_ties_even(x: f32) -> f32 {
    if x.abs() < MAGIC {
        // A signed value keeps the addition in a binade with a unit in the
        // last place.
        let magic = MAGIC.copysign(x);
        let rounded = (x + magic) - magic;
        if rounded == 0.0 {
            // The add and subtract pair lose the sign of a zero result.
            0.0f32.copysign(x)
        } else {
            rounded
        }
    } else {
        // Finite values are integral. NaN and infinities stay unchanged.
        x
    }
}

/// Applies [`round_ties_even`] to each element of a vector.
///
/// The vector path handles bounded values. Other values use the scalar
/// helper. The three elements are independent. Their order does not change
/// the result.
#[inline]
#[must_use]
pub(crate) fn round_ties_even_vec3a(v: Vec3A) -> Vec3A {
    let limit = Vec3A::splat(MAGIC);
    if v.abs().cmplt(limit).all() {
        let magic = limit.copysign(v);
        let rounded = (v + magic) - magic;
        Vec3A::select(rounded.cmpeq(Vec3A::ZERO), rounded.copysign(v), rounded)
    } else {
        Vec3A::new(
            round_ties_even(v.x),
            round_ties_even(v.y),
            round_ties_even(v.z),
        )
    }
}
