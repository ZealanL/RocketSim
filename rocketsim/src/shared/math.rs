//! Scalar math selected at compile time.
//!
//! The default path uses platform math. `native-math` selects fixed-runtime
//! algorithms for the engine-sensitive operations.

#[cfg(feature = "native-math")]
pub(crate) use super::ucrt_math::{
    cosf, sinf, ucrt_acosf as acosf, ucrt_atan2f as atan2f, ucrt_powf as powf,
};

/// Return the inverse cosine of `x` in radians with platform math.
///
/// An input outside `[-1, 1]` returns NaN.
#[cfg(not(feature = "native-math"))]
#[inline]
#[must_use]
pub(crate) fn acosf(x: f32) -> f32 {
    x.acos()
}

/// Return the angle of `(x, y)` in radians with platform math.
#[cfg(not(feature = "native-math"))]
#[inline]
#[must_use]
pub(crate) fn atan2f(y: f32, x: f32) -> f32 {
    y.atan2(x)
}

/// Raise `x` to `y` with platform math.
#[cfg(not(feature = "native-math"))]
#[inline]
#[must_use]
pub(crate) fn powf(x: f32, y: f32) -> f32 {
    x.powf(y)
}
