//! Scalar math for consistent simulation results.
//!
//! Keep the specified `f32` and `f64` operations in order.
//! Only explicit `mul_add` calls fuse. Constants use exact bit patterns.
//!
//! # Supported ranges
//!
//! [`ucrt_sinf`] is defined for `abs_bits(x) < UCRT_SINF_LIMIT`.
//! [`trig::ucrt_cosf`] is defined for `abs_bits(x) < UCRT_COSF_LIMIT`.
//!
//! [`sinf`] and [`cosf`] cover all `f32` inputs. They call [`f32::sin`]
//! or [`f32::cos`] outside the core range. The platform fallback can differ
//! from a different C runtime in the last bits. Do not use the fallback when
//! the caller needs exact output bits.
//!
//! [`ucrt_atan2f`], [`ucrt_powf`], and [`ucrt_acosf`] accept all `f32` inputs.
//! They handle NaN, infinities, and signed zeros. NaN results are quiet and
//! keep the payload bits that the algorithm selects.

// Explicit paths keep the module tree resolvable when a standalone tool
// includes this file with its own `#[path]` attribute.
#[path = "ucrt_math/acos.rs"]
mod acos;
#[path = "ucrt_math/atan2.rs"]
mod atan2;
#[path = "ucrt_math/bits.rs"]
mod bits;
#[path = "ucrt_math/common.rs"]
mod common;
#[path = "ucrt_math/pow.rs"]
mod pow;
#[path = "ucrt_math/tables/mod.rs"]
mod tables;
#[path = "ucrt_math/trig.rs"]
mod trig;

pub(crate) use acos::ucrt_acosf;
pub(crate) use atan2::ucrt_atan2f;
pub(crate) use pow::ucrt_powf;
#[allow(unused_imports)] // The standalone comparison tool calls the bounded cosine function.
pub(crate) use trig::ucrt_cosf;
pub(crate) use trig::{cosf, sinf, ucrt_sinf};
