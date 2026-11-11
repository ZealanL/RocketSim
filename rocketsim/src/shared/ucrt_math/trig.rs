//! Sine and cosine for `f32`.
//!
//! The core functions use a fixed argument reduction and `f64` polynomials.
//! The wrapper functions fall back to the platform sine or cosine outside the
//! core range.
//
// Ported from the scalar helpers in `include/rocketgpu/detail/math.hpp` (MIT
// License). Keep every operation in the same order.

use super::{bits::abs_bits, common::ONE_SIXTH};

/// Exclusive upper bound on `abs_bits(x)` for [`ucrt_sinf`].
pub(crate) const UCRT_SINF_LIMIT: u32 = 0x4b80_0456;

/// Exclusive upper bound on `abs_bits(x)` for [`ucrt_cosf`].
pub(crate) const UCRT_COSF_LIMIT: u32 = 0x4f49_0fdb;

// Sin polynomial.
const SIN_C1: f64 = f64::from_bits(0x3ec71de3a556c734); // 0x1.71de3a556c734p-19
const SIN_C2: f64 = f64::from_bits(0xbf2a01a01a01a01a); // -0x1.a01a01a01a01ap-13
const SIN_C3: f64 = f64::from_bits(0x3f81111111111111); // 0x1.1111111111111p-7
const SIN_C4: f64 = f64::from_bits(0xbfc5555555555555); // -0x1.5555555555555p-3

// Cos polynomial.
const COS_C1: f64 = f64::from_bits(0xbe927e4fb7789f5c); // -0x1.27e4fb7789f5cp-22
const COS_C2: f64 = f64::from_bits(0x3efa01a01a01a019); // 0x1.a01a01a01a019p-16
const COS_C3: f64 = f64::from_bits(0xbf56c16c16c16c16); // -0x1.6c16c16c16c16p-10
const COS_C4: f64 = f64::from_bits(0x3fa5555555555555); // 0x1.5555555555555p-5

// Argument reduction.
const INV_PIO2: f64 = f64::from_bits(0x3fe45f306dc9c883); // 0x1.45f306dc9c883p-1
const PIO2_HI: f64 = f64::from_bits(0x3ff921fb54400000); // 0x1.921fb54400000p+0
const PIO2_LO: f64 = f64::from_bits(0x3dd0b4611a626331); // 0x1.0b4611a626331p-34

/// Evaluates the sine polynomial at `r`. Uses one fused multiply-add per step.
#[inline]
fn sin_poly(r: f64) -> f64 {
    let r2 = r * r;
    let p = r2.mul_add(SIN_C1, SIN_C2);
    let p = p.mul_add(r2, SIN_C3);
    let p = p.mul_add(r2, SIN_C4);
    p.mul_add(r * r2, r)
}

/// Evaluates the cosine polynomial at `r`. Uses one fused multiply-add per step.
#[inline]
fn cos_poly(r: f64) -> f64 {
    let r2 = r * r;
    let base = 1.0 - r2 * 0.5;
    let p = r2.mul_add(COS_C1, COS_C2);
    let p = p.mul_add(r2, COS_C3);
    let p = p.mul_add(r2, COS_C4);
    p.mul_add(r2 * r2, base)
}

/// Reduces `a` by multiples of pi/2.
///
/// Writes the quadrant of `a` to `quadrant` and returns the remainder in
/// `[-pi/4, pi/4]`. The supported input range keeps the quotient inside
/// `i32`.
#[inline]
fn reduce(a: f64, quadrant: &mut i32) -> f64 {
    let n = a.mul_add(INV_PIO2, 0.5) as i32;
    let nd = f64::from(n);
    let r1 = (-nd).mul_add(PIO2_HI, a);
    *quadrant = n & 3;
    r1 - nd * PIO2_LO
}

/// Returns the sine of `x` for `abs_bits(x) < UCRT_SINF_LIMIT`.
///
/// Inputs below 2^-13 return unchanged. Inputs below 2^-7 use the cubic
/// correction `x - x^3 / 6`. Inputs up to pi/4 use the direct sine polynomial.
/// Larger inputs use the pi/2 reduction and the quadrant sign rule.
///
/// Inputs at or above the limit are outside the supported range. Use
/// [`sinf`] for those inputs.
#[inline]
#[must_use]
pub(crate) fn ucrt_sinf(x: f32) -> f32 {
    let ax = abs_bits(x);
    if ax < 0x3900_0000 {
        return x;
    }
    let d = f64::from(x);
    if ax <= 0x3f49_0fdb {
        if ax < 0x3c00_0000 {
            return (-((d * d) * d)).mul_add(ONE_SIXTH, d) as f32;
        }
        return sin_poly(d) as f32;
    }
    let mut q = 0;
    let r = reduce(d.abs(), &mut q);
    let y = if q & 1 != 0 { cos_poly(r) } else { sin_poly(r) };
    if (q >= 2) != (x < 0.0) {
        (-y) as f32
    } else {
        y as f32
    }
}

/// Returns the cosine of `x` for `abs_bits(x) < UCRT_COSF_LIMIT`.
///
/// Inputs below 2^-13 return `1.0`. Inputs below 2^-7 use the quadratic
/// correction `1 - x^2 / 2`. Inputs up to pi/4 use the direct cosine
/// polynomial. Larger inputs use the pi/2 reduction and the quadrant sign
/// rule.
///
/// Inputs at or above the limit are outside the supported range. Use
/// [`cosf`] for those inputs.
#[inline]
#[must_use]
pub(crate) fn ucrt_cosf(x: f32) -> f32 {
    let ax = abs_bits(x);
    if ax < 0x3900_0000 {
        return 1.0;
    }
    let d = f64::from(x);
    if ax <= 0x3f49_0fdb {
        if ax < 0x3c00_0000 {
            return (-(d * 0.5)).mul_add(d, 1.0) as f32;
        }
        return cos_poly(d) as f32;
    }
    let mut q = 0;
    let r = reduce(d.abs(), &mut q);
    let y = if q & 1 != 0 { sin_poly(r) } else { cos_poly(r) };
    if q == 1 || q == 2 {
        (-y) as f32
    } else {
        y as f32
    }
}

/// Returns the sine of `x` for all `f32` inputs.
///
/// Magnitudes below `UCRT_SINF_LIMIT` use [`ucrt_sinf`]. Larger magnitudes call
/// [`f32::sin`]. The fallback can differ from a platform C `sinf` in the last
/// bits. Exact output bits outside the core range depend on the platform.
#[inline]
#[must_use]
pub(crate) fn sinf(x: f32) -> f32 {
    if abs_bits(x) < UCRT_SINF_LIMIT {
        ucrt_sinf(x)
    } else {
        x.sin()
    }
}

/// Returns the cosine of `x` for all `f32` inputs.
///
/// Magnitudes below `UCRT_COSF_LIMIT` use [`ucrt_cosf`]. Larger magnitudes call
/// [`f32::cos`]. The fallback can differ from a platform C `cosf` in the last
/// bits. Exact output bits outside the core range depend on the platform.
#[inline]
#[must_use]
pub(crate) fn cosf(x: f32) -> f32 {
    if abs_bits(x) < UCRT_COSF_LIMIT {
        ucrt_cosf(x)
    } else {
        x.cos()
    }
}
