//! Inverse cosine for `f32`.

use super::{
    bits::{F32_CANONICAL_NAN_BITS, F32_INF_BITS, QUIET_BIT},
    common::{F32_PI, F32_PI_OVER_2, PIO, PIO2},
};

// acosf constants.
const ACOS_OFFSET: f64 = f64::from_bits(0x3c91a62633145c07); // 0x1.1a62633145c07p-54
const ACOS_C0: f32 = f32::from_bits(0x3b81ce6b);
const ACOS_C1: f32 = f32::from_bits(0xbc5b3fe1);
const ACOS_C2: f32 = f32::from_bits(0x3d678bdd);
const ACOS_C3: f32 = f32::from_bits(0x3e3c94dc);
const ACOS_D0: f32 = f32::from_bits(0x3f561f0d);
const ACOS_D1: f32 = f32::from_bits(0x3f8d6fa5);

/// Returns the inverse cosine of `x` for all `f32` inputs.
///
/// The result is in `[0, pi]`. Magnitudes below 2^-26 return `pi/2` directly.
/// Small magnitudes just above the threshold also round to `pi/2` through the
/// polynomial. An input of 1 returns 0. An input of -1 returns pi. Other
/// magnitudes at or above 1 return a canonical negative NaN. Inputs near 1 use
/// the square-root branch. A NaN input returns a quiet NaN.
#[inline]
#[must_use]
pub(crate) fn ucrt_acosf(x: f32) -> f32 {
    let ux = x.to_bits();
    let ax = ux & 0x7fff_ffff;
    let ex = (ux >> 23) & 0xff;
    if ax > F32_INF_BITS {
        return f32::from_bits(ux | QUIET_BIT);
    }
    if ex < 0x65 {
        return F32_PI_OVER_2;
    }
    if ex >= 0x7f {
        if x == 1.0 {
            return 0.0;
        }
        if x == -1.0 {
            return F32_PI;
        }
        return f32::from_bits(F32_CANONICAL_NAN_BITS);
    }
    let a = x.abs();
    let big = ex >= 0x7e;
    let z = if big { (1.0 - a) * 0.5 } else { a * a };
    let s = if big { z.sqrt() } else { 0.0 };
    let mut t = ACOS_C0;
    t = (-t).mul_add(z, ACOS_C1);
    t = t.mul_add(z, -ACOS_C2);
    t = t.mul_add(z, ACOS_C3);
    let r = (t * z) / (-ACOS_D0).mul_add(z, ACOS_D1);
    if !big {
        let lo = ACOS_OFFSET - f64::from(r * x);
        return (PIO2 - (f64::from(x) - lo)) as f32;
    }
    if (ux as i32) < 0 {
        let h = (f64::from(s * r) - ACOS_OFFSET) + f64::from(s);
        return (PIO - (h + h)) as f32;
    }
    let hi = f32::from_bits(s.to_bits() & 0xffff_0000);
    let c = (-hi).mul_add(hi, z) / (s + hi);
    r.mul_add(s + s, c + c) + (hi + hi)
}
