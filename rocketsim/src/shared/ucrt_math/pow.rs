//! Power and exponential helpers for `f32`.

use super::{
    bits::{
        EXP_MASK, F32_CANONICAL_NAN_BITS, F32_INF_BITS, F32_ONE_BITS, F32_QUIET_NAN_BITS,
        QUIET_BIT, SIGN_BIT,
    },
    common::{ONE_SIXTH, TWO_POW_NEG_4},
    tables::{UCRT_POW_EXP2, UCRT_POW_INV_C, UCRT_POW_LOG_C},
};

// powf exp helper.
const EXP_OVERFLOW: f64 = f64::from_bits(0x40562e4300000000); // 0x1.62e43p6
const EXP_UNDERFLOW: f64 = f64::from_bits(0xc059d1da00000000); // -0x1.9d1dap6
const EXP_SCALE: f64 = f64::from_bits(0x40571547652b82fe); // 0x1.71547652b82fep+6
const LN2_OVER_64: f64 = f64::from_bits(0x3f862e42fefa39ef); // 0x1.62e42fefa39efp-7

// powf log path.
const LN2: f64 = f64::from_bits(0x3fe62e42fefa39ef); // 0x1.62e42fefa39efp-1
const LOG1P_A: f64 = f64::from_bits(0x3f89999999bac6d4); // 0x1.9999999bac6d4p-7
const LOG1P_B: f64 = f64::from_bits(0x3fb55555555554e6); // 0x1.55555555554e6p-4
const LOG1P_C: f64 = f64::from_bits(0x3f3c8034c85dfff0); // 0x1.c8034c85dfff0p-12
const LOG1P_D: f64 = f64::from_bits(0x3f62492307f1519f); // 0x1.2492307f1519fp-9

const ONE_THIRD: f64 = f64::from_bits(0x3fd5555555555555); // 0x1.5555555555555p-2

/// Computes `exp(t)` and applies a sign bit.
///
/// `sign` is `0` or `0x8000_0000`. `fused` selects the fused multiply-add form
/// of the small remainder. The result is infinity above the overflow limit and
/// signed zero below the underflow limit. A NaN argument also returns signed
/// zero because the underflow test is false for NaN.
#[inline]
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn powf_exp(t: f64, sign: u32, fused: bool) -> f32 {
    if t > EXP_OVERFLOW {
        return f32::from_bits(F32_INF_BITS | sign);
    }
    if !(t > EXP_UNDERFLOW) {
        return f32::from_bits(sign);
    }
    let n = (t * EXP_SCALE).round_ties_even() as i32;
    let tab = f64::from_bits(UCRT_POW_EXP2[(n & 63) as usize]);
    let res = if fused {
        let r = (-f64::from(n)).mul_add(LN2_OVER_64, t);
        let poly = ONE_SIXTH.mul_add(r, 0.5).mul_add(r, 1.0);
        (r * poly).mul_add(tab, tab)
    } else {
        let r = t - f64::from(n) * LN2_OVER_64;
        ((r * r) * (ONE_SIXTH * r + 0.5) + r) * tab + tab
    };
    let mut rb = res.to_bits();
    rb = rb.wrapping_add((((n >> 6) as i64) as u64) << 52);
    let f = f64::from_bits(rb) as f32;
    f32::from_bits(f.to_bits() | sign)
}

/// Returns `x` raised to the power `y` for all `f32` inputs.
///
/// The function handles NaN, infinities, signed zeros, negative bases, and
/// integer exponents. A finite, negative, nonzero base with a non-integer
/// exponent returns a canonical negative NaN. A near-one base uses a `log1p`-style series. Other
/// bases use a table-based logarithm and exponential.
#[inline]
#[must_use]
pub(crate) fn ucrt_powf(x: f32, y: f32) -> f32 {
    let ux = x.to_bits();
    let uy = y.to_bits();
    let ax = ux & 0x7fff_ffff;
    let ay = uy & 0x7fff_ffff;

    let odd_int = || {
        if (ay & EXP_MASK) > 0x4b00_0000 {
            return false;
        }
        let yr = y.round_ties_even();
        yr == y && ((yr as i32) & 1) != 0
    };

    if ay >= F32_INF_BITS {
        if ay > F32_INF_BITS {
            return if ux == F32_ONE_BITS {
                if ay < F32_QUIET_NAN_BITS { 1.5 } else { 1.0 }
            } else {
                f32::from_bits((if ax > F32_INF_BITS { ux } else { uy }) | QUIET_BIT)
            };
        }
        if ax > F32_INF_BITS {
            return f32::from_bits(ux | QUIET_BIT);
        }
        if ax == F32_ONE_BITS {
            return 1.0;
        }
        return f32::from_bits(if ((uy as i32) < 0) == (ax < F32_ONE_BITS) {
            F32_INF_BITS
        } else {
            0
        });
    }
    if ay == 0 {
        return if ax > F32_INF_BITS && ax < F32_QUIET_NAN_BITS {
            f32::from_bits(ux | QUIET_BIT)
        } else {
            1.0
        };
    }
    if uy == F32_ONE_BITS {
        return if ax > F32_INF_BITS {
            f32::from_bits(ux | QUIET_BIT)
        } else {
            x
        };
    }
    if ax > F32_INF_BITS {
        return f32::from_bits(ux | QUIET_BIT);
    }
    if ax == F32_INF_BITS || ax == 0 {
        let mag = if ((uy as i32) < 0) == (ax == 0) {
            F32_INF_BITS
        } else {
            0
        };
        let sign = if odd_int() { ux & SIGN_BIT } else { 0 };
        return f32::from_bits(mag | sign);
    }

    let mut sign = 0;
    if (ux as i32) < 0 && (ay & EXP_MASK) <= 0x4b00_0000 {
        if y.round_ties_even() != y {
            return f32::from_bits(F32_CANONICAL_NAN_BITS);
        }
        if odd_int() {
            sign = SIGN_BIT;
        }
    }

    let dx = f64::from(x);
    let dy = f64::from(y);

    // Signed comparison: the reference tests the raw bit pattern against
    // 0x3f880000 as a signed int. Negative x satisfies it and then fails the
    // `abs(f) < 2^-4` test below.
    if (ux as i32) < 0x3f88_0000 {
        let f = dx - 1.0;
        if f.abs() < TWO_POW_NEG_4 {
            let u = f / (f + 2.0);
            let w = u + u;
            let v = w * w;
            let w3 = w * v;
            // The reference writes `v * A + B` without an explicit fma here.
            let a = v * LOG1P_A + LOG1P_B;
            let b = v * LOG1P_C + LOG1P_D;
            let s = (((v * v) * w3) * b + w3 * a) - u * f;
            return powf_exp(dy * (f + s), sign, false);
        }
    }

    let bx = dx.to_bits();
    let mant = bx & 0x000f_ffff_ffff_ffff;
    let idx = ((mant >> 44) + ((mant >> 43) & 1)) as usize;
    let cb = (0x3fe00u64 | idx as u64) << 44;
    let mb = mant | 0x3fe0_0000_0000_0000;
    let c = f64::from_bits(cb);
    let m = f64::from_bits(mb);
    let e = (((bx >> 52) & 0x7ff) as i32) - 0x3ff;
    let r = (c - m) * f64::from_bits(UCRT_POW_INV_C[idx]);
    let rq = r * (ONE_THIRD.mul_add(r, 0.5).mul_add(r, 1.0));
    let log_x = (f64::from(e) * LN2 + f64::from_bits(UCRT_POW_LOG_C[idx])) - rq;
    powf_exp(dy * log_x, sign, true)
}
