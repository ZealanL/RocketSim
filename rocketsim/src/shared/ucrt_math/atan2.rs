//! Two-argument arctangent for `f32`.

use super::{
    bits::{F64_INF_BITS, quiet},
    common::{F32_PI, F32_PI_OVER_2, PIO, PIO2, TWO_POW_NEG_4},
    tables::UCRT_ATAN_TAB,
};

// atan2 polynomial and constants.
const ATAN_POLY_A: f64 = f64::from_bits(0xbfc2492482bd6be1); // -0x1.2492482bd6be1p-3
const ATAN_POLY_B: f64 = f64::from_bits(0x3fc99999999643a3); // 0x1.99999999643a3p-3
const ATAN_POLY_C: f64 = f64::from_bits(0x3fd5555555555538); // 0x1.5555555555538p-2
const ATAN_CUBIC: f64 = f64::from_bits(0x3fd5555555550877); // 0x1.5555555550877p-2
const ATAN_SMALL: f64 = f64::from_bits(0x3f1a36e2eb1c432d); // 0x1.a36e2eb1c432dp-14

// Float constants returned by bit pattern.
const F32_PI_OVER_4: f32 = f32::from_bits(0x3f49_0fdb);
const F32_NEG_PI_OVER_4: f32 = f32::from_bits(0xbf49_0fdb);
const F32_NEG_PI_OVER_2: f32 = f32::from_bits(0xbfc9_0fdb);
const F32_NEG_PI: f32 = f32::from_bits(0xc049_0fdb);
const F32_THREE_PI_OVER_4: f32 = f32::from_bits(0x4016_cbe4);
const F32_NEG_THREE_PI_OVER_4: f32 = f32::from_bits(0xc016_cbe4);

/// Returns the four-quadrant arctangent of `y / x` for all `f32` inputs.
///
/// The result is in `[-pi, pi]`. The function handles signed zeros,
/// infinities, and NaN. A NaN input returns a quiet NaN. When the exponents
/// differ enough to make the ratio negligible, the function returns a
/// quadrant constant instead of the ratio.
#[inline]
#[must_use]
pub(crate) fn ucrt_atan2f(y: f32, x: f32) -> f32 {
    let yd = f64::from(y);
    let xd = f64::from(x);
    let yb = yd.to_bits();
    let xb = xd.to_bits();
    let ax = xb & 0x7fff_ffff_ffff_ffff;
    let ay = yb & 0x7fff_ffff_ffff_ffff;
    let yneg = (yb >> 63) != 0;
    let xneg = (xb >> 63) != 0;
    let diff = (((yb >> 52) & 0x7ff) as i32) - (((xb >> 52) & 0x7ff) as i32);

    if ax > F64_INF_BITS {
        return quiet(x);
    }
    if ay > F64_INF_BITS {
        return quiet(y);
    }
    if ay == 0 {
        return if xneg {
            if yneg { F32_NEG_PI } else { F32_PI }
        } else {
            y
        };
    }
    if ax == 0 || diff > 26 {
        return if yneg {
            F32_NEG_PI_OVER_2
        } else {
            F32_PI_OVER_2
        };
    }
    if diff < -13 && !xneg {
        if diff < -150 {
            return if yneg { -0.0 } else { 0.0 };
        }
        return (yd / xd) as f32;
    }
    if diff < -26 {
        return if yneg { F32_NEG_PI } else { F32_PI };
    }
    if ay == F64_INF_BITS {
        if !xneg {
            return if yneg {
                F32_NEG_PI_OVER_4
            } else {
                F32_PI_OVER_4
            };
        }
        return if yneg {
            F32_NEG_THREE_PI_OVER_4
        } else {
            F32_THREE_PI_OVER_4
        };
    }

    let mut u = yd.abs();
    let mut v = xd.abs();
    let swapped = u > v;
    if swapped {
        core::mem::swap(&mut u, &mut v);
    }
    let ratio = u / v;
    let mut res;
    if ratio > TWO_POW_NEG_4 {
        let i = 256.0f64.mul_add(ratio, 0.5) as i32;
        let id = f64::from(i);
        let r = u.mul_add(256.0, -(id * v)) / u.mul_add(id, v * 256.0);
        res = (-((r * r) * r)).mul_add(
            ATAN_CUBIC,
            r + f64::from_bits(UCRT_ATAN_TAB[(i - 16) as usize]),
        );
    } else if ratio < ATAN_SMALL {
        res = ratio;
    } else {
        let r2 = ratio * ratio;
        let p = ATAN_POLY_A.mul_add(r2, ATAN_POLY_B);
        let p = (-p).mul_add(r2, ATAN_POLY_C);
        res = (-(r2 * ratio)).mul_add(p, ratio);
    }
    if swapped {
        res = PIO2 - res;
    }
    if xneg {
        res = PIO - res;
    }
    if yneg { (-res) as f32 } else { res as f32 }
}
