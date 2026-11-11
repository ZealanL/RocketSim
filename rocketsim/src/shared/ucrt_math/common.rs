//! Constants that more than one scalar math module uses.
//!
//! The bit patterns are exact. Do not replace a pattern with a decimal value.

// 1/6, used by the small-angle sine correction and the exponential series.
pub(super) const ONE_SIXTH: f64 = f64::from_bits(0x3fc5555555555555); // 0x1.5555555555555p-3

// 2^-4, the near-one threshold for `powf` and the atan2 ratio split point.
pub(super) const TWO_POW_NEG_4: f64 = f64::from_bits(0x3fb0000000000000); // 0x1p-4

// pi/2 as `f64`, used by the arctangent and inverse-cosine results.
pub(super) const PIO2: f64 = f64::from_bits(0x3ff921fb54442d18); // 0x1.921fb54442d18p+0

// pi as `f64`, used by the arctangent and inverse-cosine results.
pub(super) const PIO: f64 = f64::from_bits(0x400921fb54442d18); // 0x1.921fb54442d18p+1

// pi/2 as `f32`, returned for small inverse-cosine inputs.
pub(super) const F32_PI_OVER_2: f32 = f32::from_bits(0x3fc9_0fdb);

// pi as `f32`, returned for an inverse-cosine input of -1.
pub(super) const F32_PI: f32 = f32::from_bits(0x4049_0fdb);
