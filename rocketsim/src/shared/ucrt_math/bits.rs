//! Bit helpers and float bit patterns shared by the scalar math modules.
//!
//! The patterns are exact. Do not replace a pattern with a decimal value.

/// Returns the magnitude bits of a float.
///
/// The result clears the sign bit and keeps the exponent and mantissa.
#[inline]
#[must_use]
pub(crate) fn abs_bits(x: f32) -> u32 {
    x.to_bits() & 0x7fff_ffff
}

/// Sets the quiet bit in the bit pattern.
///
/// On a signaling NaN, the payload is kept and the value becomes quiet. The
/// function is used only for NaN inputs.
#[inline]
pub(super) fn quiet(f: f32) -> f32 {
    f32::from_bits(f.to_bits() | QUIET_BIT)
}

pub(super) const F32_ONE_BITS: u32 = 0x3f80_0000;
pub(super) const F32_INF_BITS: u32 = 0x7f80_0000;
pub(super) const F32_QUIET_NAN_BITS: u32 = 0x7fc0_0000;
pub(super) const F32_CANONICAL_NAN_BITS: u32 = 0xffc0_0000;
pub(super) const QUIET_BIT: u32 = 0x0040_0000;
pub(super) const SIGN_BIT: u32 = 0x8000_0000;
pub(super) const EXP_MASK: u32 = 0x7f80_0000;
pub(super) const F64_INF_BITS: u64 = 0x7ff0_0000_0000_0000;
