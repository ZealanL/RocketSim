//! Private lookup tables for the scalar math modules.
//!
//! Each table holds exact bit patterns. Keep the values unchanged.

mod atan2;
mod pow;

pub(super) use atan2::UCRT_ATAN_TAB;
pub(super) use pow::{UCRT_POW_EXP2, UCRT_POW_INV_C, UCRT_POW_LOG_C};
