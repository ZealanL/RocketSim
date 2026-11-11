//! Conversions between body basis columns and quaternions.
//!
//! This module selects one math backend at compile time.
//!
//! - Enable the `native-math` feature to keep the engine-specific scalar math.
//!   This backend preserves its operation order and output bits.
//! - Without the feature, the default backend uses `glam` conversions and the
//!   platform `f32` sin and cos functions.
//!
//! Both backends use these integration rules:
//!
//! - State import and time integration use the selected matrix conversion.
//! - Time integration bounds the angle used by the step rotation.
//! - Quaternion normalization uses a caller-supplied minimum squared length.
//!
//! The native backend must keep its operation order. Reassociation can change
//! the output bits.

#[cfg(not(feature = "native-math"))]
mod fast;
#[cfg(feature = "native-math")]
mod native;

#[cfg(not(feature = "native-math"))]
pub(crate) use fast::{
    bullet_quat_from_basis_columns, game_get_rotation_scalar,
    game_integrate_orientation_basis_columns, game_quat_normalize_scalar,
    game_set_rotation_scalar_columns,
};
#[cfg(feature = "native-math")]
pub(crate) use native::{
    bullet_quat_from_basis_columns, game_get_rotation_scalar,
    game_integrate_orientation_basis_columns, game_quat_normalize_scalar,
    game_set_rotation_scalar_columns,
};
