//! Engine rotator conversions and quaternion helpers.
//!
//! The crate has two math backends for these helpers.
//!
//! The default backend uses `glam` and platform math. It returns geometric
//! results. It does not use integer rotators or the sine table.
//!
//! The `native-math` feature selects the native backend. This backend applies
//! the engine integer rotator rules and the 16384-entry sine table.
//! Its sine-table and angle conversions retain their platform math calls.
//!
//! Both backends use these conventions:
//!
//! * The forward axis is `+X`, the right axis is `+Y`, and the up axis is
//!   `+Z`.
//! * Directions are unitless.
//!
//! The native backend uses 65536 rotator units per turn.
//!
//! All quaternion helpers assume a finite unit quaternion. All direction
//! helpers assume finite input.

#[cfg(feature = "native-math")]
mod native;

#[cfg(feature = "native-math")]
pub(crate) use native::{auto_flip_up, dodge_direction, quat_axes, quat_rotate, rotator_forward};

#[cfg(not(feature = "native-math"))]
mod fast;

#[cfg(not(feature = "native-math"))]
pub(crate) use fast::{auto_flip_up, dodge_direction, quat_axes, quat_rotate, rotator_forward};
