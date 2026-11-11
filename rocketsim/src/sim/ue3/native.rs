//! Native UE3 sine-table and rotator conversions.
//!
//! This backend applies the engine integer rotator rules and the
//! 16384-entry sine table. A full turn is 65536 rotator units. Sine lookup
//! discards the two lowest angle bits and wraps the table index.
//! Keep the rounding rules and arithmetic order in these conversions.

mod quaternion;

use glam::{Quat, Vec3A};
use quaternion::quat_to_rotator;
pub(crate) use quaternion::{axes as quat_axes, rotate as quat_rotate};

use crate::shared::round_ties_even;

/// Cosine phase offset in rotator units.
const QUARTER_TURN: i32 = 0x4000;

/// A UE3 rotator in 65536 units per turn.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Rotator {
    /// Pitch in rotator units.
    pub pitch: i32,
    /// Yaw in rotator units.
    pub yaw: i32,
    /// Roll in rotator units.
    pub roll: i32,
}

/// One entry of the UE3 16384-entry sine table.
///
/// Form the argument in `f32` as `((j + j) * pi) * 2^-14`.
/// Evaluate its sine in `f64`, then convert the result to `f32`.
#[inline]
#[must_use]
pub(crate) fn sin_tab(j: i32) -> f32 {
    let j = j as f32;
    let angle = ((j + j) * std::f32::consts::PI) * (1.0 / 16384.0);
    f64::from(angle).sin() as f32
}

/// Sine-table value for an angle in 65536 units per turn.
#[inline]
#[must_use]
pub(crate) fn rotator_sin(angle: i32) -> f32 {
    sin_tab((angle >> 2) & 0x3FFF)
}

/// Yaw, pitch, and roll rounding for a quaternion rotator.
///
/// Compute `atan2(y, x) * 32768.0 / pi` in `f32`.
/// Then apply ties-to-even rounding to `2v + 0.5` and shift right by one.
/// Both inputs must be finite.
#[inline]
#[must_use]
pub(crate) fn rotator_units(y: f32, x: f32) -> i32 {
    let units = (y.atan2(x) * 32768.0) / std::f32::consts::PI;
    (round_ties_even(units + units + 0.5) as i32) >> 1
}

/// Yaw in rotator units for the dodge direction.
///
/// Compute `atan2(y, x) * (32768.0 / pi)` in `f32`.
/// Then apply ties-to-even rounding. Unlike [`rotator_units`], this function
/// does not double and offset the result before rounding.
#[inline]
#[must_use]
pub(crate) fn dodge_yaw_units(y: f32, x: f32) -> i32 {
    let units = y.atan2(x) * (32768.0 / std::f32::consts::PI);
    round_ties_even(units) as i32
}

/// Forward direction from a direction vector.
///
/// Round the supplied direction to rotator pitch and yaw with
/// [`rotator_units`]. Then construct the forward direction from the sine
/// table. Preserve the single-precision angle conversions.
#[inline]
#[must_use]
pub(crate) fn rotator_forward(forward: Vec3A) -> Vec3A {
    let yaw = rotator_units(forward.y, forward.x);
    let xy = f64::from(forward.y * forward.y + forward.x * forward.x).sqrt() as f32;
    let pitch = rotator_units(forward.z, xy);
    let cos_pitch = rotator_sin(pitch + QUARTER_TURN);
    Vec3A::new(
        cos_pitch * rotator_sin(yaw + QUARTER_TURN),
        cos_pitch * rotator_sin(yaw),
        rotator_sin(pitch),
    )
}

/// Up direction of the body from a unit quaternion.
///
/// Convert the quaternion to a rotator. Then evaluate the rotator up axis.
#[inline]
#[must_use]
pub(crate) fn auto_flip_up(q: Quat) -> Vec3A {
    rotator_up_axis(quat_to_rotator(q))
}

/// Horizontal dodge direction from a forward vector.
///
/// Round the horizontal direction to dodge yaw units with
/// [`dodge_yaw_units`]. Then evaluate the sine table at the yaw angle and at
/// the quarter-turn offset.
#[inline]
#[must_use]
pub(crate) fn dodge_direction(forward: Vec3A) -> Vec3A {
    let yaw = dodge_yaw_units(forward.y, forward.x);
    Vec3A::new(rotator_sin(yaw + QUARTER_TURN), rotator_sin(yaw), 0.0)
}

/// Up direction of a rotator.
#[inline]
#[must_use]
pub(crate) fn rotator_up_axis(r: Rotator) -> Vec3A {
    let sy = rotator_sin(r.yaw);
    let cy = rotator_sin(r.yaw + QUARTER_TURN);
    let sp = rotator_sin(r.pitch);
    let cp = rotator_sin(r.pitch + QUARTER_TURN);
    let sr = rotator_sin(r.roll);
    let cr = rotator_sin(r.roll + QUARTER_TURN);
    let cr_sp = cr * sp;
    Vec3A::new(-(cr_sp * cy + sy * sr), cy * sr - cr_sp * sy, cp * cr)
}

/// Derive pitch, yaw, and roll from basis columns with sine-table rounding.
fn rotator_from_axes(forward: Vec3A, right: Vec3A, up: Vec3A) -> Rotator {
    let yaw = rotator_units(forward.y, forward.x);
    let forward_xy = f64::from(forward.y * forward.y + forward.x * forward.x).sqrt() as f32;
    let pitch = rotator_units(forward.z, forward_xy);
    let sy = rotator_sin(yaw);
    let cy = rotator_sin(yaw + QUARTER_TURN);
    let z_dot = (up.y * cy + up.x * -sy) + up.z * -0.0;
    let y_dot = (right.y * cy + right.x * -sy) + right.z * -0.0;
    let roll = rotator_units(z_dot, y_dot);
    Rotator { pitch, yaw, roll }
}
