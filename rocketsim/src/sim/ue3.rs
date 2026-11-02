use glam::Vec3A;

use crate::shared::round_ties_even;

/// One entry of UE3's 16384-entry sine table
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
pub(crate) fn sin_tab_angle(angle: i32) -> f32 {
    sin_tab((angle >> 2) & 0x3FFF)
}

/// Converts a direction to rotator units with the game's `appRound`.
///
/// Uses `atan2f`, `(angle * 32768) / PI`, then `rint(2v + 0.5) >> 1`.
#[inline]
#[must_use]
pub(crate) fn rotator_units(y: f32, x: f32) -> i32 {
    let units = (y.atan2(x) * 32768.0) / std::f32::consts::PI;
    (round_ties_even(units + units + 0.5) as i32) >> 1
}

/// Yaw in rotator units for the dodge direction.
///
/// The game uses `rint(atan2f(y, x) * (32768 / PI))` here, not `appRound`.
#[inline]
#[must_use]
pub(crate) fn dodge_yaw_units(y: f32, x: f32) -> i32 {
    let units = y.atan2(x) * (32768.0 / std::f32::consts::PI);
    round_ties_even(units) as i32
}

/// The game's `GetRotatorAxis(QuatToRotator(q), 0)` forward direction.
///
/// Rounds the direction to rotator pitch and yaw. Then builds the forward
/// from the 16384-entry sine table.
#[inline]
#[must_use]
pub(crate) fn rotator_forward(forward: Vec3A) -> Vec3A {
    let yaw = rotator_units(forward.y, forward.x);
    let xy = f64::from(forward.y * forward.y + forward.x * forward.x).sqrt() as f32;
    let pitch = rotator_units(forward.z, xy);
    let cos_pitch = sin_tab_angle(pitch + 0x4000);
    Vec3A::new(
        cos_pitch * sin_tab_angle(yaw + 0x4000),
        cos_pitch * sin_tab_angle(yaw),
        sin_tab_angle(pitch),
    )
}
