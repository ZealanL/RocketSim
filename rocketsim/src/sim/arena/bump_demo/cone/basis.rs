//! Body basis selection for the forward-cone test.
//!
//! Without `native-math`, the body basis columns are the axes.
//! With `native-math`, the body basis columns are converted to rotator units
//! and the axes are rebuilt from the sine table.
//!
//! The fixed conversion stays in this module. The shared UE3 entry points keep
//! their existing platform calls.

use glam::Vec3A;

use crate::CarState;

/// Return the forward, right, and up body axes, in that order.
///
/// Convert the body columns to rotator units and rebuild the axes from the
/// sine table.
#[cfg(feature = "native-math")]
#[inline]
#[must_use]
pub(super) fn body_axes(state: &CarState) -> (Vec3A, Vec3A, Vec3A) {
    fixed::axes(state.phys.rot_mat)
}

/// Return the forward, right, and up body axes, in that order.
///
/// The body columns are already the geometric axes.
#[cfg(not(feature = "native-math"))]
#[inline]
#[must_use]
pub(super) fn body_axes(state: &CarState) -> (Vec3A, Vec3A, Vec3A) {
    let m = state.phys.rot_mat;
    (m.x_axis, m.y_axis, m.z_axis)
}

/// Fixed scalar rotator conversion for the cone.
///
/// The body columns are the authoritative orientation. Do not rebuild a
/// quaternion first: that roundtrip can move one rotator unit at a boundary.
#[cfg(feature = "native-math")]
mod fixed {
    use glam::{Mat3A, Vec3A};

    use crate::shared::{math::atan2f, round_ties_even, ucrt_math::ucrt_sinf};

    /// The nearest `f32` value to pi (`0x1.921fb6p+1`).
    const PI_F32: f32 = f32::from_bits(0x4049_0fdb);

    /// Cosine phase offset in rotator units.
    const QUARTER_TURN: i32 = 0x4000;

    /// Return the fixed sine value for an angle in rotator units.
    #[inline]
    fn rotator_sin(angle: i32) -> f32 {
        let j = ((angle >> 2) & 0x3fff) as f32;
        ucrt_sinf(((j + j) * PI_F32) * (1.0 / 16384.0))
    }

    /// Convert `(y, x)` to rotator units with the fixed atan2.
    ///
    /// Round `2v + 0.5` to nearest even, then shift right by one.
    #[inline]
    fn rotator_units(y: f32, x: f32) -> i32 {
        let units = (atan2f(y, x) * 32768.0) / PI_F32;
        (round_ties_even(units + units + 0.5) as i32) >> 1
    }

    /// Return the pitch, yaw, and roll units of the body columns.
    #[must_use]
    pub(super) fn units(m: Mat3A) -> (i32, i32, i32) {
        let forward = m.x_axis;
        let right = m.y_axis;
        let up = m.z_axis;
        let yaw = rotator_units(forward.y, forward.x);
        let forward_xy = f64::from(forward.y * forward.y + forward.x * forward.x).sqrt() as f32;
        let pitch = rotator_units(forward.z, forward_xy);
        let sy = rotator_sin(yaw);
        let cy = rotator_sin(yaw + QUARTER_TURN);
        let z_dot = (up.y * cy + up.x * -sy) + up.z * -0.0;
        let y_dot = (right.y * cy + right.x * -sy) + right.z * -0.0;
        let roll = rotator_units(z_dot, y_dot);
        (pitch, yaw, roll)
    }

    /// Return the rotator axes for the given units.
    ///
    /// Keep the operation order. Reassociation can change the output bits.
    #[must_use]
    pub(super) fn axes_from_units(pitch: i32, yaw: i32, roll: i32) -> (Vec3A, Vec3A, Vec3A) {
        let sp = rotator_sin(pitch);
        let cp = rotator_sin(pitch + QUARTER_TURN);
        let sy = rotator_sin(yaw);
        let cy = rotator_sin(yaw + QUARTER_TURN);
        let sr = rotator_sin(roll);
        let cr = rotator_sin(roll + QUARTER_TURN);
        let sp_sr = sp * sr;
        let cr_sp = cr * sp;
        let x = Vec3A::new(cy * cp, cp * sy, sp);
        let y = Vec3A::new(sp_sr * cy - cr * sy, cy * cr + sp_sr * sy, -sr * cp);
        let z = Vec3A::new(-(cr_sp * cy + sy * sr), cy * sr - cr_sp * sy, cp * cr);
        (x, y, z)
    }

    /// Return the rotator axes of the body columns.
    #[inline]
    #[must_use]
    pub(super) fn axes(m: Mat3A) -> (Vec3A, Vec3A, Vec3A) {
        let (pitch, yaw, roll) = units(m);
        axes_from_units(pitch, yaw, roll)
    }
}
