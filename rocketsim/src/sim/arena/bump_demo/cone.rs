//! Forward-cone tests for bumps and demolitions.
//!
//! The test projects the victim hitbox origin on the attacker body axes.
//! It then measures two angles to the forward axis. The pitch test runs first.
//! Each angle removes the other axis component with an asymmetrical scale.
//!
//! Without `native-math`, the test uses the body basis columns. With
//! `native-math`, it uses the fixed rotator axes.

mod basis;

use basis::body_axes;
use glam::Vec3A;

use crate::{CarBodyConfig, CarState, shared::math::acosf};

/// Offset scale for the asymmetrical axis removal.
const PROJECTION_SCALE: f32 = 1.01;

/// Squared-length guard below which a direction becomes zero.
const MIN_LEN_SQ: f32 = 1e-8;

/// Radians to degrees. Multiply by the `f32` literal `57.29578`.
const RAD_TO_DEG: f32 = 57.29578;

/// Keep a vector whose squared length is already 1 bit-exact.
/// Use the zero vector below the small guard.
#[inline]
fn normalize_for_cone(v: Vec3A) -> Vec3A {
    let len_sq = v.length_squared();
    if len_sq != 1.0 {
        if len_sq < MIN_LEN_SQ {
            Vec3A::ZERO
        } else {
            v * (1.0 / len_sq.sqrt())
        }
    } else {
        v
    }
}

/// Return the body forward axis.
///
/// Without `native-math`, this is `state.phys.rot_mat.x_axis`.
#[inline]
#[must_use]
pub(super) fn forward_axis(state: &CarState) -> Vec3A {
    body_axes(state).0
}

/// Return the hitbox origin for one car.
///
/// Each component is the projection of the local offset on one body axis.
#[inline]
fn hitbox_origin(pos: Vec3A, axes: (Vec3A, Vec3A, Vec3A), offset: Vec3A) -> Vec3A {
    let (forward, right, up) = axes;
    pos + Vec3A::new(forward.dot(offset), right.dot(offset), up.dot(offset))
}

/// Return true when the victim hitbox origin is inside the forward cone.
///
/// `reverse` inverts the attacker forward axis.
/// `max_pitch_deg` limits the angle in the forward/up plane.
/// `max_yaw_deg` limits the angle in the forward/right plane.
/// The pitch test runs before the yaw test.
pub(super) fn is_within_forward_cone(
    attacker: &CarState,
    attacker_config: &CarBodyConfig,
    victim: &CarState,
    victim_config: &CarBodyConfig,
    reverse: bool,
    max_yaw_deg: f32,
    max_pitch_deg: f32,
) -> bool {
    let attacker_axes = body_axes(attacker);
    let victim_axes = body_axes(victim);

    let attacker_origin = hitbox_origin(
        attacker.phys.pos,
        attacker_axes,
        attacker_config.hitbox_pos_offset,
    );
    let victim_origin = hitbox_origin(
        victim.phys.pos,
        victim_axes,
        victim_config.hitbox_pos_offset,
    );
    let delta = normalize_for_cone(victim_origin - attacker_origin);

    let (forward, right, up) = attacker_axes;
    let forward = if reverse { -forward } else { forward };

    let angle_deg = |axis: Vec3A| {
        let mut remove = delta.dot(axis);
        remove = if remove >= 0.0 {
            remove / PROJECTION_SCALE
        } else {
            remove * PROJECTION_SCALE
        };
        let projected = normalize_for_cone(delta - axis * remove);
        acosf(forward.dot(projected).clamp(-1.0, 1.0)) * RAD_TO_DEG
    };

    // Pitch first: the gate rejects on pitch before it reads yaw.
    if angle_deg(right) > max_pitch_deg {
        return false;
    }
    angle_deg(up) <= max_yaw_deg
}
