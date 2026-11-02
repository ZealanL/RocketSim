use glam::{Mat3A, Quat, Vec3A};

use crate::{
    bullet::dynamics::rigid_body::{CollisionFlags, RigidBody},
    shared::round_ties_even,
    sim::consts::{BT_TO_UU, UU_TO_BT, quantize},
};

/// An extension of `Vec3A::signum` that keeps zero-components as zero (regardless of sign bit).
#[must_use]
fn vec3a_sign_3(v: Vec3A) -> Vec3A {
    let signum = v.signum();
    Vec3A::select(v.cmpeq(Vec3A::ZERO), Vec3A::ZERO, signum)
}

#[derive(Clone, Copy)]
enum VecQuantizeMode {
    Position,
    Velocity,
}

/// UE3-networking-style quantization of vectors.
#[must_use]
fn quantize_vec_ue3(vec: Vec3A, scale: f32, quantize_mode: VecQuantizeMode) -> Vec3A {
    match quantize_mode {
        VecQuantizeMode::Position => (vec * scale + 0.5).floor() * (1.0 / scale),
        VecQuantizeMode::Velocity => {
            let inv_scale = 1.0 / scale;
            let scaled = vec * scale;
            let rounded = truncate_scaled_velocity(scaled, inv_scale);

            const OFFSET_CORRECT_FRAC: f32 = 0.1;
            let offset_mag = OFFSET_CORRECT_FRAC * inv_scale;

            rounded + (vec3a_sign_3(rounded) * offset_mag)
        }
    }
}

/// Truncates each scaled velocity component to `i32` and multiplies it by
/// `inv_scale`.
///
/// The vector path applies to values with a magnitude below `2^31`. In that
/// range, truncation and the saturating `as i32` cast give the same integer.
/// The scalar path keeps the saturating cast for extreme values, NaN, and
/// infinities.
#[inline]
#[must_use]
fn truncate_scaled_velocity(scaled: Vec3A, inv_scale: f32) -> Vec3A {
    const INT_LIMIT: Vec3A = Vec3A::splat(2_147_483_648.0);

    if scaled.abs().cmplt(INT_LIMIT).all() {
        scaled.trunc() * inv_scale
    } else {
        let values = scaled.to_array();
        Vec3A::new(
            (values[0] as i32 as f32) * inv_scale,
            (values[1] as i32 as f32) * inv_scale,
            (values[2] as i32 as f32) * inv_scale,
        )
    }
}

// Recorded-state model of the replicated-rigid-body quaternion
// representation. These are the scalar Bullet forms. This model reproduces
// the recorded representation; it is not a recovered native routine.
fn game_set_rotation_scalar(q: Quat) -> Mat3A {
    let d = ((q.x * q.x + q.y * q.y) + q.z * q.z) + q.w * q.w;
    let s = 2.0 / d;
    let xs = q.x * s;
    let ys = q.y * s;
    let zs = q.z * s;
    let wx = q.w * xs;
    let wy = q.w * ys;
    let wz = q.w * zs;
    let xx = q.x * xs;
    let xy = q.x * ys;
    let xz = q.x * zs;
    let yy = q.y * ys;
    let yz = q.y * zs;
    let zz = q.z * zs;
    Mat3A::from_cols(
        Vec3A::new(1.0 - (yy + zz), xy + wz, xz - wy),
        Vec3A::new(xy - wz, 1.0 - (xx + zz), yz + wx),
        Vec3A::new(xz + wy, yz - wx, 1.0 - (xx + yy)),
    )
}

fn game_quat_normalize_scalar(q: Quat, min_len_sq: f32) -> Quat {
    let len_sq = ((q.x * q.x + q.y * q.y) + q.z * q.z) + q.w * q.w;
    if len_sq.partial_cmp(&min_len_sq) != Some(std::cmp::Ordering::Greater) {
        return Quat::IDENTITY;
    }
    let s = 1.0 / len_sq.sqrt();
    Quat::from_xyzw(q.x * s, q.y * s, q.z * s, q.w * s)
}

fn game_quat_quantize_component(c: f32) -> f32 {
    const MAX: i32 = (1 << 18) - 1;
    let max_f = MAX as f32;
    let mut v = ((c * std::f32::consts::SQRT_2) * 0.5 + 0.5) * max_f;
    v = (v + v) + 0.5;
    let k = ((round_ties_even(v) as i32) >> 1).clamp(0, MAX);
    let r = k as f32 / max_f - 0.5;
    (r + r) * 0.707_106_77
}

fn game_rb_state_quantize_quat(q_in: Quat) -> Quat {
    let q = game_quat_normalize_scalar(q_in, 1e-8);
    let c = [q.x, q.y, q.z, q.w];
    let a = [c[0].abs(), c[1].abs(), c[2].abs(), c[3].abs()];

    let i01 = if a[0] < a[1] { 1 } else { 0 };
    let i23 = if a[2] < a[3] { 3 } else { 2 };
    let big = if a[i01] >= a[i23] { i01 } else { i23 };
    let sign = if c[big] < 0.0 { -1.0 } else { 1.0 };

    let mut d_input = [0.0; 3];
    let mut n = 0;
    for (i, &value) in c.iter().enumerate() {
        if i != big {
            d_input[n] = value * sign;
            n += 1;
        }
    }

    let d = [
        game_quat_quantize_component(d_input[0]),
        game_quat_quantize_component(d_input[1]),
        game_quat_quantize_component(d_input[2]),
    ];

    let rest = ((1.0 - d[0] * d[0]) - d[1] * d[1]) - d[2] * d[2];
    debug_assert!(
        rest >= 0.0,
        "largest-component decode must have non-negative rest, got {rest}"
    );

    let dropped = rest.sqrt();
    let mut o = [0.0; 4];
    n = 0;

    for (i, slot) in o.iter_mut().enumerate() {
        *slot = if i == big {
            dropped
        } else {
            let value = d[n];
            n += 1;
            value
        };
    }

    game_quat_normalize_scalar(Quat::from_xyzw(o[0], o[1], o[2], o[3]), 1e-8)
}

/// Quantize position, orientation, linear velocity, and angular velocity.
/// Leave orientation unchanged when angular motion is disabled.
pub(super) fn quantize(body: &mut RigidBody) {
    let new_pos = quantize_vec_ue3(
        body.get_world_pos() * BT_TO_UU,
        quantize::POS_SCALE,
        VecQuantizeMode::Position,
    ) * UU_TO_BT;
    let new_vel = quantize_vec_ue3(
        body.lin_vel * BT_TO_UU,
        quantize::VEL_SCALE,
        VecQuantizeMode::Velocity,
    ) * UU_TO_BT;
    let new_ang_vel = quantize_vec_ue3(
        body.ang_vel,
        quantize::ANG_VEL_SCALE,
        VecQuantizeMode::Velocity,
    );

    body.set_world_pos(new_pos);
    body.set_lin_vel(new_vel);
    body.set_ang_vel(new_ang_vel);

    if body.collision_flags & CollisionFlags::NoAngularMotion != 0 {
        return;
    }

    let rot = game_rb_state_quantize_quat(body.get_world_rot());
    let mut trans = *body.get_world_trans();
    trans.matrix3 = game_set_rotation_scalar(rot);
    body.set_world_trans_and_rot(trans, rot);
}
