use glam::{IVec3, Vec3A};

use crate::{
    bullet::dynamics::rigid_body::RigidBody,
    sim::consts::{BT_TO_UU, UU_TO_BT, quantize},
};

/// An extension of `Vec3A::signum` that keeps zero-components as zero (regardless of sign bit).
#[must_use]
fn vec3a_sign_3(v: Vec3A) -> Vec3A {
    let signum = v.signum();
    Vec3A::select(v.cmpeq(Vec3A::ZERO), Vec3A::ZERO, signum)
}

enum VecQuantizeMode {
    Position,
    Velocity,
}

/// UE3-networking-style quantization of vectors.
#[must_use]
fn quantize_vec_ue3(vec: Vec3A, scale: f32, quantize_mode: VecQuantizeMode) -> Vec3A {
    match quantize_mode {
        VecQuantizeMode::Position => (vec * scale + 0.5).floor() / scale,
        VecQuantizeMode::Velocity => {
            let inv_scale = 1.0 / scale;
            let scaled = vec * scale;
            let scaled_values = scaled.to_array();
            let mut rounded = Vec3A::ZERO;
            for i in 0..3 {
                let i_val = scaled_values[i] as i32;
                rounded[i] = (i_val as f32) * inv_scale;
            }

            const OFFSET_CORRECT_FRAC: f32 = 0.1;
            let offset_mag = OFFSET_CORRECT_FRAC * inv_scale;

            rounded + (vec3a_sign_3(rounded) * offset_mag)
        }
    }
}

/// Quantizes the position, linear velocity, and angular velocity of a rigid body.
pub fn quantize(body: &mut RigidBody) {
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
        body.ang_vel * BT_TO_UU,
        quantize::ANG_VEL_SCALE,
        VecQuantizeMode::Velocity,
    ) * UU_TO_BT;

    body.set_world_pos(new_pos);
    body.set_lin_vel(new_vel);
    body.set_ang_vel(new_ang_vel);
}

/// Asymmetric 8-bit input quantization: negatives scale by 128, positives by 127.
pub fn quantize_axis_inputs(ctrls: Vec3A) -> Vec3A {
    const UPPER_BOUND: Vec3A = Vec3A::splat(128.0);
    const LOWER_BOUND: Vec3A = Vec3A::splat(127.0);

    let clamped = ctrls.clamp(Vec3A::NEG_ONE, Vec3A::ONE);
    let scale = Vec3A::select(clamped.cmplt(Vec3A::ZERO), UPPER_BOUND, LOWER_BOUND);
    let biased = clamped * scale + UPPER_BOUND;
    let w = biased + biased + Vec3A::splat(0.5);
    let byte = ((w.round().as_ivec3() >> 1i32) & IVec3::splat(0xFF)).as_vec3a();
    let s = byte - UPPER_BOUND;

    Vec3A::select(
        s.cmplt(Vec3A::ZERO),
        s * (1.0 / UPPER_BOUND),
        s / LOWER_BOUND,
    )
}

/// Single-axis form of [`quantize_axis_inputs`].
#[must_use]
pub fn quantize_axis_input(x: f32) -> f32 {
    let clamped = x.clamp(-1.0, 1.0);
    let y = if clamped < 0.0 {
        (clamped * 128.0).max(-128.0)
    } else {
        (clamped * 127.0).min(127.0)
    };
    let w = ((y + 128.0) + (y + 128.0)) + 0.5;
    let eax = w.round_ties_even() as i32;
    let byte = ((eax >> 1) & 0xFF) as u8;
    let s = (byte as f32) - 128.0;
    if byte < 0x80 {
        s * (1.0 / 128.0)
    } else {
        s / 127.0
    }
}
