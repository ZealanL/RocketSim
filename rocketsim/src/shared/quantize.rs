use glam::{IVec3, Vec3A};

use crate::{
    bullet::dynamics::rigid_body::RigidBody,
    sim::consts::{BT_TO_UU, UU_TO_BT, quantize},
};

/// An extension of `Vec3A::signum` that keeps zero-components as zero (regardless of sign bit)
///
/// Example: `sign_3([-3.0, 0.0, 0.5])` -> `[-1.0, 0.0, 1.0]`
#[must_use]
fn vec3a_sign_3(v: Vec3A) -> Vec3A {
    let signum = v.signum();
    Vec3A::select(v.cmpeq(Vec3A::ZERO), Vec3A::ZERO, signum)
}

pub enum VecQuantizeMode {
    Position,
    Velocity,
}

/// UE3-networking-style quantization of vectors
#[must_use]
fn quantize_vec_ue3(vec: Vec3A, scale: f32, quantize_mode: VecQuantizeMode) -> Vec3A {
    match quantize_mode {
        VecQuantizeMode::Position => {
            // Simple rounding method
            //
            // NOTE: Rocket League does something that's slightly different *sometimes*,
            //  possibly due to fast math float optimizations or something of that sort,
            //  but since a simple round is already perfect in 95% of cases, and the
            //  occasional errors are so tiny (e.g. 0.0001uu), I can't be bothered lol
            (vec * scale).round() / scale
        }
        VecQuantizeMode::Velocity => {
            const OFFSET_CORRECT_FRAC: f32 = 0.1;

            let inv_scale = 1.0 / scale;
            let offset_mag = OFFSET_CORRECT_FRAC * inv_scale;
            let scaled_vec = vec * scale;

            let i_vec = unsafe {
                IVec3 {
                    x: scaled_vec.x.to_int_unchecked::<i32>(),
                    y: scaled_vec.y.to_int_unchecked::<i32>(),
                    z: scaled_vec.z.to_int_unchecked::<i32>(),
                }
            };

            let rounded = i_vec.as_vec3a() * inv_scale;
            rounded + (vec3a_sign_3(rounded) * offset_mag)
        }
    }
}

/// Quantizes the position, linear velocity, and angular velocity of a rigid body
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
