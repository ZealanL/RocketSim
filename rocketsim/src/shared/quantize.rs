use glam::Vec3A;

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
            let mut rounded = Vec3A::ZERO;
            for i in 0..3 {
                let i_val = (vec[i] * scale) as i32;
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

#[cfg(test)]
mod tests {
    use glam::Vec3A;

    use super::{VecQuantizeMode, quantize_vec_ue3};

    fn quantize_position(vec: Vec3A, scale: f32) -> Vec3A {
        quantize_vec_ue3(vec, scale, VecQuantizeMode::Position)
    }

    #[test]
    fn position_quantization_rounds_positive_even_ties_up() {
        let actual = quantize_position(Vec3A::new(2.5, 4.5, 6.5), 1.0);

        assert_eq!(actual, Vec3A::new(3.0, 5.0, 7.0));
    }

    #[test]
    fn position_quantization_rounds_positive_odd_ties_up() {
        let actual = quantize_position(Vec3A::new(3.5, 5.5, 7.5), 1.0);

        assert_eq!(actual, Vec3A::new(4.0, 6.0, 8.0));
    }

    #[test]
    fn position_quantization_rounds_negative_ties_toward_zero() {
        let actual = quantize_position(Vec3A::new(-2.5, -3.5, -0.5), 1.0);

        assert_eq!(actual, Vec3A::new(-2.0, -3.0, 0.0));
    }

    #[test]
    fn position_quantization_rounds_non_ties_to_nearest_value() {
        let actual = quantize_position(Vec3A::new(2.25, 2.75, -2.75), 1.0);

        assert_eq!(actual, Vec3A::new(2.0, 3.0, -3.0));
    }
}
