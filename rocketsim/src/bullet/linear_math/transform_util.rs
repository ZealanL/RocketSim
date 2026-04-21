use std::f32::consts::FRAC_PI_4;

use glam::{Affine3A, Mat3A, Quat, Vec3A};

use super::{Mat3AExt, QuatExt};

const ANGULAR_MOTION_THRESHOLD: f32 = FRAC_PI_4;

#[inline]
pub fn integrate_trans_no_rot(cur_trans: &mut Affine3A, lin_vel: Vec3A, time_step: f32) {
    cur_trans.translation += lin_vel * time_step;
}

pub fn integrate_trans(cur_trans: &mut Affine3A, lin_vel: Vec3A, ang_vel: Vec3A, time_step: f32) {
    integrate_trans_no_rot(cur_trans, lin_vel, time_step);

    let mut angle = ang_vel.length();

    if angle * time_step > ANGULAR_MOTION_THRESHOLD {
        angle = ANGULAR_MOTION_THRESHOLD / time_step;
    }

    let axis = if angle < 0.001 {
        ang_vel
            * (0.5 * time_step - time_step * time_step * time_step * 0.020_833_334)
            * angle
            * angle
    } else {
        ang_vel * ((0.5 * angle * time_step).sin() / angle)
    };

    let dorn = Quat::from_xyzw(axis.x, axis.y, axis.z, (angle * time_step * 0.5).cos());
    let orn0 = Quat::from_mat3a(&cur_trans.matrix3);
    let predicted_orn = dorn.bullet_mul_quat(orn0).bullet_normalize();
    cur_trans.matrix3 = Mat3A::bullet_from_quat(predicted_orn);
}
