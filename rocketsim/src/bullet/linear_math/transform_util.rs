use glam::{Affine3A, Mat3A, Quat, Vec3A};

use super::game_rotation::game_integrate_orientation_basis_columns;

#[inline]
pub fn integrate_trans_no_rot(cur_trans: &mut Vec3A, lin_vel: Vec3A, time_step: f32) {
    *cur_trans += lin_vel * time_step;
}

/// Integrate a transform and its orientation by one time step.
///
/// `lin_vel` is in meters per second. `ang_vel` is in radians per second.
/// `time_step` is in seconds. Derive the orientation from the current basis
/// columns. Ignore the input value of `cur_rot`. Write both the matrix and
/// its matching normalized quaternion.
pub fn integrate_trans(
    cur_trans: &mut Affine3A,
    cur_rot: &mut Quat,
    lin_vel: Vec3A,
    ang_vel: Vec3A,
    time_step: f32,
) {
    integrate_trans_no_rot(&mut cur_trans.translation, lin_vel, time_step);

    let mut fwd = cur_trans.matrix3.x_axis;
    let mut right = cur_trans.matrix3.y_axis;
    let mut up = cur_trans.matrix3.z_axis;
    // Derive the source orientation from the current basis columns.
    // The stored quaternion is only the output of the previous step.
    game_integrate_orientation_basis_columns(
        &mut fwd, &mut right, &mut up, cur_rot, ang_vel, time_step, None,
    );
    cur_trans.matrix3 = Mat3A::from_cols(fwd, right, up);
}
