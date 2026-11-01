use std::f32::consts::PI;

use glam::{Mat3A, Vec3A};

use super::base::Car;
use crate::{
    bullet::dynamics::rigid_body::{Impulse, RigidBody},
    consts::{TICK_TIME, UU_TO_BT, car as car_consts},
    sim::ue3,
};

impl Car {
    pub(super) fn update_auto_flip(&mut self, rb: &mut RigidBody, jump_pressed: bool) {
        if jump_pressed
            && let Some(world_contact_normal) = self.state.cache.world_contact_normal
            && world_contact_normal.z > car_consts::autoflip::NORM_Z_THRESH
        {
            let rot_mat = self.state.phys.rot_mat;
            let y_z = rot_mat.y_axis.z;
            let z_z = rot_mat.z_axis.z;
            let cy = (z_z * z_z + y_z * y_z).sqrt();
            let roll = if cy > 16.0 * f32::EPSILON {
                y_z.atan2(z_z)
            } else {
                0.0
            };

            let abs_roll = roll.abs();
            if world_contact_normal.dot(self.state.get_up_dir())
                < car_consts::autoflip::UP_DOT_THRESH
            {
                self.state.auto_flip_timer = car_consts::autoflip::TIME * (abs_roll / PI);
                self.state.auto_flip_torque_scale = autoflip_impulse_sign(&self.state.phys.rot_mat);
                self.state.is_auto_flipping = true;

                let force =
                    -self.state.get_up_dir() * const { car_consts::autoflip::IMPULSE * UU_TO_BT };
                rb.add_impulse(Impulse::Linear(force), false, false);
            }
        }

        if self.state.is_auto_flipping {
            if self.state.auto_flip_timer <= 0.0 {
                self.state.is_auto_flipping = false;
                self.state.auto_flip_timer = 0.0;
            } else {
                rb.ang_vel += self.state.get_forward_dir()
                    * car_consts::autoflip::ANG_VEL_DELTA
                    * self.state.auto_flip_torque_scale;
                rb.limit_vels(car_consts::MAX_SPEED * UU_TO_BT, car_consts::MAX_ANG_SPEED);
                self.state.auto_flip_timer -= TICK_TIME;
            }
        }
    }
}

/// Roll select for the auto-flip recovery, in wrapped signed-16 units.
///
/// Runs on the matrix columns. Do not replace with `-roll.signum()`.
/// [`autoflip_impulse_sign`] maps the units to ±1.
fn autoflip_roll_units(rot_mat: &Mat3A) -> i16 {
    let col0 = rot_mat.x_axis;
    let col1 = rot_mat.y_axis;
    let col2 = rot_mat.z_axis;

    let yaw = axis_angle_to_int(col0.y, col0.x);
    let len = f64::from(col0.x).hypot(f64::from(col0.y)) as f32;
    let pitch = axis_angle_to_int(col0.z, len);
    let r1 = rebuild_row1(pitch, yaw);

    let roll = axis_angle_to_int(col2.dot(r1), col1.dot(r1));
    roll as i16
}

/// Direction sign for the auto-flip roll: `+1.0` when the roll units
/// are strictly positive, `-1.0` otherwise (zero maps to `-1`).
fn autoflip_impulse_sign(rot_mat: &Mat3A) -> f32 {
    if autoflip_roll_units(rot_mat) > 0 {
        1.0
    } else {
        -1.0
    }
}

/// Converts an `atan2(y, x)` angle to scaled integer units.
fn axis_angle_to_int(a0: f32, a1: f32) -> i32 {
    let ang = f64::from(a0).atan2(f64::from(a1));
    let u = (ang as f32 * 32768.0) / PI;
    let v = u * 2.0 + 0.5;
    (v.round_ties_even() as i32) >> 1
}

/// Rebuilds the intermediate row vector from the pitch and yaw angles.
fn rebuild_row1(pitch: i32, yaw: i32) -> Vec3A {
    let tp = ue3::sin_tab_angle(pitch);
    let ty = ue3::sin_tab_angle(yaw);
    let tr = ue3::sin_tab_angle(0);
    let tpq = ue3::sin_tab_angle(pitch + 0x4000);
    let tyq = ue3::sin_tab_angle(yaw + 0x4000);
    let trq = ue3::sin_tab_angle(0x4000);
    let r1x = tp * tr * tyq - trq * ty;
    let r1y = tyq * trq + tp * tr * ty;
    let r1z = -tr * tpq;
    Vec3A::new(r1x, r1y, r1z)
}
