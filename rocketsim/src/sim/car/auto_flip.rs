use super::base::Car;
use crate::{
    MutatorConfig,
    bullet::dynamics::rigid_body::{Impulse, RigidBody},
    consts::{TICK_TIME, UU_TO_BT, car as car_consts},
    shared::math::atan2f,
    sim::ue3,
};

/// Auto-flip recovery duration for three-wheel cars, in seconds.
/// This is half the four-wheel duration.
const THREE_WHEEL_TIME: f32 = 0.2;

impl Car {
    pub(super) fn update_auto_flip(
        &mut self,
        rb: &mut RigidBody,
        mutator_config: &MutatorConfig,
        jump_pressed: bool,
    ) {
        if jump_pressed
            && let Some(world_contact_normal) = self.state.cache.world_contact_normal
            && world_contact_normal.z > car_consts::autoflip::NORM_Z_THRESH
        {
            // The target forms the roll from the quantized basis columns.
            let rot_mat = self.state.phys.rot_mat;
            let roll = -atan2f(rot_mat.y_axis.z, rot_mat.z_axis.z);
            let abs_roll = roll.abs();

            // The gate uses the tick-start quantized body quaternion, not the
            // cached matrix up axis.
            let tick_start_up = ue3::auto_flip_up(rb.get_world_rot());
            if world_contact_normal.dot(tick_start_up) < car_consts::autoflip::UP_DOT_THRESH {
                let flip_time = if self.config.three_wheels {
                    THREE_WHEEL_TIME
                } else {
                    car_consts::autoflip::TIME
                };
                // The target forms the timer in `f64` before narrowing.
                self.state.auto_flip_timer =
                    (f64::from(flip_time) * (f64::from(abs_roll) / std::f64::consts::PI)) as f32;
                // The target maps a positive roll to `+1`; zero maps to `-1`.
                self.state.auto_flip_torque_scale = if roll > 0.0 { 1.0 } else { -1.0 };
                self.state.is_auto_flipping = true;

                // The target rotates `-up` out of the quantized body
                // quaternion, converts to a BT impulse with the mass, then
                // the official impulse path divides by the same mass.
                let (_, _, up) = ue3::quat_axes(rb.get_world_rot());
                let mass = mutator_config.car_mass;
                let impulse = ((up * -car_consts::autoflip::IMPULSE) * UU_TO_BT) * mass;
                rb.add_impulse(Impulse::Linear(impulse * (1.0 / mass)), false, false);
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
