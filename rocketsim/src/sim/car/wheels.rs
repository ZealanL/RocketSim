//! Wheel drive inputs, sticky force, and suspension mass distribution.

use glam::Vec3A;

use super::{
    base::Car,
    car_controls::{quantize_axis_input, quantize_axis_inputs},
};
use crate::{
    MutatorConfig,
    bullet::dynamics::rigid_body::{Impulse, RigidBody},
    consts::{
        TICK_TIME, UU_TO_BT,
        car::{self as car_consts, drive as drive_consts},
    },
    shared::LinearPieceCurve,
};

/// Drive-torque curve indexed by signed forward speed in Unreal units per second.
///
/// Keep the negative-speed segments. Evaluating the positive-speed segments
/// at an absolute speed can change the last output bit.
const DRIVE_SPEED_TORQUE_FACTOR: LinearPieceCurve<5> = LinearPieceCurve::new([
    (-1410.0, 0.0),
    (-1400.0, 0.1),
    (0.0, 1.0),
    (1400.0, 0.1),
    (1410.0, 0.0),
]);

impl Car {
    /// Refresh wheel suspension scales after the chassis mass changes.
    pub(crate) fn refresh_mass_dependent_state(&mut self, mass: f32) {
        let config = self.info.config;
        let inv_mass = 1.0 / mass;
        let strengths = Self::suspension_strengths(
            inv_mass,
            config.front_wheels.connection_point_offset.x * UU_TO_BT,
            config.back_wheels.connection_point_offset.x * UU_TO_BT,
        );
        for (wheel, strength) in self.bullet_vehicle.wheels.iter_mut().zip(strengths) {
            wheel.suspension_force_scale = strength;
        }
    }

    /// Split chassis mass between the front and rear wheel suspension scales.
    /// Only negative axle distances use the equal split.
    /// Zero and NaN use the normal path.
    pub(super) fn suspension_strengths(inv_mass: f32, front_x_bt: f32, rear_x_bt: f32) -> [f32; 4] {
        let mass = 1.0 / inv_mass;
        let front_dist = front_x_bt;
        let rear_dist = -rear_x_bt;
        if front_dist < 0.0 || rear_dist < 0.0 {
            let half = mass * 0.5;
            let quarter = half * 0.5;
            [quarter, quarter, quarter, quarter]
        } else {
            let total = rear_dist + front_dist;
            let front_frac = front_dist / total;
            let front_axle = front_frac * mass;
            let rear_axle = mass - front_axle;
            let front_wheel = rear_axle * 0.5;
            let rear_wheel = front_axle * 0.5;
            [front_wheel, front_wheel, rear_wheel, rear_wheel]
        }
    }

    /// Quantize the wheel throttle and steer.
    ///
    /// The scalar form is cheaper and matches the vector form for finite
    /// inputs. NaN handling can differ between the forms and targets. Keep
    /// the original vector result for a NaN axis.
    #[inline]
    fn quantize_wheel_inputs(raw_throttle: f32, raw_steer: f32, quant_throttle: f32) -> (f32, f32) {
        if raw_throttle.is_nan() || raw_steer.is_nan() {
            let [throttle, steer, _] =
                quantize_axis_inputs(Vec3A::new(raw_throttle, raw_steer, 0.0)).to_array();
            (throttle, steer)
        } else {
            (quant_throttle, quantize_axis_input(raw_steer))
        }
    }

    pub(super) fn update_wheels(
        &mut self,
        rb: &mut RigidBody,
        forward_speed_uu: f32,
        quant_throttle: f32,
        mutator_config: &MutatorConfig,
        cached_upwards_dir: &mut Option<Vec3A>,
    ) {
        let handbrake_delta = if self.state.controls.handbrake {
            drive_consts::POWERSLIDE_RISE_RATE
        } else {
            -drive_consts::POWERSLIDE_FALL_RATE
        } * TICK_TIME;
        self.state.handbrake_val = (self.state.handbrake_val + handbrake_delta).clamp(0.0, 1.0);

        let mut real_brake = 0.0;
        let raw_throttle = self.state.controls.throttle;
        let (proc_throttle, proc_steer) =
            Self::quantize_wheel_inputs(raw_throttle, self.state.controls.steer, quant_throttle);

        let real_throttle = proc_throttle;

        let abs_forward_speed_uu = forward_speed_uu.abs();
        let mut engine_throttle = real_throttle;
        if !self.state.controls.handbrake {
            if real_throttle.abs() > drive_consts::THROTTLE_DEADZONE {
                if abs_forward_speed_uu > drive_consts::STOPPING_FORWARD_VEL
                    && real_throttle.signum() != forward_speed_uu.signum()
                {
                    real_brake = 1.0;

                    if abs_forward_speed_uu > drive_consts::BRAKING_NO_THROTTLE_SPEED_THRESH {
                        engine_throttle = 0.0;
                    }
                }
            } else {
                engine_throttle = 0.0;
                real_brake = if abs_forward_speed_uu < drive_consts::STOPPING_FORWARD_VEL {
                    1.0
                } else {
                    drive_consts::COASTING_BRAKE_FACTOR
                };
            }
        }

        const ENGINE_FORCE_UU: f32 = 288_000.0;
        const BRAKE_FORCE_UU: f32 = 10_500.0;
        let drive_speed_scale = if engine_throttle != 0.0 {
            DRIVE_SPEED_TORQUE_FACTOR.get_output(forward_speed_uu)
        } else {
            1.0
        };

        let drive_engine_force =
            ((drive_speed_scale * (engine_throttle * ENGINE_FORCE_UU)) * 0.25) * UU_TO_BT;
        let drive_brake_force = ((real_brake * BRAKE_FORCE_UU) * 0.25) * UU_TO_BT;
        for wheel in &mut self.bullet_vehicle.wheels {
            wheel.engine_force = drive_engine_force;
            wheel.brake = drive_brake_force;
        }

        let mut steer_angle = self
            .config
            .steer_angle_curve
            .get_output(abs_forward_speed_uu);
        if self.state.handbrake_val != 0.0 {
            let powerslide_steer = self
                .config
                .powerslide_steer_angle_curve
                .get_output(abs_forward_speed_uu);
            steer_angle += (powerslide_steer - steer_angle) * self.state.handbrake_val;
        }

        steer_angle *= proc_steer;
        self.bullet_vehicle.wheels[0].steer_angle = steer_angle;
        self.bullet_vehicle.wheels[1].steer_angle = steer_angle;

        // fresh raycast contact must not produce sticky force within its own tick
        if self.sticky_gate_prev {
            // The wheel contacts are unchanged between the sticky force and
            // auto-roll in one tick, so share one cached upwards direction.
            let upwards_dir = match *cached_upwards_dir {
                Some(dir) => dir,
                None => {
                    let dir = self.bullet_vehicle.get_upwards_dir_from_wheel_contacts(rb);
                    *cached_upwards_dir = Some(dir);
                    dir
                }
            };

            // Sticky keeps the quantized throttle gate. The target folds
            // the full-stick amount into one expression: three-wheel cars
            // use 1.5 * (1 - |up.z|), four-wheel cars use 1.5 - |up.z|.
            let full_stick = proc_throttle != 0.0
                || abs_forward_speed_uu > car_consts::drive::STOPPING_FORWARD_VEL;
            let sticky_base = if self.config.three_wheels { 0.0 } else { 0.5 };
            let sticky_full = if self.config.three_wheels {
                1.5
            } else {
                sticky_base + 1.0
            };
            let sticky_target = if full_stick { sticky_full } else { sticky_base };
            let sticky_force_scale =
                upwards_dir.z.abs() * (sticky_base - sticky_target) + sticky_target;

            rb.add_impulse(
                Impulse::Linear(
                    upwards_dir
                        * sticky_force_scale
                        * -mutator_config.gravity.z.abs()
                        * TICK_TIME
                        * UU_TO_BT,
                ),
                false,
                true,
            );
        }
    }
}
