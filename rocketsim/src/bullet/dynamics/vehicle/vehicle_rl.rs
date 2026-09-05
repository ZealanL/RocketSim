use glam::{Quat, Vec3A};

use super::{
    NUM_WHEELS,
    raycaster::VehicleRaycaster,
    wheel_info::{FrictionCurveInput, WheelInfo},
};
use crate::bullet::{
    collision::broadphase::CollisionFilterGroups,
    dynamics::{
        discrete_dynamics_world::DiscreteDynamicsWorld,
        rigid_body::{Impulse, RigidBody},
    },
    linear_math::QuatExt,
};

pub struct VehicleRL {
    raycaster: VehicleRaycaster,
    chassis_body_idx: usize,
    pub wheels: [WheelInfo; NUM_WHEELS],
}

impl VehicleRL {
    pub const fn new(chassis_body_idx: usize, wheels: [WheelInfo; NUM_WHEELS]) -> Self {
        Self {
            raycaster: VehicleRaycaster::new(CollisionFilterGroups::DropshotFloor as u8),
            chassis_body_idx,
            wheels,
        }
    }

    pub fn get_upwards_dir_from_wheel_contacts(&self, cb: &RigidBody) -> Vec3A {
        let mut sum_contact_dir = Vec3A::ZERO;
        for wheel in &self.wheels {
            if let Some(raycast_info) = wheel.raycast_info.as_ref() {
                sum_contact_dir += raycast_info.contact_normal;
            }
        }

        sum_contact_dir
            .try_normalize()
            .unwrap_or_else(|| cb.get_up_vector())
    }

    pub const fn get_num_wheels(&self) -> usize {
        self.wheels.len()
    }

    pub fn update(
        &mut self,
        collision_world: &mut DiscreteDynamicsWorld,
        time_step: f32,
        handbrake_val: f32,
        real_throttle: f32,
        three_wheels: bool,
    ) {
        let chassis = &collision_world.bodies()[self.chassis_body_idx];
        // The chassis transform and mass are invariant for the whole update:
        // wheel impulses only change velocities, never the transform or mass.
        // Cache them once instead of re-reading them for every wheel.
        let chassis_trans = *chassis.get_world_trans();
        let chassis_translation = chassis_trans.translation;
        let friction_scale = chassis.get_mass() / 3.0;

        let mut sources = [Vec3A::ZERO; 4];
        let mut targets = [Vec3A::ZERO; 4];

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            (sources[i], targets[i]) = wheel.prepare_for_raycast(&chassis_trans);
        }

        let ray_results = self
            .raycaster
            .cast_rays(collision_world, &sources, &targets, chassis);

        // Front wheels normally share one steer angle, so their steered
        // axle is identical. Compute it lazily and reuse it while the
        // steer angle matches (each build needs a sin/cos pair). If a
        // front wheel ever carries a different angle, fall back to its
        // own axle with the original formula.
        let mut front_axle_cache: Option<(f32, Vec3A)> = None;
        let mut num_wheels_in_contact = 0;
        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            if let Some(ray_result) = ray_results[i] {
                num_wheels_in_contact += 1;
                let front = i < 2;
                let steer_angle = wheel.steer_angle;
                let axle_dir = if front {
                    match front_axle_cache {
                        Some((cached_angle, cached_axle)) if cached_angle == steer_angle => {
                            cached_axle
                        }
                        _ => {
                            let axle = Quat::from_axis_angle_simd(
                                chassis_trans.matrix3.z_axis,
                                steer_angle,
                            ) * chassis_trans.matrix3.y_axis;
                            front_axle_cache = Some((steer_angle, axle));
                            axle
                        }
                    }
                } else {
                    chassis_trans.matrix3.y_axis
                };
                wheel.apply_ray_cast(
                    chassis,
                    &chassis_trans,
                    axle_dir,
                    ray_result,
                    time_step,
                    front,
                );
                let is_dynamic_hit = !ray_result.rigid_body.is_static_obj();
                wheel.refresh_friction_curves(
                    chassis,
                    FrictionCurveInput {
                        chassis_translation,
                        contact_normal: ray_result.hit_normal_in_world,
                        handbrake_val,
                        real_throttle,
                        three_wheels,
                        is_dynamic_hit,
                    },
                );
            } else {
                wheel.reset_wheel_suspension();
            }
        }

        if num_wheels_in_contact < 3 {
            for wheel in &mut self.wheels {
                wheel.engine_force /= 4.0;
            }
        }

        // Apply dynamic-body stick before chassis suspension and friction.
        for wheel in &self.wheels {
            let Some(info) = wheel.raycast_info.as_ref() else {
                continue;
            };
            let ground_idx = info.ground_body_idx;
            if info.ground_stick == Vec3A::ZERO
                || ground_idx == self.chassis_body_idx
                || ground_idx >= collision_world.bodies().len()
            {
                continue;
            }

            let ground = &mut collision_world.bodies_mut()[ground_idx];
            if ground.is_static_obj() || ground.inv_mass == 0.0 {
                continue;
            }

            let ground_offset = info.contact_point - ground.get_world_trans().translation;
            ground.add_impulse(
                None,
                Impulse::LinearRelPos(info.ground_stick, ground_offset),
                true,
                false,
            );
        }

        let chassis = &mut collision_world.bodies_mut()[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            wheel.update_suspension(chassis, chassis_translation, time_step);
        }

        let chassis = &collision_world.bodies()[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            wheel.update_friction_impulse(chassis, time_step, friction_scale);
        }

        // note: all suspension MUST be updated before impulses are applied
        let chassis = &mut collision_world.bodies_mut()[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            wheel.apply_friction_impulses(chassis, &chassis_trans, time_step);
        }
    }
}
