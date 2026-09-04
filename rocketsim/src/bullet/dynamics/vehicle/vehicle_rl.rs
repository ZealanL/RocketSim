use glam::Vec3A;

use super::{NUM_WHEELS, raycaster::VehicleRaycaster, wheel_info::WheelInfo};
use crate::bullet::{
    collision::broadphase::CollisionFilterGroups,
    dynamics::{
        discrete_dynamics_world::DiscreteDynamicsWorld,
        rigid_body::{Impulse, RigidBody},
    },
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
        let chassis_trans = chassis.get_world_trans();

        let mut sources = [Vec3A::ZERO; 4];
        let mut targets = [Vec3A::ZERO; 4];

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            (sources[i], targets[i]) = wheel.prepare_for_raycast(chassis_trans);
        }

        let ray_results = self
            .raycaster
            .cast_rays(collision_world, &sources, &targets, chassis);

        let mut num_wheels_in_contact = 0;
        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            if let Some(ray_result) = ray_results[i] {
                num_wheels_in_contact += 1;
                wheel.apply_ray_cast(chassis, ray_result, time_step, i < 2);
                let is_dynamic_hit = !ray_result.rigid_body.is_static_obj();
                wheel.refresh_friction_curves(
                    chassis,
                    ray_result.hit_normal_in_world,
                    handbrake_val,
                    real_throttle,
                    three_wheels,
                    is_dynamic_hit,
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
            wheel.update_suspension(chassis, time_step);
        }

        let chassis = &collision_world.bodies()[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            wheel.update_friction_impulse(chassis, time_step);
        }

        // note: all suspension MUST be updated before impulses are applied
        let chassis = &mut collision_world.bodies_mut()[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            wheel.apply_friction_impulses(chassis, time_step);
        }
    }
}
