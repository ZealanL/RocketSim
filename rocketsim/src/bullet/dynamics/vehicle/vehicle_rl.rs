use glam::Vec3A;

use super::{NUM_WHEELS, raycaster::VehicleRaycaster, wheel_info::WheelInfo};
use crate::bullet::{
    collision::broadphase::CollisionFilterGroups,
    dynamics::{discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody},
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

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            if let Some(ray_result) = ray_results[i] {
                wheel.apply_ray_cast(
                    chassis,
                    ray_result,
                    time_step,
                    i < 2,
                    handbrake_val,
                    real_throttle,
                    three_wheels,
                );
            } else {
                wheel.reset_wheel_suspension();
            }
        }

        let chassis = &mut collision_world.bodies_mut()[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            wheel.update_suspension(chassis, time_step);
        }

        let bodies = collision_world.bodies();
        let chassis = &bodies[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            wheel.update_friction_impulse(chassis, bodies, time_step);
        }

        // note: all suspension MUST be updated before impulses are applied
        let chassis = &mut collision_world.bodies_mut()[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            wheel.apply_friction_impulses(chassis, time_step);
        }
    }
}
