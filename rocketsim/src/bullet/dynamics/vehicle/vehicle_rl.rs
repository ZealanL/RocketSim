use glam::Vec3A;

use super::{NUM_WHEELS, raycaster::VehicleRaycaster, wheel_info::WheelInfo};
use crate::{
    CollisionMasks,
    bullet::dynamics::{discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody},
};

pub struct VehicleRL {
    raycaster: VehicleRaycaster,
    chassis_body_idx: usize,
    pub wheels: [WheelInfo; NUM_WHEELS],
}

impl VehicleRL {
    pub const fn new(chassis_body_idx: usize, wheels: [WheelInfo; NUM_WHEELS]) -> Self {
        Self {
            raycaster: VehicleRaycaster::new(CollisionMasks::DropshotFloor as u8),
            chassis_body_idx,
            wheels,
        }
    }

    pub fn get_upwards_dir_from_wheel_contacts(&self, cb: &RigidBody) -> Vec3A {
        let mut sum_contact_dir = Vec3A::ZERO;
        for wheel in &self.wheels {
            if wheel.raycast_info.is_in_contact {
                sum_contact_dir += wheel.raycast_info.contact_normal_ws;
            }
        }

        sum_contact_dir
            .try_normalize()
            .unwrap_or_else(|| cb.get_up_vector())
    }

    pub const fn get_num_wheels(&self) -> usize {
        self.wheels.len()
    }

    pub fn update_vehicle_first(
        &mut self,
        collision_world: &DiscreteDynamicsWorld,
        time_step: f32,
    ) {
        let chassis = &collision_world.bodies()[self.chassis_body_idx];

        let (front_wheels, back_wheels) = self.wheels.split_at_mut(2);
        for wheel in front_wheels {
            wheel.update_wheel_trans::<true>(chassis);
        }

        for wheel in back_wheels {
            wheel.update_wheel_trans::<false>(chassis);
        }

        let mut sources = [Vec3A::ZERO; 4];
        let mut targets = [Vec3A::ZERO; 4];
        let mut suspension_travels = [0.0; 4];

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            (sources[i], targets[i], suspension_travels[i]) = wheel.prepare_for_raycast();
        }

        let ray_results = self
            .raycaster
            .cast_rays(collision_world, &sources, &targets, chassis);

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            if let Some(ray_result) = ray_results[i] {
                wheel.apply_ray_cast(chassis, suspension_travels[i], ray_result, time_step);
            } else {
                wheel.reset_wheel_suspension(suspension_travels[i]);
            }
        }

        let friction_scale = chassis.get_mass() / 3.0;
        for wheel in &mut self.wheels {
            wheel.calc_friction_impulses(
                chassis,
                collision_world.bodies(),
                friction_scale,
                time_step,
            );
        }
    }

    pub fn update_vehicle_second(&mut self, cb: &mut RigidBody, step: f32) {
        for wheel in &mut self.wheels {
            wheel.update_suspension(cb, step);
        }

        // note: all suspension MUST be updated before impulses are applied
        for wheel in &mut self.wheels {
            wheel.apply_friction_impulses(cb, step);
        }
    }
}
