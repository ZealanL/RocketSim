use glam::Vec3A;
use std::sync::atomic::{AtomicUsize, Ordering};

use super::{NUM_WHEELS, raycaster::VehicleRaycaster, wheel_info::WheelInfo};
use crate::bullet::{
    collision::broadphase::CollisionFilterGroups,
    dynamics::{discrete_dynamics_world::DiscreteDynamicsWorld, rigid_body::RigidBody},
};

static FRIC_TRACE_CALLS: AtomicUsize = AtomicUsize::new(0);

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

    /// Perform the raycast half of a vehicle tick. This must happen before
    /// the car's wheel/air-control update so contact count and forward speed
    /// describe the current tick, matching RocketSim v2's updateVehicleFirst.
    pub fn update_first(&mut self, collision_world: &mut DiscreteDynamicsWorld, time_step: f32) {
        let trace_call = FRIC_TRACE_CALLS.fetch_add(1, Ordering::Relaxed);
        let chassis = &collision_world.bodies()[self.chassis_body_idx];
        let chassis_trans = chassis.get_world_trans();

        let mut sources = [Vec3A::ZERO; 4];
        let mut targets = [Vec3A::ZERO; 4];

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            (sources[i], targets[i]) = wheel.prepare_for_raycast(chassis_trans);
        }

        if std::env::var("RSIM_DEBUG_WHEEL").is_ok() {
            let chassis = &collision_world.bodies()[self.chassis_body_idx];
            println!(
                "rust_vehicle_first,body={},pos={:?},lin={:?},ang={:?},up={:?}",
                self.chassis_body_idx,
                chassis.get_world_trans().translation,
                chassis.lin_vel,
                chassis.ang_vel,
                chassis.get_world_trans().matrix3.z_axis
            );
            for (i, wheel) in self.wheels.iter().enumerate() {
                match wheel.raycast_info.as_ref() {
                    Some(info) => println!(
                        "rust_wheel_first,body={},wheel={},contact={},point={:?},normal={:?},susp={},susp_rel={},clip={},hard={:?},axle={:?},vel={:?},lat={},long={},extra={}",
                        self.chassis_body_idx,
                        i,
                        info.is_in_contact_with_world,
                        info.contact_point,
                        info.contact_normal,
                        info.suspension_length,
                        info.suspension_relative_vel,
                        info.clipped_inv_contact_dot_suspension,
                        wheel.hard_point,
                        wheel.axle_dir,
                        wheel.vel_at_contact_point,
                        wheel.lat_friction,
                        wheel.long_friction,
                        wheel.extra_pushback
                    ),
                    None => println!(
                        "rust_wheel_first,body={},wheel={},contact=0",
                        self.chassis_body_idx, i
                    ),
                }
            }
        }

        let ray_results = self
            .raycaster
            .cast_rays(collision_world, &sources, &targets, chassis);

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            if let Some(ray_result) = ray_results[i] {
                wheel.apply_ray_cast(chassis, ray_result, time_step, i < 2);
            } else {
                wheel.reset_wheel_suspension();
            }
        }

        // Narrow parity diagnostics: unlike RSIM_DEBUG_WHEEL (which prints the
        // previous raycast state), this captures the freshly-applied raycast
        // values that feed suspension/friction.  It is intentionally gated so
        // normal simulation remains bit-for-bit inert.
        if std::env::var("RSIM_DEBUG_WHEEL_AFTER").is_ok() {
            let chassis = &collision_world.bodies()[self.chassis_body_idx];
            println!(
                "rust_wheel_after,body={},pos={:?},lin={:?},ang={:?},up={:?},invI={:?}",
                self.chassis_body_idx,
                chassis.get_world_trans().translation,
                chassis.lin_vel,
                chassis.ang_vel,
                chassis.get_world_trans().matrix3.z_axis,
                chassis.inv_inertia_tensor_world,
            );
            for (i, wheel) in self.wheels.iter().enumerate() {
                match wheel.raycast_info.as_ref() {
                    Some(info) => println!(
                        "rust_wheel_after_data,body={},wheel={},contact={},point={:?},normal={:?},susp={},susp_rel={},clip={},force_scale={},extra={},hard={:?},vel={:?}",
                        self.chassis_body_idx,
                        i,
                        info.is_in_contact_with_world,
                        info.contact_point,
                        info.contact_normal,
                        info.suspension_length,
                        info.suspension_relative_vel,
                        info.clipped_inv_contact_dot_suspension,
                        wheel.suspension_force_scale,
                        wheel.extra_pushback,
                        wheel.hard_point,
                        wheel.vel_at_contact_point,
                    ),
                    None => println!(
                        "rust_wheel_after_data,body={},wheel={},contact=0",
                        self.chassis_body_idx, i
                    ),
                }
            }
        }

        // RocketSim v2 calculates friction impulses in updateVehicleFirst,
        // before Car::_UpdateWheels changes this tick's engine/brake/friction
        // fields.  The values stored on each wheel are therefore the values
        // from the previous tick, exactly as in the C++ vehicle path.
        for (wheel_idx, wheel) in self.wheels.iter_mut().enumerate() {
            let Some(info) = wheel.raycast_info.as_ref() else {
                continue;
            };
            let chassis = &collision_world.bodies()[self.chassis_body_idx];
            let ground = &collision_world.bodies()[info.ground_body_idx];
            let impulse = wheel.calc_friction_impulses(
                chassis,
                ground,
                info.contact_normal,
                info.contact_point,
                time_step,
            );
            if let Some(info) = wheel.raycast_info.as_mut() {
                info.impulse = impulse;
            }
            if std::env::var("RSIM_FRIC_TRACE_CALL")
                .ok()
                .and_then(|v| v.parse::<usize>().ok())
                == Some(trace_call)
            {
                let info = wheel.raycast_info.as_ref().unwrap();
                println!(
                    "rust_fric_trace,call={},body={},wheel={},axle=({:.17e}[{:08x}],{:.17e}[{:08x}],{:.17e}[{:08x}]),normal=({:.17e}[{:08x}],{:.17e}[{:08x}],{:.17e}[{:08x}]),point=({:.17e}[{:08x}],{:.17e}[{:08x}],{:.17e}[{:08x}]),engine={:.17e}[{:08x}],brake={:.17e}[{:08x}],lat={:.17e}[{:08x}],long={:.17e}[{:08x}],impulse=({:.17e}[{:08x}],{:.17e}[{:08x}],{:.17e}[{:08x}])",
                    trace_call,
                    self.chassis_body_idx,
                    wheel_idx,
                    wheel.axle_dir.x,
                    wheel.axle_dir.x.to_bits(),
                    wheel.axle_dir.y,
                    wheel.axle_dir.y.to_bits(),
                    wheel.axle_dir.z,
                    wheel.axle_dir.z.to_bits(),
                    info.contact_normal.x,
                    info.contact_normal.x.to_bits(),
                    info.contact_normal.y,
                    info.contact_normal.y.to_bits(),
                    info.contact_normal.z,
                    info.contact_normal.z.to_bits(),
                    info.contact_point.x,
                    info.contact_point.x.to_bits(),
                    info.contact_point.y,
                    info.contact_point.y.to_bits(),
                    info.contact_point.z,
                    info.contact_point.z.to_bits(),
                    wheel.engine_force,
                    wheel.engine_force.to_bits(),
                    wheel.brake,
                    wheel.brake.to_bits(),
                    wheel.lat_friction,
                    wheel.lat_friction.to_bits(),
                    wheel.long_friction,
                    wheel.long_friction.to_bits(),
                    impulse.x,
                    impulse.x.to_bits(),
                    impulse.y,
                    impulse.y.to_bits(),
                    impulse.z,
                    impulse.z.to_bits()
                );
            }
            if std::env::var("RSIM_DEBUG_WHEEL_AFTER").is_ok() {
                println!(
                    "rust_wheel_friction,body={},wheel={},static={},lat={},long={},engine={},brake={},impulse={:?}",
                    self.chassis_body_idx,
                    wheel_idx,
                    ground.is_static_obj(),
                    wheel.lat_friction,
                    wheel.long_friction,
                    wheel.engine_force,
                    wheel.brake,
                    impulse,
                );
            }
        }
    }

    /// Calculate friction after the car has set engine/brake/handbrake state,
    /// then apply suspension and friction impulses. This is the second half
    /// of RocketSim v2's updateVehicleFirst/updateVehicleSecond sequence.
    pub fn update_second(&mut self, collision_world: &mut DiscreteDynamicsWorld, time_step: f32) {
        let chassis_idx = self.chassis_body_idx;
        let chassis = &mut collision_world.bodies_mut()[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            if std::env::var("RSIM_DEBUG_WHEEL_AFTER").is_ok() {
                if let Some(info) = wheel.raycast_info.as_ref() {
                    println!(
                        "rust_wheel_susp_input,body={},susp={},susp_rel={},clip={},force_scale={},extra={},normal={:?}",
                        self.chassis_body_idx,
                        info.suspension_length,
                        info.suspension_relative_vel,
                        info.clipped_inv_contact_dot_suspension,
                        wheel.suspension_force_scale,
                        wheel.extra_pushback,
                        info.contact_normal,
                    );
                }
            }
            wheel.update_suspension(chassis, time_step);
        }

        if std::env::var("RSIM_DEBUG_WHEEL").is_ok() {
            println!(
                "rust_vehicle_suspension_post,body={},pos={:?},lin={:?},ang={:?},up={:?}",
                self.chassis_body_idx,
                chassis.get_world_trans().translation,
                chassis.lin_vel,
                chassis.ang_vel,
                chassis.get_world_trans().matrix3.z_axis
            );
        }

        // note: all suspension MUST be updated before impulses are applied
        for wheel in &mut self.wheels {
            wheel.apply_friction_impulses(chassis, time_step);
        }

        if std::env::var("RSIM_DEBUG_WHEEL").is_ok() {
            println!(
                "rust_vehicle_second,body={},pos={:?},lin={:?},ang={:?},up={:?}",
                self.chassis_body_idx,
                chassis.get_world_trans().translation,
                chassis.lin_vel,
                chassis.ang_vel,
                chassis.get_world_trans().matrix3.z_axis
            );
        }
    }

    /// Refresh per-wheel friction coefficients after the car's current
    /// controls have been processed. This mirrors V2's `_UpdateWheels`; the
    /// impulse itself was already computed in `update_first`.
    pub fn update_friction_coefficients(
        &mut self,
        collision_world: &DiscreteDynamicsWorld,
        handbrake_val: f32,
        real_throttle: f32,
        three_wheels: bool,
    ) {
        for wheel in &mut self.wheels {
            if wheel.raycast_info.is_none() {
                continue;
            }
            let chassis = &collision_world.bodies()[self.chassis_body_idx];
            wheel.update_friction_coefficients(chassis, handbrake_val, real_throttle, three_wheels);
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        collision_world: &mut DiscreteDynamicsWorld,
        time_step: f32,
        _handbrake_val: f32,
        _real_throttle: f32,
        _three_wheels: bool,
    ) {
        self.update_first(collision_world, time_step);
        self.update_second(collision_world, time_step);
    }
}
