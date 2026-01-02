use arrayvec::ArrayVec;
use glam::{Affine3A, Mat3A, Quat, Vec3A};

use super::{
    raycaster::VehicleRaycaster,
    wheel_info::{WheelInfo, WheelInfoConstructionInfo},
};
use crate::{
    bullet::{
        collision::dispatch::collision_object::CollisionObject,
        dynamics::{
            constraint_solver::contact_constraint::{
                resolve_single_bilateral, resolve_single_collision,
            },
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::RigidBody,
            vehicle::raycaster::VehicleRaycasterResult,
        },
        linear_math::{Mat3AExt, QuatExt},
    },
    consts::{UU_TO_BT, bullet_vehicle},
    sim::consts::bullet_vehicle::SUSPENSION_SUBTRACTION,
};

const NUM_WHEELS: usize = 4;

pub struct WheelInfoRL {
    pub wheel_info: WheelInfo,
    pub is_in_contact_with_world: bool,
    pub steer_angle: f32,
    pub vel_at_contact_point: Vec3A,
    pub lat_friction: f32,
    pub long_friction: f32,
    pub impulse: Vec3A,
    pub suspsension_force_scale: f32,
    pub extra_pushback: f32,
}

impl WheelInfoRL {
    pub fn new(ci: WheelInfoConstructionInfo) -> Self {
        Self {
            wheel_info: WheelInfo::new(ci),
            is_in_contact_with_world: false,
            steer_angle: 0.0,
            vel_at_contact_point: Vec3A::ZERO,
            lat_friction: 0.0,
            long_friction: 0.0,
            impulse: Vec3A::ZERO,
            suspsension_force_scale: 0.0,
            extra_pushback: 0.0,
        }
    }

    fn update_wheel_transform_ws(&mut self, chassis_trans: &Affine3A) {
        self.is_in_contact_with_world = false;
        self.wheel_info.raycast_info.is_in_contact = false;
        self.wheel_info.raycast_info.hard_point_ws =
            chassis_trans.transform_point3a(self.wheel_info.chassis_connection_point_cs);
        self.wheel_info.raycast_info.wheel_direction_ws =
            chassis_trans.matrix3 * self.wheel_info.wheel_direction_cs;
        self.wheel_info.raycast_info.wheel_axle_ws =
            chassis_trans.matrix3 * self.wheel_info.wheel_axle_cs;
    }

    fn update_wheel_transform(&mut self, cb_co: &CollisionObject) {
        self.update_wheel_transform_ws(cb_co.get_world_transform());
        let up = -self.wheel_info.raycast_info.wheel_direction_ws;
        let right = self.wheel_info.raycast_info.wheel_axle_ws;
        let fwd = up.cross(right).normalize();

        let steering_orn = Quat::from_axis_angle_simd(up, self.steer_angle);
        let steering_mat = Mat3A::bullet_from_quat(steering_orn);

        let basis2 = Mat3A::from_cols(fwd, -right, up);
        self.wheel_info.world_transform = Affine3A {
            matrix3: steering_mat * basis2,
            translation: self.wheel_info.raycast_info.hard_point_ws
                + self.wheel_info.wheel_direction_cs
                    * self.wheel_info.raycast_info.suspension_length,
        }
    }

    fn prepare_for_raycast(&mut self, chassis: &RigidBody) -> (Vec3A, Vec3A, f32) {
        self.update_wheel_transform_ws(chassis.collision_object.get_world_transform());

        let suspension_travel = bullet_vehicle::MAX_SUSPENSION_TRAVEL * UU_TO_BT;
        let real_ray_length = self.wheel_info.suspension_rest_length_1
            + suspension_travel
            + self.wheel_info.wheels_radius
            - SUSPENSION_SUBTRACTION;

        let source = self.wheel_info.raycast_info.hard_point_ws;
        let target = source + (self.wheel_info.raycast_info.wheel_direction_ws * real_ray_length);
        self.wheel_info.raycast_info.contact_point_ws = target;
        self.wheel_info.raycast_info.ground_object = None;

        (source, target, suspension_travel)
    }

    fn reset_wheel_suspension(&mut self, suspension_travel: f32) {
        self.wheel_info.raycast_info.suspension_length =
            self.wheel_info.suspension_rest_length_1 + suspension_travel;
        self.wheel_info.suspension_relative_velocity = 0.0;
        self.wheel_info.raycast_info.contact_normal_ws =
            -self.wheel_info.raycast_info.wheel_direction_ws;
        self.wheel_info.clipped_inv_contact_dot_suspension = 1.0;
        self.extra_pushback = 0.0;
    }

    fn apply_ray_cast(
        &mut self,
        chassis: &RigidBody,
        suspension_travel: f32,
        ray_results: VehicleRaycasterResult,
        time_step: f32,
    ) {
        let co = &ray_results.rigid_body.collision_object;
        self.wheel_info.raycast_info.contact_point_ws = ray_results.hit_point_in_world;
        self.wheel_info.raycast_info.contact_normal_ws = ray_results.hit_normal_in_world;
        self.wheel_info.raycast_info.is_in_contact = true;
        self.is_in_contact_with_world = co.is_static_object();

        self.wheel_info.raycast_info.ground_object = Some(co.world_array_index);

        let up = chassis
            .collision_object
            .get_world_transform()
            .matrix3
            .z_axis;
        let wheel_trace_len_sq = (self.wheel_info.raycast_info.hard_point_ws
            - self.wheel_info.raycast_info.contact_point_ws)
            .dot(up);
        self.wheel_info.raycast_info.suspension_length =
            wheel_trace_len_sq - self.wheel_info.wheels_radius;

        let min_suspension_len = self.wheel_info.suspension_rest_length_1 - suspension_travel;
        let max_suspension_len = self.wheel_info.suspension_rest_length_1 + suspension_travel;
        self.wheel_info.raycast_info.suspension_length = self
            .wheel_info
            .raycast_info
            .suspension_length
            .clamp(min_suspension_len, max_suspension_len);

        let rel_pos = self.wheel_info.raycast_info.contact_point_ws
            - chassis.collision_object.get_world_transform().translation;
        self.vel_at_contact_point = chassis.get_velocity_in_local_point(rel_pos);

        let proj_vel = self
            .wheel_info
            .raycast_info
            .contact_normal_ws
            .dot(self.vel_at_contact_point);
        let denom = self.wheel_info.raycast_info.contact_normal_ws.dot(up);

        if denom > 0.1 {
            let inv = 1.0 / denom;
            self.wheel_info.suspension_relative_velocity = proj_vel * inv;
            self.wheel_info.clipped_inv_contact_dot_suspension = inv;
        } else {
            self.wheel_info.suspension_relative_velocity = 0.0;
            self.wheel_info.clipped_inv_contact_dot_suspension = 10.0;
        }

        if self.is_in_contact_with_world {
            let ray_pushback_thresh = self.wheel_info.suspension_rest_length_1
                + self.wheel_info.wheels_radius
                - SUSPENSION_SUBTRACTION;
            if wheel_trace_len_sq < ray_pushback_thresh {
                let wheel_trace_dist_delta = wheel_trace_len_sq - ray_pushback_thresh;

                let collision_result = resolve_single_collision(
                    chassis,
                    ray_results.rigid_body,
                    ray_results.hit_point_in_world,
                    ray_results.hit_normal_in_world,
                    time_step,
                    wheel_trace_dist_delta,
                );

                self.extra_pushback = collision_result / NUM_WHEELS as f32;
            }
        }
    }

    fn calc_friction_impulses(
        &mut self,
        chassis: &RigidBody,
        bodies: &[RigidBody],
        friction_scale: f32,
        time_step: f32,
    ) {
        let Some(ground_rb_index) = self.wheel_info.raycast_info.ground_object else {
            self.impulse = Vec3A::ZERO;
            return;
        };
        let ground_rb = &bodies[ground_rb_index];

        let mut axle_dir = self.wheel_info.world_transform.matrix3.y_axis;
        let surf_normal_ws = self.wheel_info.raycast_info.contact_normal_ws;
        let proj = axle_dir.dot(surf_normal_ws);
        axle_dir -= surf_normal_ws * proj;
        axle_dir = axle_dir.normalize_or_zero();

        let forward_dir = surf_normal_ws.cross(axle_dir).normalize_or_zero();

        let side_impulse = resolve_single_bilateral(
            chassis,
            ground_rb,
            self.wheel_info.raycast_info.contact_point_ws,
            self.wheel_info.raycast_info.contact_point_ws,
            axle_dir,
        );

        let rolling_friction = if self.wheel_info.engine_force == 0.0 {
            if self.wheel_info.brake == 0.0 {
                0.0
            } else {
                const ROLLING_FRICTION_SCALE: f32 = 113.73963;

                let contact_point = self.wheel_info.raycast_info.contact_point_ws;
                let car_rel_contact_point =
                    contact_point - chassis.collision_object.get_world_transform().translation;

                let v1 = chassis.get_velocity_in_local_point(car_rel_contact_point);
                let v2 = ground_rb.get_velocity_in_local_point(car_rel_contact_point);
                let contact_vel = v1 - v2;
                let mut rel_vel = contact_vel.dot(forward_dir);

                if time_step > 1.0 / 80.0 {
                    let threshold = 0.8 - (1.0 / (time_step * 150.0));
                    if rel_vel.abs() < threshold {
                        rel_vel = 0.0;
                    }
                }

                (-rel_vel * ROLLING_FRICTION_SCALE)
                    .clamp(-self.wheel_info.brake, self.wheel_info.brake)
            }
        } else {
            -self.wheel_info.engine_force / friction_scale
        };

        let total_friction_force = forward_dir * rolling_friction * self.long_friction
            + axle_dir * side_impulse * self.lat_friction;

        self.impulse = total_friction_force * friction_scale;
    }

    fn update_suspension(&mut self, cb: &mut RigidBody, delta_time: f32) {
        if !self.wheel_info.raycast_info.is_in_contact {
            self.wheel_info.wheels_suspension_force = 0.0;
            return;
        }

        let force = (self.wheel_info.suspension_rest_length_1
            - self.wheel_info.raycast_info.suspension_length)
            * bullet_vehicle::SUSPENSION_STIFFNESS
            * self.wheel_info.clipped_inv_contact_dot_suspension;

        let damping_vel_scale = if self.wheel_info.suspension_relative_velocity < 0.0 {
            bullet_vehicle::WHEELS_DAMPING_COMPRESSION
        } else {
            bullet_vehicle::WHEELS_DAMPING_RELAXATION
        };

        self.wheel_info.wheels_suspension_force =
            force - (damping_vel_scale * self.wheel_info.suspension_relative_velocity);

        self.wheel_info.wheels_suspension_force *= self.suspsension_force_scale;
        self.wheel_info.wheels_suspension_force = self.wheel_info.wheels_suspension_force.max(0.0);

        if self.wheel_info.wheels_suspension_force == 0.0 {
            return;
        }

        let base_force_scale =
            self.wheel_info.wheels_suspension_force * delta_time + self.extra_pushback;
        let contact_point_offset = self.wheel_info.raycast_info.contact_point_ws
            - cb.collision_object.get_world_transform().translation;

        let force = self.wheel_info.raycast_info.contact_normal_ws * base_force_scale;
        cb.apply_impulse(force, contact_point_offset);
    }

    fn apply_friction_impulses(&self, cb: &mut RigidBody, time_step: f32) {
        if self.impulse == Vec3A::ZERO {
            return;
        }

        let trans = cb.collision_object.get_world_transform();
        let wheel_contact_offset =
            self.wheel_info.raycast_info.contact_point_ws - trans.translation;
        let contact_up_dot = trans.matrix3.z_axis.dot(wheel_contact_offset);
        let wheel_rel_pos = wheel_contact_offset - trans.matrix3.z_axis * contact_up_dot;
        cb.apply_impulse(self.impulse * time_step, wheel_rel_pos);
    }
}

pub struct VehicleRL {
    // forward_ws: Vec<Vec3A>,
    // axle: Vec<Vec3A>,
    // forward_impulse: Vec<f32>,
    // side_impulse: Vec<f32>,
    raycaster: VehicleRaycaster,
    // pitch_control: f32,
    // steering_value: f32,
    pub chassis_body_idx: usize,
    pub wheels: ArrayVec<WheelInfoRL, NUM_WHEELS>,
}

impl VehicleRL {
    pub fn new(chassis_body_idx: usize, raycaster: VehicleRaycaster) -> Self {
        Self {
            raycaster,
            chassis_body_idx,
            // forward_ws: Vec::new(),
            // axle: Vec::new(),
            // forward_impulse: Vec::new(),
            // side_impulse: Vec::new(),
            // pitch_control: 0.0,
            // steering_value: 0.0,
            wheels: ArrayVec::new(),
        }
    }

    pub fn get_upwards_dir_from_wheel_contacts(&self, cb: &RigidBody) -> Vec3A {
        let mut sum_contact_dir = Vec3A::ZERO;
        for wheel in &self.wheels {
            if wheel.wheel_info.raycast_info.is_in_contact {
                sum_contact_dir += wheel.wheel_info.raycast_info.contact_normal_ws;
            }
        }

        if sum_contact_dir == Vec3A::ZERO {
            cb.get_up_vector()
        } else {
            sum_contact_dir.normalize_or_zero()
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn add_wheel(
        &mut self,
        cb_co: &CollisionObject,
        connection_point_cs: Vec3A,
        wheel_direction_cs0: Vec3A,
        wheel_axle_cs: Vec3A,
        suspension_rest_length: f32,
        wheel_radius: f32,
    ) {
        let ci = WheelInfoConstructionInfo {
            chassis_connection_cs: connection_point_cs,
            wheel_direction_cs: wheel_direction_cs0,
            wheel_axle_cs,
            suspension_rest_length,
            wheel_radius,
        };

        let mut wheel = WheelInfoRL::new(ci);
        wheel.update_wheel_transform(cb_co);
        self.wheels.push(wheel);
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

        let friction_scale = chassis.get_mass() / 3.0;
        for wheel in &mut self.wheels {
            wheel.update_wheel_transform(&chassis.collision_object);
        }

        let mut sources = [Vec3A::ZERO; 4];
        let mut targets = [Vec3A::ZERO; 4];
        let mut suspension_travels = [0.0; 4];

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            (sources[i], targets[i], suspension_travels[i]) = wheel.prepare_for_raycast(chassis);
        }

        let ray_results = self.raycaster.cast_rays(
            collision_world,
            &sources,
            &targets,
            &chassis.collision_object,
        );

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            if let Some(ray_result) = ray_results[i] {
                wheel.apply_ray_cast(chassis, suspension_travels[i], ray_result, time_step);
            } else {
                wheel.reset_wheel_suspension(suspension_travels[i]);
            }
        }

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
