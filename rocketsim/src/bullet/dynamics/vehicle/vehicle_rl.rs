use glam::{Affine3A, Quat, Vec3A};

use super::{
    raycaster::{VehicleRaycaster, VehicleRaycasterResult},
    wheel_info::WheelInfo,
};
use crate::{
    bullet::{
        dynamics::{
            constraint_solver::contact_constraint::{
                resolve_single_bilateral, resolve_single_collision,
            },
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::RigidBody,
        },
        linear_math::QuatExt,
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
    pub const DEFAULT: Self = Self {
        wheel_info: WheelInfo::DEFAULT,
        is_in_contact_with_world: false,
        steer_angle: 0.0,
        vel_at_contact_point: Vec3A::ZERO,
        lat_friction: 1.0,
        long_friction: 1.0,
        impulse: Vec3A::ZERO,
        suspsension_force_scale: 1.0,
        extra_pushback: 0.0,
    };

    pub fn set_params(
        &mut self,
        chassis_connection_cs: Vec3A,
        suspension_rest_length: f32,
        wheel_radius: f32,
        suspsension_force_scale: f32,
    ) {
        self.wheel_info.chassis_connection_point_cs = chassis_connection_cs;
        self.wheel_info.suspension_rest_length_1 = suspension_rest_length;
        self.wheel_info.wheels_radius = wheel_radius;
        self.suspsension_force_scale = suspsension_force_scale;
    }

    fn update_wheel_trans_ws(&mut self, chassis_trans: &Affine3A) {
        self.is_in_contact_with_world = false;
        self.wheel_info.raycast_info.is_in_contact = false;
        self.wheel_info.raycast_info.hard_point_ws =
            chassis_trans.transform_point3a(self.wheel_info.chassis_connection_point_cs);

        self.wheel_info.raycast_info.wheel_direction_ws = -chassis_trans.matrix3.z_axis;
        self.wheel_info.raycast_info.wheel_axle_ws = -chassis_trans.matrix3.y_axis;
    }

    fn update_wheel_trans<const FRONT: bool>(&mut self, cb_co: &RigidBody) {
        self.update_wheel_trans_ws(cb_co.get_world_trans());
        let right = self.wheel_info.raycast_info.wheel_axle_ws;

        self.wheel_info.axle_dir = if FRONT {
            let up = -self.wheel_info.raycast_info.wheel_direction_ws;
            let steering_orn = Quat::from_axis_angle_simd(up, self.steer_angle);

            steering_orn * -right
        } else {
            -right
        };
    }

    fn prepare_for_raycast(&mut self) -> (Vec3A, Vec3A, f32) {
        let suspension_travel = bullet_vehicle::MAX_SUSPENSION_TRAVEL * UU_TO_BT;
        let real_ray_length = self.wheel_info.suspension_rest_length_1
            + suspension_travel
            + self.wheel_info.wheels_radius
            - SUSPENSION_SUBTRACTION;

        let source = self.wheel_info.raycast_info.hard_point_ws;
        let target = source + (self.wheel_info.raycast_info.wheel_direction_ws * real_ray_length);
        self.wheel_info.raycast_info.contact_point_ws = target;
        self.wheel_info.raycast_info.ground_obj = None;

        (source, target, suspension_travel)
    }

    fn reset_wheel_suspension(&mut self, suspension_travel: f32) {
        self.wheel_info.raycast_info.suspension_length =
            self.wheel_info.suspension_rest_length_1 + suspension_travel;
        self.wheel_info.suspension_relative_vel = 0.0;
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
        let co = &ray_results.rigid_body;
        self.wheel_info.raycast_info.contact_point_ws = ray_results.hit_point_in_world;
        self.wheel_info.raycast_info.contact_normal_ws = ray_results.hit_normal_in_world;
        self.wheel_info.raycast_info.is_in_contact = true;
        self.is_in_contact_with_world = co.is_static_obj();

        self.wheel_info.raycast_info.ground_obj = Some(co.world_array_idx);

        let up = chassis.get_world_trans().matrix3.z_axis;
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

        let rel_pos =
            self.wheel_info.raycast_info.contact_point_ws - chassis.get_world_trans().translation;
        self.vel_at_contact_point = chassis.get_vel_in_local_point(rel_pos);

        let proj_vel = self
            .wheel_info
            .raycast_info
            .contact_normal_ws
            .dot(self.vel_at_contact_point);
        let denom = self.wheel_info.raycast_info.contact_normal_ws.dot(up);

        if denom > 0.1 {
            let inv = 1.0 / denom;
            self.wheel_info.suspension_relative_vel = proj_vel * inv;
            self.wheel_info.clipped_inv_contact_dot_suspension = inv;
        } else {
            self.wheel_info.suspension_relative_vel = 0.0;
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
        let Some(ground_rb_idx) = self.wheel_info.raycast_info.ground_obj else {
            self.impulse = Vec3A::ZERO;
            return;
        };
        let ground_rb = &bodies[ground_rb_idx];

        let mut axle_dir = self.wheel_info.axle_dir;
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
                let car_rel_contact_point = contact_point - chassis.get_world_trans().translation;

                let v1 = chassis.get_vel_in_local_point(car_rel_contact_point);
                let v2 = ground_rb.get_vel_in_local_point(car_rel_contact_point);
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

        let damping_vel_scale = if self.wheel_info.suspension_relative_vel < 0.0 {
            bullet_vehicle::WHEELS_DAMPING_COMPRESSION
        } else {
            bullet_vehicle::WHEELS_DAMPING_RELAXATION
        };

        self.wheel_info.wheels_suspension_force =
            force - (damping_vel_scale * self.wheel_info.suspension_relative_vel);

        self.wheel_info.wheels_suspension_force *= self.suspsension_force_scale;
        self.wheel_info.wheels_suspension_force = self.wheel_info.wheels_suspension_force.max(0.0);

        if self.wheel_info.wheels_suspension_force == 0.0 {
            return;
        }

        let base_force_scale =
            self.wheel_info.wheels_suspension_force * delta_time + self.extra_pushback;
        let contact_point_offset =
            self.wheel_info.raycast_info.contact_point_ws - cb.get_world_trans().translation;

        let force = self.wheel_info.raycast_info.contact_normal_ws * base_force_scale;
        cb.apply_impulse(force, contact_point_offset);
    }

    fn apply_friction_impulses(&self, cb: &mut RigidBody, time_step: f32) {
        if self.impulse == Vec3A::ZERO {
            return;
        }

        let trans = cb.get_world_trans();
        let wheel_contact_offset =
            self.wheel_info.raycast_info.contact_point_ws - trans.translation;
        let contact_up_dot = trans.matrix3.z_axis.dot(wheel_contact_offset);
        let wheel_rel_pos = wheel_contact_offset - trans.matrix3.z_axis * contact_up_dot;
        cb.apply_impulse(self.impulse * time_step, wheel_rel_pos);
    }
}

pub struct VehicleRL {
    raycaster: VehicleRaycaster,
    chassis_body_idx: usize,
    pub wheels: [WheelInfoRL; NUM_WHEELS],
}

impl VehicleRL {
    pub const fn new(
        chassis_body_idx: usize,
        wheels: [WheelInfoRL; NUM_WHEELS],
        raycaster: VehicleRaycaster,
    ) -> Self {
        Self {
            raycaster,
            chassis_body_idx,
            wheels,
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
