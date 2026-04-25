use glam::{Affine3A, Quat, Vec3A};

use super::{NUM_WHEELS, raycaster::VehicleRaycasterResult};
use crate::{
    bullet::{
        dynamics::{
            constraint_solver::contact_constraint::{
                resolve_single_bilateral, resolve_single_collision,
            },
            rigid_body::RigidBody,
        },
        linear_math::QuatExt,
    },
    consts::{UU_TO_BT, bullet_vehicle},
};

pub struct RaycastInfo {
    pub contact_normal_ws: Vec3A,
    pub contact_point_ws: Vec3A,
    pub suspension_length: f32,
    pub hard_point_ws: Vec3A,
    pub wheel_direction_ws: Vec3A,
    pub wheel_axle_ws: Vec3A,
    pub is_in_contact: bool,
    pub ground_obj: Option<usize>,
}

impl RaycastInfo {
    const DEFAULT: Self = Self {
        contact_normal_ws: Vec3A::ZERO,
        contact_point_ws: Vec3A::ZERO,
        suspension_length: 0.0,
        hard_point_ws: Vec3A::ZERO,
        wheel_direction_ws: Vec3A::ZERO,
        wheel_axle_ws: Vec3A::ZERO,
        is_in_contact: false,
        ground_obj: None,
    };
}

pub struct WheelInfo {
    pub raycast_info: RaycastInfo,
    pub axle_dir: Vec3A,
    pub chassis_connection_point_cs: Vec3A,
    pub suspension_rest_length_1: f32,
    pub wheels_radius: f32,
    pub engine_force: f32,
    pub brake: f32,
    pub clipped_inv_contact_dot_suspension: f32,
    pub suspension_relative_vel: f32,
    pub wheels_suspension_force: f32,
    pub is_in_contact_with_world: bool,
    pub steer_angle: f32,
    pub vel_at_contact_point: Vec3A,
    pub lat_friction: f32,
    pub long_friction: f32,
    pub impulse: Option<Vec3A>,
    pub suspsension_force_scale: f32,
    pub extra_pushback: f32,
}

impl WheelInfo {
    pub const DEFAULT: Self = Self {
        chassis_connection_point_cs: Vec3A::ZERO,
        suspension_rest_length_1: 0.0,
        wheels_radius: 0.0,
        engine_force: 0.0,
        brake: 0.0,
        raycast_info: RaycastInfo::DEFAULT,
        axle_dir: Vec3A::ZERO,
        clipped_inv_contact_dot_suspension: 0.0,
        suspension_relative_vel: 0.0,
        wheels_suspension_force: 0.0,
        is_in_contact_with_world: false,
        steer_angle: 0.0,
        vel_at_contact_point: Vec3A::ZERO,
        lat_friction: 1.0,
        long_friction: 1.0,
        impulse: None,
        suspsension_force_scale: 1.0,
        extra_pushback: 0.0,
    };

    pub const fn set_params(
        &mut self,
        chassis_connection_cs: Vec3A,
        suspension_rest_length: f32,
        wheel_radius: f32,
        suspsension_force_scale: f32,
    ) {
        self.chassis_connection_point_cs = chassis_connection_cs;
        self.suspension_rest_length_1 = suspension_rest_length;
        self.wheels_radius = wheel_radius;
        self.suspsension_force_scale = suspsension_force_scale;
    }

    fn update_wheel_trans_ws(&mut self, chassis_trans: &Affine3A) {
        self.is_in_contact_with_world = false;
        self.raycast_info.is_in_contact = false;
        self.raycast_info.hard_point_ws =
            chassis_trans.transform_point3a(self.chassis_connection_point_cs);

        self.raycast_info.wheel_direction_ws = -chassis_trans.matrix3.z_axis;
        self.raycast_info.wheel_axle_ws = -chassis_trans.matrix3.y_axis;
    }

    pub fn update_wheel_trans<const FRONT: bool>(&mut self, cb_co: &RigidBody) {
        self.update_wheel_trans_ws(cb_co.get_world_trans());
        let right = self.raycast_info.wheel_axle_ws;

        self.axle_dir = if FRONT {
            let up = -self.raycast_info.wheel_direction_ws;
            let steering_orn = Quat::from_axis_angle_simd(up, self.steer_angle);

            steering_orn * -right
        } else {
            -right
        };
    }

    pub fn prepare_for_raycast(&mut self) -> (Vec3A, Vec3A, f32) {
        let suspension_travel = bullet_vehicle::MAX_SUSPENSION_TRAVEL * UU_TO_BT;
        let real_ray_length =
            self.suspension_rest_length_1 + suspension_travel + self.wheels_radius
                - bullet_vehicle::SUSPENSION_SUBTRACTION;

        let source = self.raycast_info.hard_point_ws;
        let target = source + (self.raycast_info.wheel_direction_ws * real_ray_length);
        self.raycast_info.contact_point_ws = target;
        self.raycast_info.ground_obj = None;

        (source, target, suspension_travel)
    }

    pub fn reset_wheel_suspension(&mut self, suspension_travel: f32) {
        self.raycast_info.suspension_length = self.suspension_rest_length_1 + suspension_travel;
        self.suspension_relative_vel = 0.0;
        self.raycast_info.contact_normal_ws = -self.raycast_info.wheel_direction_ws;
        self.clipped_inv_contact_dot_suspension = 1.0;
        self.extra_pushback = 0.0;
    }

    pub fn apply_ray_cast(
        &mut self,
        chassis: &RigidBody,
        suspension_travel: f32,
        ray_results: VehicleRaycasterResult,
        time_step: f32,
    ) {
        let co = &ray_results.rigid_body;
        self.raycast_info.contact_point_ws = ray_results.hit_point_in_world;
        self.raycast_info.contact_normal_ws = ray_results.hit_normal_in_world;
        self.raycast_info.is_in_contact = true;
        self.is_in_contact_with_world = co.is_static_obj();

        self.raycast_info.ground_obj = Some(co.world_array_idx);

        let up = chassis.get_world_trans().matrix3.z_axis;
        let wheel_trace_len_sq =
            (self.raycast_info.hard_point_ws - self.raycast_info.contact_point_ws).dot(up);
        self.raycast_info.suspension_length = wheel_trace_len_sq - self.wheels_radius;

        let min_suspension_len = self.suspension_rest_length_1 - suspension_travel;
        let max_suspension_len = self.suspension_rest_length_1 + suspension_travel;
        self.raycast_info.suspension_length = self
            .raycast_info
            .suspension_length
            .clamp(min_suspension_len, max_suspension_len);

        let rel_pos = self.raycast_info.contact_point_ws - chassis.get_world_trans().translation;
        self.vel_at_contact_point = chassis.get_vel_in_local_point(rel_pos);

        let proj_vel = self
            .raycast_info
            .contact_normal_ws
            .dot(self.vel_at_contact_point);
        let denom = self.raycast_info.contact_normal_ws.dot(up);

        if denom > 0.1 {
            let inv = 1.0 / denom;
            self.suspension_relative_vel = proj_vel * inv;
            self.clipped_inv_contact_dot_suspension = inv;
        } else {
            self.suspension_relative_vel = 0.0;
            self.clipped_inv_contact_dot_suspension = 10.0;
        }

        if self.is_in_contact_with_world {
            let ray_pushback_thresh = self.suspension_rest_length_1 + self.wheels_radius
                - bullet_vehicle::SUSPENSION_SUBTRACTION;
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

    pub fn calc_friction_impulses(
        &mut self,
        chassis: &RigidBody,
        bodies: &[RigidBody],
        friction_scale: f32,
        time_step: f32,
    ) {
        let Some(ground_rb_idx) = self.raycast_info.ground_obj else {
            self.impulse = None;
            return;
        };
        let ground_rb = &bodies[ground_rb_idx];

        let mut axle_dir = self.axle_dir;
        let surf_normal_ws = self.raycast_info.contact_normal_ws;
        let proj = axle_dir.dot(surf_normal_ws);
        axle_dir -= surf_normal_ws * proj;
        axle_dir = axle_dir.normalize_or_zero();

        let forward_dir = surf_normal_ws.cross(axle_dir).normalize_or_zero();

        let side_impulse = resolve_single_bilateral(
            chassis,
            ground_rb,
            self.raycast_info.contact_point_ws,
            self.raycast_info.contact_point_ws,
            axle_dir,
        );

        let rolling_friction = if self.engine_force == 0.0 {
            if self.brake == 0.0 {
                0.0
            } else {
                const ROLLING_FRICTION_SCALE: f32 = 113.73963;

                let contact_point = self.raycast_info.contact_point_ws;
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

                (-rel_vel * ROLLING_FRICTION_SCALE).clamp(-self.brake, self.brake)
            }
        } else {
            -self.engine_force / friction_scale
        };

        let total_friction_force = forward_dir * rolling_friction * self.long_friction
            + axle_dir * side_impulse * self.lat_friction;

        self.impulse = Some(total_friction_force * friction_scale);
    }

    pub fn update_suspension(&mut self, cb: &mut RigidBody, delta_time: f32) {
        if !self.raycast_info.is_in_contact {
            self.wheels_suspension_force = 0.0;
            return;
        }

        let force = (self.suspension_rest_length_1 - self.raycast_info.suspension_length)
            * bullet_vehicle::SUSPENSION_STIFFNESS
            * self.clipped_inv_contact_dot_suspension;

        let damping_vel_scale = if self.suspension_relative_vel < 0.0 {
            bullet_vehicle::WHEELS_DAMPING_COMPRESSION
        } else {
            bullet_vehicle::WHEELS_DAMPING_RELAXATION
        };

        self.wheels_suspension_force = force - (damping_vel_scale * self.suspension_relative_vel);
        if self.wheels_suspension_force <= 0.0 {
            self.wheels_suspension_force = 0.0;
            return;
        }

        self.wheels_suspension_force *= self.suspsension_force_scale;
        let base_force_scale = self.wheels_suspension_force * delta_time + self.extra_pushback;
        let contact_point_offset =
            self.raycast_info.contact_point_ws - cb.get_world_trans().translation;

        let force = self.raycast_info.contact_normal_ws * base_force_scale;
        cb.apply_impulse(force, contact_point_offset);
    }

    pub fn apply_friction_impulses(&self, cb: &mut RigidBody, time_step: f32) {
        let Some(impulse) = self.impulse else {
            return;
        };

        let trans = cb.get_world_trans();
        let wheel_contact_offset = self.raycast_info.contact_point_ws - trans.translation;
        let contact_up_dot = trans.matrix3.z_axis.dot(wheel_contact_offset);
        let wheel_rel_pos = wheel_contact_offset - trans.matrix3.z_axis * contact_up_dot;
        cb.apply_impulse(impulse * time_step, wheel_rel_pos);
    }
}
