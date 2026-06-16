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
    pub contact_normal: Vec3A,
    pub contact_point: Vec3A,
    pub suspension_length: f32,
    pub impulse: Vec3A,
    pub is_in_contact_with_world: bool,
    pub clipped_inv_contact_dot_suspension: f32,
    pub suspension_relative_vel: f32,
}

pub struct WheelInfo {
    pub raycast_info: Option<RaycastInfo>,
    pub hard_point: Vec3A,
    pub axle_dir: Vec3A,
    pub chassis_connection_point_cs: Vec3A,
    pub suspension_rest_length_1: f32,
    pub wheels_radius: f32,
    pub engine_force: f32,
    pub brake: f32,
    pub steer_angle: f32,
    pub vel_at_contact_point: Vec3A,
    pub lat_friction: f32,
    pub long_friction: f32,
    pub suspsension_force_scale: f32,
    pub extra_pushback: f32,
    pub real_ray_length: f32,
}

impl WheelInfo {
    pub const DEFAULT: Self = Self {
        raycast_info: None,
        hard_point: Vec3A::ZERO,
        axle_dir: Vec3A::ZERO,
        chassis_connection_point_cs: Vec3A::ZERO,
        suspension_rest_length_1: 0.0,
        wheels_radius: 0.0,
        engine_force: 0.0,
        brake: 0.0,
        steer_angle: 0.0,
        vel_at_contact_point: Vec3A::ZERO,
        lat_friction: 1.0,
        long_friction: 1.0,
        suspsension_force_scale: 1.0,
        extra_pushback: 0.0,
        real_ray_length: 0.0,
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

        let suspension_travel = bullet_vehicle::MAX_SUSPENSION_TRAVEL * UU_TO_BT;
        self.real_ray_length =
            self.suspension_rest_length_1 + suspension_travel + self.wheels_radius
                - bullet_vehicle::SUSPENSION_SUBTRACTION;
    }

    pub fn prepare_for_raycast(&mut self, chassis_trans: &Affine3A) -> (Vec3A, Vec3A) {
        self.hard_point = chassis_trans.transform_point3a(self.chassis_connection_point_cs);
        let target = self.hard_point - (chassis_trans.matrix3.z_axis * self.real_ray_length);

        (self.hard_point, target)
    }

    pub fn reset_wheel_suspension(&mut self) {
        self.extra_pushback = 0.0;
        self.raycast_info = None;
    }

    pub fn apply_ray_cast(
        &mut self,
        chassis: &RigidBody,
        ray_results: VehicleRaycasterResult,
        time_step: f32,
        front: bool,
    ) {
        let contact_point = ray_results.hit_point_in_world;
        let contact_normal = ray_results.hit_normal_in_world;
        let is_in_contact_with_world = ray_results.rigid_body.is_static_obj();

        let chassis_trans = chassis.get_world_trans();
        self.axle_dir = if front {
            Quat::from_axis_angle_simd(chassis_trans.matrix3.z_axis, self.steer_angle)
                * chassis_trans.matrix3.y_axis
        } else {
            chassis_trans.matrix3.y_axis
        };

        let up = chassis_trans.matrix3.z_axis;
        let wheel_trace_len_sq = (self.hard_point - contact_point).dot(up);
        let mut suspension_length = wheel_trace_len_sq - self.wheels_radius;

        let suspension_travel = bullet_vehicle::MAX_SUSPENSION_TRAVEL * UU_TO_BT;
        let min_suspension_len = self.suspension_rest_length_1 - suspension_travel;
        let max_suspension_len = self.suspension_rest_length_1 + suspension_travel;
        suspension_length = suspension_length.clamp(min_suspension_len, max_suspension_len);

        let rel_pos = contact_point - chassis_trans.translation;
        self.vel_at_contact_point = chassis.get_vel_in_local_point(rel_pos);

        let proj_vel = contact_normal.dot(self.vel_at_contact_point);
        let denom = contact_normal.dot(up);

        let suspension_relative_vel;
        let clipped_inv_contact_dot_suspension;
        if denom > 0.1 {
            let inv = 1.0 / denom;
            suspension_relative_vel = proj_vel * inv;
            clipped_inv_contact_dot_suspension = inv;
        } else {
            suspension_relative_vel = 0.0;
            clipped_inv_contact_dot_suspension = 10.0;
        }

        if is_in_contact_with_world {
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

        let impulse = self.calc_friction_impulses(
            chassis,
            ray_results.rigid_body,
            contact_normal,
            contact_point,
            time_step,
        );

        self.raycast_info = Some(RaycastInfo {
            contact_normal,
            contact_point,
            suspension_length,
            impulse,
            is_in_contact_with_world,
            clipped_inv_contact_dot_suspension,
            suspension_relative_vel,
        });
    }

    pub fn calc_friction_impulses(
        &mut self,
        chassis: &RigidBody,
        ground_rb: &RigidBody,
        contact_normal: Vec3A,
        contact_point: Vec3A,
        time_step: f32,
    ) -> Vec3A {
        let friction_scale = chassis.get_mass() / 3.0;
        let mut axle_dir = self.axle_dir;
        let proj = axle_dir.dot(contact_normal);
        axle_dir -= contact_normal * proj;
        axle_dir = axle_dir.normalize_or_zero();

        let forward_dir = contact_normal.cross(axle_dir).normalize_or_zero();

        let side_impulse =
            resolve_single_bilateral(chassis, ground_rb, contact_point, contact_point, axle_dir);

        let rolling_friction = if self.engine_force == 0.0 {
            if self.brake == 0.0 {
                0.0
            } else {
                const ROLLING_FRICTION_SCALE: f32 = 113.73963;

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
        total_friction_force * friction_scale
    }

    pub fn update_suspension(&mut self, cb: &mut RigidBody, delta_time: f32) {
        let Some(raycast_info) = self.raycast_info.as_ref() else {
            return;
        };

        let force = (self.suspension_rest_length_1 - raycast_info.suspension_length)
            * bullet_vehicle::SUSPENSION_STIFFNESS
            * raycast_info.clipped_inv_contact_dot_suspension;

        let damping_vel_scale = if raycast_info.suspension_relative_vel < 0.0 {
            bullet_vehicle::WHEELS_DAMPING_COMPRESSION
        } else {
            bullet_vehicle::WHEELS_DAMPING_RELAXATION
        };

        let mut wheels_suspension_force =
            force - (damping_vel_scale * raycast_info.suspension_relative_vel);
        wheels_suspension_force *= self.suspsension_force_scale;
        if wheels_suspension_force <= 0.0 {
            return;
        }
        let base_force_scale = wheels_suspension_force * delta_time + self.extra_pushback;
        let contact_point_offset = raycast_info.contact_point - cb.get_world_trans().translation;

        let force = raycast_info.contact_normal * base_force_scale;
        cb.apply_impulse(force, contact_point_offset);
    }

    pub fn apply_friction_impulses(&self, cb: &mut RigidBody, time_step: f32) {
        let Some(raycast_info) = self.raycast_info.as_ref() else {
            return;
        };

        let trans = cb.get_world_trans();
        let wheel_contact_offset = raycast_info.contact_point - trans.translation;
        let contact_up_dot = trans.matrix3.z_axis.dot(wheel_contact_offset);
        let wheel_rel_pos = wheel_contact_offset - trans.matrix3.z_axis * contact_up_dot;
        cb.apply_impulse(raycast_info.impulse * time_step, wheel_rel_pos);
    }
}
