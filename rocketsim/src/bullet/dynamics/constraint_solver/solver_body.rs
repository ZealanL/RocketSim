use glam::{Affine3A, Vec3A};

use crate::bullet::dynamics::rigid_body::RigidBody;

pub struct SolverBody {
    pub world_trans: Affine3A,
    pub delta_lin_vel: Vec3A,
    pub delta_ang_vel: Vec3A,
    pub inv_mass: Vec3A,
    pub push_vel: Vec3A,
    pub turn_vel: Vec3A,
    pub lin_vel: Vec3A,
    pub ang_vel: Vec3A,
    pub external_force_impulse: Vec3A,
    pub external_torque_impulse: Vec3A,
    pub original_body: Option<usize>,
}

impl SolverBody {
    pub const DEFAULT: Self = Self {
        world_trans: Affine3A::IDENTITY,
        delta_lin_vel: Vec3A::ZERO,
        delta_ang_vel: Vec3A::ZERO,
        inv_mass: Vec3A::ZERO,
        push_vel: Vec3A::ZERO,
        turn_vel: Vec3A::ZERO,
        lin_vel: Vec3A::ZERO,
        ang_vel: Vec3A::ZERO,
        external_force_impulse: Vec3A::ZERO,
        external_torque_impulse: Vec3A::ZERO,
        original_body: None,
    };

    pub fn new(rb: &RigidBody, time_step: f32) -> Self {
        Self {
            world_trans: *rb.get_world_trans(),
            delta_lin_vel: Vec3A::ZERO,
            delta_ang_vel: Vec3A::ZERO,
            inv_mass: rb.inv_mass,
            push_vel: Vec3A::ZERO,
            turn_vel: Vec3A::ZERO,
            lin_vel: rb.lin_vel,
            ang_vel: rb.ang_vel,
            external_force_impulse: rb.total_force * rb.inverse_mass * time_step,
            external_torque_impulse: rb.inv_inertia_tensor_world * rb.total_torque * time_step,
            original_body: Some(rb.world_array_idx),
        }
    }

    pub fn internal_apply_impulse(
        &mut self,
        linear_component: Vec3A,
        angular_component: Vec3A,
        impulse_magnitude: f32,
    ) {
        self.delta_lin_vel += linear_component * impulse_magnitude;
        self.delta_ang_vel += angular_component * impulse_magnitude;
    }

    pub fn get_vel_in_local_point_no_delta(&self, rel_pos: Vec3A) -> Vec3A {
        self.lin_vel
            + self.external_force_impulse
            + (self.ang_vel + self.external_torque_impulse).cross(rel_pos)
    }
}
