use glam::{Affine3A, Vec3A};

use crate::bullet::dynamics::rigid_body::RigidBody;

pub struct SolverBody {
    pub world_trans: Affine3A,
    pub delta_linear_velocity: Vec3A,
    pub delta_angular_velocity: Vec3A,
    pub inv_mass: Vec3A,
    pub push_velocity: Vec3A,
    pub turn_velocity: Vec3A,
    pub linear_velocity: Vec3A,
    pub angular_velocity: Vec3A,
    pub external_force_impulse: Vec3A,
    pub external_torque_impulse: Vec3A,
    pub original_body: Option<usize>,
}

impl SolverBody {
    pub const DEFAULT: Self = Self {
        world_trans: Affine3A::IDENTITY,
        delta_linear_velocity: Vec3A::ZERO,
        delta_angular_velocity: Vec3A::ZERO,
        inv_mass: Vec3A::ZERO,
        push_velocity: Vec3A::ZERO,
        turn_velocity: Vec3A::ZERO,
        linear_velocity: Vec3A::ZERO,
        angular_velocity: Vec3A::ZERO,
        external_force_impulse: Vec3A::ZERO,
        external_torque_impulse: Vec3A::ZERO,
        original_body: None,
    };

    pub fn new(rb: &RigidBody, time_step: f32) -> Self {
        Self {
            world_trans: *rb.get_world_trans(),
            delta_linear_velocity: Vec3A::ZERO,
            delta_angular_velocity: Vec3A::ZERO,
            inv_mass: rb.inv_mass,
            push_velocity: Vec3A::ZERO,
            turn_velocity: Vec3A::ZERO,
            linear_velocity: rb.linear_velocity,
            angular_velocity: rb.angular_velocity,
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
        self.delta_linear_velocity += linear_component * impulse_magnitude;
        self.delta_angular_velocity += angular_component * impulse_magnitude;
    }

    #[must_use]
    pub fn get_velocity_in_local_point_no_delta(&self, rel_pos: Vec3A) -> Vec3A {
        self.linear_velocity
            + self.external_force_impulse
            + (self.angular_velocity + self.external_torque_impulse).cross(rel_pos)
    }
}
