use glam::Vec3A;

use crate::bullet::{
    dynamics::constraint_solver::solver_body::SolverBody, linear_math::plane_space_2,
};

#[derive(Debug, Default, Copy, Clone)]
pub struct ManifoldPoint {
    pub local_point_a: Vec3A,
    pub local_point_b: Vec3A,
    pub pos_world_on_b: Vec3A,
    pub pos_world_on_a: Vec3A,
    pub normal_world_on_b: Vec3A,
    pub distance_1: f32,
    pub combined_friction: f32,
    pub combined_restitution: f32,
    pub applied_impulse: f32,
    pub lateral_friction_dir_1: Vec3A,
    pub lateral_friction_dir_2: Vec3A,
    pub is_special: bool,
}

impl ManifoldPoint {
    pub const fn new(point_a: Vec3A, point_b: Vec3A, normal: Vec3A, distance: f32) -> Self {
        Self {
            local_point_a: point_a,
            local_point_b: point_b,
            pos_world_on_a: Vec3A::ZERO,
            pos_world_on_b: Vec3A::ZERO,
            normal_world_on_b: normal,
            distance_1: distance,
            combined_friction: 0.0,
            combined_restitution: 0.0,
            applied_impulse: 0.0,
            lateral_friction_dir_1: Vec3A::ZERO,
            lateral_friction_dir_2: Vec3A::ZERO,
            is_special: false,
        }
    }

    pub fn calc_lat_friction_dir(
        &mut self,
        solver_body_a: &SolverBody,
        solver_body_b: &SolverBody,
        rel_pos1: Vec3A,
        rel_pos2: Vec3A,
    ) {
        let vel1 = solver_body_a.get_vel_in_local_point_no_delta(rel_pos1);
        let vel2 = solver_body_b.get_vel_in_local_point_no_delta(rel_pos2);

        let vel = vel1 - vel2;
        let rel_vel = self.normal_world_on_b.dot(vel);

        self.lateral_friction_dir_1 = vel - self.normal_world_on_b * rel_vel;
        let lat_rel_vel = self.lateral_friction_dir_1.length_squared();

        if lat_rel_vel > f32::EPSILON {
            self.lateral_friction_dir_1 *= 1.0 / lat_rel_vel.sqrt();
        } else {
            (self.lateral_friction_dir_1, self.lateral_friction_dir_2) =
                plane_space_2(self.normal_world_on_b);
        }
    }
}
