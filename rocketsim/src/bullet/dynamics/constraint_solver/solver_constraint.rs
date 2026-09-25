use glam::Vec3A;

use super::{contact_solver_info, solver_body::SolverBody};
use crate::bullet::{
    collision::narrowphase::manifold_point::ManifoldPoint, dynamics::rigid_body::RigidBody,
};

fn bullet_dot(vec0: Vec3A, vec1: Vec3A) -> f32 {
    let result = vec0 * vec1;
    result.x + (result.y + result.z)
}

#[derive(Default)]
pub struct SolverConstraint {
    pub rel_pos1_cross_normal: Vec3A,
    pub contact_normal_1: Vec3A,
    pub rel_pos2_cross_normal: Vec3A,
    pub contact_normal_2: Vec3A,
    pub angular_component_a: Vec3A,
    pub angular_component_b: Vec3A,
    pub applied_push_impulse: f32,
    pub applied_impulse: f32,
    pub friction: f32,
    pub jac_diag_ab_inv: f32,
    pub rhs: f32,
    pub lower_limit: f32,
    pub upper_limit: f32,
    pub rhs_penetration: f32,
    pub friction_idx: usize,
    pub solver_body_id_a: usize,
    pub solver_body_id_b: usize,
    pub is_split_only: bool,
}

impl SolverConstraint {
    /// Sentinel parent for rows that own no friction child.
    pub const NO_FRICTION_PARENT: usize = usize::MAX;

    pub fn get_friction_constraint(
        (solver_body_id_a, solver_body_id_b): (usize, usize),
        (solver_body_a, solver_body_b): (&mut SolverBody, &mut SolverBody),
        (rb0, rb1): (Option<&RigidBody>, Option<&RigidBody>),
        (rel_pos1, rel_pos2): (Vec3A, Vec3A),
        friction: f32,
        normal_axis: Vec3A,
        friction_idx: usize,
    ) -> Self {
        let mut constraint = Self {
            friction_idx,
            solver_body_id_a,
            solver_body_id_b,
            lower_limit: -friction,
            upper_limit: friction,
            friction,
            ..Default::default()
        };

        constraint.setup_friction_constraint(
            (solver_body_a, solver_body_b),
            (rb0, rb1),
            (rel_pos1, rel_pos2),
            normal_axis,
        );

        constraint
    }

    #[allow(clippy::too_many_arguments)]
    pub fn get_contact_constraint(
        (solver_body_id_a, solver_body_id_b): (usize, usize),
        (solver_body_a, solver_body_b): (&mut SolverBody, &mut SolverBody),
        (rb0, rb1): (Option<&RigidBody>, Option<&RigidBody>),
        (rel_pos1, rel_pos2): (Vec3A, Vec3A),
        cp: &ManifoldPoint,
        friction_idx: usize,
        time_step: f32,
    ) -> Self {
        let mut constraint = Self {
            solver_body_id_a,
            solver_body_id_b,
            friction_idx,
            friction: cp.combined_friction,
            lower_limit: 0.0,
            upper_limit: 1e10,
            ..Default::default()
        };

        constraint.setup_contact_constraint(
            (solver_body_a, solver_body_b),
            (rb0, rb1),
            (rel_pos1, rel_pos2),
            cp,
            time_step,
            true,
        );

        constraint
    }

    pub(super) fn get_split_only_contact_constraint(
        (solver_body_id_a, solver_body_id_b): (usize, usize),
        (_solver_body_a, _solver_body_b): (&mut SolverBody, &mut SolverBody),
        (rb0, rb1): (Option<&RigidBody>, Option<&RigidBody>),
        (rel_pos1, rel_pos2): (Vec3A, Vec3A),
        cp: &ManifoldPoint,
        time_step: f32,
    ) -> Self {
        let mut constraint = Self {
            solver_body_id_a,
            solver_body_id_b,
            friction_idx: Self::NO_FRICTION_PARENT,
            friction: cp.combined_friction,
            lower_limit: 0.0,
            upper_limit: 1e10,
            is_split_only: true,
            ..Default::default()
        };

        constraint.setup_split_contact_constraint(rb0, rb1, rel_pos1, rel_pos2, cp, time_step);

        constraint
    }

    fn setup_split_contact_constraint(
        &mut self,
        rb0: Option<&RigidBody>,
        rb1: Option<&RigidBody>,
        rel_pos1: Vec3A,
        rel_pos2: Vec3A,
        cp: &ManifoldPoint,
        time_step: f32,
    ) {
        let denom0 = rb0.map_or(0.0, |rb| {
            let torque_axis = rel_pos1.cross(cp.normal_world_on_b);
            self.angular_component_a = rb.inv_inertia_tensor_world.mul_transpose_vec3a(torque_axis);
            let vec = self.angular_component_a.cross(rel_pos1);
            let denom = rb.inv_mass + cp.normal_world_on_b.dot(vec);
            self.contact_normal_1 = cp.normal_world_on_b;
            self.rel_pos1_cross_normal = torque_axis;
            denom
        });

        let denom1 = rb1.map_or(0.0, |rb| {
            let torque_axis = rel_pos2.cross(cp.normal_world_on_b);
            self.angular_component_b = rb
                .inv_inertia_tensor_world
                .mul_transpose_vec3a(-torque_axis);
            let vec = (-self.angular_component_b).cross(rel_pos2);
            let denom = rb.inv_mass + cp.normal_world_on_b.dot(vec);
            self.contact_normal_2 = -cp.normal_world_on_b;
            self.rel_pos2_cross_normal = -torque_axis;
            denom
        });

        self.jac_diag_ab_inv = contact_solver_info::SOR / (denom0 + denom1);

        let inv_time_step = 1.0 / time_step;
        let penetration = cp.distance_1;
        let positional_error = if penetration > 0.0 {
            0.0
        } else {
            -penetration * contact_solver_info::ERP_2 * inv_time_step
        };
        let penetration_impulse = positional_error * self.jac_diag_ab_inv;
        self.rhs_penetration =
            if penetration > contact_solver_info::SPLIT_IMPULSE_PENETRATION_THRESHOLD {
                0.0
            } else {
                penetration_impulse
            };
    }

    #[inline]
    pub(super) fn make_dynamic_side_a(&mut self, dynamic_is_b: bool) {
        if dynamic_is_b {
            std::mem::swap(&mut self.solver_body_id_a, &mut self.solver_body_id_b);
            std::mem::swap(
                &mut self.rel_pos1_cross_normal,
                &mut self.rel_pos2_cross_normal,
            );
            std::mem::swap(&mut self.contact_normal_1, &mut self.contact_normal_2);
            std::mem::swap(&mut self.angular_component_a, &mut self.angular_component_b);
        }
    }

    pub fn restitution_curve(rel_vel: f32, restitution: f32) -> f32 {
        (restitution * -rel_vel).max(0.0)
    }

    fn setup_contact_constraint(
        &mut self,
        (solver_body_a, solver_body_b): (&mut SolverBody, &mut SolverBody),
        (rb0, rb1): (Option<&RigidBody>, Option<&RigidBody>),
        (rel_pos1, rel_pos2): (Vec3A, Vec3A),
        cp: &ManifoldPoint,
        time_step: f32,
        apply_warmstart: bool,
    ) {
        let inv_time_step = 1.0 / time_step;
        let erp = contact_solver_info::ERP_2;
        self.applied_impulse = if apply_warmstart {
            cp.applied_impulse * contact_solver_info::WARMSTARTING_FACTOR
        } else {
            0.0
        };

        let (denom0, vel0, vel_1_dot_n) = rb0.map_or((0.0, Vec3A::ZERO, 0.0), |rb| {
            let torque_axis = rel_pos1.cross(cp.normal_world_on_b);
            self.angular_component_a = rb.inv_inertia_tensor_world.mul_transpose_vec3a(torque_axis);
            let vec = self.angular_component_a.cross(rel_pos1);
            let denom = rb.inv_mass + cp.normal_world_on_b.dot(vec);
            let vel = rb.get_vel_in_local_point(rel_pos1);

            self.contact_normal_1 = cp.normal_world_on_b;
            self.rel_pos1_cross_normal = torque_axis;

            if apply_warmstart {
                solver_body_a.internal_apply_impulse(
                    cp.normal_world_on_b * solver_body_a.inv_mass,
                    self.angular_component_a,
                    self.applied_impulse,
                );
            }

            let vel_dot_n = self
                .contact_normal_1
                .dot(solver_body_a.lin_vel + solver_body_a.external_force_impulse)
                + self
                    .rel_pos1_cross_normal
                    .dot(solver_body_a.ang_vel + solver_body_a.external_torque_impulse);

            (denom, vel, vel_dot_n)
        });

        let (denom1, vel1, vel_2_dot_n) = rb1.map_or((0.0, Vec3A::ZERO, 0.0), |rb| {
            let torque_axis = rel_pos2.cross(cp.normal_world_on_b);
            self.angular_component_b = rb
                .inv_inertia_tensor_world
                .mul_transpose_vec3a(-torque_axis);
            let vec = (-self.angular_component_b).cross(rel_pos2);
            let denom = rb.inv_mass + cp.normal_world_on_b.dot(vec);
            let vel = rb.get_vel_in_local_point(rel_pos2);
            self.contact_normal_2 = -cp.normal_world_on_b;
            self.rel_pos2_cross_normal = -torque_axis;

            if apply_warmstart {
                solver_body_b.internal_apply_impulse(
                    cp.normal_world_on_b * solver_body_b.inv_mass,
                    -self.angular_component_b,
                    -self.applied_impulse,
                );
            }

            let vel_dot_n = self
                .contact_normal_2
                .dot(solver_body_b.lin_vel + solver_body_b.external_force_impulse)
                + self
                    .rel_pos2_cross_normal
                    .dot(solver_body_b.ang_vel + solver_body_b.external_torque_impulse);

            (denom, vel, vel_dot_n)
        });

        self.jac_diag_ab_inv = contact_solver_info::SOR / (denom0 + denom1);

        let vel = vel0 - vel1;
        let rel_vel = cp.normal_world_on_b.dot(vel);
        let restitution = Self::restitution_curve(rel_vel, cp.combined_restitution);

        let penetration = cp.distance_1;
        let positional_error = if penetration > 0.0 {
            0.0
        } else {
            -penetration * erp * inv_time_step
        };

        let rel_vel = vel_1_dot_n + vel_2_dot_n;
        let vel_error = restitution - rel_vel;

        let penetration_impulse = positional_error * self.jac_diag_ab_inv;
        let vel_impulse = vel_error * self.jac_diag_ab_inv;

        (self.rhs, self.rhs_penetration) =
            if penetration > contact_solver_info::SPLIT_IMPULSE_PENETRATION_THRESHOLD {
                (penetration_impulse + vel_impulse, 0.0)
            } else {
                (vel_impulse, penetration_impulse)
            };
    }

    fn setup_friction_constraint(
        &mut self,
        (solver_body_a, solver_body_b): (&SolverBody, &SolverBody),
        (rb0, rb1): (Option<&RigidBody>, Option<&RigidBody>),
        (rel_pos1, rel_pos2): (Vec3A, Vec3A),
        normal_axis: Vec3A,
    ) {
        let (vel_1_dot_n, denom1) = rb0.map_or((0.0, 0.0), |rb| {
            self.contact_normal_1 = normal_axis;
            self.rel_pos1_cross_normal = rel_pos1.cross(self.contact_normal_1);
            self.angular_component_a = rb
                .inv_inertia_tensor_world
                .mul_transpose_vec3a(self.rel_pos1_cross_normal);

            let vec = self.angular_component_a.cross(rel_pos1);
            let denom = rb.inv_mass + normal_axis.dot(vec);

            let vel_dot_n = self
                .contact_normal_1
                .dot(solver_body_a.lin_vel + solver_body_a.external_force_impulse)
                + self.rel_pos1_cross_normal.dot(solver_body_a.ang_vel);

            (vel_dot_n, denom)
        });

        let (vel_2_dot_n, denom2) = rb1.map_or((0.0, 0.0), |rb| {
            self.contact_normal_2 = -normal_axis;
            self.rel_pos2_cross_normal = rel_pos2.cross(self.contact_normal_2);
            self.angular_component_b = rb
                .inv_inertia_tensor_world
                .mul_transpose_vec3a(self.rel_pos2_cross_normal);

            let vec = (-self.angular_component_b).cross(rel_pos2);
            let denom = rb.inv_mass + normal_axis.dot(vec);

            let vel_dot_n = self
                .contact_normal_2
                .dot(solver_body_b.lin_vel + solver_body_b.external_force_impulse)
                + self.rel_pos2_cross_normal.dot(solver_body_b.ang_vel);

            (vel_dot_n, denom)
        });

        self.jac_diag_ab_inv = contact_solver_info::SOR / (denom1 + denom2);

        let rel_vel = vel_1_dot_n + vel_2_dot_n;
        let vel_error = -rel_vel;
        self.rhs = vel_error * self.jac_diag_ab_inv;
    }
}

impl SolverConstraint {
    #[inline]
    pub fn resolve_single_constraint_row_lower_limit_one_dynamic(
        &mut self,
        body: &mut SolverBody,
    ) -> f32 {
        let mut delta_impulse = self.rhs;
        let normal = self.contact_normal_1;
        let lever = self.rel_pos1_cross_normal;
        let angular_component = self.angular_component_a;

        let delta_vel_dot_n =
            bullet_dot(normal, body.delta_lin_vel) + bullet_dot(lever, body.delta_ang_vel);
        delta_impulse -= delta_vel_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_impulse;
            self.applied_impulse = self.lower_limit;
        } else {
            self.applied_impulse = sum;
        }

        body.delta_lin_vel += normal * body.inv_mass * delta_impulse;
        body.delta_ang_vel += angular_component * delta_impulse;
        delta_impulse / self.jac_diag_ab_inv
    }

    #[inline]
    pub fn resolve_single_constraint_row_generic_one_dynamic(
        &mut self,
        body: &mut SolverBody,
    ) -> f32 {
        let mut delta_impulse = self.rhs;
        let normal = self.contact_normal_1;
        let lever = self.rel_pos1_cross_normal;
        let angular_component = self.angular_component_a;

        let delta_vel_dot_n =
            bullet_dot(normal, body.delta_lin_vel) + bullet_dot(lever, body.delta_ang_vel);
        delta_impulse -= delta_vel_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_impulse;
            self.applied_impulse = self.lower_limit;
        } else if sum > self.upper_limit {
            delta_impulse = self.upper_limit - self.applied_impulse;
            self.applied_impulse = self.upper_limit;
        } else {
            self.applied_impulse = sum;
        }

        body.delta_lin_vel += normal * body.inv_mass * delta_impulse;
        body.delta_ang_vel += angular_component * delta_impulse;
        delta_impulse / self.jac_diag_ab_inv
    }

    #[inline]
    pub fn resolve_split_penetration_impulse_one_dynamic(&mut self, body: &mut SolverBody) -> f32 {
        if self.rhs_penetration == 0.0 {
            return 0.0;
        }

        let mut delta_impulse = self.rhs_penetration;
        let normal = self.contact_normal_1;
        let lever = self.rel_pos1_cross_normal;
        let angular_component = self.angular_component_a;

        let delta_vel_dot_n = bullet_dot(normal, body.push_vel) + bullet_dot(lever, body.turn_vel);
        delta_impulse -= delta_vel_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_push_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_push_impulse;
            self.applied_push_impulse = self.lower_limit;
        } else {
            self.applied_push_impulse = sum;
        }

        body.push_vel += normal * body.inv_mass * delta_impulse;
        body.turn_vel += angular_component * delta_impulse;
        delta_impulse / self.jac_diag_ab_inv
    }

    pub fn resolve_single_constraint_row_generic(
        &mut self,
        body_a: &mut SolverBody,
        body_b: &mut SolverBody,
    ) -> f32 {
        let mut delta_impulse = self.rhs;

        let delta_vel_1_dot_n = if body_a.original_body.is_some() {
            bullet_dot(self.contact_normal_1, body_a.delta_lin_vel)
                + bullet_dot(self.rel_pos1_cross_normal, body_a.delta_ang_vel)
        } else {
            0.0
        };
        let delta_vel_2_dot_n = if body_b.original_body.is_some() {
            bullet_dot(self.contact_normal_2, body_b.delta_lin_vel)
                + bullet_dot(self.rel_pos2_cross_normal, body_b.delta_ang_vel)
        } else {
            0.0
        };

        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;
        delta_impulse -= delta_vel_2_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_impulse;
            self.applied_impulse = self.lower_limit;
        } else if sum > self.upper_limit {
            delta_impulse = self.upper_limit - self.applied_impulse;
            self.applied_impulse = self.upper_limit;
        } else {
            self.applied_impulse = sum;
        }

        if body_a.original_body.is_some() {
            body_a.delta_lin_vel += self.contact_normal_1 * body_a.inv_mass * delta_impulse;
            body_a.delta_ang_vel += self.angular_component_a * delta_impulse;
        }

        if body_b.original_body.is_some() {
            body_b.delta_lin_vel += self.contact_normal_2 * body_b.inv_mass * delta_impulse;
            body_b.delta_ang_vel += self.angular_component_b * delta_impulse;
        }

        delta_impulse / self.jac_diag_ab_inv
    }

    pub fn resolve_single_constraint_row_lower_limit(
        &mut self,
        body_a: &mut SolverBody,
        body_b: &mut SolverBody,
    ) -> f32 {
        let mut delta_impulse = self.rhs;

        let delta_vel_1_dot_n = if body_a.original_body.is_some() {
            bullet_dot(self.contact_normal_1, body_a.delta_lin_vel)
                + bullet_dot(self.rel_pos1_cross_normal, body_a.delta_ang_vel)
        } else {
            0.0
        };
        let delta_vel_2_dot_n = if body_b.original_body.is_some() {
            bullet_dot(self.contact_normal_2, body_b.delta_lin_vel)
                + bullet_dot(self.rel_pos2_cross_normal, body_b.delta_ang_vel)
        } else {
            0.0
        };

        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;
        delta_impulse -= delta_vel_2_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_impulse;
            self.applied_impulse = self.lower_limit;
        } else {
            self.applied_impulse = sum;
        }

        if body_a.original_body.is_some() {
            body_a.delta_lin_vel += self.contact_normal_1 * body_a.inv_mass * delta_impulse;
            body_a.delta_ang_vel += self.angular_component_a * delta_impulse;
        }

        if body_b.original_body.is_some() {
            body_b.delta_lin_vel += self.contact_normal_2 * body_b.inv_mass * delta_impulse;
            body_b.delta_ang_vel += self.angular_component_b * delta_impulse;
        }

        delta_impulse / self.jac_diag_ab_inv
    }

    pub fn resolve_split_penetration_impulse(
        &mut self,
        body_a: &mut SolverBody,
        body_b: &mut SolverBody,
    ) -> f32 {
        if self.rhs_penetration == 0.0 {
            return 0.0;
        }

        let mut delta_impulse = self.rhs_penetration;

        let delta_vel_1_dot_n = if body_a.original_body.is_some() {
            bullet_dot(self.contact_normal_1, body_a.push_vel)
                + bullet_dot(self.rel_pos1_cross_normal, body_a.turn_vel)
        } else {
            0.0
        };
        let delta_vel_2_dot_n = if body_b.original_body.is_some() {
            bullet_dot(self.contact_normal_2, body_b.push_vel)
                + bullet_dot(self.rel_pos2_cross_normal, body_b.turn_vel)
        } else {
            0.0
        };

        delta_impulse -= delta_vel_1_dot_n * self.jac_diag_ab_inv;
        delta_impulse -= delta_vel_2_dot_n * self.jac_diag_ab_inv;

        let sum = self.applied_push_impulse + delta_impulse;
        if sum < self.lower_limit {
            delta_impulse = self.lower_limit - self.applied_push_impulse;
            self.applied_push_impulse = self.lower_limit;
        } else {
            self.applied_push_impulse = sum;
        }

        if body_a.original_body.is_some() {
            body_a.push_vel += self.contact_normal_1 * body_a.inv_mass * delta_impulse;
            body_a.turn_vel += self.angular_component_a * delta_impulse;
        }

        if body_b.original_body.is_some() {
            body_b.push_vel += self.contact_normal_2 * body_b.inv_mass * delta_impulse;
            body_b.turn_vel += self.angular_component_b * delta_impulse;
        }

        delta_impulse / self.jac_diag_ab_inv
    }
}
