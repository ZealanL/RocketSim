use glam::Vec3A;

use super::{contact_solver_info, solver_body::SolverBody, solver_constraint::SolverConstraint};
use crate::bullet::{
    collision::narrowphase::{
        manifold_point::ManifoldPoint, persistent_manifold::PersistentManifold,
    },
    dynamics::rigid_body::{CollisionFlags, RigidBody},
    linear_math::{integrate_trans, integrate_trans_no_rot, plane_space_1},
};

struct SpecialResolveInfo {
    pub obj_idx: usize,
    pub num_special_collisions: u16,
    pub total_normal: Vec3A,
    pub total_dist: f32,
    pub restitution: f32,
    pub friction: f32,
}

impl SpecialResolveInfo {
    pub const DEFAULT: Self = Self {
        obj_idx: 0,
        num_special_collisions: 0,
        total_normal: Vec3A::ZERO,
        total_dist: 0.0,
        restitution: 0.0,
        friction: 0.0,
    };

    fn add_special_collision(
        &mut self,
        body0: &RigidBody,
        body1: &RigidBody,
        cp: &ManifoldPoint,
        rel_pos1: Vec3A,
        rel_pos2: Vec3A,
    ) {
        for (obj, rel_pos) in [(&body0, rel_pos1), (&body1, rel_pos2)] {
            if !obj.is_static_obj() {
                self.obj_idx = obj.world_array_idx;
                self.num_special_collisions += 1;
                self.friction = cp.combined_friction;
                self.restitution = cp.combined_restitution;
                self.total_normal += cp.normal_world_on_b;
                self.total_dist += rel_pos.length();
            }
        }
    }
}

pub struct SeqImpulseConstraintSolver {
    tmp_solver_body_pool: Vec<SolverBody>,
    tmp_solver_contact_constraint_pool: Vec<SolverConstraint>,
    tmp_solver_contact_friction_constraint_pool: Vec<SolverConstraint>,
    fixed_body_id: Option<usize>,
    least_squares_residual: f32,
    special_resolve_info: SpecialResolveInfo,
}

impl Default for SeqImpulseConstraintSolver {
    fn default() -> Self {
        Self {
            tmp_solver_body_pool: Vec::new(),
            tmp_solver_contact_constraint_pool: Vec::new(),
            tmp_solver_contact_friction_constraint_pool: Vec::new(),
            fixed_body_id: None,
            least_squares_residual: 0.0,
            special_resolve_info: SpecialResolveInfo::DEFAULT,
        }
    }
}

impl SeqImpulseConstraintSolver {
    fn get_or_init_solver_body(&mut self, rb: &mut RigidBody) -> usize {
        if let Some(companion_id) = rb.companion_id {
            return companion_id;
        }

        if !rb.is_static_obj() && rb.inv_mass != 0.0 {
            let solver_body_id = self.tmp_solver_body_pool.len();
            rb.companion_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::new(rb));
            return solver_body_id;
        }

        if let Some(fixed_body_id) = self.fixed_body_id {
            fixed_body_id
        } else {
            let solver_body_id = self.tmp_solver_body_pool.len();
            rb.companion_id = Some(solver_body_id);
            self.fixed_body_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::DEFAULT);
            solver_body_id
        }
    }

    pub fn solve_group(
        &mut self,
        collision_objs: &mut [RigidBody],
        non_static_bodies: &[usize],
        manifolds: &mut Vec<PersistentManifold>,
        time_step: f32,
    ) {
        self.solve_group_setup(collision_objs, non_static_bodies, manifolds, time_step);
        self.solve_group_iterations();
        self.solve_group_finish(collision_objs, time_step);
    }

    fn solve_group_setup(
        &mut self,
        collision_objs: &mut [RigidBody],
        non_static_bodies: &[usize],
        manifolds: &mut Vec<PersistentManifold>,
        time_step: f32,
    ) {
        self.setup_solver_bodies(collision_objs, non_static_bodies);

        for manifold in manifolds.iter_mut() {
            debug_assert!(manifold.body0_idx < collision_objs.len());
            debug_assert!(manifold.body1_idx < collision_objs.len());
            debug_assert_ne!(manifold.body0_idx, manifold.body1_idx);
            let [body0, body1] = unsafe {
                collision_objs.get_disjoint_unchecked_mut([manifold.body0_idx, manifold.body1_idx])
            };

            let solver_body_id_a = self.get_or_init_solver_body(body0);
            let solver_body_id_b = self.get_or_init_solver_body(body1);

            debug_assert!(solver_body_id_a < self.tmp_solver_body_pool.len());
            debug_assert!(solver_body_id_b < self.tmp_solver_body_pool.len());
            debug_assert_ne!(solver_body_id_a, solver_body_id_b);
            let [solver_body_a, solver_body_b] = unsafe {
                self.tmp_solver_body_pool
                    .get_disjoint_unchecked_mut([solver_body_id_a, solver_body_id_b])
            };

            body0.companion_id = Some(solver_body_id_a);
            body1.companion_id = Some(solver_body_id_b);

            for cp in &mut manifold.point_cache {
                assert!(cp.distance_1 <= manifold.contact_processing_threshold);

                let rel_pos1 = cp.pos_world_on_a - body0.get_world_trans().translation;
                let rel_pos2 = cp.pos_world_on_b - body1.get_world_trans().translation;

                let rb0 = solver_body_a.original_body.map(|_| &*body0);
                let rb1 = solver_body_b.original_body.map(|_| &*body1);
                let friction_idx = self.tmp_solver_contact_friction_constraint_pool.len();

                // V2 retains the original special manifold in the contact pool.
                // Its split-impulse pass therefore applies the manifold's
                // penetration correction, while the regular velocity pass
                // skips it via SolverConstraint::is_special.  The aggregated
                // special constraint below supplies the ball/world velocity
                // response.  Do not discard the manifold before split impulse.
                if cp.is_special {
                    self.special_resolve_info
                        .add_special_collision(body0, body1, cp, rel_pos1, rel_pos2);
                }

                self.tmp_solver_contact_constraint_pool.push(
                    SolverConstraint::get_contact_constraint(
                        (solver_body_id_a, solver_body_id_b),
                        (solver_body_a, solver_body_b),
                        (rb0, rb1),
                        (rel_pos1, rel_pos2),
                        cp,
                        friction_idx,
                        time_step,
                    ),
                );

                cp.calc_lat_friction_dir(solver_body_a, solver_body_b, rel_pos1, rel_pos2);

                self.tmp_solver_contact_friction_constraint_pool.push(
                    SolverConstraint::get_friction_constraint(
                        (solver_body_id_a, solver_body_id_b),
                        (solver_body_a, solver_body_b),
                        (rb0, rb1),
                        (rel_pos1, rel_pos2),
                        cp,
                        friction_idx,
                    ),
                );
            }
        }

        manifolds.clear();

        if self.special_resolve_info.num_special_collisions > 0 {
            let body = &mut collision_objs[self.special_resolve_info.obj_idx];
            self.convert_contact_special(body, time_step);
            self.special_resolve_info = SpecialResolveInfo::DEFAULT;
        }
    }

    fn setup_solver_bodies(
        &mut self,
        collision_objs: &mut [RigidBody],
        non_static_bodies: &[usize],
    ) {
        self.fixed_body_id = None;

        self.tmp_solver_body_pool
            .reserve(non_static_bodies.len() + 1);
        self.tmp_solver_contact_constraint_pool
            .reserve(non_static_bodies.len() * 2);
        self.tmp_solver_contact_friction_constraint_pool
            .reserve(non_static_bodies.len() * 2);

        for rb in &mut *collision_objs {
            rb.companion_id = None;
        }

        for &rb_idx in non_static_bodies {
            let rb = &mut collision_objs[rb_idx];
            debug_assert_ne!(rb.inv_mass, 0.0);

            if !rb.is_active() {
                continue;
            }

            let solver_body_id = self.tmp_solver_body_pool.len();
            rb.companion_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::new(rb));
        }
    }

    fn convert_contact_special(&mut self, body: &RigidBody, time_step: f32) {
        let sri = &self.special_resolve_info;
        let num_collisions = f32::from(sri.num_special_collisions);
        let distance = sri.total_dist / num_collisions;
        let normal_world_on_b = (sri.total_normal / num_collisions).normalize_or_zero();

        if std::env::var("RSIM_DEBUG_SPECIAL").is_ok() {
            println!(
                "rust_special,num={},total_normal={:?},total_dist={},avg_normal={:?},avg_dist={},friction={},restitution={},body={},pos={:?},lin={:?},ang={:?}",
                sri.num_special_collisions,
                sri.total_normal,
                sri.total_dist,
                normal_world_on_b,
                distance,
                sri.friction,
                sri.restitution,
                body.world_array_idx,
                body.get_world_trans().translation,
                body.lin_vel,
                body.ang_vel,
            );
        }

        let friction_idx = self.tmp_solver_contact_constraint_pool.len();

        let solver_body_id_a = body.companion_id.unwrap();
        let solver_body_id_b = if let Some(fixed_body_id) = self.fixed_body_id {
            fixed_body_id
        } else {
            let solver_body_id = self.tmp_solver_body_pool.len();
            self.fixed_body_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::DEFAULT);
            solver_body_id
        };

        let solver_body_a = &mut self.tmp_solver_body_pool[solver_body_id_a];

        let rel_pos1 = normal_world_on_b * -distance;
        let relaxation = contact_solver_info::SOR;

        let inv_time_step = 1.0 / time_step;
        let erp = contact_solver_info::ERP_2;

        let torque_axis_0 = rel_pos1.cross(normal_world_on_b);
        let angular_component_a = body
            .inv_inertia_tensor_world
            .mul_transpose_vec3a(torque_axis_0);

        let denom = {
            let vec = angular_component_a.cross(rel_pos1);
            body.inv_mass + normal_world_on_b.dot(vec)
        };
        let jac_diag_ab_inv = relaxation / denom;

        let (contact_normal_1, rel_pos1_cross_normal) = (normal_world_on_b, torque_axis_0);

        let penetration = distance;

        let vel = body.get_vel_in_local_point(rel_pos1);
        let rel_vel = normal_world_on_b.dot(vel);

        let restitution = SolverConstraint::restitution_curve(rel_vel, sri.restitution).max(0.0);

        let (external_force_impulse_a, external_torque_impulse_a) = (
            solver_body_a.external_force_impulse,
            solver_body_a.external_torque_impulse,
        );

        let rel_vel = contact_normal_1.dot(solver_body_a.lin_vel + external_force_impulse_a)
            + rel_pos1_cross_normal.dot(solver_body_a.ang_vel + external_torque_impulse_a);

        let positional_error = if penetration > 0.0 {
            0.0
        } else {
            -penetration * erp * inv_time_step
        };

        let vel_error = restitution - rel_vel;

        let penetration_impulse = positional_error * jac_diag_ab_inv;
        let vel_impulse = vel_error * jac_diag_ab_inv;

        let (rhs, rhs_penetration) =
            if penetration > contact_solver_info::SPLIT_IMPULSE_PENETRATION_THRESHOLD {
                (penetration_impulse + vel_impulse, 0.0)
            } else {
                (vel_impulse, penetration_impulse)
            };

        // Temporary diagnostic for the one remaining full-replay divergence.  The
        // coordinate gate keeps this focused on the ball near the floor contact
        // around transition 2130; remove with the other debug instrumentation once
        // the solver parity fix is identified.
        let p = body.get_world_trans().translation;
        if std::env::var("RSIM_DEBUG_SPECIAL_DETAIL").is_ok()
            && p.x > -11.0
            && p.x < -9.0
            && p.y > 79.0
            && p.y < 81.0
            && p.z < 3.0
        {
            println!(
                "rust_special_detail,body={},pos={:?},distance={},normal={:?},rel_pos={:?},body_lin={:?},body_ang={:?},solver_lin={:?},solver_ang={:?},ext_lin={:?},ext_ang={:?},rel_vel_pre={},restitution={},jac={},penetration={},pos_error={},vel_error={},rhs={},rhs_pen={},applied={}",
                body.world_array_idx,
                p,
                distance,
                normal_world_on_b,
                rel_pos1,
                body.lin_vel,
                body.ang_vel,
                solver_body_a.lin_vel,
                solver_body_a.ang_vel,
                external_force_impulse_a,
                external_torque_impulse_a,
                rel_vel,
                restitution,
                jac_diag_ab_inv,
                penetration,
                positional_error,
                vel_error,
                rhs,
                rhs_penetration,
                0.0f32,
            );
        }

        self.tmp_solver_contact_constraint_pool
            .push(SolverConstraint {
                solver_body_id_a,
                solver_body_id_b,
                angular_component_a,
                jac_diag_ab_inv,
                contact_normal_1,
                rel_pos1_cross_normal,
                rhs,
                rhs_penetration,
                friction: sri.friction,
                lower_limit: 0.0,
                upper_limit: 1e10,
                ..Default::default()
            });

        let vel = solver_body_a.get_vel_in_local_point_no_delta(rel_pos1);
        let rel_vel = normal_world_on_b.dot(vel);

        let mut lateral_friction_dir_1 = vel - normal_world_on_b * rel_vel;
        let lat_rel_vel = lateral_friction_dir_1.length_squared();

        if lat_rel_vel > f32::EPSILON {
            lateral_friction_dir_1 *= 1.0 / lat_rel_vel.sqrt();
        } else {
            lateral_friction_dir_1 = plane_space_1(normal_world_on_b);
        }

        // addFrictionConstraint
        let (contact_normal_1, rel_pos1_cross_normal, angular_component_a) = {
            let torque_axis = rel_pos1.cross(lateral_friction_dir_1);

            (
                lateral_friction_dir_1,
                torque_axis,
                body.inv_inertia_tensor_world
                    .mul_transpose_vec3a(torque_axis),
            )
        };

        let denom = {
            let vec = angular_component_a.cross(rel_pos1);
            body.inv_mass + lateral_friction_dir_1.dot(vec)
        };
        let jac_diag_ab_inv = relaxation / denom;

        let rel_vel = contact_normal_1.dot(solver_body_a.lin_vel + external_force_impulse_a)
            + rel_pos1_cross_normal.dot(solver_body_a.ang_vel + external_torque_impulse_a);

        let vel_error = -rel_vel;
        let vel_impulse = vel_error * jac_diag_ab_inv;

        self.tmp_solver_contact_friction_constraint_pool
            .push(SolverConstraint {
                friction_idx,
                solver_body_id_a,
                solver_body_id_b,
                contact_normal_1,
                rel_pos1_cross_normal,
                angular_component_a,
                jac_diag_ab_inv,
                rhs: vel_impulse,
                lower_limit: -sri.friction,
                upper_limit: sri.friction,
                friction: sri.friction,
                ..Default::default()
            });
    }

    fn solve_group_split_impulse_iterations(&mut self) {
        let mut should_run = (1u64 << self.tmp_solver_contact_constraint_pool.len()) - 1;

        for _ in 0..contact_solver_info::NUM_ITERATIONS {
            for (i, contact) in self
                .tmp_solver_contact_constraint_pool
                .iter_mut()
                .enumerate()
            {
                let mask = 1 << i;
                if should_run & mask == 0 {
                    continue;
                }

                debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
                let [body_a, body_b] = unsafe {
                    self.tmp_solver_body_pool.get_disjoint_unchecked_mut([
                        contact.solver_body_id_a,
                        contact.solver_body_id_b,
                    ])
                };

                let residual = contact.resolve_split_penetration_impulse(body_a, body_b);
                if residual * residual == 0.0 {
                    should_run ^= mask;
                }
            }

            if should_run == 0 {
                break;
            }
        }
    }

    fn solve_single_iteration(&mut self) -> f32 {
        let mut least_squares_residual = 0.0;

        for (contact_idx, contact) in self
            .tmp_solver_contact_constraint_pool
            .iter_mut()
            .enumerate()
        {
            if contact.is_special {
                continue;
            }

            debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
            let [body_a, body_b] = unsafe {
                self.tmp_solver_body_pool.get_disjoint_unchecked_mut([
                    contact.solver_body_id_a,
                    contact.solver_body_id_b,
                ])
            };

            let detail = std::env::var("RSIM_DEBUG_SPECIAL_DETAIL").is_ok()
                && (body_a.original_body == Some(20) || body_b.original_body == Some(20))
                && body_a.world_trans.translation.x > -11.0
                && body_a.world_trans.translation.x < -9.0
                && body_a.world_trans.translation.y > 79.0
                && body_a.world_trans.translation.y < 81.0
                && body_a.world_trans.translation.z < 3.0;
            if detail {
                println!(
                    "rust_constraint_before,idx={},special={},a={},b={},rhs={},rhs_pen={},jac={},applied={},lower={},upper={},normal={:?},delta_lin_a={:?},delta_ang_a={:?}",
                    contact_idx,
                    contact.is_special,
                    contact.solver_body_id_a,
                    contact.solver_body_id_b,
                    contact.rhs,
                    contact.rhs_penetration,
                    contact.jac_diag_ab_inv,
                    contact.applied_impulse,
                    contact.lower_limit,
                    contact.upper_limit,
                    contact.contact_normal_1,
                    body_a.delta_lin_vel,
                    body_a.delta_ang_vel,
                );
            }

            let residual = contact.resolve_single_constraint_row_lower_limit(body_a, body_b);
            if detail {
                println!(
                    "rust_constraint_after,idx={},residual={},applied={},delta_lin_a={:?},delta_ang_a={:?}",
                    contact_idx,
                    residual,
                    contact.applied_impulse,
                    body_a.delta_lin_vel,
                    body_a.delta_ang_vel,
                );
            }
            least_squares_residual = (residual * residual).max(least_squares_residual);
        }

        for contact in &mut self.tmp_solver_contact_friction_constraint_pool {
            let total_impulse =
                self.tmp_solver_contact_constraint_pool[contact.friction_idx].applied_impulse;
            if total_impulse <= 0.0 {
                continue;
            }

            let limit = contact.friction * total_impulse;
            contact.lower_limit = -limit;
            contact.upper_limit = limit;

            debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
            let [body_a, body_b] = unsafe {
                self.tmp_solver_body_pool.get_disjoint_unchecked_mut([
                    contact.solver_body_id_a,
                    contact.solver_body_id_b,
                ])
            };

            let residual = contact.resolve_single_constraint_row_generic(body_a, body_b);
            least_squares_residual = (residual * residual).max(least_squares_residual);
        }

        least_squares_residual
    }

    fn solve_group_iterations(&mut self) {
        self.solve_group_split_impulse_iterations();

        for _ in 0..contact_solver_info::NUM_ITERATIONS {
            self.least_squares_residual = self.solve_single_iteration();
            if self.least_squares_residual == 0.0 {
                break;
            }
        }
    }

    fn solve_group_finish(&mut self, collision_objs: &mut [RigidBody], time_step: f32) {
        // writeBackBodies
        for solver in &mut self.tmp_solver_body_pool {
            let Some(body) = solver.original_body.map(|idx| &mut collision_objs[idx]) else {
                continue;
            };

            if std::env::var("RSIM_SOLVER_DEBUG").is_ok()
                && body.world_array_idx == 20
                && solver.push_vel.length_squared() > 0.0
            {
                println!(
                    "rust_solver_body,idx={},pre_pos={:?},pre_lin={:?},pre_ang={:?},delta_lin={:?},delta_ang={:?},push={:?},turn={:?},inv_mass={:?}",
                    body.world_array_idx,
                    body.get_world_trans().translation,
                    body.lin_vel,
                    body.ang_vel,
                    solver.delta_lin_vel,
                    solver.delta_ang_vel,
                    solver.push_vel,
                    solver.turn_vel,
                    solver.inv_mass,
                );
            }

            solver.lin_vel += solver.delta_lin_vel;
            solver.ang_vel += solver.delta_ang_vel;

            if solver.push_vel.length_squared() != 0.0 || solver.turn_vel.length_squared() != 0.0 {
                if body.collision_flags & CollisionFlags::NoAngularMotion != 0 {
                    integrate_trans_no_rot(
                        &mut solver.world_trans.translation,
                        solver.push_vel,
                        time_step,
                    );
                } else {
                    integrate_trans(
                        &mut solver.world_trans,
                        &mut solver.world_rot,
                        solver.push_vel,
                        solver.turn_vel * contact_solver_info::SPLIT_IMPULSE_TURN_ERP,
                        time_step,
                    );
                }
            }

            body.set_lin_vel(solver.lin_vel + solver.external_force_impulse);
            body.set_ang_vel(solver.ang_vel + solver.external_torque_impulse);

            body.set_world_trans(solver.world_trans);
        }

        self.tmp_solver_body_pool.clear();
        self.tmp_solver_contact_constraint_pool.clear();
        self.tmp_solver_contact_friction_constraint_pool.clear();
    }
}
