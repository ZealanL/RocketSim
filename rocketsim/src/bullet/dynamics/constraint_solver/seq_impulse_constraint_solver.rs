use glam::Vec3A;

use super::{contact_solver_info, solver_body::SolverBody, solver_constraint::SolverConstraint};
use crate::bullet::{
    collision::narrowphase::{
        manifold_point::ManifoldPoint, persistent_manifold::PersistentManifold,
    },
    dynamics::rigid_body::{CollisionFlags, RigidBody},
    linear_math::{integrate_trans, integrate_trans_no_rot},
};

/// Hold one flagged sample for the per-body accumulator.
/// Store the adjusted normal. Store the lever length. Store materials.
#[derive(Clone, Copy)]
struct SpecialSample {
    obj_idx: usize,
    normal_world_on_b: Vec3A,
    lever_len: f32,
    friction: f32,
    restitution: f32,
}

/// Hold plain sums for one dynamic body.
/// Keep first-seen order in the accumulator vector.
#[derive(Clone, Copy, Default)]
struct SpecialAccumulator {
    obj_idx: usize,
    total_normal: Vec3A,
    total_lever_len: f32,
    total_friction: f32,
    total_restitution: f32,
    count: u32,
}

/// Add one flagged sample to its body accumulator.
/// Create the body entry on first sight. Keep first-seen order.
fn accumulate_special_sample(accumulators: &mut Vec<SpecialAccumulator>, sample: SpecialSample) {
    if let Some(acc) = accumulators
        .iter_mut()
        .find(|acc| acc.obj_idx == sample.obj_idx)
    {
        acc.total_normal += sample.normal_world_on_b;
        acc.total_lever_len += sample.lever_len;
        acc.total_friction += sample.friction;
        acc.total_restitution += sample.restitution;
        acc.count += 1;
        return;
    }

    accumulators.push(SpecialAccumulator {
        obj_idx: sample.obj_idx,
        total_normal: sample.normal_world_on_b,
        total_lever_len: sample.lever_len,
        total_friction: sample.friction,
        total_restitution: sample.restitution,
        count: 1,
    });
}

/// Return the dynamic side for one flagged point.
/// Return the dynamic body index and its lever arm.
/// Return `None` when both bodies are static.
fn special_dynamic_side(
    body0: &RigidBody,
    body1: &RigidBody,
    rel_pos1: Vec3A,
    rel_pos2: Vec3A,
) -> Option<(usize, Vec3A)> {
    if !body0.is_static_obj() {
        Some((body0.world_array_idx, rel_pos1))
    } else if !body1.is_static_obj() {
        Some((body1.world_array_idx, rel_pos2))
    } else {
        None
    }
}

pub struct SeqImpulseConstraintSolver {
    tmp_solver_body_pool: Vec<SolverBody>,
    tmp_solver_contact_constraint_pool: Vec<SolverConstraint>,
    tmp_solver_contact_friction_constraint_pool: Vec<SolverConstraint>,
    fixed_body_id: Option<usize>,
    least_squares_residual: f32,
    tmp_special_accumulators: Vec<SpecialAccumulator>,
    tmp_split_should_run: Vec<bool>,
    /// Ball-only arenas do not need split rows for separated special points.
    pub skip_separated_special_rows: bool,
}

impl Default for SeqImpulseConstraintSolver {
    fn default() -> Self {
        Self {
            tmp_solver_body_pool: Vec::new(),
            tmp_solver_contact_constraint_pool: Vec::new(),
            tmp_solver_contact_friction_constraint_pool: Vec::new(),
            fixed_body_id: None,
            least_squares_residual: 0.0,
            tmp_special_accumulators: Vec::new(),
            tmp_split_should_run: Vec::new(),
            skip_separated_special_rows: false,
        }
    }
}

impl SeqImpulseConstraintSolver {
    fn get_or_init_solver_body(&mut self, rb: &mut RigidBody) -> usize {
        if rb.is_static_obj() {
            return if let Some(fixed_body_id) = self.fixed_body_id {
                fixed_body_id
            } else {
                let solver_body_id = self.tmp_solver_body_pool.len();
                rb.companion_id = Some(solver_body_id);
                self.fixed_body_id = Some(solver_body_id);

                self.tmp_solver_body_pool.push(SolverBody::DEFAULT);
                solver_body_id
            };
        }

        if let Some(companion_id) = rb.companion_id {
            return companion_id;
        }

        if rb.inv_mass != 0.0 {
            let solver_body_id = self.tmp_solver_body_pool.len();
            rb.companion_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::new(rb));
            solver_body_id
        } else if let Some(fixed_body_id) = self.fixed_body_id {
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
        manifolds: &mut [PersistentManifold],
        active_manifold_idcs: &mut Vec<usize>,
        time_step: f32,
    ) {
        self.solve_group_setup(
            collision_objs,
            non_static_bodies,
            manifolds,
            active_manifold_idcs,
            time_step,
        );
        self.solve_group_iterations();
        self.solve_group_finish(collision_objs, time_step);
    }

    fn solve_group_setup(
        &mut self,
        collision_objs: &mut [RigidBody],
        non_static_bodies: &[usize],
        manifolds: &mut [PersistentManifold],
        active_manifold_idcs: &mut Vec<usize>,
        time_step: f32,
    ) {
        self.setup_solver_bodies(collision_objs, non_static_bodies);
        self.tmp_special_accumulators.clear();

        for &manifold_idx in active_manifold_idcs.iter() {
            let manifold = &mut manifolds[manifold_idx];

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

            let trans0 = body0.get_world_trans().translation;
            let trans1 = body1.get_world_trans().translation;

            for cp in &mut manifold.point_cache {
                assert!(cp.distance_1 <= manifold.contact_processing_threshold);

                let rel_pos1 = cp.pos_world_on_a - trans0;
                let rel_pos2 = cp.pos_world_on_b - trans1;

                if cp.is_special {
                    debug_assert!(body0.is_static_obj() != body1.is_static_obj());
                    let rb0 = solver_body_a.original_body.map(|_| &*body0);
                    let rb1 = solver_body_b.original_body.map(|_| &*body1);

                    if !self.skip_separated_special_rows || cp.distance_1 < 0.0 {
                        let mut constraint = SolverConstraint::get_split_only_contact_constraint(
                            (solver_body_id_a, solver_body_id_b),
                            (solver_body_a, solver_body_b),
                            (rb0, rb1),
                            (rel_pos1, rel_pos2),
                            cp,
                            time_step,
                        );
                        if self.skip_separated_special_rows {
                            constraint.make_dynamic_side_a(rb0.is_none() && rb1.is_some());
                        }
                        self.tmp_solver_contact_constraint_pool.push(constraint);
                    }

                    if let Some((obj_idx, lever_arm)) =
                        special_dynamic_side(body0, body1, rel_pos1, rel_pos2)
                    {
                        let sample = SpecialSample {
                            obj_idx,
                            normal_world_on_b: cp.normal_world_on_b,
                            lever_len: lever_arm.length(),
                            friction: cp.combined_friction,
                            restitution: cp.combined_restitution,
                        };

                        if self.skip_separated_special_rows
                            && let Some(acc) = self.tmp_special_accumulators.first_mut()
                            && acc.obj_idx == sample.obj_idx
                        {
                            acc.total_normal += sample.normal_world_on_b;
                            acc.total_lever_len += sample.lever_len;
                            acc.total_friction += sample.friction;
                            acc.total_restitution += sample.restitution;
                            acc.count += 1;
                        } else {
                            accumulate_special_sample(&mut self.tmp_special_accumulators, sample);
                        }
                    }

                    continue;
                }

                let rb0 = solver_body_a.original_body.map(|_| &*body0);
                let rb1 = solver_body_b.original_body.map(|_| &*body1);
                let friction_idx = self.tmp_solver_contact_constraint_pool.len();

                let lateral_friction_dir_1 =
                    cp.calc_lat_friction_dir(solver_body_a, solver_body_b, rel_pos1, rel_pos2);

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

                self.tmp_solver_contact_friction_constraint_pool.push(
                    SolverConstraint::get_friction_constraint(
                        (solver_body_id_a, solver_body_id_b),
                        (solver_body_a, solver_body_b),
                        (rb0, rb1),
                        (rel_pos1, rel_pos2),
                        cp.combined_friction,
                        lateral_friction_dir_1,
                        friction_idx,
                    ),
                );
            }
        }

        // Persistent manifolds keep their points and warmstart impulses for the next tick.
        active_manifold_idcs.clear();

        if !self.tmp_special_accumulators.is_empty() {
            self.emit_special_synthetics(collision_objs, time_step);
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

        for &rb_idx in non_static_bodies {
            let rb = &mut collision_objs[rb_idx];
            rb.companion_id = None;
            debug_assert_ne!(rb.inv_mass, 0.0);

            if !rb.is_active() {
                continue;
            }

            let solver_body_id = self.tmp_solver_body_pool.len();
            rb.companion_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::new(rb));
        }
    }

    /// Emit one synthetic row plus its friction row per dynamic body.
    /// Use plain means. Normalize the mean normal. Use a radial lever.
    /// Use distance `0.0` from the evidence-backed template. Keep first-seen order.
    fn emit_special_synthetics(&mut self, collision_objs: &[RigidBody], time_step: f32) {
        let num_bodies = self.tmp_special_accumulators.len();
        for i in 0..num_bodies {
            let acc = self.tmp_special_accumulators[i];
            self.push_special_synthetic(collision_objs, &acc, time_step);
        }
    }

    fn push_special_synthetic(
        &mut self,
        collision_objs: &[RigidBody],
        acc: &SpecialAccumulator,
        time_step: f32,
    ) {
        debug_assert!(acc.count > 0);
        let num_samples = acc.count as f32;
        let mean_normal = (acc.total_normal / num_samples).normalize_or_zero();
        let mean_lever_len = acc.total_lever_len / num_samples;
        let mean_friction = acc.total_friction / num_samples;
        let mean_restitution = acc.total_restitution / num_samples;

        let body = &collision_objs[acc.obj_idx];
        let solver_body_id_a = body.companion_id.unwrap();
        let solver_body_id_b = if let Some(fixed_body_id) = self.fixed_body_id {
            fixed_body_id
        } else {
            let solver_body_id = self.tmp_solver_body_pool.len();
            self.fixed_body_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::DEFAULT);
            solver_body_id
        };

        debug_assert!(solver_body_id_a < self.tmp_solver_body_pool.len());
        debug_assert!(solver_body_id_b < self.tmp_solver_body_pool.len());
        debug_assert_ne!(solver_body_id_a, solver_body_id_b);
        let [solver_body_a, solver_body_b] = unsafe {
            self.tmp_solver_body_pool
                .get_disjoint_unchecked_mut([solver_body_id_a, solver_body_id_b])
        };

        // Use a radial lever. Use template distance `0.0`.
        let rel_pos1 = mean_normal * -mean_lever_len;
        let rel_pos2 = Vec3A::ZERO;
        let synthetic_point = ManifoldPoint {
            normal_world_on_b: mean_normal,
            combined_friction: mean_friction,
            combined_restitution: mean_restitution,
            distance_1: 0.0,
            applied_impulse: 0.0,
            ..Default::default()
        };

        let rb0 = solver_body_a.original_body.map(|_| body);
        let friction_idx = self.tmp_solver_contact_constraint_pool.len();

        self.tmp_solver_contact_constraint_pool
            .push(SolverConstraint::get_contact_constraint(
                (solver_body_id_a, solver_body_id_b),
                (solver_body_a, solver_body_b),
                (rb0, None),
                (rel_pos1, rel_pos2),
                &synthetic_point,
                friction_idx,
                time_step,
            ));

        let lateral_friction_dir_1 =
            synthetic_point.calc_lat_friction_dir(solver_body_a, solver_body_b, rel_pos1, rel_pos2);

        self.tmp_solver_contact_friction_constraint_pool.push(
            SolverConstraint::get_friction_constraint(
                (solver_body_id_a, solver_body_id_b),
                (solver_body_a, solver_body_b),
                (rb0, None),
                (rel_pos1, rel_pos2),
                synthetic_point.combined_friction,
                lateral_friction_dir_1,
                friction_idx,
            ),
        );
    }

    fn solve_group_split_impulse_iterations_one_dynamic(&mut self) {
        let row_count = self.tmp_solver_contact_constraint_pool.len();
        self.tmp_split_should_run.clear();
        self.tmp_split_should_run.resize(row_count, true);
        let mut remaining = row_count;

        for _ in 0..contact_solver_info::NUM_ITERATIONS {
            if remaining == 0 {
                break;
            }
            for (i, contact) in self
                .tmp_solver_contact_constraint_pool
                .iter_mut()
                .enumerate()
            {
                if !self.tmp_split_should_run[i] {
                    continue;
                }

                debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
                let body_a = &mut self.tmp_solver_body_pool[contact.solver_body_id_a];
                let residual = contact.resolve_split_penetration_impulse_one_dynamic(body_a);
                if residual * residual == 0.0 {
                    self.tmp_split_should_run[i] = false;
                    remaining -= 1;
                }
            }
        }
    }

    fn solve_group_split_impulse_iterations(&mut self) {
        if self.skip_separated_special_rows {
            self.solve_group_split_impulse_iterations_one_dynamic();
            return;
        }

        let row_count = self.tmp_solver_contact_constraint_pool.len();
        self.tmp_split_should_run.clear();
        self.tmp_split_should_run.resize(row_count, true);
        let mut remaining = row_count;

        for _ in 0..contact_solver_info::NUM_ITERATIONS {
            if remaining == 0 {
                break;
            }
            for (i, contact) in self
                .tmp_solver_contact_constraint_pool
                .iter_mut()
                .enumerate()
            {
                if !self.tmp_split_should_run[i] {
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
                    self.tmp_split_should_run[i] = false;
                    remaining -= 1;
                }
            }
        }
    }

    fn solve_single_iteration_one_dynamic(&mut self) -> f32 {
        let Some(contact) = self.tmp_solver_contact_constraint_pool.last_mut() else {
            return 0.0;
        };
        debug_assert!(!contact.is_split_only);
        let body_a = &mut self.tmp_solver_body_pool[contact.solver_body_id_a];
        let residual = contact.resolve_single_constraint_row_lower_limit_one_dynamic(body_a);
        let mut least_squares_residual = residual * residual;

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
            let body_a = &mut self.tmp_solver_body_pool[contact.solver_body_id_a];
            let residual = contact.resolve_single_constraint_row_generic_one_dynamic(body_a);
            least_squares_residual = (residual * residual).max(least_squares_residual);
        }

        least_squares_residual
    }

    fn solve_single_iteration(&mut self) -> f32 {
        if self.skip_separated_special_rows {
            return self.solve_single_iteration_one_dynamic();
        }

        let mut least_squares_residual = 0.0;

        for contact in &mut self.tmp_solver_contact_constraint_pool {
            if contact.is_split_only {
                continue;
            }

            debug_assert_ne!(contact.solver_body_id_a, contact.solver_body_id_b);
            let [body_a, body_b] = unsafe {
                self.tmp_solver_body_pool.get_disjoint_unchecked_mut([
                    contact.solver_body_id_a,
                    contact.solver_body_id_b,
                ])
            };

            let residual = contact.resolve_single_constraint_row_lower_limit(body_a, body_b);
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
        for solver in &mut self.tmp_solver_body_pool {
            let Some(body) = solver.original_body.map(|idx| &mut collision_objs[idx]) else {
                continue;
            };

            solver.lin_vel += solver.delta_lin_vel;
            solver.ang_vel += solver.delta_ang_vel;

            if solver.push_vel.length_squared() != 0.0 || solver.turn_vel.length_squared() != 0.0 {
                let mut world_trans = *body.get_world_trans();
                if body.collision_flags & CollisionFlags::NoAngularMotion != 0 {
                    integrate_trans_no_rot(
                        &mut world_trans.translation,
                        solver.push_vel,
                        time_step,
                    );
                } else {
                    let mut world_rot = body.get_world_rot();
                    integrate_trans(
                        &mut world_trans,
                        &mut world_rot,
                        solver.push_vel,
                        solver.turn_vel * contact_solver_info::SPLIT_IMPULSE_TURN_ERP,
                        time_step,
                    );
                }

                body.set_world_trans(world_trans);
            }

            body.set_lin_vel(solver.lin_vel + solver.external_force_impulse);
            body.set_ang_vel(solver.ang_vel + solver.external_torque_impulse);
        }

        self.tmp_solver_body_pool.clear();
        self.tmp_solver_contact_constraint_pool.clear();
        self.tmp_solver_contact_friction_constraint_pool.clear();
        self.tmp_split_should_run.clear();
    }
}
