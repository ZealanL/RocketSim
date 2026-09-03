use glam::Vec3A;

use super::{contact_solver_info, solver_body::SolverBody, solver_constraint::SolverConstraint};
use crate::bullet::{
    collision::{
        narrowphase::{
            manifold_point::ManifoldPoint,
            persistent_manifold::{MANIFOLD_CACHE_SIZE, PersistentManifold},
        },
        shapes::collision_shape::CollisionShapes,
    },
    dynamics::rigid_body::{CollisionFlags, RigidBody},
    linear_math::{integrate_trans, integrate_trans_no_rot, plane_space_1},
};

/// Use Bullet's linear slop to classify shallow support contacts.
/// Treat deeper contacts as impacts.
const SPECIAL_LINEAR_SLOP: f32 = 0.04;

/// Blend samples only for upward-facing support contacts.
/// Use facet responses for steep contacts.
const SUPPORT_NORMAL_MIN_Z: f32 = 0.866;

#[derive(Clone, Copy)]
struct SpecialContact {
    obj_idx: usize,
    static_obj_idx: usize,
    pos_world_on_a: Vec3A,
    normal_world_on_b: Vec3A,
    /// Save the closest-feature normal before edge adjustment.
    raw_normal_world_on_b: Vec3A,
    /// Store the lever arm from the body center to the contact point.
    lever_arm: Vec3A,
    distance: f32,
    friction: f32,
    restitution: f32,
}

impl SpecialContact {
    fn is_penetrating(&self) -> bool {
        self.distance < 0.0
    }

    /// Return the contact speed along the normal. Negative means closing.
    fn approach_speed(&self, lin_vel: Vec3A, ang_vel: Vec3A) -> f32 {
        let contact_vel = lin_vel + ang_vel.cross(self.lever_arm);
        self.normal_world_on_b.dot(contact_vel)
    }
}

fn special_contact_from_point(
    body0: &RigidBody,
    body1: &RigidBody,
    cp: &ManifoldPoint,
    rel_pos1: Vec3A,
    rel_pos2: Vec3A,
) -> Option<SpecialContact> {
    let (obj_idx, static_obj_idx, lever_arm) = if !body0.is_static_obj() {
        (body0.world_array_idx, body1.world_array_idx, rel_pos1)
    } else if !body1.is_static_obj() {
        (body1.world_array_idx, body0.world_array_idx, rel_pos2)
    } else {
        return None;
    };

    Some(SpecialContact {
        obj_idx,
        static_obj_idx,
        pos_world_on_a: cp.pos_world_on_a,
        normal_world_on_b: cp.normal_world_on_b,
        raw_normal_world_on_b: cp.raw_normal_world_on_b,
        lever_arm,
        distance: cp.distance_1,
        friction: cp.combined_friction,
        restitution: cp.combined_restitution,
    })
}

/// Select the deepest penetrating sample.
/// Otherwise, select the fastest-closing sample.
fn dominant_contact(cluster: &[SpecialContact], lin_vel: Vec3A, ang_vel: Vec3A) -> &SpecialContact {
    let mut best: Option<&SpecialContact> = None;
    for contact in cluster {
        let take = match best {
            None => true,
            Some(b) => {
                if contact.is_penetrating() == b.is_penetrating() {
                    if contact.is_penetrating() {
                        contact.distance < b.distance
                    } else {
                        let v_contact = contact.approach_speed(lin_vel, ang_vel);
                        let v_best = b.approach_speed(lin_vel, ang_vel);
                        v_contact < v_best
                    }
                } else {
                    // Prefer penetrating samples.
                    contact.is_penetrating()
                }
            }
        };
        if take {
            best = Some(contact);
        }
    }
    best.unwrap()
}

/// Group nearby samples from one dynamic body as one physical touch.
const SPECIAL_CONTACT_CLUSTER_LEVER_FRACTION: f32 = 0.02;

pub struct SeqImpulseConstraintSolver {
    tmp_solver_body_pool: Vec<SolverBody>,
    tmp_solver_contact_constraint_pool: Vec<SolverConstraint>,
    tmp_solver_contact_friction_constraint_pool: Vec<SolverConstraint>,
    fixed_body_id: Option<usize>,
    least_squares_residual: f32,
    tmp_special_contact_pool: Vec<SpecialContact>,
    tmp_special_contact_group_ends: Vec<usize>,
    tmp_special_resolved_touches: Vec<SpecialContact>,
    tmp_special_cluster_pool: Vec<SpecialContact>,
    tmp_special_cluster_ranges: Vec<(usize, usize)>,
    tmp_special_representatives: Vec<SpecialContact>,
    tmp_special_shallow_reduced: Vec<SpecialContact>,
    tmp_special_shallow_edge_group_pool: Vec<SpecialContact>,
    tmp_special_shallow_edge_group_ranges: Vec<(usize, usize)>,
    tmp_special_shallow_deduplicated: Vec<SpecialContact>,
}

impl Default for SeqImpulseConstraintSolver {
    fn default() -> Self {
        Self {
            tmp_solver_body_pool: Vec::new(),
            tmp_solver_contact_constraint_pool: Vec::new(),
            tmp_solver_contact_friction_constraint_pool: Vec::new(),
            fixed_body_id: None,
            least_squares_residual: 0.0,
            tmp_special_contact_pool: Vec::new(),
            tmp_special_contact_group_ends: Vec::new(),
            tmp_special_resolved_touches: Vec::new(),
            tmp_special_cluster_pool: Vec::new(),
            tmp_special_cluster_ranges: Vec::new(),
            tmp_special_representatives: Vec::new(),
            tmp_special_shallow_reduced: Vec::new(),
            tmp_special_shallow_edge_group_pool: Vec::new(),
            tmp_special_shallow_edge_group_ranges: Vec::new(),
            tmp_special_shallow_deduplicated: Vec::new(),
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
        self.tmp_special_contact_pool.clear();
        self.tmp_special_contact_group_ends.clear();
        self.tmp_special_resolved_touches.clear();

        for manifold in manifolds.iter_mut() {
            let evicted_point = manifold.most_recently_evicted_point;
            let special_contact_start = self.tmp_special_contact_pool.len();

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

            let is_compound_mesh = matches!(
                (body0.get_collision_shape(), body1.get_collision_shape()),
                (
                    CollisionShapes::Compound(_),
                    CollisionShapes::TriangleMesh(_)
                ) | (
                    CollisionShapes::TriangleMesh(_),
                    CollisionShapes::Compound(_)
                )
            );
            let restitution_velocity_threshold = if is_compound_mesh {
                contact_solver_info::COMPOUND_MESH_RESTITUTION_VELOCITY_THRESHOLD
            } else {
                contact_solver_info::RESTITUTION_VELOCITY_THRESHOLD
            };

            for cp in &mut manifold.point_cache {
                assert!(cp.distance_1 <= manifold.contact_processing_threshold);

                let rel_pos1 = cp.pos_world_on_a - body0.get_world_trans().translation;
                let rel_pos2 = cp.pos_world_on_b - body1.get_world_trans().translation;

                if cp.is_special {
                    if let Some(contact) =
                        special_contact_from_point(body0, body1, cp, rel_pos1, rel_pos2)
                    {
                        self.tmp_special_contact_pool.push(contact);
                    }

                    // Process special contacts separately.
                    continue;
                }

                let rb0 = solver_body_a.original_body.map(|_| &*body0);
                let rb1 = solver_body_b.original_body.map(|_| &*body1);
                let friction_idx = self.tmp_solver_contact_constraint_pool.len();

                self.tmp_solver_contact_constraint_pool.push(
                    SolverConstraint::get_contact_constraint(
                        (solver_body_id_a, solver_body_id_b),
                        (solver_body_a, solver_body_b),
                        (rb0, rb1),
                        (rel_pos1, rel_pos2),
                        cp,
                        friction_idx,
                        restitution_velocity_threshold,
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

            let evicted_special_contact = evicted_point.and_then(|cp| {
                if !cp.is_special {
                    return None;
                }

                let rel_pos1 = cp.pos_world_on_a - body0.get_world_trans().translation;
                let rel_pos2 = cp.pos_world_on_b - body1.get_world_trans().translation;
                special_contact_from_point(body0, body1, &cp, rel_pos1, rel_pos2)
            });

            if let Some(evicted_contact) = evicted_special_contact {
                replace_shallower_duplicate(
                    &mut self.tmp_special_contact_pool[special_contact_start..],
                    evicted_contact,
                );
            }

            if self.tmp_special_contact_pool.len() != special_contact_start {
                self.tmp_special_contact_group_ends
                    .push(self.tmp_special_contact_pool.len());
            }
        }

        manifolds.clear();

        let special_contacts = std::mem::take(&mut self.tmp_special_contact_pool);
        if special_contacts
            .iter()
            .all(|contact| !contact.is_penetrating())
        {
            if !special_contacts.is_empty() {
                self.convert_contact_special(collision_objs, &special_contacts, time_step);
            }
        } else {
            // Resolve each physical touch once across duplicate mesh groups.
            let mut group_start = 0;
            for group_idx in 0..self.tmp_special_contact_group_ends.len() {
                let group_end = self.tmp_special_contact_group_ends[group_idx];
                let group = &special_contacts[group_start..group_end];
                let has_impact = group.iter().any(|contact| contact.is_penetrating());
                if has_impact {
                    let is_duplicate_view = group.iter().all(|contact| {
                        self.tmp_special_resolved_touches
                            .iter()
                            .any(|resolved| same_touch(resolved, contact))
                    });
                    if is_duplicate_view {
                        group_start = group_end;
                        continue;
                    }

                    self.tmp_special_resolved_touches.extend_from_slice(group);
                }

                self.convert_contact_special(collision_objs, group, time_step);
                group_start = group_end;
            }
        }

        self.tmp_special_contact_group_ends.clear();
        self.tmp_special_contact_pool = special_contacts;
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

    fn convert_contact_special(
        &mut self,
        collision_objs: &[RigidBody],
        special_contacts: &[SpecialContact],
        time_step: f32,
    ) {
        // Reduce duplicate reports before classifying shallow support contacts.
        // Use one representative for each deep physical touch.
        let has_reduced_contacts = reduce_shallow_support_contacts(
            special_contacts,
            &mut self.tmp_special_shallow_reduced,
            &mut self.tmp_special_shallow_edge_group_pool,
            &mut self.tmp_special_shallow_edge_group_ranges,
            &mut self.tmp_special_shallow_deduplicated,
        );

        let aggregate = {
            let classification_contacts: &[SpecialContact] = if has_reduced_contacts {
                &self.tmp_special_shallow_deduplicated
            } else {
                special_contacts
            };
            let min_distance = classification_contacts
                .iter()
                .map(|contact| contact.distance)
                .fold(f32::MAX, f32::min);
            let mut total_classification_normal = Vec3A::ZERO;
            for contact in classification_contacts {
                total_classification_normal += if has_reduced_contacts {
                    contact.normal_world_on_b
                } else {
                    contact.raw_normal_world_on_b
                };
            }
            let classification_mean =
                total_classification_normal / classification_contacts.len() as f32;
            let all_non_penetrating = special_contacts
                .iter()
                .all(|contact| !contact.is_penetrating());

            if all_non_penetrating
                || (min_distance > -SPECIAL_LINEAR_SLOP
                    && classification_mean.z >= SUPPORT_NORMAL_MIN_Z)
            {
                let aggregate_contacts = classification_contacts;
                let mut total_normal = Vec3A::ZERO;
                let mut total_lever_len = 0.0;
                for contact in aggregate_contacts {
                    total_normal += if has_reduced_contacts {
                        contact.normal_world_on_b
                    } else {
                        contact.raw_normal_world_on_b
                    };
                    total_lever_len += contact.lever_arm.length();
                }

                let num_samples = aggregate_contacts.len() as f32;
                SpecialAggregate {
                    normal_world_on_b: (total_normal / num_samples).normalize(),
                    lever_len: total_lever_len / num_samples,
                    min_distance,
                    obj_idx: special_contacts[0].obj_idx,
                }
            } else {
                // Group deep samples by physical touch.
                // Use one representative for each touch.
                self.tmp_special_cluster_pool.clear();
                self.tmp_special_cluster_ranges.clear();
                self.tmp_special_representatives.clear();
                self.tmp_special_cluster_pool
                    .reserve(special_contacts.len());
                self.tmp_special_cluster_ranges
                    .reserve(special_contacts.len());
                self.tmp_special_representatives
                    .reserve(special_contacts.len());

                for contact in special_contacts {
                    let matching_cluster =
                        self.tmp_special_cluster_ranges.iter().enumerate().find_map(
                            |(cluster_idx, &(start, end))| {
                                if self.tmp_special_cluster_pool[start..end]
                                    .iter()
                                    .any(|member| same_touch(member, contact))
                                {
                                    Some(cluster_idx)
                                } else {
                                    None
                                }
                            },
                        );

                    if let Some(cluster_idx) = matching_cluster {
                        let insert_at = self.tmp_special_cluster_ranges[cluster_idx].1;
                        self.tmp_special_cluster_pool.insert(insert_at, *contact);
                        for (range_idx, range) in self
                            .tmp_special_cluster_ranges
                            .iter_mut()
                            .enumerate()
                            .skip(cluster_idx)
                        {
                            range.1 += 1;
                            if range_idx > cluster_idx {
                                range.0 += 1;
                            }
                        }
                    } else {
                        let start = self.tmp_special_cluster_pool.len();
                        self.tmp_special_cluster_pool.push(*contact);
                        self.tmp_special_cluster_ranges.push((start, start + 1));
                    }
                }

                for cluster_idx in 0..self.tmp_special_cluster_ranges.len() {
                    let representative = {
                        let (start, end) = self.tmp_special_cluster_ranges[cluster_idx];
                        let cluster = &self.tmp_special_cluster_pool[start..end];
                        // Use the touched body's velocities for every sample in the cluster.
                        let body = &collision_objs[cluster[0].obj_idx];
                        let solver_body = &self.tmp_solver_body_pool[body.companion_id.unwrap()];
                        *dominant_contact(cluster, solver_body.lin_vel, solver_body.ang_vel)
                    };
                    self.tmp_special_representatives.push(representative);
                }

                // Ignore stationary touches when another touch approaches.
                const STATIONARY_SPEED: f32 = 0.01;
                let any_approaching = self.tmp_special_representatives.iter().any(|contact| {
                    let body = &collision_objs[contact.obj_idx];
                    let solver_body = &self.tmp_solver_body_pool[body.companion_id.unwrap()];
                    contact.approach_speed(solver_body.lin_vel, solver_body.ang_vel)
                        < -STATIONARY_SPEED
                });
                if any_approaching {
                    self.tmp_special_representatives.retain(|contact| {
                        let body = &collision_objs[contact.obj_idx];
                        let solver_body = &self.tmp_solver_body_pool[body.companion_id.unwrap()];
                        contact.approach_speed(solver_body.lin_vel, solver_body.ang_vel)
                            < -STATIONARY_SPEED
                    });
                }

                let mut total_normal = Vec3A::ZERO;
                let mut total_lever_len = 0.0;
                for representative in &self.tmp_special_representatives {
                    total_normal += representative.normal_world_on_b;
                    total_lever_len += representative.lever_arm.length();
                }

                let num_representatives = self.tmp_special_representatives.len() as f32;
                SpecialAggregate {
                    normal_world_on_b: (total_normal / num_representatives).normalize(),
                    lever_len: total_lever_len / num_representatives,
                    min_distance: self
                        .tmp_special_representatives
                        .iter()
                        .map(|contact| contact.distance)
                        .fold(f32::MAX, f32::min),
                    obj_idx: self.tmp_special_representatives[0].obj_idx,
                }
            }
        };

        let contact = SpecialContact {
            normal_world_on_b: aggregate.normal_world_on_b,
            lever_arm: aggregate.normal_world_on_b * -aggregate.lever_len,
            distance: aggregate.min_distance,
            friction: special_contacts[0].friction,
            restitution: special_contacts[0].restitution,
            obj_idx: aggregate.obj_idx,
            ..special_contacts[0]
        };

        let body = &collision_objs[contact.obj_idx];
        let relaxation = contact_solver_info::SOR;

        let solver_body_id_a = body.companion_id.unwrap();
        let solver_body_id_b = if let Some(fixed_body_id) = self.fixed_body_id {
            fixed_body_id
        } else {
            let solver_body_id = self.tmp_solver_body_pool.len();
            self.fixed_body_id = Some(solver_body_id);

            self.tmp_solver_body_pool.push(SolverBody::DEFAULT);
            solver_body_id
        };

        let normal_world_on_b = contact.normal_world_on_b;
        let rel_pos1 = contact.lever_arm;
        let penetration = contact.distance;

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

        let vel = body.get_vel_in_local_point(rel_pos1);
        let rel_vel = normal_world_on_b.dot(vel);

        // Use a low threshold for special contacts.
        // Preserve shallow impacts without bouncing at rest.
        let restitution =
            if rel_vel.abs() >= contact_solver_info::SPECIAL_RESTITUTION_VELOCITY_THRESHOLD {
                (contact.restitution * -rel_vel).max(0.0)
            } else {
                0.0
            };

        let (external_force_impulse_a, external_torque_impulse_a) = {
            let solver_body_a = &self.tmp_solver_body_pool[solver_body_id_a];
            (
                solver_body_a.external_force_impulse,
                solver_body_a.external_torque_impulse,
            )
        };

        let rel_vel = {
            let solver_body_a = &self.tmp_solver_body_pool[solver_body_id_a];
            contact_normal_1.dot(solver_body_a.lin_vel + external_force_impulse_a)
                + rel_pos1_cross_normal.dot(solver_body_a.ang_vel + external_torque_impulse_a)
        };

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

        let friction_idx = self.tmp_solver_contact_constraint_pool.len();

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
                friction: contact.friction,
                lower_limit: 0.0,
                upper_limit: 1e10,
                ..Default::default()
            });

        let vel = {
            let solver_body_a = &self.tmp_solver_body_pool[solver_body_id_a];
            solver_body_a.get_vel_in_local_point_no_delta(rel_pos1)
        };
        let rel_vel = normal_world_on_b.dot(vel);

        let mut lateral_friction_dir_1 = vel - normal_world_on_b * rel_vel;
        let lat_rel_vel = lateral_friction_dir_1.length_squared();

        if lat_rel_vel > f32::EPSILON {
            lateral_friction_dir_1 *= 1.0 / lat_rel_vel.sqrt();
        } else {
            lateral_friction_dir_1 = plane_space_1(normal_world_on_b);
        }

        // Add the friction constraint.
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

        let rel_vel = {
            let solver_body_a = &self.tmp_solver_body_pool[solver_body_id_a];
            contact_normal_1.dot(solver_body_a.lin_vel + external_force_impulse_a)
                + rel_pos1_cross_normal.dot(solver_body_a.ang_vel + external_torque_impulse_a)
        };

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
                lower_limit: -contact.friction,
                upper_limit: contact.friction,
                friction: contact.friction,
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

        for contact in &mut self.tmp_solver_contact_constraint_pool {
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
        // Write back body state.
        for solver in &mut self.tmp_solver_body_pool {
            let Some(body) = solver.original_body.map(|idx| &mut collision_objs[idx]) else {
                continue;
            };

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

/// Store the aggregate response for one special contact region.
struct SpecialAggregate {
    normal_world_on_b: Vec3A,
    lever_len: f32,
    min_distance: f32,
    obj_idx: usize,
}

/// Return whether two samples belong to one physical touch.
fn same_touch(a: &SpecialContact, b: &SpecialContact) -> bool {
    if a.obj_idx != b.obj_idx {
        return false;
    }

    let dx = (a.pos_world_on_a.x - b.pos_world_on_a.x)
        .abs()
        .min((a.pos_world_on_a.x + b.pos_world_on_a.x).abs());
    let dy = (a.pos_world_on_a.y - b.pos_world_on_a.y)
        .abs()
        .min((a.pos_world_on_a.y + b.pos_world_on_a.y).abs());
    let dz = (a.pos_world_on_a.z - b.pos_world_on_a.z).abs();
    let tolerance = a.lever_arm.length() * SPECIAL_CONTACT_CLUSTER_LEVER_FRACTION;
    dx * dx + dy * dy + dz * dz <= tolerance * tolerance
}

const SPECIAL_NORMAL_ADJUSTMENT_EPSILON: f32 = 1e-4;

fn same_contact_position(a: &SpecialContact, b: &SpecialContact) -> bool {
    let tolerance =
        a.lever_arm.length().max(b.lever_arm.length()) * SPECIAL_CONTACT_CLUSTER_LEVER_FRACTION;
    (a.pos_world_on_a - b.pos_world_on_a).length_squared() <= tolerance * tolerance
}

fn same_adjusted_normal(a: &SpecialContact, b: &SpecialContact) -> bool {
    a.normal_world_on_b.dot(b.normal_world_on_b) >= 1.0 - SPECIAL_NORMAL_ADJUSTMENT_EPSILON
}

fn replace_shallower_duplicate(
    special_contacts: &mut [SpecialContact],
    evicted_contact: SpecialContact,
) {
    if special_contacts.len() != MANIFOLD_CACHE_SIZE
        || special_contacts
            .iter()
            .any(|contact| same_adjusted_normal(contact, &evicted_contact))
    {
        return;
    }

    for first_idx in 0..special_contacts.len() {
        for second_idx in first_idx + 1..special_contacts.len() {
            if !same_adjusted_normal(&special_contacts[first_idx], &special_contacts[second_idx]) {
                continue;
            }

            let replacement_idx =
                if special_contacts[first_idx].distance > special_contacts[second_idx].distance {
                    first_idx
                } else {
                    second_idx
                };
            special_contacts[replacement_idx] = evicted_contact;
            return;
        }
    }
}

fn is_edge_adjusted(contact: &SpecialContact) -> bool {
    contact.raw_normal_world_on_b.dot(contact.normal_world_on_b)
        < 1.0 - SPECIAL_NORMAL_ADJUSTMENT_EPSILON
}

/// Reduce duplicate reports before aggregating shallow contacts.
/// Keep one representative for each physical feature.
fn reduce_shallow_support_contacts(
    special_contacts: &[SpecialContact],
    reduced: &mut Vec<SpecialContact>,
    edge_group_contacts: &mut Vec<SpecialContact>,
    edge_group_ranges: &mut Vec<(usize, usize)>,
    deduplicated: &mut Vec<SpecialContact>,
) -> bool {
    reduced.clear();
    edge_group_contacts.clear();
    edge_group_ranges.clear();
    deduplicated.clear();

    if !special_contacts.iter().any(is_edge_adjusted) {
        return false;
    }

    reduced.reserve(special_contacts.len());
    edge_group_contacts.reserve(special_contacts.len());
    edge_group_ranges.reserve(special_contacts.len());
    deduplicated.reserve(special_contacts.len());

    for contact in special_contacts {
        if is_edge_adjusted(contact) {
            continue;
        }

        let duplicate = reduced.iter().any(|existing| {
            existing.static_obj_idx == contact.static_obj_idx
                && same_contact_position(existing, contact)
                && same_adjusted_normal(existing, contact)
        });
        if !duplicate {
            reduced.push(*contact);
        }
    }

    for contact in special_contacts {
        if !is_edge_adjusted(contact) {
            continue;
        }

        let matching_group =
            edge_group_ranges
                .iter()
                .enumerate()
                .find_map(|(group_idx, &(start, _))| {
                    if edge_group_contacts[start].static_obj_idx == contact.static_obj_idx
                        && same_contact_position(&edge_group_contacts[start], contact)
                    {
                        Some(group_idx)
                    } else {
                        None
                    }
                });

        if let Some(group_idx) = matching_group {
            let insert_at = edge_group_ranges[group_idx].1;
            edge_group_contacts.insert(insert_at, *contact);
            for (range_idx, range) in edge_group_ranges.iter_mut().enumerate().skip(group_idx) {
                range.1 += 1;
                if range_idx > group_idx {
                    range.0 += 1;
                }
            }
        } else {
            let start = edge_group_contacts.len();
            edge_group_contacts.push(*contact);
            edge_group_ranges.push((start, start + 1));
        }
    }

    for &(start, end) in edge_group_ranges.iter() {
        let group = &edge_group_contacts[start..end];
        let mut adjusted_normal = Vec3A::ZERO;
        for contact in group {
            adjusted_normal += contact.normal_world_on_b;
        }
        adjusted_normal = adjusted_normal.normalize();

        let all_normals_agree = group.iter().all(|contact| {
            contact.normal_world_on_b.dot(adjusted_normal)
                >= 1.0 - SPECIAL_NORMAL_ADJUSTMENT_EPSILON
        });

        if all_normals_agree {
            let mut representative = group[0];
            representative.normal_world_on_b = adjusted_normal;
            reduced.push(representative);
            continue;
        }

        let has_stable_owner = group.iter().all(|edge| {
            special_contacts.iter().any(|facet| {
                !is_edge_adjusted(facet)
                    && facet.static_obj_idx == edge.static_obj_idx
                    && same_adjusted_normal(facet, edge)
            })
        });
        if !has_stable_owner {
            let representative = group
                .iter()
                .min_by(|a, b| a.distance.total_cmp(&b.distance))
                .copied()
                .unwrap();
            reduced.push(representative);
        }
    }

    for contact in reduced.iter() {
        let duplicate_penetrating_feature = contact.is_penetrating()
            && deduplicated.iter().any(|existing| {
                existing.is_penetrating()
                    && existing.static_obj_idx != contact.static_obj_idx
                    && same_contact_position(existing, contact)
                    && same_adjusted_normal(existing, contact)
            });
        if !duplicate_penetrating_feature {
            deduplicated.push(*contact);
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use glam::Vec3A;

    use super::{SpecialContact, replace_shallower_duplicate};

    fn contact(normal: Vec3A, distance: f32, marker: f32) -> SpecialContact {
        SpecialContact {
            obj_idx: 1,
            static_obj_idx: 2,
            pos_world_on_a: Vec3A::new(marker, 0.0, 0.0),
            normal_world_on_b: normal,
            raw_normal_world_on_b: normal,
            lever_arm: Vec3A::Z,
            distance,
            friction: 0.5,
            restitution: 0.0,
        }
    }

    #[test]
    fn evicted_contact_replaces_shallower_duplicate() {
        let evicted = contact(Vec3A::Z, -0.08, 9.0);
        let mut contacts = [
            contact(Vec3A::X, -0.05, 1.0),
            contact(Vec3A::X, -0.20, 2.0),
            contact(Vec3A::Y, -0.10, 3.0),
            contact(Vec3A::new(0.0, 0.0, -1.0), -0.10, 4.0),
        ];

        replace_shallower_duplicate(&mut contacts, evicted);

        assert_eq!(contacts[0].pos_world_on_a, evicted.pos_world_on_a);
        assert_eq!(contacts[0].normal_world_on_b, evicted.normal_world_on_b);
        assert_eq!(contacts[0].distance, evicted.distance);
        assert_eq!(contacts[1].pos_world_on_a.x, 2.0);
        assert_eq!(contacts[1].normal_world_on_b, Vec3A::X);
        assert_eq!(contacts[1].distance, -0.20);
        assert_eq!(contacts[2].pos_world_on_a.x, 3.0);
        assert_eq!(contacts[3].pos_world_on_a.x, 4.0);
    }
}
