use glam::Vec3A;

use super::{
    constraint_solver::seq_impulse_constraint_solver::SeqImpulseConstraintSolver,
    rigid_body::{ActivationState, RigidBody},
};
use crate::bullet::{
    collision::{
        broadphase::{CollisionFilterGroups, GridBroadphase},
        dispatch::{
            collision_world::CollisionWorld,
            quad_ray_callbacks::{QuadRayCallback, QuadRayResultCallback},
        },
        narrowphase::persistent_manifold::ContactAddedCallback,
    },
    dynamics::rigid_body::Impulse,
};

pub struct DiscreteDynamicsWorld {
    collision_world: CollisionWorld,
    solver: SeqImpulseConstraintSolver,
    dynamic_body_idcs: Vec<usize>,
    gravity: Vec3A,
}

impl DiscreteDynamicsWorld {
    pub fn new(pair_cache: GridBroadphase, gravity: Vec3A) -> Self {
        Self {
            collision_world: CollisionWorld::new(pair_cache),
            solver: SeqImpulseConstraintSolver::default(),
            dynamic_body_idcs: Vec::new(),
            gravity,
        }
    }

    #[inline]
    pub fn bodies_mut(&mut self) -> &mut [RigidBody] {
        &mut self.collision_world.collision_objs
    }

    #[inline]
    pub fn bodies(&self) -> &[RigidBody] {
        &self.collision_world.collision_objs
    }

    pub fn ray_test<T: QuadRayResultCallback>(
        &self,
        ray_from_world: &[Vec3A; 4],
        ray_to_world: &[Vec3A; 4],
        result_callback: &mut T,
    ) {
        let mut ray_cb = QuadRayCallback::new(
            ray_from_world,
            ray_to_world,
            &self.collision_world,
            result_callback,
        );

        self.collision_world.broadphase_pair_cache.ray_test(
            ray_from_world,
            ray_to_world,
            &mut ray_cb,
        );
    }

    #[inline]
    fn add_collision_obj(&mut self, body: RigidBody, group: u8, mask: u8) -> usize {
        self.collision_world.add_collision_obj(body, group, mask)
    }

    pub fn add_rigid_body_default(&mut self, body: RigidBody) -> usize {
        let (group, mask) = if body.is_static_obj() {
            (
                CollisionFilterGroups::Static as u8,
                CollisionFilterGroups::ALL ^ CollisionFilterGroups::Static,
            )
        } else {
            (
                CollisionFilterGroups::Default as u8,
                CollisionFilterGroups::ALL,
            )
        };

        let rb_idx = self.add_collision_obj(body, group, mask);

        let rb = &mut self.collision_world.collision_objs[rb_idx];
        if rb.is_static_obj() {
            rb.set_activation_state(ActivationState::Sleeping);
        } else {
            self.dynamic_body_idcs.push(rb_idx);
        }

        rb_idx
    }

    pub fn add_rigid_body(&mut self, body: RigidBody, group: u8, mask: u8) -> usize {
        let rb_idx = self.add_collision_obj(body, group, mask);

        let rb = &mut self.collision_world.collision_objs[rb_idx];
        if rb.is_static_obj() {
            rb.set_activation_state(ActivationState::Sleeping);
        } else {
            self.dynamic_body_idcs.push(rb_idx);
        }

        rb_idx
    }

    fn apply_gravity(&mut self, time_step: f32) {
        for &body in &self.dynamic_body_idcs {
            let body = &mut self.collision_world.collision_objs[body];
            if body.is_active() {
                body.add_impulse(None, Impulse::Linear(self.gravity * time_step), false, true);
            }
        }
    }

    fn predict_unconstraint_motion(&mut self, time_step: f32) {
        for &body in &self.dynamic_body_idcs {
            let body = &mut self.collision_world.collision_objs[body];
            debug_assert!(!body.is_static_obj());

            body.apply_damping(time_step);
            let predicted_trans = body.predict_integration_trans(time_step);
            body.interp_world_trans = predicted_trans;
        }
    }

    #[inline]
    fn solve_constraints(&mut self, time_step: f32) {
        // Disjoint dispatcher fields: the solver reads the persistent
        // manifolds for this tick's active pair indices.
        let dispatcher = &mut self.collision_world.dispatcher1;
        self.solver.solve_group(
            &mut self.collision_world.collision_objs,
            &self.dynamic_body_idcs,
            &mut dispatcher.persistent_manifolds,
            &mut dispatcher.active_manifolds,
            time_step,
        );
    }

    fn integrate_trans_internal(&mut self, time_step: f32) {
        for &body in &self.dynamic_body_idcs {
            let body = &mut self.collision_world.collision_objs[body];

            debug_assert!(!body.is_static_obj());
            if !body.is_active() {
                continue;
            }

            let predicted_trans = body.predict_integration_trans(time_step);
            body.set_center_of_mass_trans(predicted_trans);
        }
    }

    fn integrate_trans(&mut self, time_step: f32) {
        if !self.dynamic_body_idcs.is_empty() {
            self.integrate_trans_internal(time_step);
        }
    }

    fn update_activation_state(&mut self, time_step: f32) {
        for &body in &self.dynamic_body_idcs {
            let body = &mut self.collision_world.collision_objs[body];
            body.update_activation_state(time_step);
        }
    }

    pub fn clear_accum_forces(&mut self) {
        for &body in &self.dynamic_body_idcs {
            self.collision_world.collision_objs[body].clear_accum_vels();
        }
    }

    fn internal_single_step_simulation<T: ContactAddedCallback>(
        &mut self,
        time_step: f32,
        contact_added_callback: &mut T,
    ) {
        self.predict_unconstraint_motion(time_step);

        self.collision_world
            .perform_discrete_collision_detection(contact_added_callback);

        self.solve_constraints(time_step);
        self.integrate_trans(time_step);
        self.update_activation_state(time_step);
    }

    pub fn step_simulation<T: ContactAddedCallback>(
        &mut self,
        time_step: f32,
        contact_added_callback: &mut T,
    ) {
        self.apply_gravity(time_step);
        self.internal_single_step_simulation(time_step, contact_added_callback);
    }
}
