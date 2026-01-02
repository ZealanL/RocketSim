use glam::Vec3A;

use super::{
    constraint_solver::seq_impulse_constraint_solver::SeqImpulseConstraintSolver,
    rigid_body::{RigidBody, RigidBodyFlags},
    world::DynamicsWorld,
};
use crate::bullet::collision::{
    broadphase::{CollisionFilterGroups, GridBroadphase},
    dispatch::{collision_dispatcher::CollisionDispatcher, collision_object::ActivationState},
    narrowphase::persistent_manifold::ContactAddedCallback,
};

pub struct DiscreteDynamicsWorld {
    pub dynamics_world: DynamicsWorld,
    solver: SeqImpulseConstraintSolver,
    non_static_rigid_bodies: Vec<usize>,
    gravity: Vec3A,
    apply_speculative_contact_restitution: bool,
}

impl DiscreteDynamicsWorld {
    #[must_use]
    pub fn new(
        dispatcher: CollisionDispatcher,
        pair_cache: GridBroadphase,
        constraint_solver: SeqImpulseConstraintSolver,
    ) -> Self {
        Self {
            dynamics_world: DynamicsWorld::new(dispatcher, pair_cache),
            solver: constraint_solver,
            gravity: Vec3A::new(0.0, -10.0, 0.0),
            apply_speculative_contact_restitution: false,
            non_static_rigid_bodies: Vec::with_capacity(8),
        }
    }

    #[inline]
    pub fn bodies_mut(&mut self) -> &mut [RigidBody] {
        &mut self.dynamics_world.collision_world.collision_objects
    }

    #[inline]
    pub fn bodies(&self) -> &[RigidBody] {
        &self.dynamics_world.collision_world.collision_objects
    }

    pub const fn set_gravity(&mut self, gravity: Vec3A) {
        self.gravity = gravity;
    }

    fn add_collision_object(&mut self, body: RigidBody, group: u8, mask: u8) -> usize {
        self.dynamics_world
            .collision_world
            .add_collision_object(body, group, mask)
    }

    pub fn add_rigid_body_default(&mut self, mut body: RigidBody) -> usize {
        if !body.co.is_static_object()
            && body.get_flags() & RigidBodyFlags::DisableWorldGravity as u8 == 0
        {
            body.set_gravity(self.gravity);
        }

        let (group, mask) = if body.co.is_static_object() {
            (
                CollisionFilterGroups::Static as u8,
                CollisionFilterGroups::All as u8 ^ CollisionFilterGroups::Static as u8,
            )
        } else {
            (
                CollisionFilterGroups::Default as u8,
                CollisionFilterGroups::All as u8,
            )
        };

        let rb_idx = self.add_collision_object(body, group, mask);

        let rb = &mut self.dynamics_world.collision_world.collision_objects[rb_idx];
        if rb.co.is_static_object() {
            rb.co
                .set_activation_state(ActivationState::Sleeping);
        } else {
            self.non_static_rigid_bodies.push(rb_idx);
        }

        rb_idx
    }

    pub fn add_rigid_body(&mut self, mut body: RigidBody, group: u8, mask: u8) -> usize {
        if !body.co.is_static_object()
            && body.get_flags() & RigidBodyFlags::DisableWorldGravity as u8 == 0
        {
            body.set_gravity(self.gravity);
        }

        let rb_idx = self.add_collision_object(body, group, mask);

        let rb = &mut self.dynamics_world.collision_world.collision_objects[rb_idx];
        if rb.co.is_static_object() {
            rb.co
                .set_activation_state(ActivationState::Sleeping);
        } else {
            self.non_static_rigid_bodies.push(rb_idx);
        }

        rb_idx
    }

    fn apply_gravity(&mut self) {
        for &body in &self.non_static_rigid_bodies {
            let body = &mut self.dynamics_world.collision_world.collision_objects[body];
            if body.co.is_active() {
                body.apply_gravity();
            }
        }
    }

    fn predict_unconstraint_motion(&mut self, time_step: f32) {
        for &body in &self.non_static_rigid_bodies {
            let body = &mut self.dynamics_world.collision_world.collision_objects[body];
            debug_assert!(!body.co.is_static_object());

            body.apply_damping(time_step);
            let predicted_trans = body.predict_integration_trans(time_step);
            body.co.interp_world_trans = predicted_trans;
        }
    }

    fn solve_constraints(&mut self, time_step: f32) {
        self.solver.solve_group(
            &mut self.dynamics_world.collision_world.collision_objects,
            &self.non_static_rigid_bodies,
            &mut self.dynamics_world.collision_world.dispatcher1.manifolds,
            time_step,
        );
    }

    fn integrate_transs_internal(&mut self, time_step: f32) {
        for &body in &self.non_static_rigid_bodies {
            let body = &mut self.dynamics_world.collision_world.collision_objects[body];

            debug_assert!(!body.co.is_static_object());
            if !body.co.is_active() {
                continue;
            }

            let predicted_trans = body.predict_integration_trans(time_step);
            body.set_center_of_mass_trans(predicted_trans);
        }
    }

    fn integrate_transs(&mut self, time_step: f32) {
        if !self.non_static_rigid_bodies.is_empty() {
            self.integrate_transs_internal(time_step);
        }

        debug_assert!(!self.apply_speculative_contact_restitution);
    }

    fn update_activation_state(&mut self, time_step: f32) {
        for &body in &self.non_static_rigid_bodies {
            let body = &mut self.dynamics_world.collision_world.collision_objects[body];
            body.update_activation_state(time_step);
        }
    }

    fn clear_forces(&mut self) {
        for &body in &self.non_static_rigid_bodies {
            self.dynamics_world.collision_world.collision_objects[body].clear_forces();
        }
    }

    fn internal_single_step_simulation<T: ContactAddedCallback>(
        &mut self,
        time_step: f32,
        contact_added_callback: &mut T,
    ) {
        self.predict_unconstraint_motion(time_step);

        self.dynamics_world
            .collision_world
            .perform_discrete_collision_detection(contact_added_callback);

        self.solve_constraints(time_step);
        self.integrate_transs(time_step);
        self.update_activation_state(time_step);
    }

    pub fn step_simulation<T: ContactAddedCallback>(
        &mut self,
        time_step: f32,
        contact_added_callback: &mut T,
    ) {
        self.apply_gravity();
        self.internal_single_step_simulation(time_step, contact_added_callback);

        self.clear_forces();
    }
}
