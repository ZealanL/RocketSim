use glam::Vec3A;

use super::{
    constraint_solver::seq_impulse_constraint_solver::SeqImpulseConstraintSolver,
    rigid_body::{ActivationState, RigidBody, RigidBodyConstructionInfo},
};
use crate::{
    bullet::{
        collision::{
            broadphase::{CollisionFilterGroups, GridBroadphase},
            dispatch::{
                collision_dispatcher::CollisionDispatcher,
                collision_world::CollisionWorld,
                ray_packet_callbacks::{
                    ClosestRayResultCallback, QuadRayCallback, RayResultCallback,
                },
            },
            narrowphase::persistent_manifold::ContactAddedCallback,
            shapes::{collision_shape::CollisionShapes, sphere_shape::SphereShape},
        },
        dynamics::rigid_body::CollisionFlags,
    },
    shared::Aabb,
};

pub struct DiscreteDynamicsWorld {
    collision_world: CollisionWorld,
    solver: SeqImpulseConstraintSolver,
    non_static_rigid_bodies: Vec<usize>,
    gravity: Vec3A,
}

impl DiscreteDynamicsWorld {
    pub fn new(pair_cache: GridBroadphase, gravity: Vec3A) -> Self {
        Self {
            collision_world: CollisionWorld::new(pair_cache),
            solver: SeqImpulseConstraintSolver::default(),
            non_static_rigid_bodies: Vec::new(),
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

    pub fn test_collision(&self, body_a: &RigidBody, body_b: &RigidBody) -> bool {
        CollisionDispatcher::test_collision(body_a, body_b)
    }

    /// Test a packet of 4 rays against all static bodies, returning whether each ray hit anything.
    pub fn test_ray_packet(&self, ray_from: &[Vec3A; 4], ray_to: &[Vec3A; 4]) -> [bool; 4] {
        let mut query_body = RigidBody::new(RigidBodyConstructionInfo::new(
            0.0,
            CollisionShapes::Sphere(SphereShape::new(0.0)),
        ));
        query_body.world_array_idx = usize::MAX;

        let ray_min = ray_from[0]
            .min(ray_from[1])
            .min(ray_from[2])
            .min(ray_from[3])
            .min(ray_to[0])
            .min(ray_to[1])
            .min(ray_to[2])
            .min(ray_to[3]);
        let ray_max = ray_from[0]
            .max(ray_from[1])
            .max(ray_from[2])
            .max(ray_from[3])
            .max(ray_to[0])
            .max(ray_to[1])
            .max(ray_to[2])
            .max(ray_to[3]);
        let ray_aabb = Aabb::new(ray_min, ray_max);

        let mut callback = ClosestRayResultCallback::new(ray_from, ray_to, &query_body);

        for body in &self.collision_world.collision_objs {
            if !body.is_static_obj() {
                continue;
            }

            let body_aabb = body.get_collision_shape().get_aabb(body.get_world_trans());

            if !ray_aabb.intersects(&body_aabb) {
                continue;
            }

            CollisionWorld::quad_ray_test(
                ray_from,
                ray_to,
                body,
                body.world_array_idx,
                &mut callback,
            );
        }

        [
            callback.has_hit(0),
            callback.has_hit(1),
            callback.has_hit(2),
            callback.has_hit(3),
        ]
    }

    pub fn ray_test<T: RayResultCallback>(
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

    pub fn add_rigid_body_default(&mut self, mut body: RigidBody) -> usize {
        if !body.is_static_obj() && (body.collision_flags & CollisionFlags::NoWorldGravity) == 0 {
            body.set_gravity(self.gravity);
        }

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
            self.non_static_rigid_bodies.push(rb_idx);
        }

        rb_idx
    }

    pub fn add_rigid_body(&mut self, mut body: RigidBody, group: u8, mask: u8) -> usize {
        if !body.is_static_obj() && (body.collision_flags & CollisionFlags::NoWorldGravity) == 0 {
            body.set_gravity(self.gravity);
        }

        let rb_idx = self.add_collision_obj(body, group, mask);

        let rb = &mut self.collision_world.collision_objs[rb_idx];
        if rb.is_static_obj() {
            rb.set_activation_state(ActivationState::Sleeping);
        } else {
            self.non_static_rigid_bodies.push(rb_idx);
        }

        rb_idx
    }

    fn apply_gravity(&mut self) {
        for &body in &self.non_static_rigid_bodies {
            let body = &mut self.collision_world.collision_objs[body];
            if body.is_active() {
                body.apply_gravity();
            }
        }
    }

    fn predict_unconstraint_motion(&mut self, time_step: f32) {
        for &body in &self.non_static_rigid_bodies {
            let body = &mut self.collision_world.collision_objs[body];
            debug_assert!(!body.is_static_obj());

            body.apply_damping(time_step);
            let predicted_trans = body.predict_integration_trans(time_step);
            body.interp_world_trans = predicted_trans;
        }
    }

    #[inline]
    fn solve_constraints(&mut self, time_step: f32) {
        self.solver.solve_group(
            &mut self.collision_world.collision_objs,
            &self.non_static_rigid_bodies,
            &mut self.collision_world.dispatcher1.manifolds,
            time_step,
        );
    }

    fn integrate_trans_internal(&mut self, time_step: f32) {
        for &body in &self.non_static_rigid_bodies {
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
        if !self.non_static_rigid_bodies.is_empty() {
            self.integrate_trans_internal(time_step);
        }
    }

    fn update_activation_state(&mut self, time_step: f32) {
        for &body in &self.non_static_rigid_bodies {
            let body = &mut self.collision_world.collision_objs[body];
            body.update_activation_state(time_step);
        }
    }

    fn clear_forces(&mut self) {
        for &body in &self.non_static_rigid_bodies {
            self.collision_world.collision_objs[body].clear_forces();
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
        self.apply_gravity();
        self.internal_single_step_simulation(time_step, contact_added_callback);

        self.clear_forces();
    }
}
