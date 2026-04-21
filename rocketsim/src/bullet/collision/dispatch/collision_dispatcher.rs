use super::{
    collision_obj_wrapper::RigidBodyWrapper, compound_collision_alg, convex_concave_collision_alg,
    convex_plane_collision_alg, obb_obb_collision_alg, sphere_concave_collision_alg,
    sphere_obb_collision_alg,
};
use crate::bullet::{
    collision::{
        broadphase::{GridBroadphase, GridBroadphaseProxy},
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::collision_shape::CollisionShapes,
    },
    dynamics::rigid_body::RigidBody,
};

pub struct CollisionDispatcher {
    pub manifolds: Vec<PersistentManifold>,
}

impl Default for CollisionDispatcher {
    fn default() -> Self {
        Self {
            manifolds: Vec::with_capacity(8),
        }
    }
}

impl CollisionDispatcher {
    fn process_collision<'a, T: ContactAddedCallback>(
        col_obj_a: &'a RigidBody,
        col_obj_b: &'a RigidBody,
        contact_added_callback: &'a mut T,
    ) -> Option<PersistentManifold> {
        match col_obj_a.get_collision_shape() {
            CollisionShapes::StaticPlane(plane) => match col_obj_b.get_collision_shape() {
                CollisionShapes::Sphere(_) => convex_plane_collision_alg::process_collision(
                    RigidBodyWrapper {
                        obj: col_obj_b,
                        world_trans: *col_obj_b.get_world_trans(),
                    },
                    col_obj_a,
                    plane,
                    contact_added_callback,
                ),
                CollisionShapes::Compound(compound) => compound_collision_alg::process_collision(
                    col_obj_b,
                    compound,
                    col_obj_a,
                    contact_added_callback,
                ),
                CollisionShapes::ConvexHull(_) => convex_plane_collision_alg::process_collision(
                    RigidBodyWrapper {
                        obj: col_obj_b,
                        world_trans: *col_obj_b.get_world_trans(),
                    },
                    col_obj_a,
                    plane,
                    contact_added_callback,
                ),
                _ => unreachable!(),
            },
            CollisionShapes::Sphere(sphere) => match col_obj_b.get_collision_shape() {
                CollisionShapes::StaticPlane(plane) => {
                    convex_plane_collision_alg::process_collision(
                        RigidBodyWrapper {
                            obj: col_obj_a,
                            world_trans: *col_obj_a.get_world_trans(),
                        },
                        col_obj_b,
                        plane,
                        contact_added_callback,
                    )
                }
                CollisionShapes::TriangleMesh(mesh) => {
                    sphere_concave_collision_alg::process_collision(
                        col_obj_a,
                        sphere,
                        col_obj_b,
                        mesh,
                        contact_added_callback,
                    )
                }
                CollisionShapes::Compound(compound) => sphere_obb_collision_alg::process_collision(
                    col_obj_a,
                    sphere,
                    col_obj_b,
                    compound,
                    contact_added_callback,
                ),
                _ => unreachable!(),
            },
            CollisionShapes::TriangleMesh(mesh) => match col_obj_b.get_collision_shape() {
                CollisionShapes::Sphere(sphere) => sphere_concave_collision_alg::process_collision(
                    col_obj_b,
                    sphere,
                    col_obj_a,
                    mesh,
                    contact_added_callback,
                ),
                CollisionShapes::Compound(compound) => compound_collision_alg::process_collision(
                    col_obj_b,
                    compound,
                    col_obj_a,
                    contact_added_callback,
                ),
                CollisionShapes::ConvexHull(_) => convex_concave_collision_alg::process_collision(
                    col_obj_b,
                    col_obj_a,
                    mesh,
                    contact_added_callback,
                ),
                _ => unreachable!(),
            },
            CollisionShapes::Compound(compound_a) => match col_obj_b.get_collision_shape() {
                CollisionShapes::StaticPlane(_) | CollisionShapes::TriangleMesh(_) => {
                    compound_collision_alg::process_collision(
                        col_obj_a,
                        compound_a,
                        col_obj_b,
                        contact_added_callback,
                    )
                }
                CollisionShapes::Sphere(sphere) => sphere_obb_collision_alg::process_collision(
                    col_obj_b,
                    sphere,
                    col_obj_a,
                    compound_a,
                    contact_added_callback,
                ),
                CollisionShapes::Compound(compound_b) => obb_obb_collision_alg::process_collision(
                    col_obj_a,
                    compound_a,
                    col_obj_b,
                    compound_b,
                    contact_added_callback,
                ),
                CollisionShapes::ConvexHull(_) => compound_collision_alg::process_collision(
                    col_obj_a,
                    compound_a,
                    col_obj_b,
                    contact_added_callback,
                ),
                CollisionShapes::Triangle(_) => unreachable!(),
            },
            CollisionShapes::ConvexHull(_) => match col_obj_b.get_collision_shape() {
                CollisionShapes::StaticPlane(plane) => {
                    convex_plane_collision_alg::process_collision(
                        RigidBodyWrapper {
                            obj: col_obj_a,
                            world_trans: *col_obj_a.get_world_trans(),
                        },
                        col_obj_b,
                        plane,
                        contact_added_callback,
                    )
                }
                CollisionShapes::Compound(compound) => compound_collision_alg::process_collision(
                    col_obj_b,
                    compound,
                    col_obj_a,
                    contact_added_callback,
                ),
                CollisionShapes::TriangleMesh(mesh) => {
                    convex_concave_collision_alg::process_collision(
                        col_obj_a,
                        col_obj_b,
                        mesh,
                        contact_added_callback,
                    )
                }
                _ => unreachable!(),
            },
            CollisionShapes::Triangle(_) => unreachable!(),
        }
    }

    pub fn near_callback<T: ContactAddedCallback>(
        &mut self,
        collision_objs: &[RigidBody],
        proxy0: &GridBroadphaseProxy,
        proxy1: &GridBroadphaseProxy,
        contact_added_callback: &mut T,
    ) {
        let rb0 = &collision_objs[proxy0.client_obj_idx];
        let rb1 = &collision_objs[proxy1.client_obj_idx];

        if !rb0.is_active() && !rb1.is_active()
            || !rb0.has_contact_response()
            || !rb1.has_contact_response()
        {
            return;
        }

        if let Some(manifold) = Self::process_collision(rb0, rb1, contact_added_callback) {
            self.manifolds.push(manifold);
        }
    }

    pub fn dispatch_all_collision_pairs<T: ContactAddedCallback>(
        &mut self,
        collision_objs: &[RigidBody],
        pair_cache: &mut GridBroadphase,
        contact_added_callback: &mut T,
    ) {
        pair_cache.process_all_overlapping_pairs(collision_objs, self, contact_added_callback);
    }
}
