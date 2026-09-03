use super::{
    collision_obj_wrapper::RigidBodyWrapper, compound_collision_alg, convex_concave_collision_alg,
    convex_plane_collision_alg, obb_obb_collision_alg, sphere_concave_collision_alg,
    sphere_obb_collision_alg,
};
use crate::bullet::{
    collision::{
        broadphase::{BroadphaseProxy, GridBroadphase},
        dispatch::convex_convex_collision_alg,
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{
            bvh_triangle_mesh_shape::BvhTriangleMeshShape, collision_shape::CollisionShapes,
            sphere_shape::SphereShape,
        },
    },
    dynamics::rigid_body::RigidBody,
};

pub struct CollisionDispatcher {
    pub manifolds: Vec<PersistentManifold>,
    persistent_manifolds: Vec<PersistentManifold>,
    sphere_contact_scratch: Vec<sphere_concave_collision_alg::PendingSphereContact>,
}

enum MeshCollision<'a> {
    Sphere {
        convex_obj: &'a RigidBody,
        sphere: &'a SphereShape,
        concave_obj: &'a RigidBody,
        tri_mesh: &'a BvhTriangleMeshShape,
    },
    Convex {
        convex_obj: &'a RigidBody,
        concave_obj: &'a RigidBody,
        tri_mesh: &'a BvhTriangleMeshShape,
    },
}

impl MeshCollision<'_> {
    fn bodies(&self) -> (&RigidBody, &RigidBody) {
        match self {
            Self::Sphere {
                convex_obj,
                concave_obj,
                ..
            }
            | Self::Convex {
                convex_obj,
                concave_obj,
                ..
            } => (*convex_obj, *concave_obj),
        }
    }
}

impl Default for CollisionDispatcher {
    fn default() -> Self {
        Self {
            manifolds: Vec::with_capacity(8),
            persistent_manifolds: Vec::with_capacity(8),
            sphere_contact_scratch: Vec::new(),
        }
    }
}

impl CollisionDispatcher {
    fn mesh_collision<'a>(
        col_obj_a: &'a RigidBody,
        col_obj_b: &'a RigidBody,
    ) -> Option<MeshCollision<'a>> {
        match (
            col_obj_a.get_collision_shape(),
            col_obj_b.get_collision_shape(),
        ) {
            (CollisionShapes::Sphere(sphere), CollisionShapes::TriangleMesh(tri_mesh)) => {
                Some(MeshCollision::Sphere {
                    convex_obj: col_obj_a,
                    sphere,
                    concave_obj: col_obj_b,
                    tri_mesh,
                })
            }
            (CollisionShapes::TriangleMesh(tri_mesh), CollisionShapes::Sphere(sphere)) => {
                Some(MeshCollision::Sphere {
                    convex_obj: col_obj_b,
                    sphere,
                    concave_obj: col_obj_a,
                    tri_mesh,
                })
            }
            (CollisionShapes::ConvexHull(_), CollisionShapes::TriangleMesh(tri_mesh)) => {
                Some(MeshCollision::Convex {
                    convex_obj: col_obj_a,
                    concave_obj: col_obj_b,
                    tri_mesh,
                })
            }
            (CollisionShapes::TriangleMesh(tri_mesh), CollisionShapes::ConvexHull(_)) => {
                Some(MeshCollision::Convex {
                    convex_obj: col_obj_b,
                    concave_obj: col_obj_a,
                    tri_mesh,
                })
            }
            _ => None,
        }
    }

    fn process_collision<'a, T: ContactAddedCallback>(
        col_obj_a: &'a RigidBody,
        col_obj_b: &'a RigidBody,
        contact_added_callback: &'a mut T,
    ) -> Option<PersistentManifold> {
        match col_obj_a.get_collision_shape() {
            CollisionShapes::StaticPlane(plane) => match col_obj_b.get_collision_shape() {
                CollisionShapes::Sphere(_) | CollisionShapes::ConvexHull(_) => {
                    convex_plane_collision_alg::process_collision(
                        &RigidBodyWrapper {
                            obj: col_obj_b,
                            world_trans: *col_obj_b.get_world_trans(),
                            child_shape_override: None,
                        },
                        col_obj_a,
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
                _ => unreachable!(),
            },
            CollisionShapes::Sphere(sphere) => match col_obj_b.get_collision_shape() {
                CollisionShapes::StaticPlane(plane) => {
                    convex_plane_collision_alg::process_collision(
                        &RigidBodyWrapper {
                            obj: col_obj_a,
                            world_trans: *col_obj_a.get_world_trans(),
                            child_shape_override: None,
                        },
                        col_obj_b,
                        plane,
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
                CollisionShapes::ConvexHull(_) => convex_convex_collision_alg::process_collision(
                    &RigidBodyWrapper {
                        obj: col_obj_a,
                        world_trans: *col_obj_a.get_world_trans(),
                        child_shape_override: None,
                    },
                    col_obj_b,
                    contact_added_callback,
                ),
                _ => unreachable!(),
            },
            CollisionShapes::TriangleMesh(_) => match col_obj_b.get_collision_shape() {
                CollisionShapes::Compound(compound) => compound_collision_alg::process_collision(
                    col_obj_b,
                    compound,
                    col_obj_a,
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
                        &RigidBodyWrapper {
                            obj: col_obj_a,
                            world_trans: *col_obj_a.get_world_trans(),
                            child_shape_override: None,
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
                CollisionShapes::Sphere(_) => convex_convex_collision_alg::process_collision(
                    &RigidBodyWrapper {
                        obj: col_obj_a,
                        world_trans: *col_obj_a.get_world_trans(),
                        child_shape_override: None,
                    },
                    col_obj_b,
                    contact_added_callback,
                ),
                _ => unreachable!(),
            },
            CollisionShapes::Triangle(_) => unreachable!(),
        }
    }

    fn process_mesh_collision_into<T: ContactAddedCallback>(
        mesh_collision: MeshCollision<'_>,
        manifold: &mut PersistentManifold,
        sphere_contact_scratch: &mut Vec<sphere_concave_collision_alg::PendingSphereContact>,
        contact_added_callback: &mut T,
    ) -> bool {
        match mesh_collision {
            MeshCollision::Sphere {
                convex_obj,
                sphere,
                concave_obj,
                tri_mesh,
            } => sphere_concave_collision_alg::process_collision_into(
                convex_obj,
                sphere,
                concave_obj,
                tri_mesh,
                manifold,
                sphere_contact_scratch,
                contact_added_callback,
            ),
            MeshCollision::Convex {
                convex_obj,
                concave_obj,
                tri_mesh,
            } => convex_concave_collision_alg::process_collision_into(
                convex_obj,
                concave_obj,
                tri_mesh,
                manifold,
                contact_added_callback,
            ),
        }
    }

    pub fn near_callback<T: ContactAddedCallback>(
        &mut self,
        collision_objs: &[RigidBody],
        proxy0: &BroadphaseProxy,
        proxy1: &BroadphaseProxy,
        contact_added_callback: &mut T,
    ) {
        let rb0_idx = proxy0.client_obj_idx as usize;
        let rb1_idx = proxy1.client_obj_idx as usize;
        let rb0 = &collision_objs[rb0_idx];
        let rb1 = &collision_objs[rb1_idx];

        if !rb0.is_active() && !rb1.is_active()
            || !rb0.has_contact_response()
            || !rb1.has_contact_response()
        {
            return;
        }

        let pair_min = rb0_idx.min(rb1_idx);
        let pair_max = rb0_idx.max(rb1_idx);
        let cached_idx = self.persistent_manifolds.iter().position(|manifold| {
            manifold.body0_idx.min(manifold.body1_idx) == pair_min
                && manifold.body0_idx.max(manifold.body1_idx) == pair_max
        });

        if let Some(mesh_collision) = Self::mesh_collision(rb0, rb1) {
            let (convex_obj, concave_obj) = mesh_collision.bodies();
            let persistent_idx = if let Some(cached_idx) = cached_idx {
                cached_idx
            } else {
                self.persistent_manifolds
                    .push(PersistentManifold::new(convex_obj, concave_obj));
                self.persistent_manifolds.len() - 1
            };
            let has_contacts = Self::process_mesh_collision_into(
                mesh_collision,
                &mut self.persistent_manifolds[persistent_idx],
                &mut self.sphere_contact_scratch,
                contact_added_callback,
            );

            if has_contacts {
                self.manifolds
                    .push(self.persistent_manifolds[persistent_idx].clone());
            }
            return;
        }

        let fresh_manifold = Self::process_collision(rb0, rb1, contact_added_callback);

        let manifold = match (cached_idx, fresh_manifold) {
            (Some(cached_idx), Some(fresh_manifold)) => {
                self.persistent_manifolds[cached_idx].merge_contact_points(fresh_manifold);
                let (body0_idx, body1_idx) = {
                    let manifold = &self.persistent_manifolds[cached_idx];
                    (manifold.body0_idx, manifold.body1_idx)
                };
                self.persistent_manifolds[cached_idx]
                    .refresh_contact_points(&collision_objs[body0_idx], &collision_objs[body1_idx]);
                self.persistent_manifolds[cached_idx].clone()
            }
            (Some(cached_idx), None) => {
                let (body0_idx, body1_idx) = {
                    let manifold = &self.persistent_manifolds[cached_idx];
                    (manifold.body0_idx, manifold.body1_idx)
                };
                self.persistent_manifolds[cached_idx]
                    .refresh_contact_points(&collision_objs[body0_idx], &collision_objs[body1_idx]);
                self.persistent_manifolds[cached_idx].clone()
            }
            (None, Some(fresh_manifold)) => {
                self.persistent_manifolds.push(fresh_manifold);
                self.persistent_manifolds.last().unwrap().clone()
            }
            (None, None) => return,
        };

        if !manifold.point_cache.is_empty() {
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
