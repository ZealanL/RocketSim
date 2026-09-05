use super::{
    collision_obj_wrapper::RigidBodyWrapper, compound_collision_alg, convex_concave_collision_alg,
    convex_plane_collision_alg, obb_obb_collision_alg, sphere_concave_collision_alg,
    sphere_obb_collision_alg,
};
use crate::bullet::{
    collision::{
        broadphase::{BroadphaseProxy, GridBroadphase},
        dispatch::convex_convex_collision_alg,
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold, pair_key},
        shapes::{
            bvh_triangle_mesh_shape::BvhTriangleMeshShape, collision_shape::CollisionShapes,
            sphere_shape::SphereShape,
        },
    },
    dynamics::rigid_body::RigidBody,
};

pub struct CollisionDispatcher {
    pub persistent_manifolds: Vec<PersistentManifold>,
    /// Indices into `persistent_manifolds` with contacts this tick.
    /// Pushed in pair-processing order, so the solver sees the same
    /// contact order as the old per-tick manifold clones.
    pub active_manifolds: Vec<usize>,
    /// Dense index from ordered body pair to `persistent_manifolds` index.
    /// Cell `min * stride + max` holds the index plus one (`0` = none).
    /// Push-only manifold vector, so indices stay stable. Maintain at push
    /// sites below; any removal/clear must rebuild this table too.
    manifold_table: Vec<u32>,
    manifold_stride: usize,
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
            persistent_manifolds: Vec::with_capacity(8),
            active_manifolds: Vec::with_capacity(8),
            manifold_table: Vec::new(),
            manifold_stride: 0,
            // Ball/convex-vs-mesh sweeps collect a few triangle hits;
            // keep the buffer across ticks instead of reallocating.
            sphere_contact_scratch: Vec::with_capacity(16),
        }
    }
}

impl CollisionDispatcher {
    /// Push a first-seen pair's manifold and record its index.
    fn insert_persistent_manifold(&mut self, key: u64, manifold: PersistentManifold) -> usize {
        debug_assert_eq!(manifold.pair_key, key);
        // Read indices before push; growth rebuild covers prior manifolds only.
        let (lo, hi) = (
            manifold.body0_idx.min(manifold.body1_idx),
            manifold.body0_idx.max(manifold.body1_idx),
        );
        if hi >= self.manifold_stride {
            self.grow_manifold_table(hi + 1);
        }
        self.persistent_manifolds.push(manifold);
        let idx = self.persistent_manifolds.len() - 1;
        let table_idx = lo * self.manifold_stride + hi;
        debug_assert_eq!(self.manifold_table[table_idx], 0);
        self.manifold_table[table_idx] = u32::try_from(idx + 1).expect("manifold index overflow");
        idx
    }

    /// Grow the table to cover `needed - 1` and reinsert existing mappings.
    fn grow_manifold_table(&mut self, needed: usize) {
        // Round stride to 16 to avoid reallocating per body.
        let new_stride = (needed + 15) & !15;
        let mut new_table = vec![0u32; new_stride * new_stride];
        for (idx, manifold) in self.persistent_manifolds.iter().enumerate() {
            let lo = manifold.body0_idx.min(manifold.body1_idx);
            let hi = manifold.body0_idx.max(manifold.body1_idx);
            new_table[lo * new_stride + hi] =
                u32::try_from(idx + 1).expect("manifold index overflow");
        }
        self.manifold_table = new_table;
        self.manifold_stride = new_stride;
    }

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

    // Miss leaves None; hit writes Some.
    fn process_collision<'a, T: ContactAddedCallback>(
        col_obj_a: &'a RigidBody,
        col_obj_b: &'a RigidBody,
        contact_added_callback: &'a mut T,
        out: &mut Option<PersistentManifold>,
    ) {
        debug_assert!(out.is_none());
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
                        out,
                    );
                }
                CollisionShapes::Compound(compound) => compound_collision_alg::process_collision(
                    col_obj_b,
                    compound,
                    col_obj_a,
                    contact_added_callback,
                    out,
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
                        out,
                    );
                }
                CollisionShapes::Compound(compound) => sphere_obb_collision_alg::process_collision(
                    col_obj_a,
                    sphere,
                    col_obj_b,
                    compound,
                    contact_added_callback,
                    out,
                ),
                CollisionShapes::ConvexHull(_) => convex_convex_collision_alg::process_collision(
                    &RigidBodyWrapper {
                        obj: col_obj_a,
                        world_trans: *col_obj_a.get_world_trans(),
                        child_shape_override: None,
                    },
                    col_obj_b,
                    contact_added_callback,
                    out,
                ),
                _ => unreachable!(),
            },
            CollisionShapes::TriangleMesh(_) => match col_obj_b.get_collision_shape() {
                CollisionShapes::Compound(compound) => compound_collision_alg::process_collision(
                    col_obj_b,
                    compound,
                    col_obj_a,
                    contact_added_callback,
                    out,
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
                        out,
                    );
                }
                CollisionShapes::Sphere(sphere) => sphere_obb_collision_alg::process_collision(
                    col_obj_b,
                    sphere,
                    col_obj_a,
                    compound_a,
                    contact_added_callback,
                    out,
                ),
                CollisionShapes::Compound(compound_b) => obb_obb_collision_alg::process_collision(
                    col_obj_a,
                    compound_a,
                    col_obj_b,
                    compound_b,
                    contact_added_callback,
                    out,
                ),
                CollisionShapes::ConvexHull(_) => compound_collision_alg::process_collision(
                    col_obj_a,
                    compound_a,
                    col_obj_b,
                    contact_added_callback,
                    out,
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
                        out,
                    );
                }
                CollisionShapes::Compound(compound) => compound_collision_alg::process_collision(
                    col_obj_b,
                    compound,
                    col_obj_a,
                    contact_added_callback,
                    out,
                ),
                CollisionShapes::Sphere(_) => convex_convex_collision_alg::process_collision(
                    &RigidBodyWrapper {
                        obj: col_obj_a,
                        world_trans: *col_obj_a.get_world_trans(),
                        child_shape_override: None,
                    },
                    col_obj_b,
                    contact_added_callback,
                    out,
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

        // Dense table lookup; insertion order is unchanged.
        let wanted = pair_key(rb0_idx, rb1_idx);
        let (lo, hi) = (rb0_idx.min(rb1_idx), rb0_idx.max(rb1_idx));
        let cached_idx = if hi < self.manifold_stride {
            let cell = self.manifold_table[lo * self.manifold_stride + hi];
            if cell == 0 {
                None
            } else {
                Some(cell as usize - 1)
            }
        } else {
            None
        };
        if let Some(cached_idx) = cached_idx {
            // Push-only vector, so a hit must reference this exact pair.
            let manifold = &self.persistent_manifolds[cached_idx];
            debug_assert_eq!(manifold.pair_key, wanted);
            debug_assert!(
                (manifold.body0_idx == rb0_idx && manifold.body1_idx == rb1_idx)
                    || (manifold.body0_idx == rb1_idx && manifold.body1_idx == rb0_idx),
                "stale pair index"
            );
        }

        if let Some(mesh_collision) = Self::mesh_collision(rb0, rb1) {
            let (convex_obj, concave_obj) = mesh_collision.bodies();
            let persistent_idx = if let Some(cached_idx) = cached_idx {
                cached_idx
            } else {
                self.insert_persistent_manifold(
                    wanted,
                    PersistentManifold::new(convex_obj, concave_obj),
                )
            };
            let has_contacts = Self::process_mesh_collision_into(
                mesh_collision,
                &mut self.persistent_manifolds[persistent_idx],
                &mut self.sphere_contact_scratch,
                contact_added_callback,
            );

            if has_contacts {
                self.active_manifolds.push(persistent_idx);
            }
            return;
        }

        let mut fresh: Option<PersistentManifold> = None;
        Self::process_collision(rb0, rb1, contact_added_callback, &mut fresh);

        // Share the persistent manifold with the solver by index instead
        // of pushing a per-tick clone. Merge and refresh order is
        // unchanged from the clone path above.
        let active_idx = match (cached_idx, fresh.as_ref()) {
            (Some(cached_idx), Some(fresh_manifold)) => {
                // Cull separated points before merging fresh detections.
                // This mirrors the target's two BWCACHE rounds: round 1 is
                // the refresh that culls carried points to empty (observed
                // before-count 2 -> after-count 0); round 2 is the leaf
                // fresh-add pass (`284EE0`/`29EF00`, `life=0`) in detection
                // order, kept by the leaf-tail refresh. Merging into an
                // unrefreshed cache lets a fresh point hijack a stale slot
                // (same position, different face), scrambling row order for
                // the order-dependent sequential-impulse solve.
                //
                // No second refresh follows the merge: it is a proven no-op
                // here. Fresh points were detected this tick, so their
                // projected distance is within threshold with ~zero drift;
                // surviving old points were re-projected above at identical
                // transforms (bodies do not move between the two calls), so
                // re-running the deterministic recompute changes no value
                // and culls nothing. (The target tail refresh additionally
                // ages lifetimes and fires survivor callbacks, which this
                // manifold does not model; revisit if that ever changes.)
                let (body0_idx, body1_idx) = {
                    let manifold = &self.persistent_manifolds[cached_idx];
                    (manifold.body0_idx, manifold.body1_idx)
                };
                // Skip the no-op empty refresh (see `refresh_contact_points`);
                // the merge below behaves identically on an empty cache either way.
                if !self.persistent_manifolds[cached_idx].point_cache.is_empty() {
                    self.persistent_manifolds[cached_idx].refresh_contact_points(
                        &collision_objs[body0_idx],
                        &collision_objs[body1_idx],
                    );
                }
                self.persistent_manifolds[cached_idx].merge_contact_points(fresh_manifold);
                cached_idx
            }
            (Some(cached_idx), None) => {
                let (body0_idx, body1_idx) = {
                    let manifold = &self.persistent_manifolds[cached_idx];
                    (manifold.body0_idx, manifold.body1_idx)
                };
                // Skip the no-op empty refresh; an empty manifold stays
                // empty and the active push below is already guarded.
                if !self.persistent_manifolds[cached_idx].point_cache.is_empty() {
                    self.persistent_manifolds[cached_idx].refresh_contact_points(
                        &collision_objs[body0_idx],
                        &collision_objs[body1_idx],
                    );
                }
                cached_idx
            }
            (None, Some(_)) => {
                self.insert_persistent_manifold(wanted, fresh.take().expect("fresh hit"))
            }
            (None, None) => return,
        };

        if !self.persistent_manifolds[active_idx].point_cache.is_empty() {
            self.active_manifolds.push(active_idx);
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
