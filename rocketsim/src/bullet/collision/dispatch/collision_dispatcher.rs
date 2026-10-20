use super::{
    collision_obj_wrapper::RigidBodyWrapper, compound_collision_alg, convex_concave_collision_alg,
    convex_plane_collision_alg, obb_obb_collision_alg, sphere_concave_collision_alg,
    sphere_obb_collision_alg,
};
use crate::{
    bullet::{
        collision::{
            broadphase::{BroadphaseProxy, GridBroadphase},
            dispatch::convex_convex_collision_alg,
            narrowphase::persistent_manifold::{
                ContactAddedCallback, PersistentManifold, pair_key,
            },
            shapes::{
                bvh_triangle_mesh_shape::BvhTriangleMeshShape, collision_shape::CollisionShapes,
                compound_shape::CompoundShape, sphere_shape::SphereShape,
            },
        },
        dynamics::rigid_body::RigidBody,
    },
    shared::Aabb,
};

#[derive(Clone)]
pub struct CollisionDispatcher {
    pub persistent_manifolds: Vec<PersistentManifold>,
    /// Indices into `persistent_manifolds` with contacts this tick.
    /// Pushed in pair-processing order, so the solver sees the same
    /// contact order as the old per-tick manifold clones.
    pub active_manifolds: Vec<usize>,
    /// Dense index from ordered body pair to manifold index plus one.
    /// Zero means that the pair has no retained manifold.
    manifold_table: Vec<u32>,
    manifold_stride: usize,
    raw_body_aabbs: Vec<Option<Aabb>>,
    compound_child_aabbs: Vec<Option<Aabb>>,
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
            raw_body_aabbs: Vec::new(),
            compound_child_aabbs: Vec::new(),
        }
    }
}

impl CollisionDispatcher {
    /// Clear cached contacts after a teleport.
    ///
    /// Reused prediction arenas teleport to varied poses. Drop stale
    /// contacts. Match a fresh arena.
    pub fn clear_persistent_manifolds(&mut self) {
        self.persistent_manifolds.clear();
        self.active_manifolds.clear();
        self.manifold_table.clear();
        self.manifold_stride = 0;
    }

    /// Push a first-seen pair's manifold and record its index.
    pub(crate) fn insert_persistent_manifold(
        &mut self,
        key: u64,
        manifold: PersistentManifold,
    ) -> usize {
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

    fn lookup_persistent_manifold(&self, body0_idx: usize, body1_idx: usize) -> Option<usize> {
        let (lo, hi) = (body0_idx.min(body1_idx), body0_idx.max(body1_idx));
        if hi >= self.manifold_stride {
            return None;
        }
        let entry = self.manifold_table[lo * self.manifold_stride + hi];
        if entry == 0 {
            None
        } else {
            Some(entry as usize - 1)
        }
    }

    pub(crate) fn release_manifold(&mut self, body0_idx: usize, body1_idx: usize) {
        let Some(index) = self.lookup_persistent_manifold(body0_idx, body1_idx) else {
            return;
        };
        let (lo, hi) = (body0_idx.min(body1_idx), body0_idx.max(body1_idx));
        let table_index = lo * self.manifold_stride + hi;
        let last_index = self.persistent_manifolds.len() - 1;

        self.persistent_manifolds[index].point_cache.clear();
        self.manifold_table[table_index] = 0;

        // A child-algorithm cleanup can release a manifold in the middle of
        // pair dispatch. Keep earlier active rows and remap the swap target.
        self.active_manifolds.retain(|&active| active != index);
        self.persistent_manifolds.swap_remove(index);
        if index != last_index {
            for active in &mut self.active_manifolds {
                if *active == last_index {
                    *active = index;
                }
            }
            let moved = &self.persistent_manifolds[index];
            let moved_lo = moved.body0_idx.min(moved.body1_idx);
            let moved_hi = moved.body0_idx.max(moved.body1_idx);
            self.manifold_table[moved_lo * self.manifold_stride + moved_hi] =
                u32::try_from(index + 1).expect("manifold index overflow");
        }
    }

    /// Grow the table to cover `needed - 1` and reinsert existing mappings.
    fn grow_manifold_table(&mut self, needed: usize) {
        // Round stride to 16 to avoid reallocating per body.
        let new_stride = (needed + 15) & !15;
        self.manifold_table.clear();
        self.manifold_table.resize(new_stride * new_stride, 0);
        for (idx, manifold) in self.persistent_manifolds.iter().enumerate() {
            let lo = manifold.body0_idx.min(manifold.body1_idx);
            let hi = manifold.body0_idx.max(manifold.body1_idx);
            self.manifold_table[lo * new_stride + hi] =
                u32::try_from(idx + 1).expect("manifold index overflow");
        }

        self.manifold_stride = new_stride;
    }

    fn compound_child_pair<'a>(
        col_obj_a: &'a RigidBody,
        col_obj_b: &'a RigidBody,
    ) -> Option<(&'a RigidBody, &'a CompoundShape, &'a RigidBody)> {
        // `Compound` versus `Compound` keeps the OBB-OBB path. The native
        // compound child cleanup is recovered only for one compound side.
        let is_other = |shape: &CollisionShapes| {
            !matches!(
                shape,
                CollisionShapes::Compound(_) | CollisionShapes::Triangle(_)
            )
        };

        match (
            col_obj_a.get_collision_shape(),
            col_obj_b.get_collision_shape(),
        ) {
            (CollisionShapes::Compound(compound), other) if is_other(other) => {
                Some((col_obj_a, compound, col_obj_b))
            }
            (other, CollisionShapes::Compound(compound)) if is_other(other) => {
                Some((col_obj_b, compound, col_obj_a))
            }
            _ => None,
        }
    }

    pub(crate) fn prepare_dispatch_bounds(&mut self, body_count: usize) {
        self.raw_body_aabbs.clear();
        self.raw_body_aabbs.resize(body_count, None);
        self.compound_child_aabbs.clear();
        self.compound_child_aabbs.resize(body_count, None);
    }

    fn cached_raw_body_aabb(&mut self, body_idx: usize, body: &RigidBody) -> Aabb {
        if let Some(aabb) = self.raw_body_aabbs[body_idx] {
            return aabb;
        }

        let aabb = body.get_collision_shape().get_aabb(body.get_world_trans());
        self.raw_body_aabbs[body_idx] = Some(aabb);
        aabb
    }

    fn cached_compound_child_overlaps_other(
        &mut self,
        compound_idx: usize,
        compound_obj: &RigidBody,
        compound_shape: &CompoundShape,
        other_idx: usize,
        other_obj: &RigidBody,
    ) -> bool {
        let child_aabb = match self.compound_child_aabbs[compound_idx] {
            Some(aabb) => aabb,
            None => {
                let aabb = compound_collision_alg::child_world_aabb(compound_obj, compound_shape);
                self.compound_child_aabbs[compound_idx] = Some(aabb);
                aabb
            }
        };
        let other_aabb = self.cached_raw_body_aabb(other_idx, other_obj);
        child_aabb.intersects(&other_aabb)
    }

    fn empty_manifold_bodies<'a>(
        col_obj_a: &'a RigidBody,
        col_obj_b: &'a RigidBody,
    ) -> (&'a RigidBody, &'a RigidBody) {
        use CollisionShapes as Shape;

        match (
            col_obj_a.get_collision_shape(),
            col_obj_b.get_collision_shape(),
        ) {
            (Shape::StaticPlane(_), _) => (col_obj_b, col_obj_a),
            (Shape::Sphere(_), Shape::Compound(_)) => (col_obj_a, col_obj_b),
            (Shape::Sphere(_), Shape::TriangleMesh(_)) => (col_obj_a, col_obj_b),
            (Shape::TriangleMesh(_), Shape::Sphere(_) | Shape::ConvexHull(_)) => {
                (col_obj_b, col_obj_a)
            }
            (Shape::TriangleMesh(_), Shape::Compound(_)) => (col_obj_b, col_obj_a),
            (Shape::ConvexHull(_), Shape::Compound(_)) => (col_obj_b, col_obj_a),
            (Shape::ConvexHull(_), _) => (col_obj_a, col_obj_b),
            (Shape::Compound(_), Shape::Sphere(_)) => (col_obj_b, col_obj_a),
            _ => (col_obj_a, col_obj_b),
        }
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
    pub fn process_collision<'a, T: ContactAddedCallback>(
        col_obj_a: &'a RigidBody,
        col_obj_b: &'a RigidBody,
        contact_added_callback: &'a mut T,
        out: &mut Option<PersistentManifold>,
    ) {
        Self::process_collision_inner(col_obj_a, col_obj_b, contact_added_callback, out);
    }

    fn process_collision_inner<T: ContactAddedCallback>(
        col_obj_a: &RigidBody,
        col_obj_b: &RigidBody,
        contact_added_callback: &mut T,
        out: &mut Option<PersistentManifold>,
    ) {
        debug_assert!(out.is_none());

        if let Some(mesh_collision) = Self::mesh_collision(col_obj_a, col_obj_b) {
            let (convex_obj, concave_obj) = mesh_collision.bodies();
            let mut manifold = PersistentManifold::new(convex_obj, concave_obj);

            if Self::process_mesh_collision_into(
                mesh_collision,
                &mut manifold,
                &mut *contact_added_callback,
            ) {
                *out = Some(manifold);
            }

            return;
        }

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

    pub(crate) fn near_callback_with_cached_bounds<T: ContactAddedCallback>(
        &mut self,
        collision_objs: &[RigidBody],
        proxy0: &BroadphaseProxy,
        proxy1: &BroadphaseProxy,
        contact_added_callback: &mut T,
    ) {
        self.near_callback_with_bounds::<true, T>(
            collision_objs,
            proxy0,
            proxy1,
            contact_added_callback,
        );
    }

    fn near_callback_with_bounds<const USE_CACHED_BOUNDS: bool, T: ContactAddedCallback>(
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
        let cached_idx = self.lookup_persistent_manifold(rb0_idx, rb1_idx);

        if let Some(cached_idx) = cached_idx {
            // The dense index must reference this exact body pair.
            let manifold = &self.persistent_manifolds[cached_idx];
            debug_assert_eq!(manifold.pair_key, pair_key(rb0_idx, rb1_idx));
            debug_assert!(
                (manifold.body0_idx == rb0_idx && manifold.body1_idx == rb1_idx)
                    || (manifold.body0_idx == rb1_idx && manifold.body1_idx == rb0_idx),
                "stale pair index"
            );
        }

        let compound_pair = Self::compound_child_pair(rb0, rb1);
        let compound_gate = compound_pair.map(|(compound_obj, compound, other_obj)| {
            if USE_CACHED_BOUNDS {
                let (compound_idx, other_idx) = if std::ptr::eq(compound_obj, rb0) {
                    (rb0_idx, rb1_idx)
                } else {
                    (rb1_idx, rb0_idx)
                };
                self.cached_compound_child_overlaps_other(
                    compound_idx,
                    compound_obj,
                    compound,
                    other_idx,
                    other_obj,
                )
            } else {
                compound_collision_alg::child_overlaps_other(compound_obj, compound, other_obj)
            }
        });
        if compound_gate == Some(false) {
            self.release_manifold(rb0_idx, rb1_idx);
            return;
        }

        if let Some(mesh_collision) = Self::mesh_collision(rb0, rb1) {
            let (convex_obj, concave_obj) = mesh_collision.bodies();
            let persistent_idx = if let Some(cached_idx) = cached_idx {
                cached_idx
            } else {
                self.insert_persistent_manifold(
                    pair_key(rb0_idx, rb1_idx),
                    PersistentManifold::new(convex_obj, concave_obj),
                )
            };

            Self::process_mesh_collision_into(
                mesh_collision,
                &mut self.persistent_manifolds[persistent_idx],
                contact_added_callback,
            );

            self.active_manifolds.push(persistent_idx);
            return;
        }

        let mut fresh: Option<PersistentManifold> = None;
        match compound_pair {
            Some((compound_obj, compound_shape, other_obj))
                if matches!(
                    other_obj.get_collision_shape(),
                    CollisionShapes::StaticPlane(_)
                        | CollisionShapes::TriangleMesh(_)
                        | CollisionShapes::ConvexHull(_)
                ) =>
            {
                compound_collision_alg::process_collision_after_child_gate(
                    compound_obj,
                    compound_shape,
                    other_obj,
                    contact_added_callback,
                    &mut fresh,
                );
            }
            _ => Self::process_collision_inner(rb0, rb1, contact_added_callback, &mut fresh),
        }

        // Share the persistent manifold with the solver by index instead
        // of pushing a per-tick clone. Merge and refresh order is
        // unchanged from the clone path above.
        let active_idx = match (cached_idx, fresh.as_ref()) {
            (Some(cached_idx), Some(fresh_manifold)) => {
                let (body0_idx, body1_idx) = {
                    let manifold = &self.persistent_manifolds[cached_idx];
                    (manifold.body0_idx, manifold.body1_idx)
                };

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

                if !self.persistent_manifolds[cached_idx].point_cache.is_empty() {
                    self.persistent_manifolds[cached_idx].refresh_contact_points(
                        &collision_objs[body0_idx],
                        &collision_objs[body1_idx],
                    );
                }

                // No fresh point reported this pair. Report the retained contacts
                // after refresh removes invalid points.
                for i in 0..self.persistent_manifolds[cached_idx].point_cache.len() {
                    let point = self.persistent_manifolds[cached_idx].point_cache[i];
                    contact_added_callback.contact_processed(
                        &point,
                        &collision_objs[body0_idx],
                        &collision_objs[body1_idx],
                    );
                }

                cached_idx
            }
            (None, Some(_)) => self.insert_persistent_manifold(
                pair_key(rb0_idx, rb1_idx),
                fresh.take().expect("fresh hit"),
            ),
            (None, None) => {
                // The child algorithm exists because the gate above passed.
                // Its convex-concave constructor creates the manifold before
                // the first point, so keep an empty retained row.
                let (body0, body1) = Self::empty_manifold_bodies(rb0, rb1);
                self.insert_persistent_manifold(
                    pair_key(rb0_idx, rb1_idx),
                    PersistentManifold::new(body0, body1),
                )
            }
        };

        self.active_manifolds.push(active_idx);
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
