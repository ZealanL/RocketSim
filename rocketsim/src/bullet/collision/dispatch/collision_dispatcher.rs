use super::{
    convex_concave_collision_alg::ConvexConcaveCollisionAlgorithm,
    convex_plane_collision_alg::ConvexPlaneCollisionAlgorithm,
};
use crate::bullet::collision::{
    broadphase::{CollisionAlgorithm, GridBroadphase, GridBroadphaseProxy},
    dispatch::{
        collision_obj_wrapper::RigidBodyWrapper,
        compound_collision_alg::CompoundCollisionAlgorithm,
        obb_obb_collision_alg::ObbObbCollisionAlgorithm,
        sphere_obb_collision_alg::SphereObbCollisionAlgorithm,
    },
    narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
    shapes::{
        bvh_triangle_mesh_shape::BvhTriangleMeshShape, collision_shape::CollisionShapes,
        compound_shape::CompoundShape, sphere_shape::SphereShape,
        static_plane_shape::StaticPlaneShape,
    },
};
use crate::bullet::dynamics::rigid_body::RigidBody;

enum Algorithms<'a, T: ContactAddedCallback> {
    ConvexPlane(ConvexPlaneCollisionAlgorithm<'a, T>),
    ConvexConcave(ConvexConcaveCollisionAlgorithm<'a, T>),
    SphereObb(SphereObbCollisionAlgorithm<'a, T>),
    Compound(CompoundCollisionAlgorithm<'a, T>),
    ObbObb(ObbObbCollisionAlgorithm<'a, T>),
}

impl<'a, T: ContactAddedCallback> Algorithms<'a, T> {
    const fn new_convex_plane(
        convex_obj: RigidBodyWrapper<'a>,
        plane_obj: &'a RigidBody,
        plane_shape: &'a StaticPlaneShape,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::ConvexPlane(ConvexPlaneCollisionAlgorithm::new(
            convex_obj,
            plane_obj,
            plane_shape,
            is_swapped,
            contact_added_callback,
        ))
    }

    const fn new_convex_concave(
        convex_obj: &'a RigidBody,
        sphere_shape: &'a SphereShape,
        concave_obj: &'a RigidBody,
        tri_mesh: &'a BvhTriangleMeshShape,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::ConvexConcave(ConvexConcaveCollisionAlgorithm::new(
            convex_obj,
            sphere_shape,
            concave_obj,
            tri_mesh,
            is_swapped,
            contact_added_callback,
        ))
    }

    const fn new_sphere_obb(
        sphere_obj: &'a RigidBody,
        sphere_shape: &'a SphereShape,
        obb_obj: &'a RigidBody,
        obb_shape: &'a CompoundShape,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::SphereObb(SphereObbCollisionAlgorithm::new(
            sphere_obj,
            sphere_shape,
            obb_obj,
            obb_shape,
            is_swapped,
            contact_added_callback,
        ))
    }

    const fn new_obb_obb(
        compound_a_obj: &'a RigidBody,
        compound_a_shape: &'a CompoundShape,
        compound_b_obj: &'a RigidBody,
        compound_b_shape: &'a CompoundShape,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::ObbObb(ObbObbCollisionAlgorithm::new(
            compound_a_obj,
            compound_a_shape,
            compound_b_obj,
            compound_b_shape,
            contact_added_callback,
        ))
    }

    const fn new_compound(
        compound_obj: &'a RigidBody,
        compound_shape: &'a CompoundShape,
        other_obj: &'a RigidBody,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self::Compound(CompoundCollisionAlgorithm::new(
            compound_obj,
            compound_shape,
            other_obj,
            is_swapped,
            contact_added_callback,
        ))
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for Algorithms<'_, T> {
    fn process_collision(self) -> Option<PersistentManifold> {
        match self {
            Self::ConvexPlane(alg) => alg.process_collision(),
            Self::ConvexConcave(alg) => alg.process_collision(),
            Self::SphereObb(alg) => alg.process_collision(),
            Self::Compound(alg) => alg.process_collision(),
            Self::ObbObb(alg) => alg.process_collision(),
        }
    }
}

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
    fn find_alg<'a, T: ContactAddedCallback>(
        col_obj_a: &'a RigidBody,
        col_obj_b: &'a RigidBody,
        contact_added_callback: &'a mut T,
    ) -> Algorithms<'a, T> {
        match col_obj_a.get_collision_shape() {
            CollisionShapes::StaticPlane(plane) => match col_obj_b.get_collision_shape() {
                CollisionShapes::Sphere(_) => Algorithms::new_convex_plane(
                    RigidBodyWrapper {
                        obj: col_obj_b,
                        world_trans: *col_obj_b.get_world_trans(),
                    },
                    col_obj_a,
                    plane,
                    true,
                    contact_added_callback,
                ),
                CollisionShapes::Compound(compound) => Algorithms::new_compound(
                    col_obj_b,
                    compound,
                    col_obj_a,
                    true,
                    contact_added_callback,
                ),
                _ => unimplemented!(),
            },
            CollisionShapes::Sphere(sphere) => match col_obj_b.get_collision_shape() {
                CollisionShapes::StaticPlane(plane) => Algorithms::new_convex_plane(
                    RigidBodyWrapper {
                        obj: col_obj_a,
                        world_trans: *col_obj_a.get_world_trans(),
                    },
                    col_obj_b,
                    plane,
                    false,
                    contact_added_callback,
                ),
                CollisionShapes::TriangleMesh(mesh) => Algorithms::new_convex_concave(
                    col_obj_a,
                    sphere,
                    col_obj_b,
                    mesh,
                    false,
                    contact_added_callback,
                ),
                CollisionShapes::Compound(compound) => Algorithms::new_sphere_obb(
                    col_obj_a,
                    sphere,
                    col_obj_b,
                    compound,
                    false,
                    contact_added_callback,
                ),
                CollisionShapes::Sphere(_) => unimplemented!(),
            },
            CollisionShapes::TriangleMesh(mesh) => match col_obj_b.get_collision_shape() {
                CollisionShapes::Sphere(sphere) => Algorithms::new_convex_concave(
                    col_obj_b,
                    sphere,
                    col_obj_a,
                    mesh,
                    true,
                    contact_added_callback,
                ),
                CollisionShapes::Compound(compound) => Algorithms::new_compound(
                    col_obj_b,
                    compound,
                    col_obj_a,
                    true,
                    contact_added_callback,
                ),
                _ => unimplemented!(),
            },
            CollisionShapes::Compound(compound_a) => match col_obj_b.get_collision_shape() {
                CollisionShapes::StaticPlane(_) | CollisionShapes::TriangleMesh(_) => {
                    Algorithms::new_compound(
                        col_obj_a,
                        compound_a,
                        col_obj_b,
                        false,
                        contact_added_callback,
                    )
                }
                CollisionShapes::Sphere(sphere) => Algorithms::new_sphere_obb(
                    col_obj_b,
                    sphere,
                    col_obj_a,
                    compound_a,
                    true,
                    contact_added_callback,
                ),
                CollisionShapes::Compound(compound_b) => Algorithms::new_obb_obb(
                    col_obj_a,
                    compound_a,
                    col_obj_b,
                    compound_b,
                    contact_added_callback,
                ),
            },
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

        let algorithm = Self::find_alg(&rb0, &rb1, contact_added_callback);

        if let Some(manifold) = algorithm.process_collision() {
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
