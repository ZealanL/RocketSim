use glam::Affine3A;

use super::{
    box_mesh_collision_alg, collision_obj_wrapper::RigidBodyWrapper, convex_convex_collision_alg,
    convex_plane_collision_alg,
};
use crate::{
    bullet::{
        collision::{
            narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
            shapes::{collision_shape::CollisionShapes, compound_shape::CompoundShape},
        },
        dynamics::rigid_body::RigidBody,
    },
    shared::Aabb,
};

/// World transform of the compound child shape.
///
/// The function uses the same composition order as the child AABB gate. A
/// caller can thus reuse one value for the gate and the contact pass without a
/// rounding change.
#[inline]
pub(crate) fn child_world_trans(
    compound_obj: &RigidBody,
    compound_shape: &CompoundShape,
) -> Affine3A {
    *compound_obj.get_world_trans() * compound_shape.child_trans
}

#[inline]
pub(crate) fn child_world_aabb_from_trans(
    compound_shape: &CompoundShape,
    new_child_world_trans: &Affine3A,
) -> Aabb {
    compound_shape.child_shape.get_aabb(new_child_world_trans)
}

pub(crate) fn child_world_aabb(compound_obj: &RigidBody, compound_shape: &CompoundShape) -> Aabb {
    child_world_aabb_from_trans(
        compound_shape,
        &child_world_trans(compound_obj, compound_shape),
    )
}

pub(crate) fn child_overlaps_other(
    compound_obj: &RigidBody,
    compound_shape: &CompoundShape,
    other_obj: &RigidBody,
) -> bool {
    let child_aabb = child_world_aabb(compound_obj, compound_shape);
    let other_aabb = other_obj
        .get_collision_shape()
        .get_aabb(other_obj.get_world_trans());
    child_aabb.intersects(&other_aabb)
}

fn child_overlaps_other_with_trans(
    compound_shape: &CompoundShape,
    new_child_world_trans: &Affine3A,
    other_obj: &RigidBody,
) -> bool {
    let child_aabb = child_world_aabb_from_trans(compound_shape, new_child_world_trans);
    let other_aabb = other_obj
        .get_collision_shape()
        .get_aabb(other_obj.get_world_trans());
    child_aabb.intersects(&other_aabb)
}

pub fn process_collision<T: ContactAddedCallback>(
    compound_obj: &RigidBody,
    compound_shape: &CompoundShape,
    other_obj: &RigidBody,
    contact_added_callback: &mut T,
    out: &mut Option<PersistentManifold>,
) {
    debug_assert!(out.is_none());
    let new_child_world_trans = child_world_trans(compound_obj, compound_shape);
    if !child_overlaps_other_with_trans(compound_shape, &new_child_world_trans, other_obj) {
        return;
    }

    process_collision_after_child_gate(
        compound_obj,
        compound_shape,
        new_child_world_trans,
        other_obj,
        contact_added_callback,
        out,
    );
}

/// Process a compound child after the caller applied the child AABB gate.
///
/// `new_child_world_trans` is the world transform of the child shape. `out`
/// must be `None`.
pub(crate) fn process_collision_after_child_gate<T: ContactAddedCallback>(
    compound_obj: &RigidBody,
    compound_shape: &CompoundShape,
    new_child_world_trans: Affine3A,
    other_obj: &RigidBody,
    contact_added_callback: &mut T,
    out: &mut Option<PersistentManifold>,
) {
    debug_assert!(out.is_none());

    let box_shape = &compound_shape.child_shape;
    let other_col_shape = other_obj.get_collision_shape();

    match other_col_shape {
        CollisionShapes::TriangleMesh(tri_mesh) => {
            if let Some(manifold) = box_mesh_collision_alg::process_box_mesh_collision(
                compound_obj,
                compound_shape,
                new_child_world_trans,
                other_obj,
                tri_mesh,
                contact_added_callback,
            ) {
                *out = Some(manifold);
            }
        }
        CollisionShapes::StaticPlane(plane) => {
            let compound_obj_wrap = RigidBodyWrapper {
                obj: compound_obj,
                world_trans: new_child_world_trans,
                child_shape_override: Some(box_shape),
            };
            // The child gate above already compared this AABB with
            // `plane.aabb_cache`. Do not repeat the same test here.
            convex_plane_collision_alg::process_collision_after_gate(
                &compound_obj_wrap,
                other_obj,
                plane,
                contact_added_callback,
                out,
            )
        }
        CollisionShapes::ConvexHull(_) => {
            let compound_obj_wrap = RigidBodyWrapper {
                obj: compound_obj,
                world_trans: new_child_world_trans,
                child_shape_override: Some(box_shape),
            };
            convex_convex_collision_alg::process_collision(
                &compound_obj_wrap,
                other_obj,
                contact_added_callback,
                out,
            )
        }
        _ => unimplemented!(),
    }
}
