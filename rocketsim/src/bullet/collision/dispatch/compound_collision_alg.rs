use glam::{Affine3A, Vec3A};

use super::{
    collision_obj_wrapper::RigidBodyWrapper, convex_convex_collision_alg,
    convex_plane_collision_alg,
};
use crate::{
    bullet::{
        collision::{
            narrowphase::{
                box_triangle_sat::box_triangle_sat,
                persistent_manifold::{ContactAddedCallback, PersistentManifold},
            },
            shapes::{
                collision_shape::CollisionShapes, compound_shape::CompoundShape,
                triangle_callback::ProcessTriangle, triangle_shape::TriangleShape,
            },
        },
        dynamics::rigid_body::{CollisionFlags, RigidBody},
        linear_math::AffineExt,
    },
    shared::Aabb,
};

/// Cached static-mesh transform facts for one compound/mesh pair.
///
/// `NoAngularMotion` and `IdentityTransform` are maintained by the
/// `RigidBody` transform setters. Do not test the matrix again.
#[derive(Clone, Copy)]
struct MeshTransformFlags {
    no_angular_motion: bool,
    identity: bool,
}

impl MeshTransformFlags {
    fn from_body(body: &RigidBody) -> Self {
        let flags = body.collision_flags;
        Self {
            no_angular_motion: flags & CollisionFlags::NoAngularMotion != 0,
            identity: flags & CollisionFlags::IdentityTransform != 0,
        }
    }

    /// The body matrix is identity. `IdentityTransform` implies this too.
    const fn is_flat(self) -> bool {
        self.no_angular_motion || self.identity
    }
}

/// Build the box transform in triangle-mesh space.
///
/// The generic path is the original expression. The flat paths use the
/// `NoAngularMotion` matrix guarantee. Only `IdentityTransform` skips the
/// translation subtraction.
#[inline]
fn mesh_pair_transform(
    mesh_trans: &Affine3A,
    flags: MeshTransformFlags,
    child_world_trans: Affine3A,
) -> Affine3A {
    if flags.identity {
        child_world_trans
    } else if flags.no_angular_motion {
        Affine3A {
            matrix3: child_world_trans.matrix3,
            translation: child_world_trans.translation - mesh_trans.translation,
        }
    } else {
        let mesh_to_local = mesh_trans.transpose();
        Affine3A {
            matrix3: mesh_to_local.matrix3 * child_world_trans.matrix3,
            translation: mesh_to_local.transform_point3a(child_world_trans.translation),
        }
    }
}

/// Build the triangle-mesh to box transform.
///
/// Compute `box_to_local` once. The flat path replaces only the
/// `box_to_local.matrix3 * I` product. The translation keeps the original
/// `Affine3A` multiply order because it feeds contact geometry.
#[inline]
fn mesh_to_box_transform(
    box_trans: &Affine3A,
    mesh_trans: &Affine3A,
    flags: MeshTransformFlags,
) -> Affine3A {
    let box_to_local = box_trans.transpose();
    let matrix3 = if flags.is_flat() {
        box_to_local.matrix3
    } else {
        box_to_local.matrix3 * mesh_trans.matrix3
    };

    Affine3A {
        matrix3,
        translation: box_to_local.transform_point3a(mesh_trans.translation),
    }
}

struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    manifold: Option<PersistentManifold>,
    convex_obj: &'a RigidBody,
    tri_obj: &'a RigidBody,
    local_convex_aabb: Aabb,
    half: Vec3A,
    margin: f32,
    maximum_distance: f32,
    box_trans: Affine3A,
    tri_trans: &'a Affine3A,
    tri_flags: MeshTransformFlags,
    /// Composed on the first triangle that passes the local AABB test.
    /// A BVH walk with no leaf candidate never pays for the transform.
    mesh_to_box: Option<Affine3A>,
    contact_added_callback: &'a mut T,
}

impl<T: ContactAddedCallback> ProcessTriangle for ConvexTriangleCallback<'_, T> {
    fn process_triangle(&mut self, triangle: &TriangleShape, triangle_idx: usize) {
        let tri_min = triangle.points[0]
            .min(triangle.points[1])
            .min(triangle.points[2]);
        let tri_max = triangle.points[0]
            .max(triangle.points[1])
            .max(triangle.points[2]);
        if !(tri_min.cmple(self.local_convex_aabb.max).all()
            && tri_max.cmpge(self.local_convex_aabb.min).all())
        {
            return;
        }

        let mesh_to_box = match self.mesh_to_box {
            Some(mesh_to_box) => mesh_to_box,
            None => {
                let mesh_to_box =
                    mesh_to_box_transform(&self.box_trans, self.tri_trans, self.tri_flags);
                self.mesh_to_box = Some(mesh_to_box);
                mesh_to_box
            }
        };
        let q = [
            mesh_to_box.transform_point3a(triangle.points[0]),
            mesh_to_box.transform_point3a(triangle.points[1]),
            mesh_to_box.transform_point3a(triangle.points[2]),
        ];
        let Some(contact) = box_triangle_sat(
            &self.box_trans,
            self.half,
            self.margin,
            &q,
            self.maximum_distance,
        ) else {
            return;
        };

        let emit_normal = self.tri_trans.matrix3 * triangle.normal;
        let (convex_obj, tri_obj) = (self.convex_obj, self.tri_obj);
        let manifold = self
            .manifold
            .get_or_insert_with(|| PersistentManifold::new(convex_obj, tri_obj));
        let callback = &mut *self.contact_added_callback;
        manifold.add_contact_point(
            convex_obj,
            tri_obj,
            emit_normal,
            contact.point_on_b_world,
            contact.distance,
            Some(triangle_idx),
            callback,
        );
    }
}

/// World transform of the compound child shape.
///
/// This uses the same composition order as the gate AABB computation, so a
/// caller can reuse one value for both the gate and the contact pass without
/// changing rounding.
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
            let other_trans = other_obj.get_world_trans();
            let tri_flags = MeshTransformFlags::from_body(other_obj);
            let convex_in_triangle_space =
                mesh_pair_transform(other_trans, tri_flags, new_child_world_trans);
            let aabb_in_triangle = box_shape.get_aabb(&convex_in_triangle_space);

            // Constant for the whole BVH walk; cache here. Direct threshold
            // fields avoid constructing a manifold for empty calls.
            let half = box_shape.get_half_extents();
            let margin = box_shape.get_margin();
            let breaking = compound_obj
                .get_contact_breaking_threshold()
                .min(other_obj.get_contact_breaking_threshold());
            let maximum_distance = margin + breaking;

            let mut convex_triangle_callback = ConvexTriangleCallback {
                manifold: None,
                convex_obj: compound_obj,
                tri_obj: other_obj,
                local_convex_aabb: aabb_in_triangle,
                half,
                margin,
                maximum_distance,
                box_trans: new_child_world_trans,
                tri_trans: other_trans,
                tri_flags,
                mesh_to_box: None,
                contact_added_callback,
            };

            tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb_in_triangle);

            if let Some(mut manifold) = convex_triangle_callback.manifold {
                manifold.refresh_contact_points(compound_obj, other_obj);
                if !manifold.point_cache.is_empty() {
                    *out = Some(manifold);
                }
            }
        }
        CollisionShapes::StaticPlane(plane) => {
            let compound_obj_wrap = RigidBodyWrapper {
                obj: compound_obj,
                world_trans: new_child_world_trans,
                child_shape_override: Some(box_shape),
            };
            // The compound gate above already compared this child AABB with
            // `plane.aabb_cache`. Do not repeat the identical gate here.
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
