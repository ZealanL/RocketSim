use glam::{Affine3A, Mat3A, Vec3A};

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
        dynamics::rigid_body::RigidBody,
        linear_math::AffineExt,
    },
    shared::Aabb,
};

struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    manifold: Option<PersistentManifold>,
    convex_obj: &'a RigidBody,
    tri_obj: &'a RigidBody,
    local_convex_aabb: Aabb,
    half: Vec3A,
    margin: f32,
    maximum_distance: f32,
    box_trans: Affine3A,
    mesh_to_box: Affine3A,
    tri_matrix: Mat3A,
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

        let q = [
            self.mesh_to_box.transform_point3a(triangle.points[0]),
            self.mesh_to_box.transform_point3a(triangle.points[1]),
            self.mesh_to_box.transform_point3a(triangle.points[2]),
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

        let emit_normal = self.tri_matrix * triangle.normal;
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

pub fn process_collision<T: ContactAddedCallback>(
    compound_obj: &RigidBody,
    compound_shape: &CompoundShape,
    other_obj: &RigidBody,
    contact_added_callback: &mut T,
    out: &mut Option<PersistentManifold>,
) {
    debug_assert!(out.is_none());
    let org_trans = *compound_obj.get_world_trans();
    let child_trans = &compound_shape.child_trans;
    let new_child_world_trans = org_trans * child_trans;

    let box_shape = &compound_shape.child_shape;
    let aabb1 = box_shape.get_aabb(&new_child_world_trans);

    let other_col_shape = other_obj.get_collision_shape();
    let aabb2 = other_col_shape.get_aabb(other_obj.get_world_trans());

    if !aabb1.intersects(&aabb2) {
        return;
    }

    match other_col_shape {
        CollisionShapes::TriangleMesh(tri_mesh) => {
            let other_trans = *other_obj.get_world_trans();
            let xform1 = other_trans.transpose();
            let xform2 = new_child_world_trans;
            let convex_in_triangle_space = Affine3A {
                matrix3: xform1.matrix3 * xform2.matrix3,
                translation: xform1.transform_point3a(xform2.translation),
            };
            let aabb_in_triangle = box_shape.get_aabb(&convex_in_triangle_space);

            // Mesh-to-box transform, composed once per compound/mesh collision.
            let mesh_to_box = new_child_world_trans.transpose() * other_trans;
            // Constant for the whole BVH walk; cache here. Direct threshold
            // fields avoid constructing a manifold for empty calls.
            let half = box_shape.get_half_extents();
            let margin = box_shape.get_margin();
            let breaking = compound_obj
                .get_contact_breaking_threshold()
                .min(other_obj.get_contact_breaking_threshold());
            let maximum_distance = margin + breaking;
            let tri_matrix = other_trans.matrix3;

            let mut convex_triangle_callback = ConvexTriangleCallback {
                manifold: None,
                convex_obj: compound_obj,
                tri_obj: other_obj,
                local_convex_aabb: aabb_in_triangle,
                half,
                margin,
                maximum_distance,
                box_trans: new_child_world_trans,
                mesh_to_box,
                tri_matrix,
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
            convex_plane_collision_alg::process_collision(
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
