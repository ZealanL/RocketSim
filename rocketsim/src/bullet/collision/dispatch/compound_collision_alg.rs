use arrayvec::ArrayVec;
use glam::{Affine3A, Vec3A};

use crate::bullet::dynamics::rigid_body::RigidBody;
use crate::bullet::{
    collision::{
        broadphase::CollisionAlgorithm,
        dispatch::{
            collision_object_wrapper::RigidBodyWrapper,
            convex_plane_collision_alg::ConvexPlaneCollisionAlgorithm,
        },
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{
            box_shape::BoxShape, collision_shape::CollisionShapes, compound_shape::CompoundShape,
            triangle_callback::ProcessTriangle, triangle_shape::TriangleShape,
        },
    },
    linear_math::AffineExt,
};
use crate::shared::Aabb;

struct SatResult {
    penetration: f32,
    axis: Vec3A,
    axis_type: AxisType,
}

enum AxisType {
    TriFace,
    ObbFace(usize),
    EdgeEdge { box_axis: usize, tri_edge: usize },
}

struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: PersistentManifold,
    pub convex_obj: RigidBodyWrapper<'a>,
    pub tri_obj: &'a RigidBody,
    pub local_convex_aabb: &'a Aabb,
    pub box_shape: &'a BoxShape,
    pub contact_added_callback: &'a mut T,
}

impl<T: ContactAddedCallback> ProcessTriangle for ConvexTriangleCallback<'_, T> {
    fn process_triangle(&mut self, triangle: &TriangleShape, tri_aabb: &Aabb, triangle_idx: usize) {
        if !tri_aabb.intersects(self.local_convex_aabb) {
            return;
        }

        let half_extents = self.box_shape.get_half_extents();

        // Transform stuff to local space
        // (because colliding a triangle and an AABB is a *lot* simpler)
        let world_to_box = self.convex_obj.world_trans.inverse();
        let tri_to_world = self.tri_obj.get_world_trans();
        let tri_to_box = world_to_box * tri_to_world;

        let tri_verts_rel = [
            tri_to_box.transform_point3a(triangle.points[0]),
            tri_to_box.transform_point3a(triangle.points[1]),
            tri_to_box.transform_point3a(triangle.points[2]),
        ];

        let tri_edges_rel = [
            // This is probably faster than transforming all 3 "triangle.edges[]"
            tri_verts_rel[1] - tri_verts_rel[0],
            tri_verts_rel[2] - tri_verts_rel[1],
            tri_verts_rel[0] - tri_verts_rel[2],
        ];

        let mut min_penetration = f32::INFINITY;
        let mut best_result: Option<SatResult> = None;

        // Face normal tests
        for i in 0..3 {
            let tri_min = tri_verts_rel[0][i]
                .min(tri_verts_rel[1][i])
                .min(tri_verts_rel[2][i]);
            let tri_max = tri_verts_rel[0][i]
                .max(tri_verts_rel[1][i])
                .max(tri_verts_rel[2][i]);

            if tri_max < -half_extents[i] || tri_min > half_extents[i] {
                return;
            }

            let p_neg = tri_max + half_extents[i];
            let p_pos = half_extents[i] - tri_min;
            let (penetration, side) = if p_neg < p_pos {
                (p_neg, -1.0)
            } else {
                (p_pos, 1.0)
            };

            if penetration < min_penetration {
                min_penetration = penetration;
                let mut axis = Vec3A::ZERO;
                axis[i] = side;
                best_result = Some(SatResult {
                    penetration,
                    axis,
                    axis_type: AxisType::ObbFace(i),
                });
            }
        }

        // Triangle face normal test
        let tri_normal = tri_edges_rel[0].cross(tri_edges_rel[1]);
        if let Some(norm_axis) = tri_normal.try_normalize() {
            let plane_dist = norm_axis.dot(tri_verts_rel[0]);
            let projection_radius = half_extents.dot(norm_axis.abs());

            if plane_dist.abs() > projection_radius {
                return;
            }

            let penetration = projection_radius - plane_dist.abs();
            if penetration < min_penetration {
                min_penetration = penetration;
                best_result = Some(SatResult {
                    penetration,
                    axis: if plane_dist > 0.0 {
                        norm_axis
                    } else {
                        -norm_axis
                    },
                    axis_type: AxisType::TriFace,
                });
            }
        }

        // Edge-edge tests
        for box_axis_idx in 0..3 {
            let mut box_axis = Vec3A::ZERO;
            box_axis[box_axis_idx] = 1.0;

            for (edge_idx, &edge_vec) in tri_edges_rel.iter().enumerate() {
                let axis = box_axis.cross(edge_vec);
                if let Some(norm_axis) = axis.try_normalize() {
                    let p = Vec3A::new(
                        norm_axis.dot(tri_verts_rel[0]),
                        norm_axis.dot(tri_verts_rel[1]),
                        norm_axis.dot(tri_verts_rel[2]),
                    );
                    let (tri_min, tri_max) = (p.min_element(), p.max_element());
                    let projection_radius = half_extents.dot(norm_axis.abs());

                    if tri_max < -projection_radius || tri_min > projection_radius {
                        return;
                    }

                    let p_neg = tri_max + projection_radius;
                    let p_pos = projection_radius - tri_min;
                    let (penetration, axis_dir) = if p_neg < p_pos {
                        (p_neg, -norm_axis)
                    } else {
                        (p_pos, norm_axis)
                    };

                    if penetration < min_penetration {
                        min_penetration = penetration;
                        best_result = Some(SatResult {
                            penetration,
                            axis: axis_dir,
                            axis_type: AxisType::EdgeEdge {
                                box_axis: box_axis_idx,
                                tri_edge: edge_idx,
                            },
                        });
                    }
                }
            }
        }

        let Some(result) = best_result else {
            return;
        };

        // Calculate the actual contact point
        let contact_point_local = match result.axis_type {
            AxisType::TriFace => {
                let tri_centroid = (tri_verts_rel[0] + tri_verts_rel[1] + tri_verts_rel[2]) / 3.0;
                let plane_dist = result.axis.dot(tri_verts_rel[0]);

                let mut best_point = Vec3A::ZERO;
                let mut min_dist_sq = f32::INFINITY;
                for i in 0..8 {
                    let corner = half_extents
                        * Vec3A::new(
                            if i & 1 != 0 { 1.0 } else { -1.0 },
                            if i & 2 != 0 { 1.0 } else { -1.0 },
                            if i & 4 != 0 { 1.0 } else { -1.0 },
                        );

                    let dist_to_plane = result.axis.dot(corner) - plane_dist;
                    let projected = corner - result.axis * dist_to_plane;

                    let dist_sq = (projected - tri_centroid).length_squared();
                    if dist_sq < min_dist_sq && dist_to_plane.abs() < 0.1 {
                        min_dist_sq = dist_sq;
                        best_point = projected;
                    }
                }
                best_point
            }

            AxisType::ObbFace(face_idx) => {
                let mut contact_points = ArrayVec::<Vec3A, 6>::new();
                let face_coord = if result.axis[face_idx] > 0.0 {
                    half_extents[face_idx]
                } else {
                    -half_extents[face_idx]
                };

                for &vert in &tri_verts_rel {
                    if (vert[face_idx] - face_coord).abs() < 0.05 {
                        let diff = (vert.abs() - half_extents).max_element();
                        if diff < 0.05 {
                            contact_points.push(vert);
                        }
                    }
                }

                for i in 0..3 {
                    let start = tri_verts_rel[i];
                    let end = tri_verts_rel[(i + 1) % 3];
                    let (da, db) = (start[face_idx] - face_coord, end[face_idx] - face_coord);

                    if da * db < 0.0 {
                        let t = da / (da - db);
                        let intersection = start + (end - start) * t;
                        if (intersection.abs() - half_extents).max_element() < 0.05 {
                            contact_points.push(intersection);
                        }
                    }
                }

                if contact_points.is_empty() {
                    let mut deepest_vert = tri_verts_rel[0];
                    let mut max_dot = result.axis.dot(tri_verts_rel[0]);

                    for &vert in &tri_verts_rel[1..] {
                        let dot = result.axis.dot(vert);
                        if dot > max_dot {
                            max_dot = dot;
                            deepest_vert = vert;
                        }
                    }
                    deepest_vert
                } else {
                    let mean_contact_point =
                        contact_points.iter().sum::<Vec3A>() / (contact_points.len() as f32);
                    mean_contact_point
                }
            }

            AxisType::EdgeEdge { box_axis, tri_edge } => {
                let edge_pos = half_extents * result.axis.signum();
                let mut edge_start = edge_pos;
                let mut edge_end = edge_pos;
                edge_start[box_axis] = half_extents[box_axis];
                edge_end[box_axis] = -half_extents[box_axis];

                crate::shared::closest_points_between_segments(
                    edge_start,
                    edge_end,
                    tri_verts_rel[tri_edge],
                    tri_verts_rel[(tri_edge + 1) % 3],
                )
            }
        };

        let contact_point_world = self
            .convex_obj
            .world_trans
            .transform_point3a(contact_point_local);

        let normal_world_on_b = -self.convex_obj.world_trans.transform_vector3a(result.axis);
        let distance = -(result.penetration + self.box_shape.get_margin());
        self.manifold.add_contact_point(
            self.convex_obj.object,
            self.tri_obj,
            normal_world_on_b,
            contact_point_world,
            distance,
            -1,
            triangle_idx as i32,
            self.contact_added_callback,
        );
    }
}

pub struct CompoundCollisionAlgorithm<'a, T: ContactAddedCallback> {
    compound_obj: &'a RigidBody,
    compound_shape: &'a CompoundShape,
    other_obj: &'a RigidBody,
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<T: ContactAddedCallback> CompoundCollisionAlgorithm<'_, T> {
    pub fn process_child_shape(&mut self) -> Option<PersistentManifold> {
        let org_trans = *self.compound_obj.get_world_trans();

        let child_trans = &self.compound_shape.child_trans;
        let new_child_world_trans = org_trans * child_trans;

        let box_shape = &self.compound_shape.child_shape;
        let aabb1 = box_shape.get_aabb(&new_child_world_trans);

        let other_col_shape = self.other_obj.get_collision_shape();
        let aabb2 = other_col_shape.get_aabb(self.other_obj.get_world_trans());

        if !aabb1.intersects(&aabb2) {
            return None;
        }

        let compound_obj_wrap = RigidBodyWrapper {
            object: self.compound_obj,
            world_trans: new_child_world_trans,
        };

        match other_col_shape {
            CollisionShapes::TriangleMesh(tri_mesh) => {
                let xform1 = self.other_obj.get_world_trans().transpose();
                let xform2 = new_child_world_trans;
                let convex_in_triangle_space = Affine3A {
                    matrix3: xform1.matrix3 * xform2.matrix3,
                    translation: xform1.transform_point3a(xform2.translation),
                };
                let aabb_in_triangle = box_shape.get_aabb(&convex_in_triangle_space);

                let mut convex_triangle_callback = {
                    ConvexTriangleCallback {
                        manifold: PersistentManifold::new(
                            self.compound_obj,
                            self.other_obj,
                            self.is_swapped,
                        ),
                        convex_obj: compound_obj_wrap,
                        tri_obj: self.other_obj,
                        local_convex_aabb: &aabb_in_triangle,
                        box_shape,
                        contact_added_callback: self.contact_added_callback,
                    }
                };

                tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb_in_triangle);

                convex_triangle_callback
                    .manifold
                    .refresh_contact_points(self.compound_obj, self.other_obj);
                if convex_triangle_callback.manifold.point_cache.is_empty() {
                    None
                } else {
                    Some(convex_triangle_callback.manifold)
                }
            }
            CollisionShapes::StaticPlane(plane) => ConvexPlaneCollisionAlgorithm::new(
                compound_obj_wrap,
                self.other_obj,
                plane,
                self.is_swapped,
                self.contact_added_callback,
            )
            .process_collision(),

            // Unimplemented
            _ => todo!(),
        }
    }
}

impl<'a, T: ContactAddedCallback> CompoundCollisionAlgorithm<'a, T> {
    pub const fn new(
        compound_obj: &'a RigidBody,
        compound_shape: &'a CompoundShape,
        other_obj: &'a RigidBody,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            compound_obj,
            compound_shape,
            other_obj,
            is_swapped,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for CompoundCollisionAlgorithm<'_, T> {
    fn process_collision(mut self) -> Option<PersistentManifold> {
        self.process_child_shape()
            .and_then(|manifold| (!manifold.point_cache.is_empty()).then_some(manifold))
    }
}
