use arrayvec::ArrayVec;
use glam::{Affine3A, Vec3A};

use super::{
    collision_obj_wrapper::RigidBodyWrapper, convex_convex_collision_alg,
    convex_plane_collision_alg,
};
use crate::{
    bullet::{
        collision::{
            narrowphase::persistent_manifold::{
                CONTACT_BREAKING_THRESHOLD, ContactAddedCallback, PersistentManifold,
            },
            shapes::{
                box_shape::BoxShape, collision_shape::CollisionShapes,
                compound_shape::CompoundShape, triangle_callback::ProcessTriangle,
                triangle_shape::TriangleShape,
            },
        },
        dynamics::rigid_body::RigidBody,
        linear_math::AffineExt,
    },
    shared::{Aabb, rsmath},
};

#[derive(Clone, Copy)]
struct SatResult {
    penetration: f32,
    axis: Vec3A,
    axis_type: AxisType,
}

#[derive(Clone, Copy)]
enum AxisType {
    TriFace,
    ObbFace(usize),
    EdgeEdge { box_axis: usize, tri_edge: usize },
}

fn point_in_triangle(point: Vec3A, vertices: &[Vec3A; 3]) -> bool {
    const BARYCENTRIC_EPSILON: f32 = 1e-4;

    let edge_0 = vertices[1] - vertices[0];
    let edge_1 = vertices[2] - vertices[0];
    let point_offset = point - vertices[0];
    let dot_00 = edge_0.dot(edge_0);
    let dot_01 = edge_0.dot(edge_1);
    let dot_11 = edge_1.dot(edge_1);
    let dot_20 = point_offset.dot(edge_0);
    let dot_21 = point_offset.dot(edge_1);
    let denominator = dot_00 * dot_11 - dot_01 * dot_01;
    if denominator.abs() <= f32::EPSILON {
        return false;
    }

    let barycentric_0 = (dot_11 * dot_20 - dot_01 * dot_21) / denominator;
    let barycentric_1 = (dot_00 * dot_21 - dot_01 * dot_20) / denominator;
    barycentric_0 >= -BARYCENTRIC_EPSILON
        && barycentric_1 >= -BARYCENTRIC_EPSILON
        && barycentric_0 + barycentric_1 <= 1.0 + BARYCENTRIC_EPSILON
}

fn triangle_face_contact_point(axis: Vec3A, box_shape: &BoxShape, vertices: &[Vec3A; 3]) -> Vec3A {
    let plane_dist = axis.dot(vertices[0]);
    let corner = box_shape.local_get_supporting_vertex_without_margin(axis);
    let dist_to_plane = axis.dot(corner) - plane_dist;
    corner - axis * dist_to_plane
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
    fn process_triangle(&mut self, triangle: &TriangleShape, triangle_idx: usize) {
        if !triangle.aabb().intersects(self.local_convex_aabb) {
            return;
        }

        let half_extents = self.box_shape.get_half_extents();
        let sat_half_extents = half_extents + self.box_shape.get_margin();
        let contact_margin = self.manifold.contact_breaking_threshold;

        let world_to_box = self.convex_obj.world_trans.inverse();
        let tri_to_world = self.tri_obj.get_world_trans();
        let tri_to_box = world_to_box * tri_to_world;

        let tri_verts_rel = [
            tri_to_box.transform_point3a(triangle.points[0]),
            tri_to_box.transform_point3a(triangle.points[1]),
            tri_to_box.transform_point3a(triangle.points[2]),
        ];

        let tri_edges_rel = [
            tri_verts_rel[1] - tri_verts_rel[0],
            tri_verts_rel[2] - tri_verts_rel[1],
            tri_verts_rel[0] - tri_verts_rel[2],
        ];

        let mut min_non_face_penetration = f32::INFINITY;
        let mut best_result: Option<SatResult> = None;
        let mut triangle_face_result: Option<SatResult> = None;

        for i in 0..3 {
            let tri_min = tri_verts_rel[0][i]
                .min(tri_verts_rel[1][i])
                .min(tri_verts_rel[2][i]);
            let tri_max = tri_verts_rel[0][i]
                .max(tri_verts_rel[1][i])
                .max(tri_verts_rel[2][i]);

            let axis_extent = sat_half_extents[i] + contact_margin;
            if tri_max < -axis_extent || tri_min > axis_extent {
                return;
            }

            let p_neg = tri_max + axis_extent;
            let p_pos = axis_extent - tri_min;
            let (penetration, side) = if p_neg < p_pos {
                (p_neg, -1.0)
            } else {
                (p_pos, 1.0)
            };

            if penetration < min_non_face_penetration {
                min_non_face_penetration = penetration;
                let mut axis = Vec3A::ZERO;
                axis[i] = side;
                best_result = Some(SatResult {
                    penetration,
                    axis,
                    axis_type: AxisType::ObbFace(i),
                });
            }
        }

        let tri_normal = tri_edges_rel[0].cross(tri_edges_rel[1]);
        if let Some(norm_axis) = tri_normal.try_normalize() {
            let plane_dist = norm_axis.dot(tri_verts_rel[0]);
            let proj_radius = sat_half_extents.dot(norm_axis.abs()) + contact_margin;

            if plane_dist.abs() > proj_radius {
                return;
            }

            let penetration = proj_radius - plane_dist.abs();
            let face_result = SatResult {
                penetration,
                axis: if plane_dist > 0.0 {
                    norm_axis
                } else {
                    -norm_axis
                },
                axis_type: AxisType::TriFace,
            };
            triangle_face_result = Some(face_result);
        }

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
                    let proj_radius = sat_half_extents.dot(norm_axis.abs()) + contact_margin;

                    if tri_max < -proj_radius || tri_min > proj_radius {
                        return;
                    }

                    let p_neg = tri_max + proj_radius;
                    let p_pos = proj_radius - tri_min;
                    let (penetration, axis_dir) = if p_neg < p_pos {
                        (p_neg, -norm_axis)
                    } else {
                        (p_pos, norm_axis)
                    };

                    if penetration < min_non_face_penetration {
                        min_non_face_penetration = penetration;
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

        let triangle_face_result = triangle_face_result.filter(|result| {
            let contact_point =
                triangle_face_contact_point(result.axis, self.box_shape, &tri_verts_rel);
            point_in_triangle(contact_point, &tri_verts_rel)
        });
        let Some(result) = triangle_face_result.or(best_result) else {
            return;
        };

        let contact_point_local = match result.axis_type {
            AxisType::TriFace => {
                triangle_face_contact_point(result.axis, self.box_shape, &tri_verts_rel)
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
                    contact_points.iter().sum::<Vec3A>() / (contact_points.len() as f32)
                }
            }

            AxisType::EdgeEdge { box_axis, tri_edge } => {
                let edge_pos = half_extents * result.axis.signum();
                let mut edge_start = edge_pos;
                let mut edge_end = edge_pos;
                edge_start[box_axis] = half_extents[box_axis];
                edge_end[box_axis] = -half_extents[box_axis];

                rsmath::closest_points_between_segments(
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
        let triangle_normal_world = self
            .tri_obj
            .get_world_trans()
            .transform_vector3a(triangle.normal);
        if normal_world_on_b.dot(triangle_normal_world) < 0.0 {
            return;
        }
        let distance = self.manifold.contact_breaking_threshold - result.penetration;
        self.manifold.add_contact_point(
            self.convex_obj.obj,
            self.tri_obj,
            normal_world_on_b,
            contact_point_world,
            distance,
            Some(triangle_idx),
            self.contact_added_callback,
        );
    }
}

pub fn process_collision<T: ContactAddedCallback>(
    compound_obj: &RigidBody,
    compound_shape: &CompoundShape,
    other_obj: &RigidBody,
    contact_added_callback: &mut T,
) -> Option<PersistentManifold> {
    let org_trans = *compound_obj.get_world_trans();
    let child_trans = &compound_shape.child_trans;
    let new_child_world_trans = org_trans * child_trans;

    let box_shape = &compound_shape.child_shape;
    let aabb1 = box_shape.get_aabb(&new_child_world_trans);

    let other_col_shape = other_obj.get_collision_shape();
    let aabb2 = other_col_shape.get_aabb(other_obj.get_world_trans());

    if !aabb1.intersects(&aabb2) {
        return None;
    }

    let compound_obj_wrap = RigidBodyWrapper {
        obj: compound_obj,
        world_trans: new_child_world_trans,
        child_shape_override: Some(box_shape),
    };

    match other_col_shape {
        CollisionShapes::TriangleMesh(tri_mesh) => {
            let xform1 = other_obj.get_world_trans().transpose();
            let xform2 = new_child_world_trans;
            let convex_in_triangle_space = Affine3A {
                matrix3: xform1.matrix3 * xform2.matrix3,
                translation: xform1.transform_point3a(xform2.translation),
            };
            let aabb_in_triangle = box_shape.get_aabb(&convex_in_triangle_space);

            let mut manifold = PersistentManifold::new(compound_obj, other_obj);
            manifold.contact_breaking_threshold =
                box_shape.get_half_extents().length() * CONTACT_BREAKING_THRESHOLD;

            let mut convex_triangle_callback = ConvexTriangleCallback {
                manifold,
                convex_obj: compound_obj_wrap,
                tri_obj: other_obj,
                local_convex_aabb: &aabb_in_triangle,
                box_shape,
                contact_added_callback,
            };

            tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb_in_triangle);

            convex_triangle_callback
                .manifold
                .refresh_contact_points(compound_obj, other_obj);

            if convex_triangle_callback.manifold.point_cache.is_empty() {
                None
            } else {
                Some(convex_triangle_callback.manifold)
            }
        }
        CollisionShapes::StaticPlane(plane) => convex_plane_collision_alg::process_collision(
            &compound_obj_wrap,
            other_obj,
            plane,
            contact_added_callback,
        ),
        CollisionShapes::ConvexHull(_) => convex_convex_collision_alg::process_collision(
            &compound_obj_wrap,
            other_obj,
            contact_added_callback,
        ),
        _ => unimplemented!(),
    }
}
