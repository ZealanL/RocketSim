use arrayvec::ArrayVec;
use glam::{Affine3A, Vec3A};

use super::{
    collision_obj_wrapper::RigidBodyWrapper, convex_convex_collision_alg,
    convex_plane_collision_alg,
};
use crate::{
    bullet::{
        collision::{
            narrowphase::gjk::{ClosestPointInput, GjkPairDetector, GjkResult},
            narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
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

struct NoopGjkResult;

impl GjkResult for NoopGjkResult {
    fn add_contact_point(&mut self, _normal_on_b: Vec3A, _point_on_b_world: Vec3A, _depth: f32) {}
}

fn clip_polygon_against_plane(
    input: &[Vec3A],
    normal: Vec3A,
    plane_eq: f32,
) -> ArrayVec<Vec3A, 16> {
    let mut output = ArrayVec::new();
    if input.len() < 2 {
        return output;
    }

    let mut first = *input.last().unwrap();
    let mut ds = normal.dot(first) + plane_eq;
    for &end in input {
        let de = normal.dot(end) + plane_eq;
        if ds < 0.0 {
            if de < 0.0 {
                output.push(end);
            } else {
                output.push(first.lerp(end, ds / (ds - de)));
            }
        } else if de < 0.0 {
            output.push(first.lerp(end, ds / (ds - de)));
            output.push(end);
        }
        first = end;
        ds = de;
    }
    output
}

struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: PersistentManifold,
    pub convex_obj: RigidBodyWrapper<'a>,
    pub tri_obj: &'a RigidBody,
    pub local_convex_aabb: &'a Aabb,
    pub box_shape: &'a BoxShape,
    pub gjk_shape: Option<&'a CollisionShapes>,
    pub convex_world_trans: Affine3A,
    pub current_triangle_index: usize,
    pub contact_added_callback: &'a mut T,
}

impl<T: ContactAddedCallback> GjkResult for ConvexTriangleCallback<'_, T> {
    fn add_contact_point(&mut self, normal_on_b: Vec3A, point_on_b_world: Vec3A, depth: f32) {
        self.manifold.add_contact_point(
            self.convex_obj.obj,
            self.tri_obj,
            normal_on_b,
            point_on_b_world,
            depth,
            Some(self.current_triangle_index),
            self.contact_added_callback,
        );
    }
}

impl<T: ContactAddedCallback> ProcessTriangle for ConvexTriangleCallback<'_, T> {
    fn stable_triangle_order(&self) -> bool {
        true
    }

    fn process_triangle(&mut self, triangle: &TriangleShape, triangle_idx: usize) {
        if !triangle.aabb().intersects(self.local_convex_aabb) {
            return;
        }

        self.current_triangle_index = triangle_idx;

        let half_extents = self.box_shape.get_half_extents();
        let contact_margin = self.box_shape.get_margin() + self.manifold.contact_breaking_threshold;

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

        let mut min_penetration = f32::INFINITY;
        let mut best_result: Option<SatResult> = None;

        for i in 0..3 {
            let tri_min = tri_verts_rel[0][i]
                .min(tri_verts_rel[1][i])
                .min(tri_verts_rel[2][i]);
            let tri_max = tri_verts_rel[0][i]
                .max(tri_verts_rel[1][i])
                .max(tri_verts_rel[2][i]);

            let axis_extent = half_extents[i] + contact_margin;
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

        let tri_normal = tri_edges_rel[0].cross(tri_edges_rel[1]);
        if let Some(norm_axis) = tri_normal.try_normalize() {
            let plane_dist = norm_axis.dot(tri_verts_rel[0]);
            let proj_radius = half_extents.dot(norm_axis.abs()) + contact_margin;

            if plane_dist.abs() > proj_radius {
                return;
            }

            let penetration = proj_radius - plane_dist.abs();
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
                    let proj_radius = half_extents.dot(norm_axis.abs()) + contact_margin;

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

        // Diagnostic escape hatch: the legacy SAT contact construction is useful
        // when comparing the narrowphase against Bullet's per-triangle GJK.
        // It is disabled by default and must never affect normal runs.
        if let Some(gjk_shape) = self.gjk_shape {
            if std::env::var_os("RSIM_COMPOUND_SAT").is_some() {
                // Diagnostic only: continue with the legacy SAT contact below.
            } else if std::env::var_os("RSIM_CLIP_TRIANGLES").is_some() {
                let triangle_shape = CollisionShapes::Triangle(*triangle);
                let convex_world_trans = self.convex_world_trans;
                let contact_breaking_threshold = self.manifold.contact_breaking_threshold;
                let input = ClosestPointInput::new(
                    &convex_world_trans,
                    self.tri_obj.get_world_trans(),
                    self.box_shape.get_margin() + contact_breaking_threshold,
                );
                let mut noop = NoopGjkResult;
                let (cached_axis, cached_distance, found_contact) =
                    GjkPairDetector::new(self.box_shape.get_margin(), 0.0)
                        .get_closest_points_with_cache(
                            &input,
                            gjk_shape,
                            &triangle_shape,
                            &mut noop,
                        );

                if !found_contact || cached_axis.length_squared() <= f32::EPSILON {
                    return;
                }

                // Match Bullet's polyhedral-vs-triangle path: GJK supplies the
                // separating direction, then the triangle is clipped against the
                // incident box face and all resulting contacts are emitted.
                let sep_normal = cached_axis / cached_axis.length_squared();
                let min_dist = cached_distance - self.box_shape.get_margin();
                let min_clip = min_dist - self.manifold.contact_breaking_threshold;
                let max_clip = self.manifold.contact_breaking_threshold;
                let tri_world_trans = self.tri_obj.get_world_trans();
                let inv_box = convex_world_trans.inverse();
                let tri_local = [
                    inv_box
                        .transform_point3a(tri_world_trans.transform_point3a(triangle.points[0])),
                    inv_box
                        .transform_point3a(tri_world_trans.transform_point3a(triangle.points[1])),
                    inv_box
                        .transform_point3a(tri_world_trans.transform_point3a(triangle.points[2])),
                ];
                let full_half =
                    self.box_shape.get_half_extents() + Vec3A::splat(self.box_shape.get_margin());

                let mut face_axis = 0usize;
                let mut face_sign = 1.0f32;
                let mut face_dot = f32::INFINITY;
                for axis in 0..3 {
                    for sign in [-1.0f32, 1.0] {
                        let mut local_normal = Vec3A::ZERO;
                        local_normal[axis] = sign;
                        let world_face_normal = convex_world_trans.transform_vector3a(local_normal);
                        let dot = world_face_normal.dot(sep_normal);
                        if dot < face_dot {
                            face_dot = dot;
                            face_axis = axis;
                            face_sign = sign;
                        }
                    }
                }

                let mut polygon = ArrayVec::<Vec3A, 16>::new();
                polygon.extend(tri_local);
                for axis in 0..3 {
                    if axis == face_axis {
                        continue;
                    }
                    for sign in [-1.0f32, 1.0] {
                        let mut plane_normal = Vec3A::ZERO;
                        plane_normal[axis] = sign;
                        polygon =
                            clip_polygon_against_plane(&polygon, plane_normal, -full_half[axis]);
                        if polygon.is_empty() {
                            return;
                        }
                    }
                }

                let mut face_normal_local = Vec3A::ZERO;
                face_normal_local[face_axis] = face_sign;
                let face_plane = -full_half[face_axis];
                for point_local in polygon {
                    let mut depth = face_normal_local.dot(point_local) + face_plane;
                    if depth <= min_clip {
                        depth = min_clip;
                    }
                    if depth <= max_clip {
                        let point_world = convex_world_trans.transform_point3a(point_local);
                        self.manifold.add_contact_point(
                            self.convex_obj.obj,
                            self.tri_obj,
                            sep_normal,
                            point_world,
                            depth,
                            Some(triangle_idx),
                            self.contact_added_callback,
                        );
                    }
                }
                return;
            } else {
                let triangle_shape = CollisionShapes::Triangle(*triangle);
                let convex_world_trans = self.convex_world_trans;
                let input = ClosestPointInput::new(
                    &convex_world_trans,
                    self.tri_obj.get_world_trans(),
                    self.box_shape.get_margin() + self.manifold.contact_breaking_threshold,
                );
                GjkPairDetector::new(self.box_shape.get_margin(), 0.0).get_closest_points(
                    &input,
                    gjk_shape,
                    &triangle_shape,
                    self,
                );
                return;
            }
        }

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
        let distance = -(result.penetration + self.box_shape.get_margin());
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
            let gjk_shape = compound_shape.gjk_shape.as_ref();
            let xform1 = other_obj.get_world_trans().transpose();
            let xform2 = new_child_world_trans;
            let convex_in_triangle_space = Affine3A {
                matrix3: xform1.matrix3 * xform2.matrix3,
                translation: xform1.transform_point3a(xform2.translation),
            };
            let aabb_in_triangle = box_shape.get_aabb(&convex_in_triangle_space);

            let mut convex_triangle_callback = ConvexTriangleCallback {
                manifold: PersistentManifold::new(compound_obj, other_obj),
                convex_obj: compound_obj_wrap,
                tri_obj: other_obj,
                local_convex_aabb: &aabb_in_triangle,
                box_shape,
                gjk_shape: Some(gjk_shape),
                convex_world_trans: new_child_world_trans,
                current_triangle_index: 0,
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
