use std::{f32::consts::PI, mem};

use glam::{Quat, Vec3, Vec3A};

use super::tri_bvh_util::NodeOverlapCallback;
use crate::{
    bullet::{
        collision::{
            narrowphase::manifold_point::ManifoldPoint,
            shapes::{
                collision_shape::CollisionShapes,
                triangle_callback::ProcessTriangle,
                triangle_info_map::{TriInfoFlag, TriangleInfoMap},
                triangle_mesh::TriangleMesh,
                triangle_shape::TriangleShape,
            },
        },
        dynamics::rigid_body::RigidBody,
        linear_math::{AffineExt, QuatExt},
    },
    shared::bvh::Tree,
};

fn get_angle(edge_a: Vec3A, normal_a: Vec3A, normal_b: Vec3A) -> f32 {
    normal_b.dot(edge_a).atan2(normal_b.dot(normal_a))
}

struct ConnectivityProcessor<'a> {
    idx: usize,
    shape: &'a TriangleShape,
    info_map: &'a mut TriangleInfoMap,
}

impl ProcessTriangle for ConnectivityProcessor<'_> {
    fn process_triangle(&mut self, tri: &TriangleShape, triangle_idx: usize) {
        if self.idx == triangle_idx
            || tri.normal_length < TriangleInfoMap::EQUAL_VERTEX_THRESHOLD
            || self.shape.normal_length < TriangleInfoMap::EQUAL_VERTEX_THRESHOLD
        {
            return;
        }

        let mut num_shared = 0;
        let mut shared_verts_a = [0; 2];
        let mut shared_verts_b = [0; 2];

        for (i, vert_a) in self.shape.points.into_iter().enumerate() {
            for (j, vert) in tri.points.into_iter().enumerate() {
                if vert_a.distance_squared(vert) < TriangleInfoMap::EQUAL_VERTEX_THRESHOLD {
                    debug_assert!(num_shared < 2, "degenerate triangle");

                    shared_verts_a[num_shared] = i;
                    shared_verts_b[num_shared] = j;
                    num_shared += 1;
                }
            }
        }

        if num_shared == 2 {
            if shared_verts_a[0] == 0 && shared_verts_a[1] == 2 {
                shared_verts_a[0] = 2;
                shared_verts_a[1] = 0;

                let [a, b] = unsafe { shared_verts_b.get_disjoint_unchecked_mut([0, 1]) };
                mem::swap(a, b);
            }

            let info = &mut self.info_map[self.idx];
            let sum_verts_a = shared_verts_a[0] + shared_verts_a[1];
            let other_idx_a = 3 - sum_verts_a;

            let edge = (self.shape.points[shared_verts_a[1]]
                - self.shape.points[shared_verts_a[0]])
                .normalize();

            let other_idx_b = 3 - (shared_verts_b[0] + shared_verts_b[1]);

            let mut edge_cross_a = edge.cross(self.shape.normal).normalize();
            {
                let tmp = self.shape.points[other_idx_a] - self.shape.points[shared_verts_a[0]];
                if edge_cross_a.dot(tmp) < 0.0 {
                    edge_cross_a *= -1.0;
                }
            }

            let mut edge_cross_b = edge.cross(tri.normal).normalize();
            {
                let tmp = tri.points[other_idx_b] - tri.points[shared_verts_b[0]];
                if edge_cross_b.dot(tmp) < 0.0 {
                    edge_cross_b *= -1.0;
                }
            }

            let calculated_edge = edge_cross_a.cross(edge_cross_b);
            let len2 = calculated_edge.length_squared();

            let mut corrected_angle = 0.0;
            let mut is_convex = false;

            if len2 >= TriangleInfoMap::PLANAR_EPSILON {
                let calculated_edge = calculated_edge.normalize();
                let calculated_normal_a = calculated_edge.cross(edge_cross_a).normalize();
                let angle2 = get_angle(calculated_normal_a, edge_cross_a, edge_cross_b);
                let ang4 = PI - angle2;

                let dot_a = self.shape.normal.dot(edge_cross_b);
                is_convex = dot_a < 0.0;
                corrected_angle = if is_convex { ang4 } else { -ang4 };
            }

            match sum_verts_a {
                1 => {
                    let edge = (-self.shape.edges[0]).normalize();
                    let orn = Quat::from_axis_angle_simd(edge, -corrected_angle);
                    let mut computed_normal_b = orn * self.shape.normal;
                    if computed_normal_b.dot(tri.normal) < 0.0 {
                        computed_normal_b *= -1.0;
                        info.flags |= TriInfoFlag::V0V1SwapNormalB as u8;
                    }

                    info.edge_v0_v1_angle = -corrected_angle;
                    if is_convex {
                        info.flags |= TriInfoFlag::V0V1Convex as u8;
                    }
                }
                2 => {
                    let edge = (-self.shape.edges[2]).normalize();
                    let orn = Quat::from_axis_angle_simd(edge, -corrected_angle);
                    let mut computed_normal_b = orn * self.shape.normal;
                    if computed_normal_b.dot(tri.normal) < 0.0 {
                        computed_normal_b *= -1.0;
                        info.flags |= TriInfoFlag::V2V0SwapNormalB as u8;
                    }

                    info.edge_v2_v0_angle = -corrected_angle;
                    if is_convex {
                        info.flags |= TriInfoFlag::V2V0Convex as u8;
                    }
                }
                3 => {
                    let edge = (-self.shape.edges[1]).normalize();
                    let orn = Quat::from_axis_angle_simd(edge, -corrected_angle);
                    let mut computed_normal_b = orn * self.shape.normal;
                    if computed_normal_b.dot(tri.normal) < 0.0 {
                        computed_normal_b *= -1.0;
                        info.flags |= TriInfoFlag::V1V2SwapNormalB as u8;
                    }

                    info.edge_v1_v2_angle = -corrected_angle;
                    if is_convex {
                        info.flags |= TriInfoFlag::V1V2Convex as u8;
                    }
                }
                _ => unreachable!(),
            }
        }
    }
}

pub fn generate_internal_edge_info(bvh: &Tree, mesh_interface: &TriangleMesh) -> TriangleInfoMap {
    let mut triangle_info_map = TriangleInfoMap::new(mesh_interface.get_total_num_faces());

    for (i, triangle_a) in mesh_interface.get_tris().iter().enumerate() {
        let mut connectivity_processor = ConnectivityProcessor {
            idx: i,
            shape: triangle_a,
            info_map: &mut triangle_info_map,
        };

        let mut my_node_callback =
            NodeOverlapCallback::new(mesh_interface, &mut connectivity_processor);
        bvh.report_aabb_overlapping_node(&mut my_node_callback, &triangle_a.aabb);
    }

    triangle_info_map
}

fn nearst_point_in_line_segment(point: Vec3A, line0: Vec3A, line1: Vec3A) -> Vec3A {
    let line_delta = line1 - line0;

    if line_delta.length_squared() < f32::EPSILON * f32::EPSILON {
        line0
    } else {
        let delta = (point - line0).dot(line_delta) / line_delta.dot(line_delta);
        line0 + line_delta * delta.clamp(0.0, 1.0)
    }
}

fn clamp_normal(
    edge: Vec3A,
    tri_normal_org: Vec3A,
    local_contact_normal_on_b: Vec3A,
    corrected_edge_angle: f32,
) -> Option<Vec3A> {
    let edge_cross = edge.cross(tri_normal_org).normalize();
    let cur_angle = get_angle(edge_cross, tri_normal_org, local_contact_normal_on_b);

    if (corrected_edge_angle < 0.0 && cur_angle < corrected_edge_angle)
        || (corrected_edge_angle >= 0.0 && cur_angle > corrected_edge_angle)
    {
        let diff_angle = corrected_edge_angle - cur_angle;
        let rotation = Quat::from_axis_angle_simd(edge.normalize(), diff_angle);
        Some(rotation * local_contact_normal_on_b)
    } else {
        None
    }
}

enum BestEdge {
    None,
    X,
    Y,
    Z,
}

pub fn adjust_internal_edge_contacts(
    cp: &mut ManifoldPoint,
    tri_mesh_col_obj: &RigidBody,
    idx: usize,
) {
    let CollisionShapes::TriangleMesh(tri_mesh) = tri_mesh_col_obj.get_collision_shape() else {
        unreachable!();
    };

    let info = &tri_mesh.get_triangle_info_map()[idx];

    let tri = tri_mesh.get_mesh_interface().get_triangle(idx);
    let nearest = nearst_point_in_line_segment(cp.local_point_b, tri.points[0], tri.points[1]);
    let contact = cp.local_point_b;

    let local_contact_normal_on_b = tri_mesh_col_obj
        .get_world_trans()
        .matrix3
        .mul_transpose_vec3a(cp.normal_world_on_b);
    debug_assert!(local_contact_normal_on_b.is_normalized());

    let mut best_edge = BestEdge::None;
    let mut dists = Vec3::MAX;
    let mut dist_to_best_edge = f32::MAX;

    if info.edge_v0_v1_angle.abs() < TriangleInfoMap::MAX_EDGE_ANGLE_THRESHOLD {
        dists.x = (contact - nearest).length();
        if dists.x < dist_to_best_edge {
            best_edge = BestEdge::X;
            dist_to_best_edge = dists.x;
        }
    }

    if info.edge_v1_v2_angle.abs() < TriangleInfoMap::MAX_EDGE_ANGLE_THRESHOLD {
        let nearest = nearst_point_in_line_segment(cp.local_point_b, tri.points[1], tri.points[2]);
        dists.y = (contact - nearest).length();
        if dists.y < dist_to_best_edge {
            best_edge = BestEdge::Y;
            dist_to_best_edge = dists.y;
        }
    }

    if info.edge_v2_v0_angle.abs() < TriangleInfoMap::MAX_EDGE_ANGLE_THRESHOLD {
        let nearest = nearst_point_in_line_segment(cp.local_point_b, tri.points[2], tri.points[0]);
        dists.z = (contact - nearest).length();
        if dists.z < dist_to_best_edge {
            best_edge = BestEdge::Z;
        }
    }

    match best_edge {
        BestEdge::None => return,
        BestEdge::X => {
            if info.edge_v0_v1_angle.abs() >= TriangleInfoMap::MAX_EDGE_ANGLE_THRESHOLD
                || dists.x >= TriangleInfoMap::EDGE_DISTANCE_THRESHOLD
            {
                return;
            }

            if info.edge_v0_v1_angle != 0.0 {
                let is_edge_convex = (info.flags & TriInfoFlag::V0V1Convex as u8) != 0;
                let swap_factor = f32::from(is_edge_convex) * 2.0 - 1.0;

                let edge = -tri.edges[0];
                let n_a = swap_factor * tri.normal;
                let orn = Quat::from_axis_angle_simd(edge.normalize(), info.edge_v0_v1_angle);
                let mut computed_normal_b = orn * tri.normal;
                if (info.flags & TriInfoFlag::V0V1SwapNormalB as u8) != 0 {
                    computed_normal_b *= -1.0;
                }
                let n_b = swap_factor * computed_normal_b;

                let n_dot_a = local_contact_normal_on_b.dot(n_a);
                let n_dot_b = local_contact_normal_on_b.dot(n_b);
                let back_facing_normal = n_dot_a < TriangleInfoMap::CONVEX_EPSILON
                    && n_dot_b < TriangleInfoMap::CONVEX_EPSILON;

                if !back_facing_normal {
                    if let Some(clamped_local_normal) = clamp_normal(
                        edge,
                        swap_factor * tri.normal,
                        local_contact_normal_on_b,
                        info.edge_v0_v1_angle,
                    ) && clamped_local_normal.dot(tri.normal) > 0.0
                    {
                        let new_normal =
                            tri_mesh_col_obj.get_world_trans().matrix3 * clamped_local_normal;
                        cp.normal_world_on_b = new_normal;
                        cp.pos_world_on_b = cp.pos_world_on_a - new_normal * cp.distance_1;
                        cp.local_point_b = tri_mesh_col_obj
                            .get_world_trans()
                            .inv_x_form(cp.pos_world_on_b);
                    }

                    return;
                }
            }
        }
        BestEdge::Y => {
            if info.edge_v1_v2_angle.abs() >= TriangleInfoMap::MAX_EDGE_ANGLE_THRESHOLD
                || dists.y >= TriangleInfoMap::EDGE_DISTANCE_THRESHOLD
            {
                return;
            }

            if info.edge_v1_v2_angle != 0.0 {
                let is_edge_convex = (info.flags & TriInfoFlag::V1V2Convex as u8) != 0;
                let swap_factor = f32::from(is_edge_convex) * 2.0 - 1.0;

                let edge = -tri.edges[1];
                let n_a = swap_factor * tri.normal;
                let orn = Quat::from_axis_angle_simd(edge.normalize(), info.edge_v1_v2_angle);
                let mut computed_normal_b = orn * tri.normal;
                if (info.flags & TriInfoFlag::V1V2SwapNormalB as u8) != 0 {
                    computed_normal_b *= -1.0;
                }
                let n_b = swap_factor * computed_normal_b;

                let n_dot_a = local_contact_normal_on_b.dot(n_a);
                let n_dot_b = local_contact_normal_on_b.dot(n_b);
                let back_facing_normal = n_dot_a < TriangleInfoMap::CONVEX_EPSILON
                    && n_dot_b < TriangleInfoMap::CONVEX_EPSILON;

                if !back_facing_normal {
                    if let Some(clamped_local_normal) = clamp_normal(
                        edge,
                        swap_factor * tri.normal,
                        local_contact_normal_on_b,
                        info.edge_v1_v2_angle,
                    ) && clamped_local_normal.dot(tri.normal) > 0.0
                    {
                        let new_normal =
                            tri_mesh_col_obj.get_world_trans().matrix3 * clamped_local_normal;
                        cp.normal_world_on_b = new_normal;
                        cp.pos_world_on_b = cp.pos_world_on_a - new_normal * cp.distance_1;
                        cp.local_point_b = tri_mesh_col_obj
                            .get_world_trans()
                            .inv_x_form(cp.pos_world_on_b);
                    }

                    return;
                }
            }
        }
        BestEdge::Z => {
            if info.edge_v2_v0_angle.abs() >= TriangleInfoMap::MAX_EDGE_ANGLE_THRESHOLD
                || dists.z >= TriangleInfoMap::EDGE_DISTANCE_THRESHOLD
            {
                return;
            }

            if info.edge_v2_v0_angle != 0.0 {
                let is_edge_convex = (info.flags & TriInfoFlag::V2V0Convex as u8) != 0;
                let swap_factor = f32::from(is_edge_convex) * 2.0 - 1.0;

                let edge = -tri.edges[2];
                let n_a = swap_factor * tri.normal;
                let orn = Quat::from_axis_angle_simd(edge.normalize(), info.edge_v2_v0_angle);
                let mut computed_normal_b = orn * tri.normal;
                if (info.flags & TriInfoFlag::V2V0SwapNormalB as u8) != 0 {
                    computed_normal_b *= -1.0;
                }
                let n_b = swap_factor * computed_normal_b;

                let n_dot_a = local_contact_normal_on_b.dot(n_a);
                let n_dot_b = local_contact_normal_on_b.dot(n_b);
                let back_facing_normal = n_dot_a < TriangleInfoMap::CONVEX_EPSILON
                    && n_dot_b < TriangleInfoMap::CONVEX_EPSILON;

                if !back_facing_normal {
                    if let Some(clamped_local_normal) = clamp_normal(
                        edge,
                        swap_factor * tri.normal,
                        local_contact_normal_on_b,
                        info.edge_v2_v0_angle,
                    ) && clamped_local_normal.dot(tri.normal) > 0.0
                    {
                        let new_normal =
                            tri_mesh_col_obj.get_world_trans().matrix3 * clamped_local_normal;
                        cp.normal_world_on_b = new_normal;
                        cp.pos_world_on_b = cp.pos_world_on_a - new_normal * cp.distance_1;
                        cp.local_point_b = tri_mesh_col_obj
                            .get_world_trans()
                            .inv_x_form(cp.pos_world_on_b);
                    }

                    return;
                }
            }
        }
    }

    let d = tri.normal.dot(local_contact_normal_on_b);
    if d < 0.0 {
        return;
    }

    cp.normal_world_on_b = tri_mesh_col_obj.get_world_trans().matrix3 * tri.normal;
    cp.pos_world_on_b = cp.pos_world_on_a - cp.normal_world_on_b * cp.distance_1;
    cp.local_point_b = tri_mesh_col_obj
        .get_world_trans()
        .inv_x_form(cp.pos_world_on_b);
}
