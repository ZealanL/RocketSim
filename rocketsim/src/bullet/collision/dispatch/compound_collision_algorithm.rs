use glam::{Affine3A, Vec3A};

use crate::bullet::{
    collision::{
        broadphase::CollisionAlgorithm,
        dispatch::{
            collision_object::CollisionObject, collision_object_wrapper::CollisionObjectWrapper,
            convex_plane_collision_algorithm::ConvexPlaneCollisionAlgorithm,
        },
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{
            box_shape::BoxShape, collision_shape::CollisionShapes, compound_shape::CompoundShape,
            triangle_callback::ProcessTriangle, triangle_shape::TriangleShape,
        },
    },
    linear_math::{AffineExt, obb::Obb},
};
use crate::shared::Aabb;

struct Hit {
    depth: f32,
    normal: Vec3A,
    axis_index: usize,
    neg_axis: bool,
}

fn project_triangle(tri: &TriangleShape, axis: Vec3A) -> (f32, f32) {
    let projections = Vec3A::new(
        tri.points[0].dot(axis),
        tri.points[1].dot(axis),
        tri.points[2].dot(axis),
    );

    (projections.min_element(), projections.max_element())
}

pub fn project_box_radius(extent: Vec3A, axis: Vec3A) -> f32 {
    extent.dot(axis.abs())
}

fn aabb_triangle_sat(
    extent: Vec3A,
    tri: &TriangleShape,
    tri_normal_depth: f32,
    tri_normal_neg_axis: bool,
) -> Option<Hit> {
    const IDENT_AXES: [Vec3A; 3] = [Vec3A::X, Vec3A::Y, Vec3A::Z];

    let mut min_depth = tri_normal_depth;
    let mut min_axis = if tri_normal_neg_axis {
        -tri.normal
    } else {
        tri.normal
    };
    let mut min_axis_index = 0;
    let mut min_neg_axis = tri_normal_neg_axis;

    let mut axis_index = 0;
    for obb_axis in IDENT_AXES {
        axis_index += 1;

        let r_obb = project_box_radius(extent, obb_axis);
        let (tri_min, tri_max) = project_triangle(tri, obb_axis);
        if tri_max < -r_obb || tri_min > r_obb {
            return None;
        }

        let overlap_neg = tri_max + r_obb;
        let overlap_pos = r_obb - tri_min;
        let neg_axis = overlap_neg < overlap_pos;
        let (depth, normal) = if neg_axis {
            (overlap_neg, -obb_axis)
        } else {
            (overlap_pos, obb_axis)
        };

        if depth < min_depth {
            min_depth = depth;
            min_axis = normal;
            min_axis_index = axis_index;
            min_neg_axis = neg_axis;
        }
    }

    for obb_axis in IDENT_AXES {
        for &tri_edge in &tri.edges {
            axis_index += 1;
            let Some(axis) = obb_axis.cross(tri_edge).try_normalize() else {
                continue;
            };

            let r_obb = project_box_radius(extent, axis);
            let (tri_min, tri_max) = project_triangle(tri, axis);
            if tri_max < -r_obb || tri_min > r_obb {
                return None;
            }

            let overlap_neg = tri_max + r_obb;
            let overlap_pos = r_obb - tri_min;
            let neg_axis = overlap_neg < overlap_pos;
            let (depth, normal) = if neg_axis {
                (overlap_neg, -obb_axis)
            } else {
                (overlap_pos, obb_axis)
            };

            if depth < min_depth {
                min_depth = depth;
                min_axis = normal;
                min_axis_index = axis_index;
                min_neg_axis = neg_axis;
            }
        }
    }

    // No separating axis found
    Some(Hit {
        axis_index: min_axis_index,
        depth: min_depth,
        normal: min_axis,
        neg_axis: min_neg_axis,
    })
}

fn closest_point_on_segment(p: Vec3A, a: Vec3A, b: Vec3A) -> Vec3A {
    let ab = b - a;
    let t = (p - a).dot(ab) / ab.dot(ab);
    a + ab * t.clamp(0.0, 1.0)
}

struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    pub manifold: PersistentManifold,
    pub convex_obj: CollisionObjectWrapper<'a>,
    pub tri_obj: &'a CollisionObject,
    pub local_convex_aabb: &'a Aabb,
    pub box_shape: &'a BoxShape,
    pub contact_added_callback: &'a mut T,
}

impl<T: ContactAddedCallback> ConvexTriangleCallback<'_, T> {
    fn get_triangle_separation(&self, triangle: &[Vec3A; 3], inv_tri_normal: Vec3A) -> f32 {
        let local_pt = self.box_shape.local_get_supporting_vertex(inv_tri_normal);
        let proj_dist_pt = inv_tri_normal.dot(local_pt);
        let proj_dist_tr = inv_tri_normal.dot(triangle[0]);

        proj_dist_tr - proj_dist_pt
    }
}

impl<T: ContactAddedCallback> ProcessTriangle for ConvexTriangleCallback<'_, T> {
    fn process_triangle(
        &mut self,
        triangle: &TriangleShape,
        tri_aabb: &Aabb,
        triangle_index: usize,
    ) {
        if !tri_aabb.intersects(self.local_convex_aabb) {
            return;
        }

        // transform the triangle into OBB space
        let xform1 = self.convex_obj.world_transform.transpose();
        let xform2 = self.tri_obj.get_world_transform();
        let triangle_in_obb = Affine3A {
            matrix3: xform1.matrix3 * xform2.matrix3,
            translation: xform1.transform_point3a(xform2.translation),
        };

        let triangle_points_in_obb = [
            triangle_in_obb.transform_point3a(triangle.points[0]),
            triangle_in_obb.transform_point3a(triangle.points[1]),
            triangle_in_obb.transform_point3a(triangle.points[2]),
        ];

        let local_triangle = TriangleShape::new(triangle_points_in_obb);

        let back_dist =
            self.get_triangle_separation(&local_triangle.points, -local_triangle.normal);
        if back_dist > self.manifold.contact_breaking_threshold {
            return;
        }

        let front_dist =
            self.get_triangle_separation(&local_triangle.points, local_triangle.normal);
        if front_dist > self.manifold.contact_breaking_threshold {
            return;
        }

        let tri_normal_neg_axis = back_dist < front_dist;
        let tri_normal_depth = if tri_normal_neg_axis {
            back_dist
        } else {
            front_dist
        };
        if tri_normal_depth > 0.0 {
            return;
        }

        let obb = Obb::new(
            self.convex_obj.world_transform.translation,
            self.convex_obj.world_transform.matrix3,
            self.box_shape.get_half_extents(),
        );

        let Some(hit) = aabb_triangle_sat(
            obb.extent,
            &local_triangle,
            -tri_normal_depth,
            tri_normal_neg_axis,
        ) else {
            return;
        };

        let normal_on_b_in_world = self
            .convex_obj
            .world_transform
            .transform_vector3a(hit.normal);

        let point_in_world = match hit.axis_index {
            0 => {
                // Triangle normal axis
                let mut closest_pt = triangle.points[0];
                let mut min_dist_sqr = (closest_pt - obb.center).length_squared();
                for &pt in &triangle.points[1..] {
                    let dist_sqr = (pt - obb.center).length_squared();
                    if dist_sqr < min_dist_sqr {
                        min_dist_sqr = dist_sqr;
                        closest_pt = pt;
                    }
                }

                closest_pt
            }
            1..4 => {
                // Box edge axis
                let box_face_verts =
                    obb.get_face_verts(hit.axis_index - 1, if hit.neg_axis { -1.0 } else { 1.0 });

                let mut min_dist_sqr = f32::MAX;
                let mut closest_pt_tri = Vec3A::ZERO;
                for i in 0..4 {
                    let a = box_face_verts[i];
                    let b = box_face_verts[(i + 1) % 4];
                    let pt_on_seg = closest_point_on_segment(obb.center, a, b);
                    let dist_sqr = (pt_on_seg - obb.center).length_squared();
                    if dist_sqr < min_dist_sqr {
                        min_dist_sqr = dist_sqr;
                        closest_pt_tri = pt_on_seg;
                    }
                }

                closest_pt_tri
            }
            mut axis_index => {
                // Edge-edge axis
                axis_index -= 4;

                closest_point_on_segment(
                    obb.center,
                    triangle.points[axis_index % 3],
                    triangle.points[(axis_index + 1) % 3],
                )
            }
        };

        self.manifold.add_contact_point(
            self.convex_obj.object,
            self.tri_obj,
            normal_on_b_in_world,
            point_in_world,
            -hit.depth,
            -1,
            triangle_index as i32,
            self.contact_added_callback,
        );
    }
}

pub struct CompoundCollisionAlgorithm<'a, T: ContactAddedCallback> {
    compound_obj: &'a CollisionObject,
    compound_shape: &'a CompoundShape,
    other_obj: &'a CollisionObject,
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<T: ContactAddedCallback> CompoundCollisionAlgorithm<'_, T> {
    pub fn process_child_shape(&mut self) -> Option<PersistentManifold> {
        let org_trans = *self.compound_obj.get_world_transform();

        let child_trans = &self.compound_shape.child_transform;
        let new_child_world_trans = org_trans * child_trans;

        let box_shape = &self.compound_shape.child_shape;
        let aabb1 = box_shape.get_aabb(&new_child_world_trans);

        let other_col_shape = self.other_obj.get_collision_shape();
        let aabb2 = other_col_shape.get_aabb(self.other_obj.get_world_transform());

        if !aabb1.intersects(&aabb2) {
            return None;
        }

        let compound_obj_wrap = CollisionObjectWrapper {
            object: self.compound_obj,
            world_transform: new_child_world_trans,
        };

        match other_col_shape {
            CollisionShapes::TriangleMesh(tri_mesh) => {
                let xform1 = self.other_obj.get_world_transform().transpose();
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
        compound_obj: &'a CollisionObject,
        compound_shape: &'a CompoundShape,
        other_obj: &'a CollisionObject,
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
