//! Box-versus-triangle-mesh contact handling.
//!
//! The parent module owns the compound child AABB gate and the dispatch entry
//! points. This module owns the mesh transforms, the per-triangle callback, and
//! the separating-axis and GJK contact tests for one compound child box.

use glam::{Affine3A, Vec3A};

#[cfg(not(feature = "box-triangle-gjk"))]
use crate::bullet::collision::narrowphase::box_triangle_sat::box_triangle_sat;
#[cfg(feature = "box-triangle-gjk")]
use crate::bullet::collision::narrowphase::gjk::{ClosestPointInput, GjkPairDetector, GjkResult};
#[cfg(feature = "box-triangle-gjk")]
use crate::bullet::collision::shapes::collision_shape::CollisionShapes;
use crate::{
    bullet::{
        collision::{
            narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
            shapes::{
                bvh_triangle_mesh_shape::BvhTriangleMeshShape, compound_shape::CompoundShape,
                triangle_callback::ProcessTriangle, triangle_shape::TriangleShape,
            },
        },
        dynamics::rigid_body::{CollisionFlags, RigidBody},
        linear_math::AffineExt,
    },
    shared::Aabb,
};

/// Cached static-mesh flags for one compound box and mesh pair.
///
/// The `RigidBody` transform setters maintain `NoAngularMotion` and
/// `IdentityTransform`. This code reads the flags. It does not test the matrix.
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

    /// The mesh matrix is identity. `IdentityTransform` also implies this condition.
    #[cfg(not(feature = "box-triangle-gjk"))]
    const fn is_flat(self) -> bool {
        self.no_angular_motion || self.identity
    }
}

/// Build the box transform in triangle-mesh space.
///
/// `IdentityTransform` returns `box_world_trans` without changes.
/// `NoAngularMotion` removes the mesh translation only. The general path
/// applies the inverse mesh transform. All values are in Bullet physics meters.
#[inline]
fn mesh_pair_transform(
    mesh_trans: &Affine3A,
    flags: MeshTransformFlags,
    box_world_trans: Affine3A,
) -> Affine3A {
    if flags.identity {
        box_world_trans
    } else if flags.no_angular_motion {
        Affine3A {
            matrix3: box_world_trans.matrix3,
            translation: box_world_trans.translation - mesh_trans.translation,
        }
    } else {
        let mesh_to_local = mesh_trans.transpose();
        Affine3A {
            matrix3: mesh_to_local.matrix3 * box_world_trans.matrix3,
            translation: mesh_to_local.transform_point3a(box_world_trans.translation),
        }
    }
}

/// Build the triangle-mesh-to-box transform.
///
/// The flat path uses the box matrix only. The general path multiplies the box
/// matrix by the mesh matrix. The translation uses the `Affine3A` product order
/// that the contact geometry expects.
#[inline]
#[cfg(not(feature = "box-triangle-gjk"))]
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

/// Contact witness for one compound box and one mesh triangle.
///
/// `point_on_b_world` is on the triangle body and in world space.
/// `distance` is the signed separation from the selected narrowphase test.
/// A negative value is a penetration. Manifold insertion uses the mesh
/// triangle normal instead of the narrowphase normal.
#[derive(Clone, Copy, Debug)]
struct BoxTriangleContact {
    point_on_b_world: Vec3A,
    distance: f32,
}

/// Maximum GJK search distance, in Bullet physics meters.
///
/// The GJK loop uses this value directly because Bullet physics positions use
/// meters. The contact threshold is a separate input.
#[cfg(feature = "box-triangle-gjk")]
const GJK_LOOP_MAX_DISTANCE_BT: f32 = 10.0;

/// Separating-axis test for the compound box and one mesh triangle.
///
/// `q` holds the triangle points in box-local space. The result is `None` when
/// the separating distance is at or above `maximum_distance`.
#[cfg(not(feature = "box-triangle-gjk"))]
#[inline]
fn box_triangle_contact(
    box_trans: &Affine3A,
    half: Vec3A,
    margin: f32,
    q: &[Vec3A; 3],
    maximum_distance: f32,
) -> Option<BoxTriangleContact> {
    box_triangle_sat(box_trans, half, margin, q, maximum_distance).map(|contact| {
        BoxTriangleContact {
            point_on_b_world: contact.point_on_b_world,
            distance: contact.distance,
        }
    })
}

/// GJK pair test for the compound box and one mesh triangle.
///
/// Shape A is the compound shape at the box world transform. Shape B is the
/// mesh triangle at the mesh world transform. The compound shape has one child
/// box, so its support vertex and margin are the box values. The GJK loop uses
/// [`GJK_LOOP_MAX_DISTANCE_BT`] as its search bound. Contacts at or above
/// `maximum_distance` are removed after the solve.
#[cfg(feature = "box-triangle-gjk")]
fn box_triangle_contact(
    box_shape: &CollisionShapes,
    box_trans: &Affine3A,
    triangle: &TriangleShape,
    tri_trans: &Affine3A,
    maximum_distance: f32,
) -> Option<BoxTriangleContact> {
    let triangle_shape = CollisionShapes::Triangle(*triangle);
    let margin_a = box_shape.get_margin();
    let input = ClosestPointInput::new(box_trans, tri_trans, GJK_LOOP_MAX_DISTANCE_BT);
    let detector = GjkPairDetector::new(margin_a, triangle_shape.get_margin());
    let mut sink = GjkContactSink(None);
    detector.get_closest_points(&input, box_shape, &triangle_shape, &mut sink);
    sink.0.filter(|contact| contact.distance < maximum_distance)
}

#[cfg(feature = "box-triangle-gjk")]
struct GjkContactSink(Option<BoxTriangleContact>);

#[cfg(feature = "box-triangle-gjk")]
impl GjkResult for GjkContactSink {
    fn add_contact_point(&mut self, _normal_on_b: Vec3A, point_on_b_world: Vec3A, depth: f32) {
        self.0 = Some(BoxTriangleContact {
            point_on_b_world,
            distance: depth,
        });
    }
}

/// Per-triangle sink for one compound box and one triangle mesh.
///
/// The sink keeps the first manifold that receives a contact point.
struct ConvexTriangleCallback<'a, T: ContactAddedCallback> {
    manifold: Option<PersistentManifold>,
    convex_obj: &'a RigidBody,
    tri_obj: &'a RigidBody,
    local_convex_aabb: Aabb,
    #[cfg(not(feature = "box-triangle-gjk"))]
    half: Vec3A,
    #[cfg(not(feature = "box-triangle-gjk"))]
    margin: f32,
    maximum_distance: f32,
    box_trans: Affine3A,
    tri_trans: &'a Affine3A,
    /// Composed on the first triangle that passes the local AABB test. A BVH
    /// walk with no candidate does not compute the transform.
    #[cfg(not(feature = "box-triangle-gjk"))]
    mesh_to_box: Option<Affine3A>,
    #[cfg(not(feature = "box-triangle-gjk"))]
    tri_flags: MeshTransformFlags,
    /// Box support for the shared GJK path. The compound shape has one child
    /// box, so its support vertex and margin are the child box values.
    #[cfg(feature = "box-triangle-gjk")]
    box_shape: CollisionShapes,
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

        #[cfg(not(feature = "box-triangle-gjk"))]
        let contact = {
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
            box_triangle_contact(
                &self.box_trans,
                self.half,
                self.margin,
                &q,
                self.maximum_distance,
            )
        };
        #[cfg(feature = "box-triangle-gjk")]
        let contact = box_triangle_contact(
            &self.box_shape,
            &self.box_trans,
            triangle,
            self.tri_trans,
            self.maximum_distance,
        );
        let Some(contact) = contact else {
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

/// Test one compound child box against the triangles of a mesh body.
///
/// `box_world_trans` is the world transform of the child box. The function
/// walks the triangles near the box, inserts contact points into a manifold,
/// and refreshes it. The result is `None` when no contact point survives the
/// refresh.
///
/// The caller must apply the child AABB gate before this call.
pub(super) fn process_box_mesh_collision<T: ContactAddedCallback>(
    compound_obj: &RigidBody,
    compound_shape: &CompoundShape,
    box_world_trans: Affine3A,
    mesh_obj: &RigidBody,
    tri_mesh: &BvhTriangleMeshShape,
    contact_added_callback: &mut T,
) -> Option<PersistentManifold> {
    let box_shape = &compound_shape.child_shape;
    let mesh_trans = mesh_obj.get_world_trans();
    let tri_flags = MeshTransformFlags::from_body(mesh_obj);
    let box_in_mesh_space = mesh_pair_transform(mesh_trans, tri_flags, box_world_trans);
    let aabb_in_mesh = box_shape.get_aabb(&box_in_mesh_space);

    // Constant for the whole BVH walk; cache here. Direct threshold fields
    // avoid constructing a manifold for empty calls.
    #[cfg(not(feature = "box-triangle-gjk"))]
    let half = box_shape.get_half_extents();
    let margin = box_shape.get_margin();
    let breaking = compound_obj
        .get_contact_breaking_threshold()
        .min(mesh_obj.get_contact_breaking_threshold());
    let maximum_distance = margin + breaking;

    let mut callback = ConvexTriangleCallback {
        manifold: None,
        convex_obj: compound_obj,
        tri_obj: mesh_obj,
        local_convex_aabb: aabb_in_mesh,
        #[cfg(not(feature = "box-triangle-gjk"))]
        half,
        #[cfg(not(feature = "box-triangle-gjk"))]
        margin,
        maximum_distance,
        box_trans: box_world_trans,
        tri_trans: mesh_trans,
        #[cfg(not(feature = "box-triangle-gjk"))]
        tri_flags,
        #[cfg(not(feature = "box-triangle-gjk"))]
        mesh_to_box: None,
        #[cfg(feature = "box-triangle-gjk")]
        box_shape: CollisionShapes::Compound(*compound_shape),
        contact_added_callback,
    };

    tri_mesh.process_all_triangles(&mut callback, &aabb_in_mesh);

    let mut manifold = callback.manifold?;
    manifold.refresh_contact_points(compound_obj, mesh_obj);
    if manifold.point_cache.is_empty() {
        None
    } else {
        Some(manifold)
    }
}
