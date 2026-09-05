use glam::{Affine3A, Vec3A};

use crate::bullet::{
    collision::{
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{
            bvh_triangle_mesh_shape::BvhTriangleMeshShape, sphere_shape::SphereShape,
            triangle_callback::ProcessTriangle, triangle_shape::TriangleShape,
        },
    },
    dynamics::rigid_body::RigidBody,
    linear_math::AffineExt,
};

pub(crate) struct PendingSphereContact {
    normal_on_b: Vec3A,
    point_in_world: Vec3A,
    depth: f32,
    triangle_idx: usize,
}

struct SphereTriangleCallback<'a> {
    pub collected: &'a mut Vec<PendingSphereContact>,
    pub tri_obj: &'a RigidBody,
    sphere_center: Vec3A,
    sphere_radius: f32,
    contact_breaking_threshold: f32,
}

impl<'a> SphereTriangleCallback<'a> {
    pub fn new(
        collected: &'a mut Vec<PendingSphereContact>,
        tri_obj: &'a RigidBody,
        sphere_center: Vec3A,
        sphere_radius: f32,
        contact_breaking_threshold: f32,
    ) -> Self {
        Self {
            collected,
            tri_obj,
            sphere_center,
            sphere_radius,
            contact_breaking_threshold,
        }
    }
}

impl ProcessTriangle for SphereTriangleCallback<'_> {
    fn process_triangle(&mut self, triangle: &TriangleShape, triangle_idx: usize) {
        let Some(contact_info) = triangle.intersect_sphere(
            self.sphere_center,
            self.sphere_radius,
            self.contact_breaking_threshold,
        ) else {
            return;
        };

        // Keep only front-side triangle contacts.
        let center_to_tri = self.sphere_center - triangle.points[0];
        if center_to_tri.dot(triangle.normal) < 0.0 {
            return;
        }

        let tri_world = self.tri_obj.get_world_trans();
        let normal_on_b = tri_world.transform_vector3a(contact_info.result_normal);
        let point_in_world = tri_world.transform_point3a(contact_info.contact_point);

        self.collected.push(PendingSphereContact {
            normal_on_b,
            point_in_world,
            depth: contact_info.depth,
            triangle_idx,
        });
    }
}

pub(crate) fn process_collision_into<T: ContactAddedCallback>(
    convex_obj: &RigidBody,
    sphere_shape: &SphereShape,
    concave_obj: &RigidBody,
    tri_mesh: &BvhTriangleMeshShape,
    manifold: &mut PersistentManifold,
    scratch: &mut Vec<PendingSphereContact>,
    contact_added_callback: &mut T,
) -> bool {
    manifold.most_recently_evicted_point = None;
    scratch.clear();

    let xform1 = convex_obj.get_world_trans();
    let xform2 = concave_obj.get_world_trans().transpose();
    let convex_in_triangle_space = Affine3A {
        matrix3: xform2.matrix3 * xform1.matrix3,
        translation: xform2.transform_point3a(xform1.translation),
    };

    let contact_breaking_threshold = manifold.contact_breaking_threshold;
    {
        let mut convex_triangle_callback = SphereTriangleCallback::new(
            scratch,
            concave_obj,
            convex_in_triangle_space.translation,
            sphere_shape.get_radius(),
            contact_breaking_threshold,
        );

        let aabb = sphere_shape.get_aabb(&convex_in_triangle_space);
        tri_mesh.process_all_triangles(&mut convex_triangle_callback, &aabb);
    }

    for contact in scratch.iter() {
        manifold.add_contact_point(
            convex_obj,
            concave_obj,
            contact.normal_on_b,
            contact.point_in_world,
            contact.depth,
            Some(contact.triangle_idx),
            contact_added_callback,
        );
    }

    // Skip the no-op empty refresh (see `refresh_contact_points`).
    if !manifold.point_cache.is_empty() {
        manifold.refresh_contact_points(convex_obj, concave_obj);
    }

    !manifold.point_cache.is_empty()
}
