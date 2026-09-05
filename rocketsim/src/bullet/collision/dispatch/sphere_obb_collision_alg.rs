use glam::Vec3A;

use crate::bullet::{
    collision::{
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{compound_shape::CompoundShape, sphere_shape::SphereShape},
    },
    dynamics::rigid_body::RigidBody,
    linear_math::AffineExt,
};

fn get_sphere_penetration(box_extents: Vec3A, sphere_from_local: Vec3A) -> (f32, Vec3A, Vec3A) {
    let mut min_dist = box_extents.x - sphere_from_local.x;
    let mut closest = sphere_from_local;
    closest.x = box_extents.x;
    let mut normal = Vec3A::X;

    let mut test_face = |face_dist: f32, axis: usize, sign: f32| {
        if face_dist < min_dist {
            min_dist = face_dist;
            closest = sphere_from_local;
            closest[axis] = box_extents[axis] * sign;
            normal = Vec3A::ZERO;
            normal[axis] = sign;
        }
    };

    test_face(box_extents.x + sphere_from_local.x, 0, -1.0);
    test_face(box_extents.y - sphere_from_local.y, 1, 1.0);
    test_face(box_extents.y + sphere_from_local.y, 1, -1.0);
    test_face(box_extents.z - sphere_from_local.z, 2, 1.0);
    test_face(box_extents.z + sphere_from_local.z, 2, -1.0);

    (min_dist, closest, normal)
}

pub fn process_collision<T: ContactAddedCallback>(
    sphere_obj: &RigidBody,
    sphere_shape: &SphereShape,
    obb_obj: &RigidBody,
    obb_shape: &CompoundShape,
    contact_added_callback: &mut T,
    out: &mut Option<PersistentManifold>,
) {
    debug_assert!(out.is_none());
    let sphere_trans = sphere_obj.get_world_trans();
    let aabb_1 = sphere_shape.get_aabb(sphere_trans);

    let org_trans = obb_obj.get_world_trans();
    let aabb_2 = obb_shape.get_aabb(org_trans);

    if !aabb_1.intersects(&aabb_2) {
        return;
    }

    let child_trans = &obb_shape.child_trans;
    let new_child_world_trans = org_trans * child_trans;

    let box_shape = &obb_shape.child_shape;
    let box_extents = box_shape.get_half_extents();

    let sphere_from_local = new_child_world_trans.inv_x_form(sphere_trans.translation);

    let mut closest = sphere_from_local.clamp(-box_extents, box_extents);
    let mut delta = sphere_from_local - closest;
    let dist_sq = delta.length_squared();

    let radius = sphere_shape.get_radius();
    let box_margin = box_shape.get_margin();

    let mut manifold = PersistentManifold::new(sphere_obj, obb_obj);
    let intersection_dist = radius + box_margin;
    let contact_dist = intersection_dist + manifold.contact_breaking_threshold;
    if dist_sq > contact_dist * contact_dist {
        return;
    }

    let mut dist;
    if dist_sq > f32::EPSILON {
        dist = dist_sq.sqrt();
        delta /= dist;
    } else {
        (dist, closest, delta) = get_sphere_penetration(box_extents, sphere_from_local);
        dist *= -1.0;
    };

    let normal_on_box = new_child_world_trans.transform_vector3a(delta);
    let point_on_box = new_child_world_trans.transform_point3a(closest);
    let depth = dist - intersection_dist;

    // This is the official contact point on the box
    let point_on_box_plus_margin = point_on_box + (normal_on_box * box_margin);
    manifold.add_contact_point(
        sphere_obj,
        obb_obj,
        normal_on_box,
        point_on_box_plus_margin,
        depth,
        None,
        contact_added_callback,
    );
    manifold.refresh_contact_points(sphere_obj, obb_obj);

    if manifold.point_cache.is_empty() {
        return;
    }
    *out = Some(manifold);
}
