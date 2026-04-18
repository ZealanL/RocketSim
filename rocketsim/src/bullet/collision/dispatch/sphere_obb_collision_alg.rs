use glam::Vec3A;

use crate::bullet::{
    collision::{
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{compound_shape::CompoundShape, sphere_shape::SphereShape},
    },
    dynamics::rigid_body::RigidBody,
    linear_math::AffineExt,
};

pub fn process_collision<T: ContactAddedCallback>(
    sphere_obj: &RigidBody,
    sphere_shape: &SphereShape,
    obb_obj: &RigidBody,
    obb_shape: &CompoundShape,
    is_swapped: bool,
    contact_added_callback: &mut T,
) -> Option<PersistentManifold> {
    let sphere_trans = sphere_obj.get_world_trans();
    let aabb_1 = sphere_shape.get_aabb(sphere_trans);

    let org_trans = obb_obj.get_world_trans();
    let aabb_2 = obb_shape.get_aabb(org_trans);

    if !aabb_1.intersects(&aabb_2) {
        return None;
    }

    let child_trans = &obb_shape.child_trans;
    let new_child_world_trans = org_trans * child_trans;

    let box_shape = &obb_shape.child_shape;
    let box_extents = box_shape.get_half_extents();

    let sphere_from_local = new_child_world_trans.inv_x_form(sphere_trans.translation);

    let closest = sphere_from_local.clamp(-box_extents, box_extents);
    let delta = sphere_from_local - closest;
    let dist_sq = delta.length_squared();

    let radius = sphere_shape.get_radius();
    let box_margin = box_shape.get_margin();
    if dist_sq >= (radius + box_margin).powi(2) {
        return None;
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > f32::EPSILON {
        delta / dist
    } else {
        Vec3A::X
    };

    let normal_on_box = new_child_world_trans.transform_vector3a(normal);
    let point_on_box = new_child_world_trans.transform_point3a(closest);
    let depth = dist - radius - box_margin;

    // This is the official contact point on the box
    let point_on_box_plus_margin = point_on_box + (normal_on_box * box_margin);

    let mut manifold = PersistentManifold::new(sphere_obj, obb_obj, is_swapped);
    manifold.add_contact_point(
        sphere_obj,
        obb_obj,
        normal_on_box,
        point_on_box_plus_margin,
        depth,
        -1,
        -1,
        contact_added_callback,
    );
    manifold.refresh_contact_points(sphere_obj, obb_obj);

    if manifold.point_cache.is_empty() {
        None
    } else {
        Some(manifold)
    }
}
