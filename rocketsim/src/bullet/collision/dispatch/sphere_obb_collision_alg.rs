use glam::Vec3A;

use crate::bullet::dynamics::rigid_body::RigidBody;
use crate::bullet::{
    collision::{
        broadphase::CollisionAlgorithm,
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::{compound_shape::CompoundShape, sphere_shape::SphereShape},
    },
    linear_math::AffineExt,
};

pub struct SphereObbCollisionAlgorithm<'a, T: ContactAddedCallback> {
    sphere_obj: &'a RigidBody,
    sphere_shape: &'a SphereShape,
    obb_obj: &'a RigidBody,
    obb_shape: &'a CompoundShape,
    is_swapped: bool,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> SphereObbCollisionAlgorithm<'a, T> {
    pub const fn new(
        sphere_obj: &'a RigidBody,
        sphere_shape: &'a SphereShape,
        obb_obj: &'a RigidBody,
        obb_shape: &'a CompoundShape,
        is_swapped: bool,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            sphere_obj,
            sphere_shape,
            obb_obj,
            obb_shape,
            is_swapped,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for SphereObbCollisionAlgorithm<'_, T> {
    fn process_collision<'a>(self) -> Option<PersistentManifold> {
        let sphere_trans = self.sphere_obj.get_world_trans();
        let aabb_1 = self.sphere_shape.get_aabb(sphere_trans);

        let org_trans = self.obb_obj.get_world_trans();
        let aabb_2 = self.obb_shape.get_aabb(org_trans);

        if !aabb_1.intersects(&aabb_2) {
            return None;
        }

        let child_trans = &self.obb_shape.child_trans;
        let new_child_world_trans = org_trans * child_trans;

        let box_shape = &self.obb_shape.child_shape;
        let box_extents = box_shape.get_half_extents_no_margin();

        let sphere_from_local = new_child_world_trans.inv_x_form(sphere_trans.translation);

        let closest = sphere_from_local.clamp(-box_extents, box_extents);
        let delta = sphere_from_local - closest;
        let dist_sq = delta.length_squared();

        let radius = self.sphere_shape.get_radius();
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

        let mut manifold = PersistentManifold::new(self.sphere_obj, self.obb_obj, self.is_swapped);
        manifold.add_contact_point(
            self.sphere_obj,
            self.obb_obj,
            normal_on_box,
            point_on_box_plus_margin,
            depth,
            -1,
            -1,
            self.contact_added_callback,
        );
        manifold.refresh_contact_points(self.sphere_obj, self.obb_obj);

        if manifold.point_cache.is_empty() {
            None
        } else {
            Some(manifold)
        }
    }
}
