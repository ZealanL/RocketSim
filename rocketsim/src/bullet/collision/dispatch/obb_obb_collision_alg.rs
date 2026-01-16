use crate::bullet::collision::{
    broadphase::CollisionAlgorithm,
    dispatch::box_box_detector::BoxBoxDetector,
    narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
    shapes::compound_shape::CompoundShape,
};
use crate::bullet::dynamics::rigid_body::RigidBody;

pub struct ObbObbCollisionAlgorithm<'a, T: ContactAddedCallback> {
    compound_a_obj: &'a RigidBody,
    compound_a_shape: &'a CompoundShape,
    compound_b_obj: &'a RigidBody,
    compound_b_shape: &'a CompoundShape,
    contact_added_callback: &'a mut T,
}

impl<'a, T: ContactAddedCallback> ObbObbCollisionAlgorithm<'a, T> {
    pub const fn new(
        compound_a_obj: &'a RigidBody,
        compound_a_shape: &'a CompoundShape,
        compound_b_obj: &'a RigidBody,
        compound_b_shape: &'a CompoundShape,
        contact_added_callback: &'a mut T,
    ) -> Self {
        Self {
            compound_a_obj,
            compound_a_shape,
            compound_b_obj,
            compound_b_shape,
            contact_added_callback,
        }
    }
}

impl<T: ContactAddedCallback> CollisionAlgorithm for ObbObbCollisionAlgorithm<'_, T> {
    fn process_collision<'a>(self) -> Option<PersistentManifold> {
        let org_0_trans = self.compound_a_obj.get_world_trans();
        let aabb_0 = self.compound_a_shape.get_aabb(org_0_trans);
        let org_1_trans = self.compound_b_obj.get_world_trans();
        let aabb_1 = self.compound_b_shape.get_aabb(org_1_trans);
        if !aabb_0.intersects(&aabb_1) {
            return None;
        }

        let child_a_trans = &self.compound_a_shape.child_trans;
        let child_a_world_trans = org_0_trans * child_a_trans;

        let child_b_trans = &self.compound_a_shape.child_trans;
        let child_b_world_trans = org_1_trans * child_b_trans;

        let mut detector = BoxBoxDetector {
            box1: &self.compound_a_shape.child_shape,
            col1: self.compound_a_obj,
            box2: &self.compound_a_shape.child_shape,
            col2: self.compound_b_obj,
            contact_added_callback: self.contact_added_callback,
        };

        detector.get_closest_points(child_a_world_trans, child_b_world_trans)
    }
}
