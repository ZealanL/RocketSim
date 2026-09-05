use super::box_box_detector::BoxBoxDetector;
use crate::bullet::{
    collision::{
        narrowphase::persistent_manifold::{ContactAddedCallback, PersistentManifold},
        shapes::compound_shape::CompoundShape,
    },
    dynamics::rigid_body::RigidBody,
};

pub fn process_collision<T: ContactAddedCallback>(
    compound_a_obj: &RigidBody,
    compound_a_shape: &CompoundShape,
    compound_b_obj: &RigidBody,
    compound_b_shape: &CompoundShape,
    contact_added_callback: &mut T,
    out: &mut Option<PersistentManifold>,
) {
    debug_assert!(out.is_none());
    let org_0_trans = compound_a_obj.get_world_trans();
    let aabb_0 = compound_a_shape.get_aabb(org_0_trans);
    let org_1_trans = compound_b_obj.get_world_trans();
    let aabb_1 = compound_b_shape.get_aabb(org_1_trans);
    if !aabb_0.intersects(&aabb_1) {
        return;
    }

    let child_a_trans = &compound_a_shape.child_trans;
    let child_a_world_trans = org_0_trans * child_a_trans;

    let child_b_trans = &compound_b_shape.child_trans;
    let child_b_world_trans = org_1_trans * child_b_trans;

    let mut detector = BoxBoxDetector {
        box1: &compound_a_shape.child_shape,
        col1: compound_a_obj,
        box2: &compound_b_shape.child_shape,
        col2: compound_b_obj,
        contact_added_callback,
    };

    detector.get_closest_points(child_a_world_trans, child_b_world_trans, out);
}
