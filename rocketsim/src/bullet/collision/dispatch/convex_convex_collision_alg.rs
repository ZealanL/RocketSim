use super::collision_obj_wrapper::RigidBodyWrapper;
use crate::bullet::{
    collision::narrowphase::{
        gjk::{ClosestPointInput, GjkPairDetector, GjkResult},
        persistent_manifold::{ContactAddedCallback, PersistentManifold},
    },
    dynamics::rigid_body::RigidBody,
};

struct ManifoldResult<'a, T: ContactAddedCallback> {
    manifold: &'a mut PersistentManifold,
    body0: &'a RigidBody,
    body1: &'a RigidBody,
    contact_added_callback: &'a mut T,
}

impl<T: ContactAddedCallback> GjkResult for ManifoldResult<'_, T> {
    fn add_contact_point(
        &mut self,
        normal_on_b: glam::Vec3A,
        point_on_b_world: glam::Vec3A,
        depth: f32,
    ) {
        self.manifold.add_contact_point(
            self.body0,
            self.body1,
            normal_on_b,
            point_on_b_world,
            depth,
            None,
            self.contact_added_callback,
        );
    }
}

pub fn process_collision<T: ContactAddedCallback>(
    convex_obj_a: &RigidBodyWrapper,
    convex_obj_b: &RigidBody,
    contact_added_callback: &mut T,
    out: &mut Option<PersistentManifold>,
) {
    debug_assert!(out.is_none());
    let mut manifold = PersistentManifold::new(convex_obj_a.obj, convex_obj_b);

    let margin_a = convex_obj_a.obj.get_collision_shape().get_margin();
    let margin_b = convex_obj_b.get_collision_shape().get_margin();

    let input = ClosestPointInput::new(
        &convex_obj_a.world_trans,
        convex_obj_b.get_world_trans(),
        margin_a + margin_b + manifold.contact_breaking_threshold,
    );

    let detector = GjkPairDetector::new(margin_a, margin_b);
    let mut result = ManifoldResult {
        manifold: &mut manifold,
        body0: convex_obj_a.obj,
        body1: convex_obj_b,
        contact_added_callback,
    };

    detector.get_closest_points(
        &input,
        convex_obj_a.obj.get_collision_shape(),
        convex_obj_b.get_collision_shape(),
        &mut result,
    );

    if manifold.point_cache.is_empty() {
        return;
    }
    manifold.refresh_contact_points(convex_obj_a.obj, convex_obj_b);
    *out = Some(manifold);
}
