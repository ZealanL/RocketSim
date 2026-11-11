//! GJK/EPA query for the car-to-car time-of-impact sweep.
//!
//! Enable the `box-triangle-gjk` feature to use this path. It keeps the shared
//! Bullet GJK pair detector behavior and its output values.

use glam::{Affine3A, Vec3A};

use super::{CarToiBoxShape, CarToiContact, MAXIMUM_DISTANCE_SQUARED};
use crate::bullet::collision::narrowphase::gjk::{ClosestPointInput, GjkPairDetector, GjkResult};

#[derive(Default)]
struct ContactSink {
    contact: Option<CarToiContact>,
}

impl GjkResult for ContactSink {
    fn add_contact_point(&mut self, normal_on_b: Vec3A, _point_on_b_world: Vec3A, depth: f32) {
        self.contact = Some(CarToiContact {
            distance: depth,
            normal: normal_on_b,
        });
    }
}

/// Return the closest points of two boxes through the GJK pair detector.
///
/// Return `None` when the query finds no result within the distance bound.
#[must_use]
pub(crate) fn closest_points(
    shape_a: &CarToiBoxShape,
    trans_a: &Affine3A,
    shape_b: &CarToiBoxShape,
    trans_b: &Affine3A,
) -> Option<CarToiContact> {
    let input = ClosestPointInput {
        transform_a: trans_a,
        transform_b: trans_b,
        maximum_distance_squared: MAXIMUM_DISTANCE_SQUARED,
    };

    let mut sink = ContactSink::default();
    GjkPairDetector::new(shape_a.margin(), shape_b.margin()).get_closest_points(
        &input,
        &shape_a.shape,
        &shape_b.shape,
        &mut sink,
    );

    let mut contact = sink.contact?;

    // The pair query centres both transforms on their midpoint. Repeat the
    // centring so the box AABB centres match that query.
    let position_offset = (trans_a.translation + trans_b.translation) * 0.5;

    let mut local_a = *trans_a;
    let mut local_b = *trans_b;
    local_a.translation -= position_offset;
    local_b.translation -= position_offset;

    let center_a = shape_a.shape.get_aabb(&local_a).center();
    let center_b = shape_b.shape.get_aabb(&local_b).center();

    if dot(center_a - center_b, contact.normal) < 0.0 {
        contact.normal = -contact.normal;
    }

    Some(contact)
}

/// Dot product in the order `(x + y) + z`.
#[inline]
fn dot(a: Vec3A, b: Vec3A) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}
