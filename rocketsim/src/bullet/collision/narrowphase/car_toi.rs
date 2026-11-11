//! Closest points for the car-to-car time-of-impact sweep.
//!
//! The caller supplies two boxes in Bullet units. The query returns the signed
//! distance and one unit normal that points from the second box toward the
//! first box. The distance is negative during penetration.
//!
//! Query selection is a compile-time choice:
//!
//! - The `box-triangle-gjk` feature uses the shared GJK/EPA pair detector.
//! - Without the feature, use the analytic SAT query in `sat_query`.
//!
//! Both queries use the same box core, the same `BoxShape` margins, and the
//! same output sign rules.

#[cfg(feature = "box-triangle-gjk")]
mod gjk_query;
#[cfg(not(feature = "box-triangle-gjk"))]
mod sat_query;

#[cfg(feature = "box-triangle-gjk")]
pub(crate) use gjk_query::closest_points as box_sweep_closest_points;
use glam::{Affine3A, Vec3A};
#[cfg(not(feature = "box-triangle-gjk"))]
pub(crate) use sat_query::closest_points as box_sweep_closest_points;

use crate::bullet::collision::shapes::{
    box_shape::BoxShape, collision_shape::CollisionShapes, compound_shape::CompoundShape,
};

/// Largest squared distance that the query reports.
const MAXIMUM_DISTANCE_SQUARED: f32 = 1.0e18;

/// One box shape for the car sweep.
///
/// The box sits at the local origin. Use the world transform to place it.
pub(crate) struct CarToiBoxShape {
    shape: CollisionShapes,
}

impl CarToiBoxShape {
    /// Build a box from half extents that already include the shape margin.
    ///
    /// `half_extents_with_margin` is in Bullet units.
    pub(crate) fn new(half_extents_with_margin: Vec3A) -> Self {
        let child = BoxShape::new(half_extents_with_margin);
        Self {
            shape: CollisionShapes::Compound(CompoundShape::new(child, Affine3A::IDENTITY)),
        }
    }

    /// Return the shape margin in Bullet units.
    pub(crate) const fn margin(&self) -> f32 {
        self.shape.get_margin()
    }

    /// Return the radius of the bound used for the angular motion term.
    #[must_use]
    pub(crate) fn angular_motion_disc(&self) -> f32 {
        let child = &self.compound().child_shape;
        let margin = child.get_margin();
        let implicit = child.get_half_extents();

        let ex = implicit.x + margin;
        let ey = implicit.y + margin;
        let ez = implicit.z + margin;

        let dx = ex + ex;
        let dy = ey + ey;
        let dz = ez + ez;

        ((dx * dx + dy * dy) + dz * dz).sqrt() * 0.5
    }

    /// Return the compound shape that holds the box core.
    fn compound(&self) -> &CompoundShape {
        let CollisionShapes::Compound(compound) = &self.shape else {
            unreachable!()
        };

        compound
    }
}

/// One closest-point result in Bullet units.
pub(crate) struct CarToiContact {
    /// Signed distance. Negative values are penetrating.
    pub(crate) distance: f32,
    /// Unit normal that points from the second box toward the first.
    pub(crate) normal: Vec3A,
}
