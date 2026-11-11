#[cfg(not(feature = "box-triangle-gjk"))]
pub(super) mod box_triangle_sat;
pub(super) mod gjk;
pub mod manifold_point;
pub mod persistent_manifold;

mod car_toi;
pub(crate) use car_toi::{CarToiBoxShape, box_sweep_closest_points};
