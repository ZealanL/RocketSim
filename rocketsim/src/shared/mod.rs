mod aabb;
mod angle;
pub(crate) mod bvh;
pub mod geometry;
mod linear_piece_curve;
pub(crate) mod math;
mod quad_ray;
mod round;
#[cfg(feature = "native-math")]
pub(crate) mod ucrt_math;

/// Geometry helpers. Use [`geometry`] for new code.
pub mod rsmath {
    pub use super::geometry::*;
}

pub use aabb::*;
pub use angle::Angle;
pub use linear_piece_curve::LinearPieceCurve;
pub use quad_ray::*;
pub(crate) use round::{round_ties_even, round_ties_even_vec3a};
