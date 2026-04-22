mod closest_point_input;
mod gjk_pair_detector;
mod penetration;
mod solver;
mod subsimplex_convex_cast;

pub use closest_point_input::ClosestPointInput;
pub use gjk_pair_detector::GjkPairDetector;
pub use subsimplex_convex_cast::calc_time_of_impact;

pub trait GjkResult {
    fn add_contact_point(
        &mut self,
        normal_on_b: glam::Vec3A,
        point_on_b_world: glam::Vec3A,
        depth: f32,
    );
}
