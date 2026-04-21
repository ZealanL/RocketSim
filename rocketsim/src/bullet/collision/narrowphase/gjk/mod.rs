mod closest_point_input;
mod gjk_pair_detector;
mod penetration;
mod solver;

pub use closest_point_input::ClosestPointInput;
pub use gjk_pair_detector::GjkPairDetector;

pub trait GjkResult {
    fn add_contact_point(
        &mut self,
        normal_on_b: glam::Vec3A,
        point_on_b_world: glam::Vec3A,
        depth: f32,
    );
}
