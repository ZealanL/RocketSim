use glam::Vec3A;

pub trait GjkResult {
    fn add_contact_point(&mut self, normal_on_b: Vec3A, point_on_b_world: Vec3A, depth: f32);
}
