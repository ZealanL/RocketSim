use glam::Affine3A;

pub struct ClosestPointInput<'a> {
    pub transform_a: &'a Affine3A,
    pub transform_b: &'a Affine3A,
    pub maximum_distance_squared: f32,
}

impl<'a> ClosestPointInput<'a> {
    pub fn new(
        transform_a: &'a Affine3A,
        transform_b: &'a Affine3A,
        maximum_distance: f32,
    ) -> Self {
        Self {
            transform_a,
            transform_b,
            maximum_distance_squared: maximum_distance * maximum_distance,
        }
    }
}
