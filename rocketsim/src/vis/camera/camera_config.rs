use glam::Vec3A;

#[derive(Debug, Copy, Clone)]
pub struct CameraConfig {
    pub fov_degrees: f32,

    pub birds_eye_pos: Vec3A,

    // TODO: Move these to a separate struct
    pub car_cam_distance: f32,
    pub car_cam_height: f32,
    pub car_cam_tilt_ball_height: f32,
    pub car_cam_tilt_min_height_scale: f32,
    pub car_cam_tilt_exponent: f32,
    pub car_cam_tilt_dist_portion: f32,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            fov_degrees: 65.0,

            birds_eye_pos: Vec3A::new(-3000.0, 0.0, 1500.0),

            car_cam_distance: 300.0,
            car_cam_height: 130.0,
            car_cam_tilt_ball_height: 500.0,
            car_cam_tilt_min_height_scale: 0.2,
            car_cam_tilt_exponent: 1.7,
            car_cam_tilt_dist_portion: 0.5,
        }
    }
}