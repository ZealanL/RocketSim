use glam::Vec3A;

#[derive(Debug, Copy, Clone)]
pub struct CarCameraConfig {
    pub distance: f32,
    pub height: f32,
    pub tilt_ball_min_height: f32,
    pub tilt_ball_max_height: f32,
    pub tilt_min_height_scale: f32,
    pub tilt_exponent: f32,
    pub tilt_dist_portion: f32,
}

impl Default for CarCameraConfig {
    fn default() -> Self {
        Self {
            distance: 300.0,
            height: 130.0,
            tilt_ball_min_height: 100.0,
            tilt_ball_max_height: 500.0,
            tilt_min_height_scale: 0.2,
            tilt_exponent: 0.7,
            tilt_dist_portion: 0.5,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct CameraConfig {
    pub fov_degrees: f32,
    pub birds_eye_pos: Vec3A,
    pub car_cam: CarCameraConfig,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            fov_degrees: 65.0,
            birds_eye_pos: Vec3A::new(-3000.0, 0.0, 1500.0),
            car_cam: CarCameraConfig::default(),
        }
    }
}
