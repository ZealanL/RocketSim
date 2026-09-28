use glam::Vec3A;

/// Tuning for the follow-car camera.
///
/// `distance` is how far behind the car the camera sits (uu),
/// `height` is the base height above the car (uu).
/// The camera tilts up as the ball gets higher between
/// `tilt_ball_min_height` and `tilt_ball_max_height`, scaled by
/// `tilt_exponent` and floored by `tilt_min_height_scale`.
/// Higher tilt pulls the camera closer via `tilt_dist_portion` (0-1).
#[derive(Debug, Copy, Clone)]
pub struct CarCameraConfig {
    /// Follow distance behind the car in uu. Default: 300.
    pub distance: f32,
    /// Base height above the car in uu. Default: 130.
    pub height: f32,
    /// Ball height above the car where upward tilt starts. Default: 100.
    pub tilt_ball_min_height: f32,
    /// Ball height where upward tilt saturates. Default: 500.
    pub tilt_ball_max_height: f32,
    /// Minimum tilt amount (keeps some tilt even for low balls). Default: 0.2.
    pub tilt_min_height_scale: f32,
    /// Curve applied to the tilt fraction. Default: 0.7.
    pub tilt_exponent: f32,
    /// How much full tilt shortens the follow distance (0-1). Default: 0.5.
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

/// Top-level camera settings.
///
/// `birds_eye_pos` is the fixed spectator position (uu); the camera looks
/// from there at the ball each frame.
#[derive(Debug, Copy, Clone)]
pub struct CameraConfig {
    /// Vertical field of view in degrees. Default: 65.
    pub fov_degrees: f32,
    /// Fixed birds-eye camera position in uu. Default: (-3000, 0, 1500).
    pub birds_eye_pos: Vec3A,
    /// Follow-car tuning.
    pub car_cam: CarCameraConfig,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            fov_degrees: 95.0,
            birds_eye_pos: Vec3A::new(-3000.0, 0.0, 1500.0),
            car_cam: CarCameraConfig::default(),
        }
    }
}
