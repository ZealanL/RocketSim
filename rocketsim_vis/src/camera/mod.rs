//! Camera logic: birds-eye spectator view and follow-car views.
//!
//! [`CameraMan`] holds the persistent state used by [`crate::VisInst`].
//! Tune it with [`CameraConfig`] / [`CarCameraConfig`].

mod camera_config;
mod camera_man;
mod car_cam;

pub use camera_config::*;
pub use camera_man::*;
pub use car_cam::*;
