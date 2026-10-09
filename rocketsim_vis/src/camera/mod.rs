//! Camera logic: birds-eye, follow-car, director, auto-cam (stage 8a).
//!
//! RL-authentic follow semantics: rigid ball-cam yaw (frozen when the ball
//! is overhead), liftoff-frozen car-cam yaw with stiffness lag, speed zoom,
//! toggle swing, dynamic FOV. All time advances through caller `dt`,
//! clamped to `[0, 1/30]` — no wall-clock, no RNG, fully testable.

mod arena_bounds;
mod auto_cam;
mod camera_config;
mod camera_man;
mod car_cam;
mod director;
mod freecam;

pub use arena_bounds::{ArenaBounds, ArenaBoundsCache, FOLLOW_BACKOFF, FREECAM_BACKOFF, SegHit};
pub use auto_cam::*;
pub use camera_config::*;
pub use camera_man::*;
pub use car_cam::*;
pub use director::*;
pub use freecam::*;

/// Largest timestep the cameras integrate (s); larger `dt` is clamped.
pub(crate) const MAX_DT: f32 = 1.0 / 30.0;

/// Clamps a frame timestep to `[0, MAX_DT]`; non-finite input yields `0`.
pub(crate) fn clamp_dt(dt: f32) -> f32 {
    if dt.is_finite() {
        dt.clamp(0.0, MAX_DT)
    } else {
        0.0
    }
}

/// Smootherstep (`6t^5 - 15t^4 + 10t^3`) of `t` clamped to `[0, 1]`.
pub(crate) fn smootherstep01(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dt_clamp() {
        assert_eq!(clamp_dt(1.0 / 60.0), 1.0 / 60.0);
        assert_eq!(clamp_dt(10.0), MAX_DT);
        assert_eq!(clamp_dt(-1.0), 0.0);
        assert_eq!(clamp_dt(f32::NAN), 0.0);
        assert_eq!(clamp_dt(f32::INFINITY), 0.0);
    }

    #[test]
    fn smootherstep_endpoints() {
        assert_eq!(smootherstep01(0.0), 0.0);
        assert_eq!(smootherstep01(1.0), 1.0);
        let mid = smootherstep01(0.5);
        assert!((mid - 0.5).abs() < 1e-6);
    }
}
