//! Follow view for one car.
//!
//! Ball-cam: rigid orbit on the car→ball bearing, look exactly at the ball.
//! Car-cam: follows heading on the ground and horizontal velocity in the air.
//! State lives in
//! [`CarCamSmoothing`] (one slot per car); this struct only says which car
//! and which mode.

use glam::{Vec2, Vec3A};
use rocketsim::ArenaState;

use crate::camera::{CameraConfig, clamp_dt, smootherstep01};

/// Max car speed (uu/s) for the speed-zoom normalization.
const V_MAX: f32 = 2300.0;
/// Supersonic gate (uu/s) for the second FOV term.
const V_SS: f32 = 2200.0;
/// Horizontal focus→ball distance under which ball-cam yaw freezes (uu).
const YAW_EPS_H: f32 = 50.0;
/// Camera never goes below this height (uu).
const CAM_MIN_Z: f32 = 30.0;
/// Speed-zoom band: extra boom length at full speed, zero stiffness.
const ZOOM_DIST: f32 = 115.0;
/// Speed-zoom band: look-pitch shallowing at full speed, zero stiffness.
const ZOOM_ANGLE_DEG: f32 = 2.0;
/// Dynamic FOV: +5 with speed, +5 more at supersonic (always on).
const FOV_SPEED_DEG: f32 = 5.0;
const FOV_SS_DEG: f32 = 5.0;
/// Smoothing rate (/s) for the speed factor, supersonic gate, and FOV.
const K_SPEED: f32 = 5.0;
// Approximate the aerial hold reported by players. Exact RL timing is not published.
const AIR_YAW_DELAY: f32 = 1.0; // seconds
const AIR_YAW_MIN_SPEED: f32 = 100.0; // uu/s, horizontal
const AIR_YAW_RATE: f32 = 2.5; // per second

fn blend_yaw(from: Vec2, to: Vec2, fraction: f32) -> Vec2 {
    let angle = from.perp_dot(to).atan2(from.dot(to));
    Vec2::from_angle(angle * fraction).rotate(from)
}

/// Follow view for a single car.
pub struct CarCam {
    /// Index into [`ArenaState::cars`](rocketsim::ArenaState::cars).
    pub car_idx: usize,
    /// Whether to look at the ball (`true`) or follow heading (`false`).
    pub face_ball: bool,
}

impl CarCam {
    /// Follows `car_idx` with ball-cam on.
    pub fn new(car_idx: usize) -> Self {
        Self {
            car_idx,
            face_ball: true,
        }
    }

    /// Single-frame snap (fresh smoothing state, `dt = 0`).
    ///
    /// # Panics
    ///
    /// Panics if `car_idx` is out of bounds for `arena_state`.
    pub fn update_car_cam_pos_dir(
        &self,
        arena_state: &ArenaState,
        cam_config: &CameraConfig,
        pos: &mut Vec3A,
        dir: &mut Vec3A,
    ) {
        CarCamSmoothing::new().update(
            self.car_idx,
            self.face_ball,
            arena_state,
            cam_config,
            pos,
            dir,
            0.0,
        );
    }
}

/// Per-frame follow state: boom yaw, speed dyes, toggle transition.
/// `dt` is clamped to `[0, 1]` seconds:
/// * focus tracks the car rigidly (`car_pos + (0, 0, height)`);
/// * ball-cam yaw sets rigidly to the car→ball bearing (frozen when the
///   ball is nearly overhead); car-cam yaw relaxes toward heading with
///   stiffness lag on the ground. In the air, it holds yaw for one second,
///   then follows horizontal velocity above 100 uu/s. This approximates
///   Rocket League behavior. Leaving ball camera resets yaw to the car heading
///   and restarts the aerial hold. A vertical nose keeps the last valid heading;
/// * boom grows with speed (`+(1-stiffness)*115` at full speed);
/// * toggles swing boom and look around the live focus over
///   `0.6 * (2 - transition)` seconds (instant at 2.0);
/// * horizontal FOV gains +5 with speed and +5 at supersonic.
///
/// The first update after [`new`](Self::new)/[`reset`](Self::reset) snaps.
#[derive(Debug, Clone)]
pub struct CarCamSmoothing {
    yaw_dir: Vec2,
    yaw_used_last: Vec2,
    car_heading: Vec2,
    air_yaw_time: f32,
    face_ball_last: bool,
    trans_t: f32,
    yaw_from: Vec2,
    d_from: f32,
    d_used_last: f32,
    /// Look snapshot at the last toggle (blend start).
    look_from: Vec3A,
    /// Last output look (next blend start if toggled mid-swing).
    prev_look: Vec3A,
    u_s: f32,
    ss_s: f32,
    fov_h: f32,
    initialized: bool,
}

impl CarCamSmoothing {
    /// Fresh (uninitialized) state; the first update snaps.
    pub fn new() -> Self {
        Self {
            yaw_dir: Vec2::X,
            yaw_used_last: Vec2::X,
            car_heading: Vec2::X,
            air_yaw_time: 0.0,
            face_ball_last: true,
            trans_t: f32::INFINITY,
            yaw_from: Vec2::X,
            d_from: 0.0,
            d_used_last: 0.0,
            look_from: Vec3A::X,
            prev_look: Vec3A::X,
            u_s: 0.0,
            ss_s: 0.0,
            fov_h: 110.0,
            initialized: false,
        }
    }

    /// Forgets all history; the next update snaps to the target.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Current horizontal FOV in degrees (base + speed/supersonic terms).
    pub fn cur_fov_h(&self) -> f32 {
        self.fov_h
    }

    /// Advances the view by `dt` seconds (clamped to `[0, 1]`).
    ///
    /// `pos`/`dir` receive the new camera pose (`dir` normalized).
    ///
    /// # Panics
    ///
    /// Panics if `car_idx` is out of bounds for `arena_state`.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        car_idx: usize,
        face_ball: bool,
        arena_state: &ArenaState,
        cam_config: &CameraConfig,
        pos: &mut Vec3A,
        dir: &mut Vec3A,
        dt: f32,
    ) {
        let dt = clamp_dt(dt);
        let cfg = &cam_config.car_cam;
        let car_state = &arena_state.cars[car_idx].1;
        let ball_state = &arena_state.ball;
        let stiffness = cfg.stiffness.clamp(0.0, 1.0);

        // Speed dyes (both modes).
        let speed = car_state.vel.length();
        let u = (speed / V_MAX).clamp(0.0, 1.0);
        let a5 = 1.0 - (-K_SPEED * dt).exp();
        self.u_s += (u - self.u_s) * a5;
        let ss_target = if speed >= V_SS { 1.0 } else { 0.0 };
        self.ss_s += (ss_target - self.ss_s) * a5;

        let d_live = cfg.distance + (1.0 - stiffness) * ZOOM_DIST * self.u_s;
        let angle_live = cfg.angle_deg + (1.0 - stiffness) * ZOOM_ANGLE_DEG * self.u_s;

        // Toggle edge: swing boom AND look from the current pose.
        let mode_changed = self.initialized && face_ball != self.face_ball_last;
        if mode_changed {
            self.yaw_from = self.yaw_used_last;
            self.d_from = self.d_used_last;
            self.look_from = self.prev_look;
            self.trans_t = 0.0;
        }
        self.face_ball_last = face_ball;
        let t_blend = (0.6 * (2.0 - cfg.transition_speed)).max(0.0);
        let b = if !self.initialized || t_blend <= 0.0 {
            1.0
        } else {
            self.trans_t += dt;
            smootherstep01(self.trans_t / t_blend)
        };

        // Track horizontal momentum in the air, not body rotation during a flip.
        //
        // The length gate runs BEFORE normalizing: near-vertical noses
        // leave tiny horizontal residue that would swing yaw violently.
        let focus = car_state.pos + Vec3A::new(0.0, 0.0, cfg.height);
        let fwd2 = car_state.get_forward_dir().truncate();
        if fwd2.length_squared() > 1e-6 {
            self.car_heading = fwd2.normalize();
        }
        if face_ball {
            self.air_yaw_time = 0.0;
            let y = (ball_state.pos - focus).truncate();
            if y.length_squared() > YAW_EPS_H * YAW_EPS_H {
                self.yaw_dir = y.normalize();
            } else if !self.initialized {
                self.yaw_dir = Vec2::X;
            }
        } else {
            let target = self.car_heading;
            if car_state.is_on_ground() || mode_changed || !self.initialized {
                self.air_yaw_time = 0.0;
                if !self.initialized || mode_changed || stiffness >= 0.999 {
                    self.yaw_dir = target;
                } else {
                    let k = 2.0 + 18.0 * stiffness;
                    let mixed = self.yaw_dir.lerp(target, 1.0 - (-k * dt).exp());
                    self.yaw_dir = mixed.try_normalize().unwrap_or(target);
                }
            } else {
                let previous_time = self.air_yaw_time;
                self.air_yaw_time = (previous_time + dt).min(AIR_YAW_DELAY);
                let tracking_dt = (previous_time + dt - AIR_YAW_DELAY).clamp(0.0, dt);
                let velocity = car_state.vel.truncate();
                if velocity.length_squared() >= AIR_YAW_MIN_SPEED * AIR_YAW_MIN_SPEED {
                    self.yaw_dir = blend_yaw(
                        self.yaw_dir,
                        velocity.normalize(),
                        1.0 - (-AIR_YAW_RATE * tracking_dt).exp(),
                    );
                }
            }
        }
        let yaw_target = self.yaw_dir;

        // Swing the boom from the toggle snapshot toward the live target.
        let yaw_used = blend_yaw(self.yaw_from, yaw_target, b);
        let d_used = self.d_from + (d_live - self.d_from) * b;
        self.d_used_last = if self.initialized { d_used } else { d_live };
        let (yaw_used, d_used) = if self.initialized {
            (yaw_used, d_used)
        } else {
            self.d_used_last = d_live;
            (yaw_target, d_live)
        };

        self.yaw_used_last = yaw_used;

        // Pose: boom behind the (live) focus, floor-clamped; look blends
        // from the toggle snapshot exactly like the boom (an instant
        // look-snap while the boom swings reads as a broken transition).
        let mut cam = focus - yaw_used.extend(0.0).to_vec3a() * d_used;
        cam.z = cam.z.max(CAM_MIN_Z);
        let look_target = if face_ball {
            let to_ball = ball_state.pos - cam;
            if to_ball.length_squared() > 1e-8 {
                to_ball.normalize()
            } else {
                yaw_used.extend(0.0).to_vec3a()
            }
        } else {
            let pitch = angle_live.to_radians();
            (yaw_used.extend(0.0).to_vec3a() * pitch.cos() + Vec3A::Z * pitch.sin())
                .try_normalize()
                .unwrap_or(Vec3A::X)
        };
        let mixed = self.look_from.lerp(look_target, b);
        let look = if mixed.length_squared() > 1e-8 {
            mixed.normalize()
        } else {
            look_target
        };

        *pos = cam;
        *dir = look;
        self.prev_look = look;

        // Dynamic horizontal FOV (always on).
        let fov_target = cam_config.fov_degrees + FOV_SPEED_DEG * self.u_s + FOV_SS_DEG * self.ss_s;
        if self.initialized {
            self.fov_h += (fov_target - self.fov_h) * a5;
        } else {
            self.fov_h = fov_target;
        }

        self.initialized = true;
    }
}

impl Default for CarCamSmoothing {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec3A;
    use rocketsim::{ArenaState, CarBodyConfig, CarInfo, CarState, GameMode, Team};

    use super::*;

    fn test_state(ball_pos: Vec3A) -> ArenaState {
        let mut state = ArenaState::new_empty(GameMode::Soccar);
        let mut car = CarState::DEFAULT;
        car.pos = Vec3A::new(0.0, 0.0, 17.0);
        state.cars.push((
            CarInfo {
                idx: 0,
                team: Team::Blue,
                config: CarBodyConfig::OCTANE,
            },
            car,
        ));
        state.ball.pos = ball_pos;
        state.ball.vel = Vec3A::ZERO;
        state
    }

    #[test]
    fn car_view_ignores_side_ball() {
        // Ball far to the side must not move a car-cam: heading only.
        let state = test_state(Vec3A::new(5000.0, 0.0, 100.0));
        let config = CameraConfig::default();
        let cam = CarCam {
            car_idx: 0,
            face_ball: false,
        };
        let mut pos = Vec3A::ZERO;
        let mut dir = Vec3A::X;
        cam.update_car_cam_pos_dir(&state, &config, &mut pos, &mut dir);

        // Car forward is +X here (identity rot): camera sits at -X.
        assert!(pos.x < 0.0);
        assert!((pos.y).abs() < 1e-3);
        // Focus height, full distance (car is still).
        assert!((pos.z - (17.0 + config.car_cam.height)).abs() < 1e-3);
        let flat_back = Vec3A::new(0.0, 0.0, 17.0)
            .truncate()
            .distance(pos.truncate());
        assert!((flat_back - config.car_cam.distance).abs() < 1e-3);
        assert!(dir.is_finite());
    }

    #[test]
    fn ball_yaw_rigid_and_look_exact() {
        let state = test_state(Vec3A::new(0.0, 2000.0, 500.0));
        let config = CameraConfig::default();
        let cam = CarCam::new(0);
        let mut pos = Vec3A::ZERO;
        let mut dir = Vec3A::X;
        cam.update_car_cam_pos_dir(&state, &config, &mut pos, &mut dir);

        // Boom on the car->ball bearing (+Y): camera sits at -Y.
        assert!(pos.y < 0.0);
        assert!((pos.x).abs() < 1e-3);
        // Level boom (no tilt system): focus height kept.
        assert!((pos.z - (17.0 + config.car_cam.height)).abs() < 1e-3);
        // Look lands exactly on the ball.
        let to_ball = (state.ball.pos - pos).normalize();
        assert!((dir.dot(to_ball) - 1.0).abs() < 1e-5);
    }

    #[test]
    fn overhead_freezes_yaw() {
        let mut state = test_state(Vec3A::new(0.0, 0.0, 1500.0));
        let config = CameraConfig::default();
        let mut smooth = CarCamSmoothing::new();
        let mut pos = Vec3A::ZERO;
        let mut dir = Vec3A::X;
        // First: ball ahead (+Y) sets yaw.
        state.ball.pos = Vec3A::new(0.0, 2000.0, 500.0);
        smooth.update(0, true, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
        assert!(pos.y < 0.0);
        // Then: ball pinned overhead — yaw must not spin.
        state.ball.pos = Vec3A::new(10.0, 5.0, 1500.0);
        smooth.update(0, true, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
        assert!(pos.y < 0.0, "yaw spun with overhead ball: {pos:?}");
        assert!((pos.x).abs() < 1e-3);
        assert!(pos.is_finite() && dir.is_finite());
    }

    #[test]
    fn aerial_body_rotation_without_momentum_keeps_yaw() {
        use std::f32::consts::FRAC_PI_2;

        use glam::{EulerRot, Mat3A};

        let mut state = test_state(Vec3A::new(0.0, 2000.0, 500.0));
        let config = CameraConfig::default();
        let mut smooth = CarCamSmoothing::new();
        let mut pos = Vec3A::ZERO;
        let mut dir = Vec3A::X;
        // Grounded, heading +X: boom settles at -X.
        smooth.update(0, false, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
        assert!(pos.x < -100.0, "grounded boom should sit at -X: {pos:?}");

        // Launch: yaw the body 90° flat, then pitch the nose straight up.
        // The boom must NOT follow either tumble — it holds liftoff yaw.
        state.cars[0].1.set_contact_flags(false, [false; 4]);
        state.cars[0].1.rot_mat = Mat3A::from_euler(EulerRot::ZYX, FRAC_PI_2, 0.0, 0.0);
        for _ in 0..60 {
            smooth.update(0, false, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
        }
        assert!(
            pos.x < -100.0,
            "airborne boom must hold liftoff yaw: {pos:?}"
        );
        state.cars[0].1.rot_mat = Mat3A::from_euler(EulerRot::ZYX, FRAC_PI_2, FRAC_PI_2, 0.0);
        for _ in 0..60 {
            smooth.update(0, false, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
        }
        assert!(
            pos.x < -100.0,
            "vertical nose must not swing the boom: {pos:?}"
        );
        assert!(pos.is_finite() && dir.is_finite());

        // Touch down sideways: yaw resumes tracking (boom swings to -Y).
        state.cars[0].1.set_contact_flags(true, [true; 4]);
        state.cars[0].1.rot_mat = Mat3A::from_euler(EulerRot::ZYX, FRAC_PI_2, 0.0, 0.0);
        for _ in 0..240 {
            smooth.update(0, false, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
        }
        assert!(
            pos.y < -100.0,
            "landed boom should trail the new heading: {pos:?}"
        );
    }

    #[test]
    fn aerial_yaw_holds_then_tracks_momentum() {
        for hz in [30, 120] {
            for velocity in [Vec3A::Y * 1500.0, Vec3A::NEG_X * 1500.0] {
                let mut state = test_state(Vec3A::new(0.0, 2000.0, 500.0));
                let config = CameraConfig::default();
                let mut smooth = CarCamSmoothing::new();
                let mut pos = Vec3A::ZERO;
                let mut dir = Vec3A::X;
                let dt = 1.0 / hz as f32;
                smooth.update(0, false, &state, &config, &mut pos, &mut dir, dt);
                state.cars[0].1.set_contact_flags(false, [false; 4]);
                // Body heading stays +X while momentum changes sideways or reverses.
                state.cars[0].1.vel = velocity;
                for _ in 0..hz / 2 {
                    smooth.update(0, false, &state, &config, &mut pos, &mut dir, dt);
                }
                assert!(dir.truncate().normalize().dot(Vec2::X) > 0.999);
                let mut largest_step = 0.0_f32;
                for _ in 0..hz * 3 {
                    let before = dir;
                    smooth.update(0, false, &state, &config, &mut pos, &mut dir, dt);
                    largest_step = largest_step.max(before.distance(dir));
                }
                assert!(
                    dir.truncate()
                        .normalize()
                        .dot(velocity.truncate().normalize())
                        > 0.999
                );
                assert!(largest_step < 0.3, "aerial reversal must not snap");

                // Vertical flight must not select a new horizontal bearing.
                let before = dir.truncate().normalize();
                state.cars[0].1.vel = Vec3A::new(0.01, -0.01, 1800.0);
                for _ in 0..hz * 2 {
                    smooth.update(0, false, &state, &config, &mut pos, &mut dir, dt);
                }
                assert!(dir.truncate().normalize().dot(before) > 0.999);
                state.cars[0].1.vel = Vec3A::X * 1500.0;
                smooth.update(0, false, &state, &config, &mut pos, &mut dir, dt);
                assert!(
                    dir.truncate().normalize().dot(Vec2::X) > before.dot(Vec2::X) + 0.001,
                    "low horizontal speed must not restart the aerial hold"
                );
            }
        }
    }

    #[test]
    fn aerial_ball_toggle_resets_to_heading() {
        use glam::Mat3A;

        for transition_speed in [1.0, 2.0] {
            let mut state = test_state(Vec3A::new(0.0, 2000.0, 500.0));
            let mut config = CameraConfig::default();
            config.car_cam.transition_speed = transition_speed;
            let mut smooth = CarCamSmoothing::new();
            let mut pos = Vec3A::ZERO;
            let mut dir = Vec3A::X;
            smooth.update(0, false, &state, &config, &mut pos, &mut dir, 0.0);
            state.cars[0].1.set_contact_flags(false, [false; 4]);
            state.cars[0].1.rot_mat = Mat3A::from_rotation_z(std::f32::consts::PI);
            // Momentum and ball bearing both disagree with the new nose heading.
            state.cars[0].1.vel = Vec3A::Y * 1500.0;
            smooth.update(0, true, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
            let before = dir;
            smooth.update(0, false, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
            if transition_speed < 2.0 {
                assert!(
                    before.dot(dir) > 0.999,
                    "interrupted toggle must stay continuous"
                );
            } else {
                assert!(dir.x < -0.9, "instant transition must reset immediately");
            }
            for _ in 0..90 {
                let before = dir;
                smooth.update(0, false, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
                assert!(before.dot(dir) > 0.99, "toggle reversal must not snap");
            }
            assert!(
                dir.x < -0.9,
                "toggle must select nose, not ball or velocity"
            );
            assert!(pos.x > 100.0 && dir.is_finite());

            // A vertical nose has no yaw. Keep the car heading, not the ball bearing.
            smooth.update(0, true, &state, &config, &mut pos, &mut dir, 0.0);
            state.cars[0].1.rot_mat *= Mat3A::from_rotation_y(std::f32::consts::FRAC_PI_2);
            smooth.update(0, false, &state, &config, &mut pos, &mut dir, 0.75);
            assert!(dir.x < -0.9, "vertical nose must keep the last car heading");
        }
    }

    #[test]
    fn toggle_swing_over_time() {
        // Ball at +Y, car heading +X: ball view booms at -Y, car view at -X.
        let state = test_state(Vec3A::new(0.0, 2000.0, 500.0));
        let config = CameraConfig::default();
        let mut smooth = CarCamSmoothing::new();
        let mut pos = Vec3A::ZERO;
        let mut dir = Vec3A::X;
        smooth.update(0, true, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
        assert!(pos.y < -100.0, "ball view should boom at -Y: {pos:?}");

        // Flip to car-cam: first frame still near the ball-side boom...
        smooth.update(0, false, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
        assert!(
            pos.y < 0.0,
            "swing should start from the ball side: {pos:?}"
        );

        // ...and settle on the car side within the swing window (~0.5 s).
        for _ in 0..120 {
            smooth.update(0, false, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
        }
        assert!(pos.x < -100.0, "swing should end at -X: {pos:?}");
        assert!(dir.is_finite() && (dir.length() - 1.0).abs() < 1e-4);
    }

    #[test]
    fn toggle_to_ball_swings_look_without_snapping() {
        // Regression: the look used to snap to the ball instantly while
        // the boom was still swinging (car->ball felt broken; the other
        // way looked fine). Settle in car-cam, flip to ball-cam, and
        // require a continuous look path that converges on the ball.
        let state = test_state(Vec3A::new(0.0, 2000.0, 500.0));
        let config = CameraConfig::default();
        let mut smooth = CarCamSmoothing::new();
        let mut pos = Vec3A::ZERO;
        let mut dir = Vec3A::X;
        for _ in 0..120 {
            smooth.update(0, false, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
        }
        let before = dir;
        // First toggle frame: look barely moves (no snap to the ball).
        smooth.update(0, true, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
        assert!(
            before.dot(dir) > 0.999,
            "look snapped on toggle: {before:?} -> {dir:?}"
        );
        // Whole swing: small steps, unit length, ends exact on the ball.
        for _ in 0..120 {
            let prev = dir;
            smooth.update(0, true, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
            assert!(
                prev.dot(dir) > 0.995,
                "look jumped mid-swing: {prev:?} -> {dir:?}"
            );
            assert!((dir.length() - 1.0).abs() < 1e-4);
        }
        let to_ball = (state.ball.pos - pos).normalize();
        assert!((dir.dot(to_ball) - 1.0).abs() < 1e-4);
    }

    #[test]
    fn speed_grows_fov() {
        let mut state = test_state(Vec3A::new(0.0, 2000.0, 500.0));
        state.cars[0].1.vel = Vec3A::new(2300.0, 0.0, 0.0);
        let config = CameraConfig::default();
        let mut smooth = CarCamSmoothing::new();
        let mut pos = Vec3A::ZERO;
        let mut dir = Vec3A::X;
        for _ in 0..600 {
            smooth.update(0, true, &state, &config, &mut pos, &mut dir, 1.0 / 120.0);
        }
        // u_s -> 1, supersonic on: FOV gains +5 +5.
        assert!(smooth.cur_fov_h() > config.fov_degrees + 9.0);
        assert!(pos.is_finite());
    }

    #[test]
    fn huge_dt_stays_finite() {
        let state = test_state(Vec3A::new(100.0, -200.0, 500.0));
        let config = CameraConfig::default();
        let mut smooth = CarCamSmoothing::new();
        let mut pos = Vec3A::ZERO;
        let mut dir = Vec3A::X;
        for _ in 0..10 {
            smooth.update(0, true, &state, &config, &mut pos, &mut dir, 10.0);
        }
        assert!(pos.is_finite());
        assert!(dir.is_finite());
        assert!((dir.length() - 1.0).abs() < 1e-4);
    }
}
