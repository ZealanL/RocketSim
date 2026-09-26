use glam::Vec3A;
use rocketsim::{ArenaState, GameMode};

use crate::camera::{CameraConfig, CarCam};

enum CameraViewMode {
    BirdsEye,
    CarCam(CarCam),
}

/// Persistent camera state: birds-eye or following one car.
///
/// Driven every tick by [`VisInst`](crate::VisInst): press `C` to
/// [`cycle_target`](Self::cycle_target) through birds-eye then each car,
/// `Space` to [`try_toggle_ball_cam`](Self::try_toggle_ball_cam).
pub struct CameraMan {
    _game_mode: GameMode,
    config: CameraConfig,
    cur_pos: Vec3A,
    cur_dir: Vec3A,

    view_mode: CameraViewMode,
}

impl CameraMan {
    /// Starts in birds-eye view, looking from `config.birds_eye_pos`.
    pub fn new(game_mode: GameMode, config: CameraConfig) -> Self {
        Self {
            _game_mode: game_mode,
            config,
            cur_pos: config.birds_eye_pos,
            cur_dir: -config.birds_eye_pos.normalize_or_zero(),
            view_mode: CameraViewMode::BirdsEye,
        }
    }

    /// Current camera position in uu.
    pub fn cur_pos(&self) -> Vec3A {
        self.cur_pos
    }

    /// Current normalized view direction.
    pub fn cur_dir(&self) -> Vec3A {
        self.cur_dir
    }

    /// Active tuning values.
    pub fn config(&self) -> &CameraConfig {
        &self.config
    }

    /// Advances birds-eye -> car 0 -> car 1 ... -> birds-eye.
    /// No-op when the arena has no cars.
    pub fn cycle_target(&mut self, arena_state: &ArenaState) {
        if arena_state.num_cars() == 0 {
            return; // Nothing to cycle
        }

        match &mut self.view_mode {
            CameraViewMode::BirdsEye => {
                // Spectate first car
                self.view_mode = CameraViewMode::CarCam(CarCam::new(0))
            }
            CameraViewMode::CarCam(car_cam) => {
                if car_cam.car_idx < arena_state.num_cars() - 1 {
                    car_cam.car_idx += 1;
                } else {
                    self.view_mode = CameraViewMode::BirdsEye;
                }
            }
        }
    }

    /// Toggles ball-cam while following a car. No-op in birds-eye view.
    pub fn try_toggle_ball_cam(&mut self) {
        if let CameraViewMode::CarCam(car_cam) = &mut self.view_mode {
            car_cam.face_ball = !car_cam.face_ball;
        }
    }

    /// Recomputes position/direction from the latest arena state.
    /// `dt` is currently unused (camera snaps, no smoothing).
    pub fn update(&mut self, arena_state: &ArenaState, _dt: f32) {
        match &self.view_mode {
            CameraViewMode::BirdsEye => {
                self.cur_pos = self.config.birds_eye_pos;
                self.cur_dir =
                    (arena_state.ball.pos - self.config.birds_eye_pos).normalize_or_zero();
            }
            CameraViewMode::CarCam(car_cam) => {
                car_cam.update_car_cam_pos_dir(
                    arena_state,
                    &self.config,
                    &mut self.cur_pos,
                    &mut self.cur_dir,
                );
            }
        }
    }

    /// The followed car, or `None` in birds-eye view.
    pub fn car_cam(&mut self) -> Option<&CarCam> {
        if let CameraViewMode::CarCam(car_cam) = &self.view_mode {
            Some(car_cam)
        } else {
            None
        }
    }
}
