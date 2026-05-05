use glam::Vec3A;

use crate::{
    ArenaState, GameMode,
    vis::camera::{CameraConfig, CarCam},
};

enum CameraViewMode {
    BirdsEye,
    CarCam(CarCam),
}

/// Camera manager, manages what the camera is doing, persistently
pub struct CameraMan {
    _game_mode: GameMode,
    config: CameraConfig,
    cur_pos: Vec3A,
    cur_dir: Vec3A,

    view_mode: CameraViewMode,
}

impl CameraMan {
    pub fn new(game_mode: GameMode, config: CameraConfig) -> Self {
        Self {
            _game_mode: game_mode,
            config,
            cur_pos: config.birds_eye_pos,
            cur_dir: -config.birds_eye_pos.normalize_or_zero(),
            view_mode: CameraViewMode::BirdsEye,
        }
    }

    pub fn cur_pos(&self) -> Vec3A {
        self.cur_pos
    }

    pub fn cur_dir(&self) -> Vec3A {
        self.cur_dir
    }

    pub fn config(&self) -> &CameraConfig {
        &self.config
    }

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

    pub fn try_toggle_ball_cam(&mut self) {
        if let CameraViewMode::CarCam(car_cam) = &mut self.view_mode {
            car_cam.face_ball = !car_cam.face_ball;
        }
    }

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
}
