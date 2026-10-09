//! Birds-eye / follow-car camera manager (stage 8a).
//!
//! Persistent state across frames: `C` cycles birds-eye → each car,
//! `Space` toggles ball-cam on the follow view. Smoothing resets when the
//! followed car changes but survives toggles (the look blends). Demoed cars
//! freeze the camera at the death pose (look re-aims at the ball); respawn
//! snaps back to live.

use glam::Vec3A;
use rocketsim::{ArenaState, GameMode};

use crate::camera::{CameraConfig, CarCam, CarCamSmoothing, clamp_dt};

enum CameraViewMode {
    BirdsEye,
    CarCam(CarCam),
}

/// Persistent camera state: birds-eye or following one car.
pub struct CameraMan {
    _game_mode: GameMode,
    config: CameraConfig,
    cur_pos: Vec3A,
    cur_dir: Vec3A,
    smooth: CarCamSmoothing,
    /// Frozen death pose while the followed car is demoed (position holds,
    /// look re-aims at the ball). Cleared on respawn (view snaps to live).
    demo_frozen_pos: Option<Vec3A>,

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
            smooth: CarCamSmoothing::new(),
            demo_frozen_pos: None,
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

    /// Current horizontal FOV in degrees (dynamic terms in follow views,
    /// plain base FOV in birds-eye).
    pub fn cur_fov_degrees(&self) -> f32 {
        match &self.view_mode {
            CameraViewMode::BirdsEye => self.config.fov_degrees,
            CameraViewMode::CarCam(_) => self.smooth.cur_fov_h(),
        }
    }

    /// Active tuning values.
    pub fn config(&self) -> &CameraConfig {
        &self.config
    }

    /// Replaces the tuning live. Smoothing stays continuous.
    pub fn set_config(&mut self, config: CameraConfig) {
        self.config = config;
    }

    /// Advances birds-eye → car 0 → car 1 … → birds-eye.
    /// No-op when the arena has no cars.
    pub fn cycle_target(&mut self, arena_state: &ArenaState) {
        if arena_state.num_cars() == 0 {
            return;
        }
        match &mut self.view_mode {
            CameraViewMode::BirdsEye => {
                self.view_mode = CameraViewMode::CarCam(CarCam::new(0));
            }
            CameraViewMode::CarCam(car_cam) => {
                if car_cam.car_idx < arena_state.num_cars() - 1 {
                    car_cam.car_idx += 1;
                } else {
                    self.view_mode = CameraViewMode::BirdsEye;
                }
            }
        }
        self.smooth.reset();
        self.demo_frozen_pos = None;
    }

    /// Toggles ball-cam while following a car. No-op in birds-eye view.
    pub fn try_toggle_ball_cam(&mut self) {
        if let CameraViewMode::CarCam(car_cam) = &mut self.view_mode {
            car_cam.face_ball = !car_cam.face_ball;
        }
    }

    /// Recomputes position/direction from the latest arena state.
    ///
    /// `dt` (seconds, clamped to `[0, 1/30]`) advances follow-view
    /// smoothing; birds-eye still snaps.
    pub fn update(&mut self, arena_state: &ArenaState, dt: f32) {
        let follow = match &self.view_mode {
            CameraViewMode::BirdsEye => None,
            CameraViewMode::CarCam(car_cam) => Some((car_cam.car_idx, car_cam.face_ball)),
        };
        match follow {
            None => {
                self.cur_pos = self.config.birds_eye_pos;
                self.cur_dir =
                    (arena_state.ball.pos - self.config.birds_eye_pos).normalize_or_zero();
            }
            Some((car_idx, face_ball)) => {
                let demoed = arena_state
                    .cars
                    .get(car_idx)
                    .is_some_and(|(_, s)| s.is_demoed);
                if demoed {
                    if self.demo_frozen_pos.is_none() {
                        self.demo_frozen_pos = Some(self.cur_pos);
                    }
                    self.cur_pos = self.demo_frozen_pos.unwrap_or(self.cur_pos);
                    self.cur_dir = (arena_state.ball.pos - self.cur_pos).normalize_or_zero();
                } else {
                    if self.demo_frozen_pos.is_some() {
                        self.demo_frozen_pos = None;
                        self.smooth.reset();
                    }
                    self.smooth.update(
                        car_idx,
                        face_ball,
                        arena_state,
                        &self.config,
                        &mut self.cur_pos,
                        &mut self.cur_dir,
                        clamp_dt(dt),
                    );
                }
            }
        }
    }

    /// Sets ball-cam for the followed car (auto-cam drive). No-op unless
    /// following `car_idx`.
    pub fn set_follow_face_ball(&mut self, car_idx: usize, face_ball: bool) {
        if let CameraViewMode::CarCam(car_cam) = &mut self.view_mode
            && car_cam.car_idx == car_idx
        {
            car_cam.face_ball = face_ball;
        }
    }

    /// The followed car, or `None` in birds-eye view.
    pub fn car_cam(&self) -> Option<&CarCam> {
        if let CameraViewMode::CarCam(car_cam) = &self.view_mode {
            Some(car_cam)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use rocketsim::{ArenaState, CarBodyConfig, CarInfo, CarState, GameMode, Team};

    use super::*;

    fn test_state() -> ArenaState {
        let mut state = ArenaState::new_empty(GameMode::Soccar);
        for (i, x) in [0.0, 1000.0].into_iter().enumerate() {
            let mut car = CarState::DEFAULT;
            car.pos = Vec3A::new(x, -2000.0, 17.0);
            state.cars.push((
                CarInfo {
                    idx: i,
                    team: if i == 0 { Team::Blue } else { Team::Orange },
                    config: CarBodyConfig::OCTANE,
                },
                car,
            ));
        }
        state.ball.pos = Vec3A::new(0.0, 0.0, 500.0);
        state
    }

    #[test]
    fn cycle_and_toggle() {
        let state = test_state();
        let mut man = CameraMan::new(GameMode::Soccar, CameraConfig::default());
        assert!(man.car_cam().is_none());

        man.cycle_target(&state);
        assert_eq!(man.car_cam().unwrap().car_idx, 0);
        man.update(&state, 1.0 / 60.0);
        assert!(man.cur_pos().is_finite());
        assert!((man.cur_dir().length() - 1.0).abs() < 1e-4);

        man.try_toggle_ball_cam();
        assert!(!man.car_cam().unwrap().face_ball);
        // Toggle swings the boom: yaw carries over (horizontal bearing
        // barely moves in one tick) while the look pitch snaps to car-cam.
        let before = man.cur_dir();
        man.update(&state, 1.0 / 120.0);
        let flat_before = before.truncate().normalize();
        let flat_after = man.cur_dir().truncate().normalize();
        assert!(flat_before.dot(flat_after) > 0.999);

        man.cycle_target(&state);
        assert_eq!(man.car_cam().unwrap().car_idx, 1);
        man.cycle_target(&state);
        assert!(man.car_cam().is_none());
    }

    #[test]
    fn fov_matches_config_when_still() {
        let state = test_state();
        let mut man = CameraMan::new(GameMode::Soccar, CameraConfig::default());
        man.cycle_target(&state);
        man.update(&state, 1.0 / 60.0);
        // Car is still: dynamic terms are 0, so FOV equals the config.
        assert_eq!(man.cur_fov_degrees(), man.config().fov_degrees);
    }

    #[test]
    fn demo_freezes_and_respawn_snaps() {
        let mut state = test_state();
        let mut man = CameraMan::new(GameMode::Soccar, CameraConfig::default());
        man.cycle_target(&state);
        man.update(&state, 1.0 / 60.0);
        let live = man.cur_pos();

        state.cars[0].1.is_demoed = true;
        man.update(&state, 1.0 / 60.0);
        man.update(&state, 1.0 / 60.0);
        assert_eq!(man.cur_pos(), live);

        state.ball.pos = Vec3A::new(3000.0, 0.0, 100.0);
        man.update(&state, 1.0 / 60.0);
        assert_eq!(man.cur_pos(), live);
        let want = (state.ball.pos - live).normalize();
        assert!((man.cur_dir().dot(want) - 1.0).abs() < 1e-5);

        state.cars[0].1.is_demoed = false;
        state.cars[0].1.pos = Vec3A::new(-3000.0, -3000.0, 17.0);
        man.update(&state, 1.0 / 60.0);
        assert!((man.cur_pos() - live).length() > 100.0);
    }
}
