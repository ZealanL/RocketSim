//! Birds-eye / follow-car camera manager.
//!
//! `C` cycles birds-eye → each car, `Space` toggles ball-cam on the follow
//! view. Smoothing resets on target change but survives toggles. Demoed
//! cars freeze the pose (look re-aims at the ball); respawn snaps back.

use glam::Vec3A;
use rocketsim::{ArenaState, GameMode};

use crate::camera::{
    CameraConfig, CarCam, CarCamSmoothing, D_BIRDS_EYE, clamp_dt, default_birds_eye,
};

enum CameraViewMode {
    BirdsEye,
    CarCam(CarCam),
}

/// Persistent camera state: birds-eye or following one car.
pub struct CameraMan {
    game_mode: GameMode,
    config: CameraConfig,
    cur_pos: Vec3A,
    cur_dir: Vec3A,
    smooth: CarCamSmoothing,
    /// Frozen death pose while the followed car is demoed (position holds,
    /// look re-aims at the ball). Cleared on respawn (view snaps to live).
    demo_frozen_pos: Option<Vec3A>,
    followed_info_idx: Option<usize>,

    view_mode: CameraViewMode,
}

impl CameraMan {
    /// Starts in birds-eye view, looking from the effective wide anchor.
    pub fn new(game_mode: GameMode, config: CameraConfig) -> Self {
        let base = Self::birds_base(game_mode, &config);
        Self {
            game_mode,
            config,
            cur_pos: base,
            cur_dir: -base.normalize_or_zero(),
            smooth: CarCamSmoothing::new(),
            demo_frozen_pos: None,
            followed_info_idx: None,
            view_mode: CameraViewMode::BirdsEye,
        }
    }

    /// Wide-view anchor: the user's `birds_eye_pos` setting, unless it is
    /// still the Soccar default in a mode whose geometry collides with it
    /// (Hoops rim) — then the per-mode default. An explicit user value
    /// always wins, in every mode.
    fn birds_base(game_mode: GameMode, config: &CameraConfig) -> Vec3A {
        if config.birds_eye_pos == D_BIRDS_EYE {
            default_birds_eye(game_mode)
        } else {
            config.birds_eye_pos
        }
    }

    /// Current camera position in uu.
    pub fn cur_pos(&self) -> Vec3A {
        self.cur_pos
    }

    /// Retargets an existing manager at a new stream mode (mesh switches
    /// without losing follow state); the wide anchor follows on the next
    /// update.
    pub fn set_game_mode(&mut self, game_mode: GameMode) {
        self.game_mode = game_mode;
    }

    /// Current normalized view direction.
    pub fn cur_dir(&self) -> Vec3A {
        self.cur_dir
    }

    /// Overrides the current pose (arena containment pulls the follow boom
    /// inside the hull). Smoothing state is untouched, so the boom keeps
    /// pushing out and the clamp re-applies every frame — no sticking.
    pub fn override_pose(&mut self, pos: Vec3A, dir: Vec3A) {
        self.cur_pos = pos;
        self.cur_dir = dir;
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

    /// Selects wide view and clears follow-camera history.
    pub fn set_wide_view(&mut self) {
        self.view_mode = CameraViewMode::BirdsEye;
        self.smooth.reset();
        self.demo_frozen_pos = None;
        self.followed_info_idx = None;
    }

    /// Advances wide view, then each car, then wide view again.
    /// Does nothing when the arena has no cars.
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
        self.followed_info_idx = self
            .car_cam()
            .and_then(|cam| arena_state.cars.get(cam.car_idx).map(|(info, _)| info.idx));
    }

    /// Follows car `i` directly (spectator number-key select). Out-of-range
    /// indices are ignored. Resets smoothing like a target change.
    pub fn follow_car(&mut self, arena_state: &ArenaState, i: usize) {
        if i >= arena_state.num_cars() {
            return;
        }
        self.view_mode = CameraViewMode::CarCam(CarCam::new(i));
        self.smooth.reset();
        self.demo_frozen_pos = None;
        self.followed_info_idx = arena_state.cars.get(i).map(|(info, _)| info.idx);
    }

    /// Toggles ball-cam while following a car. No-op in birds-eye view.
    pub fn try_toggle_ball_cam(&mut self) {
        if let CameraViewMode::CarCam(car_cam) = &mut self.view_mode {
            car_cam.face_ball = !car_cam.face_ball;
        }
    }

    /// Recomputes position/direction from the latest arena state.
    ///
    /// `dt` (seconds, clamped to `[0, 1]`) advances follow-view smoothing;
    /// birds-eye still snaps.
    pub fn update(&mut self, arena_state: &ArenaState, dt: f32) {
        let follow = match &mut self.view_mode {
            CameraViewMode::BirdsEye => None,
            CameraViewMode::CarCam(car_cam) => {
                let Some(id) = self.followed_info_idx else {
                    self.set_wide_view();
                    return;
                };
                let Some(slot) = arena_state.cars.iter().position(|(info, _)| info.idx == id)
                else {
                    self.set_wide_view();
                    self.update(arena_state, dt);
                    return;
                };
                car_cam.car_idx = slot;
                Some((slot, car_cam.face_ball))
            }
        };
        match follow {
            None => {
                let base = Self::birds_base(self.game_mode, &self.config);
                self.cur_pos = base;
                self.cur_dir = (arena_state.ball.pos - base).normalize_or_zero();
            }
            Some((car_idx, face_ball)) => {
                let Some((_, car)) = arena_state.cars.get(car_idx) else {
                    self.set_wide_view();
                    self.update(arena_state, dt);
                    return;
                };
                let demoed = car.is_demoed;
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
    fn wide_anchor_follows_mode_until_customized() {
        // Fresh (Soccar-default) settings track the per-mode default ...
        let def = CameraConfig::default();
        assert_eq!(CameraMan::birds_base(GameMode::Soccar, &def), D_BIRDS_EYE);
        let hoops = CameraMan::birds_base(GameMode::Hoops, &def);
        assert_ne!(hoops, D_BIRDS_EYE);
        // ... and the Hoops anchor clears the rim geometry (logic AABB
        // grown by a 500uu margin; the old shared anchor sat inside it).
        let aabb = rocketsim::consts::arena::get_aabb(GameMode::Hoops);
        let m = 500.0;
        let outside = hoops.x < aabb.min.x - m
            || hoops.x > aabb.max.x + m
            || hoops.y < aabb.min.y - m
            || hoops.y > aabb.max.y + m
            || hoops.z < aabb.min.z - m
            || hoops.z > aabb.max.z + m;
        assert!(outside, "hoops wide anchor inside arena: {hoops:?}");
        // ... while an explicit user value always wins, in every mode.
        let custom = CameraConfig {
            birds_eye_pos: Vec3A::new(1.0, 2.0, 3.0),
            ..CameraConfig::default()
        };
        assert_eq!(
            CameraMan::birds_base(GameMode::Hoops, &custom),
            custom.birds_eye_pos
        );
        assert_eq!(
            CameraMan::birds_base(GameMode::Soccar, &custom),
            custom.birds_eye_pos
        );
    }

    #[test]
    fn retargeting_modes_moves_wide_anchor() {
        // Mesh switches must not lose follow state, but the wide anchor
        // follows the new mode on the next update.
        let mut man = CameraMan::new(GameMode::Soccar, CameraConfig::default());
        let state = test_state();
        man.update(&state, 1.0 / 120.0);
        assert_eq!(man.cur_pos(), D_BIRDS_EYE);
        man.set_game_mode(GameMode::Hoops);
        man.update(&state, 1.0 / 120.0);
        assert_eq!(man.cur_pos(), default_birds_eye(GameMode::Hoops));
    }

    #[test]
    fn cycle_and_toggle() {
        let state = test_state();
        let mut man = CameraMan::new(GameMode::Soccar, CameraConfig::default());
        assert!(man.car_cam().is_none());
        // Direct select valid + out-of-range (ignored, keeps current).
        man.follow_car(&state, 1);
        assert_eq!(man.car_cam().unwrap().car_idx, 1);
        man.follow_car(&state, 99);
        assert_eq!(man.car_cam().unwrap().car_idx, 1);

        // Fresh manager for the cycle flow below.
        let mut man = CameraMan::new(GameMode::Soccar, CameraConfig::default());
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
    fn follow_tracks_identity_through_reorder_and_falls_back_on_replacement() {
        let mut state = test_state();
        let mut man = CameraMan::new(GameMode::Soccar, CameraConfig::default());
        man.follow_car(&state, 0);
        state.cars.swap(0, 1);
        man.update(&state, 0.1);
        assert_eq!(man.car_cam().unwrap().car_idx, 1);

        state.cars.retain(|(info, _)| info.idx != 0);
        man.update(&state, 0.1);
        assert!(man.car_cam().is_none());

        let mut state = test_state();
        let mut man = CameraMan::new(GameMode::Soccar, CameraConfig::default());
        man.follow_car(&state, 0);
        state.cars[0].0.idx = 7;
        man.update(&state, 0.1);
        assert!(man.car_cam().is_none());
    }

    #[test]
    fn cycle_tracks_selected_identity() {
        let mut state = test_state();
        let mut man = CameraMan::new(GameMode::Soccar, CameraConfig::default());
        man.cycle_target(&state);
        man.cycle_target(&state);
        assert_eq!(man.car_cam().unwrap().car_idx, 1);
        state.cars.swap(0, 1);
        man.update(&state, 0.1);
        assert_eq!(man.car_cam().unwrap().car_idx, 0);
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

    #[test]
    fn containment_shortens_boom_at_wall() {
        use crate::camera::{ArenaBounds, FOLLOW_BACKOFF};
        crate::geometry::init_collision_meshes();
        let bounds = ArenaBounds::build(GameMode::Soccar);

        // Car hugging the +X wall, ball deep at -X: the ball-cam boom
        // points straight through the wall.
        let mut state = test_state();
        state.cars[0].1.pos = Vec3A::new(3900.0, 0.0, 17.0);
        state.ball.pos = Vec3A::new(-4000.0, 0.0, 300.0);
        let mut man = CameraMan::new(GameMode::Soccar, CameraConfig::default());
        man.follow_car(&state, 0);
        man.update(&state, 1.0 / 60.0);
        // Uncontained boom exits through the X-wall plane (4096).
        assert!(
            man.cur_pos().x > 4096.0,
            "boom should exit without containment: {:?}",
            man.cur_pos()
        );
        // The viewer wiring (focus + contain + re-aim) pulls it back in
        // and keeps the look glued to the ball.
        let cam = man.car_cam().unwrap();
        let focus =
            state.cars[cam.car_idx].1.pos + Vec3A::new(0.0, 0.0, man.config().car_cam.height);
        let (p, d) = bounds.contain_follow(
            focus,
            state.ball.pos,
            cam.face_ball,
            man.cur_pos(),
            man.cur_dir(),
        );
        man.override_pose(p, d);
        assert!(
            (man.cur_pos().x - (4096.0 - FOLLOW_BACKOFF)).abs() < 2.0,
            "boom should stop one backoff inside: {:?}",
            man.cur_pos()
        );
        let want = (state.ball.pos - man.cur_pos()).normalize();
        assert!((man.cur_dir().dot(want) - 1.0).abs() < 1e-4);
    }
}
