use std::{collections::HashMap, sync::Arc};

use glam::Vec3A;
use rocketsim::{ArenaState, GameMode};
use rocketsim_vis::proto::SceneMetadata;

use super::input_state::{freecam_dolly_delta, look_delta};
use crate::{
    camera::{
        ArenaBounds, ArenaBoundsCache, AutoCam, CameraConfig, CameraMan, CarCamSmoothing,
        DirectorState, DirectorTarget, FreeCam,
    },
    input::{Action, ActionMap, ActionValue, Evaluator, InputSnapshot},
    render::view,
    settings::ViewerSettings,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum CamMode {
    Man,
    Director,
    Free,
}

#[derive(Debug, Clone)]
pub(super) enum StartMode {
    Birds,
    Follow(usize),
    Director,
    Auto(usize),
    Free,
}

impl StartMode {
    pub(super) fn try_parse(s: &str) -> Option<Self> {
        let s = s.to_ascii_lowercase();
        let index = |prefix: &str| s.strip_prefix(prefix).and_then(|n| n.parse::<usize>().ok());
        if let Some(n) = index("follow") {
            Some(Self::Follow(n))
        } else if s == "director" {
            Some(Self::Director)
        } else if let Some(n) = index("auto") {
            Some(Self::Auto(n))
        } else if s == "free" {
            Some(Self::Free)
        } else if s == "birds" {
            Some(Self::Birds)
        } else {
            None
        }
    }

    fn mode(&self) -> CamMode {
        match self {
            Self::Director => CamMode::Director,
            Self::Free => CamMode::Free,
            _ => CamMode::Man,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DirectorIdentity {
    Overview,
    Car(usize),
}

fn director_identity(target: DirectorTarget, state: &ArenaState) -> DirectorIdentity {
    match target {
        DirectorTarget::Overview => DirectorIdentity::Overview,
        DirectorTarget::Car(index) => state
            .cars
            .get(index)
            .map(|(info, _)| DirectorIdentity::Car(info.idx))
            .unwrap_or(DirectorIdentity::Overview),
    }
}

fn director_overview_pose(mode: GameMode) -> (Vec3A, Vec3A, f32) {
    let bounds = rocketsim::consts::arena::get_aabb(mode);
    // Keep the wide view inside the side wall and below the ceiling.
    let eye = bounds.max * Vec3A::new(0.92, 0.0, 0.63);
    let target = Vec3A::new(0.0, 0.0, 100.0);
    let fov = if mode == GameMode::Hoops {
        130.0
    } else {
        110.0
    };
    (eye, target, fov)
}

struct DirectorRig {
    director: DirectorState,
    smooth: CarCamSmoothing,
    previous_identity: Option<DirectorIdentity>,
}

impl DirectorRig {
    fn new() -> Self {
        Self {
            director: DirectorState::new(),
            smooth: CarCamSmoothing::new(),
            previous_identity: None,
        }
    }
}

pub(super) struct CameraFrameInput<'a> {
    pub(super) state: Option<&'a ArenaState>,
    pub(super) metadata_mode: Option<GameMode>,
    pub(super) mode_changed: bool,
    pub(super) settings: &'a ViewerSettings,
    pub(super) action_values: &'a HashMap<Action, ActionValue>,
    pub(super) action_map: &'a ActionMap,
    pub(super) evaluator: &'a Evaluator,
    pub(super) input_snapshot: &'a InputSnapshot,
    pub(super) dt: f32,
    pub(super) focused: bool,
    pub(super) menu_open: bool,
    pub(super) action_suppressed: bool,
    pub(super) pointer_busy: bool,
}

pub(super) struct CameraPose {
    pub(super) position: Vec3A,
    pub(super) direction: Vec3A,
    pub(super) fov: f32,
    pub(super) label: String,
}

pub(super) struct CameraState {
    pub(super) init_mode: StartMode,
    pub(super) mode: CamMode,
    pub(super) manager: Option<CameraMan>,
    director: DirectorRig,
    pub(super) bounds: ArenaBoundsCache,
    pub(super) bounds_for_scene: Option<Arc<ArenaBounds>>,
    pub(super) last_metadata: Option<Arc<SceneMetadata>>,
    pub(super) auto_cam: Option<AutoCam>,
    pub(super) freecam: FreeCam,
    pub(super) last_followed: usize,
    pub(super) swivel_yaw: f32,
    pub(super) swivel_pitch: f32,
}

impl CameraState {
    pub(super) fn new(init_mode: StartMode) -> Self {
        Self {
            mode: init_mode.mode(),
            init_mode,
            manager: None,
            director: DirectorRig::new(),
            bounds: ArenaBoundsCache::new(),
            bounds_for_scene: None,
            last_metadata: None,
            auto_cam: None,
            freecam: FreeCam::new(),
            last_followed: 0,
            swivel_yaw: 0.0,
            swivel_pitch: 0.0,
        }
    }

    pub(super) fn reset_session(&mut self) {
        self.manager = None;
        self.auto_cam = None;
        self.director = DirectorRig::new();
        self.freecam = FreeCam::new();
        self.bounds.reset();
        self.bounds_for_scene = None;
        self.last_metadata = None;
        self.last_followed = 0;
        self.swivel_yaw = 0.0;
        self.swivel_pitch = 0.0;
    }

    pub(super) fn reset_after_rewind(&mut self) {
        self.manager = None;
        self.auto_cam = None;
        self.director = DirectorRig::new();
        self.freecam = FreeCam::new();
        self.swivel_yaw = 0.0;
        self.swivel_pitch = 0.0;
        self.last_followed = 0;
    }

    pub(super) fn director_target(&mut self, state: &ArenaState, dt: f32) -> DirectorTarget {
        self.director.director.update(state, dt)
    }

    pub(super) fn watched_car(&self) -> Option<u32> {
        match self.mode {
            CamMode::Man => self
                .manager
                .as_ref()
                .and_then(|manager| manager.car_cam())
                .map(|car| car.car_idx as u32),
            CamMode::Director => match self.director.director.target() {
                DirectorTarget::Car(index) => Some(index as u32),
                DirectorTarget::Overview => None,
            },
            CamMode::Free => None,
        }
    }

    pub(super) fn director_pose(
        &mut self,
        target: DirectorTarget,
        state: &ArenaState,
        settings: &crate::settings::ViewerSettings,
        dt: f32,
    ) -> (Vec3A, Vec3A, f32, String) {
        let identity = director_identity(target, state);
        if self.director.previous_identity != Some(identity) {
            self.director.smooth.reset();
            self.director.previous_identity = Some(identity);
        }
        match target {
            DirectorTarget::Overview => {
                let (eye, target, fov) = director_overview_pose(state.game_mode());
                (
                    eye,
                    (target - eye).normalize_or_zero(),
                    fov,
                    "director · wide view".to_string(),
                )
            }
            DirectorTarget::Car(index) => {
                let mut pos = Vec3A::ZERO;
                let mut dir = Vec3A::X;
                self.director.smooth.update(
                    index,
                    true,
                    state,
                    &settings.camera,
                    &mut pos,
                    &mut dir,
                    dt,
                );
                if let (Some(hull), Some((_, car))) =
                    (self.bounds_for_scene.as_ref(), state.cars.get(index))
                {
                    let focus = car.pos + Vec3A::new(0.0, 0.0, settings.camera.car_cam.height);
                    (pos, dir) = hull.contain_follow(focus, state.ball.pos, true, pos, dir);
                }
                (
                    pos,
                    dir,
                    self.director.smooth.cur_fov_h(),
                    format!("director · car {}", state.cars[index].0.idx),
                )
            }
        }
    }

    pub(super) fn update(&mut self, input: CameraFrameInput<'_>) -> CameraPose {
        let (position, mut direction, fov, label) = match input.state {
            Some(state) => {
                if input.mode_changed {
                    self.update_for_mode(state.game_mode());
                }
                let bounds = if input.metadata_mode == Some(state.game_mode()) {
                    self.bounds_for_scene.clone()
                } else {
                    None
                };
                if self.manager.is_none() {
                    let mut manager = CameraMan::new(state.game_mode(), CameraConfig::default());
                    match self.init_mode {
                        StartMode::Follow(index) if index < state.num_cars() => {
                            manager.follow_car(state, index);
                        }
                        StartMode::Auto(index) if index < state.num_cars() => {
                            manager.follow_car(state, index);
                            self.auto_cam = Some(AutoCam::new(index));
                        }
                        StartMode::Follow(index) | StartMode::Auto(index) => {
                            eprintln!(
                                "rocketsim-viewer: car {index} is unavailable; using wide view"
                            );
                        }
                        _ => {}
                    }
                    self.manager = Some(manager);
                }
                let previous_mode = self.mode;
                for (index, action) in [
                    Action::Follow1,
                    Action::Follow2,
                    Action::Follow3,
                    Action::Follow4,
                    Action::Follow5,
                    Action::Follow6,
                    Action::Follow7,
                    Action::Follow8,
                ]
                .into_iter()
                .enumerate()
                {
                    if !input.action_suppressed
                        && !input.menu_open
                        && input
                            .action_values
                            .get(&action)
                            .is_some_and(|value| value.just_pressed)
                    {
                        self.mode = CamMode::Man;
                        self.auto_cam = None;
                        if let Some(manager) = self.manager.as_mut() {
                            manager.follow_car(state, index);
                        }
                    }
                }
                if !input.action_suppressed
                    && !input.menu_open
                    && input
                        .action_values
                        .get(&Action::Director)
                        .is_some_and(|value| value.just_pressed)
                {
                    self.mode = CamMode::Director;
                    self.auto_cam = None;
                }
                if !input.action_suppressed
                    && !input.menu_open
                    && input
                        .action_values
                        .get(&Action::AutoCam)
                        .is_some_and(|value| value.just_pressed)
                {
                    self.mode = CamMode::Man;
                    let index = self.last_followed;
                    if let Some(manager) = self.manager.as_mut() {
                        manager.follow_car(state, index);
                    }
                    self.auto_cam = Some(AutoCam::new(index));
                }
                if !input.action_suppressed
                    && !input.menu_open
                    && input
                        .action_values
                        .get(&Action::FreeCam)
                        .is_some_and(|value| value.just_pressed)
                {
                    self.mode = if self.mode == CamMode::Free {
                        CamMode::Man
                    } else {
                        CamMode::Free
                    };
                    self.auto_cam = None;
                }
                if !input.action_suppressed
                    && !input.menu_open
                    && input
                        .action_values
                        .get(&Action::NextCam)
                        .is_some_and(|value| value.just_pressed)
                {
                    self.mode = CamMode::Man;
                    self.auto_cam = None;
                    if let Some(manager) = self.manager.as_mut() {
                        manager.cycle_target(state);
                    }
                }
                if self.mode != previous_mode {
                    self.swivel_yaw = 0.0;
                    self.swivel_pitch = 0.0;
                }
                match self.mode {
                    CamMode::Man => {
                        if !input.action_suppressed
                            && !input.menu_open
                            && input
                                .action_values
                                .get(&Action::BallCam)
                                .is_some_and(|value| value.just_pressed)
                            && let Some(manager) = self.manager.as_mut()
                        {
                            manager.try_toggle_ball_cam();
                        }
                        if let Some(auto) = self.auto_cam.as_mut() {
                            let index = auto.car_idx();
                            let on = auto.update(state, input.dt);
                            if let Some(manager) = self.manager.as_mut() {
                                let following = manager
                                    .car_cam()
                                    .is_some_and(|camera| camera.car_idx == index);
                                if following {
                                    manager.set_follow_face_ball(index, on);
                                } else {
                                    self.auto_cam = None;
                                }
                            }
                        }
                        let manager = self.manager.as_mut().expect("viewer camera built");
                        manager.set_config(input.settings.camera);
                        manager.update(state, input.dt);
                        if let Some(camera) = manager.car_cam() {
                            self.last_followed = camera.car_idx;
                        }
                        if let (Some(hull), Some(camera)) = (bounds.as_ref(), manager.car_cam())
                            && let Some((_, car)) = state.cars.get(camera.car_idx)
                            && !car.is_demoed
                        {
                            let focus = car.pos
                                + Vec3A::new(0.0, 0.0, input.settings.camera.car_cam.height);
                            let (position, direction) = hull.contain_follow(
                                focus,
                                state.ball.pos,
                                camera.face_ball,
                                manager.cur_pos(),
                                manager.cur_dir(),
                            );
                            manager.override_pose(position, direction);
                        }
                        let label = match (manager.car_cam(), &self.auto_cam) {
                            (None, _) => "wide view".to_string(),
                            (Some(camera), None) => format!(
                                "car {} · {}",
                                camera.car_idx,
                                if camera.face_ball {
                                    "ball cam"
                                } else {
                                    "car cam"
                                }
                            ),
                            (Some(camera), Some(auto)) => format!(
                                "car {} · auto ({})",
                                camera.car_idx,
                                auto.off_reason()
                                    .map(|reason| reason.as_str())
                                    .unwrap_or("ball cam")
                            ),
                        };
                        (
                            manager.cur_pos(),
                            manager.cur_dir(),
                            manager.cur_fov_degrees(),
                            label,
                        )
                    }
                    CamMode::Director => {
                        let target = self.director_target(state, input.dt);
                        self.director_pose(target, state, input.settings, input.dt)
                    }
                    CamMode::Free => {
                        const LOOK_SENSITIVITY: f32 = 0.005;
                        if input.focused && !input.menu_open {
                            let deadzone = crate::input::sources::clamp_deadzone(
                                input.settings.bindings.pad.stick_deadzone,
                            );
                            let yaw_delta = look_delta(
                                input.action_map,
                                input.evaluator,
                                input.input_snapshot,
                                Action::LookLeft,
                                Action::LookRight,
                                input.dt,
                                LOOK_SENSITIVITY,
                                deadzone,
                                input.pointer_busy,
                            );
                            let pitch_delta = look_delta(
                                input.action_map,
                                input.evaluator,
                                input.input_snapshot,
                                Action::LookUp,
                                Action::LookDown,
                                input.dt,
                                LOOK_SENSITIVITY,
                                deadzone,
                                input.pointer_busy,
                            );
                            self.freecam.look(yaw_delta, pitch_delta);
                        }
                        let (forward, right, up) =
                            if input.action_suppressed || input.menu_open || !input.focused {
                                (0.0, 0.0, 0.0)
                            } else {
                                let value = |action| {
                                    input
                                        .action_values
                                        .get(&action)
                                        .map_or(0.0, |value| value.value)
                                };
                                (
                                    value(Action::FreeForward) - value(Action::FreeBack),
                                    value(Action::FreeRight) - value(Action::FreeLeft),
                                    value(Action::FreeUp) - value(Action::FreeDown),
                                )
                            };
                        if !input.menu_open
                            && input
                                .action_values
                                .get(&Action::FreeReset)
                                .is_some_and(|value| value.just_pressed)
                        {
                            self.freecam = FreeCam::new();
                        }
                        let speed = if !input.menu_open
                            && input
                                .action_values
                                .get(&Action::FreeFast)
                                .is_some_and(|value| value.pressed)
                        {
                            3.0
                        } else {
                            1.0
                        };
                        self.freecam.fly_bounded(
                            forward,
                            right,
                            up,
                            input.dt * speed,
                            bounds.as_deref(),
                        );
                        if input.focused && !input.menu_open && !input.action_suppressed {
                            let dolly = freecam_dolly_delta(
                                input.action_map,
                                input.evaluator,
                                input.input_snapshot,
                                input.dt,
                                48.0,
                                220.0,
                                crate::input::sources::clamp_deadzone(
                                    input.settings.bindings.pad.stick_deadzone,
                                ),
                                input.pointer_busy,
                            );
                            self.freecam.dolly_bounded(dolly, bounds.as_deref());
                        }
                        let (position, direction) = self.freecam.pos_dir();
                        (
                            position,
                            direction,
                            input.settings.camera.fov_degrees,
                            "free camera".to_string(),
                        )
                    }
                }
            }
            None => {
                let (eye, target, fov) = if self.mode == CamMode::Director {
                    director_overview_pose(input.metadata_mode.unwrap_or(GameMode::Soccar))
                } else {
                    let (eye, target, fov) = view::arena_pose();
                    (Vec3A::from(eye), Vec3A::from(target), fov)
                };
                (
                    eye,
                    (target - eye).normalize_or_zero(),
                    fov,
                    "waiting for sim".to_string(),
                )
            }
        };
        let following = self.mode == CamMode::Man
            && self
                .manager
                .as_ref()
                .and_then(|manager| manager.car_cam())
                .is_some();
        let (look_tx, look_ty) = if following && !input.menu_open {
            follow_look_targets(input.action_values)
        } else {
            (0.0, 0.0)
        };
        let smoothing = 1.0 - (-10.0 * input.dt).exp();
        self.swivel_yaw += (look_tx - self.swivel_yaw) * smoothing;
        self.swivel_pitch += (look_ty - self.swivel_pitch) * smoothing;
        if self.swivel_yaw.abs() + self.swivel_pitch.abs() > 1e-4 {
            direction = apply_follow_look(direction, self.swivel_yaw, self.swivel_pitch);
        }
        CameraPose {
            position,
            direction,
            fov,
            label,
        }
    }

    pub(super) fn update_for_mode(&mut self, mode: GameMode) {
        if let Some(manager) = self.manager.as_mut() {
            manager.set_game_mode(mode);
        }
    }
}

pub(super) fn follow_look_targets(
    values: &std::collections::HashMap<crate::input::Action, crate::input::ActionValue>,
) -> (f32, f32) {
    const MAX_YAW: f32 = 2.0;
    const MAX_PITCH: f32 = 0.9;
    let value = |action| values.get(&action).map_or(0.0, |v| v.value.clamp(0.0, 1.0));
    (
        (value(crate::input::Action::SwivelRight) - value(crate::input::Action::SwivelLeft))
            * MAX_YAW,
        (value(crate::input::Action::SwivelUp) - value(crate::input::Action::SwivelDown))
            * MAX_PITCH,
    )
}

pub(super) fn apply_follow_look(cam_dir: Vec3A, yaw: f32, pitch: f32) -> Vec3A {
    let theta = cam_dir.y.atan2(cam_dir.x) + yaw;
    let elev = (cam_dir.z.clamp(-1.0, 1.0).asin() + pitch).clamp(-1.55, 1.55);
    Vec3A::new(
        elev.cos() * theta.cos(),
        elev.cos() * theta.sin(),
        elev.sin(),
    )
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use super::*;
    use crate::input::Action;

    #[test]
    fn boost_meter_tracks_the_watched_car_without_changing_the_driving_target() {
        use rocketsim::{CarInfo, CarState};

        use crate::ui::hud::spectated_boost;

        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.ball.pos.y = 1000.0;
        let mut state = CarState::DEFAULT;
        state.boost = 87.0;
        arena.cars.push((
            CarInfo {
                idx: 8,
                ..CarInfo::default()
            },
            state,
        ));
        let mut camera = CameraState::new(StartMode::Director);
        camera.director_target(&arena, 0.0);

        state.boost = 25.0;
        arena.cars.insert(
            0,
            (
                CarInfo {
                    idx: 3,
                    ..CarInfo::default()
                },
                state,
            ),
        );
        let mut manager = CameraMan::new(GameMode::Soccar, CameraConfig::default());
        manager.follow_car(&arena, 0);
        camera.manager = Some(manager);
        camera.director_target(&arena, 0.0);

        for (mode, expected) in [
            (CamMode::Director, Some(87.0)),
            (CamMode::Man, Some(25.0)),
            (CamMode::Free, None),
        ] {
            camera.mode = mode;
            assert_eq!(spectated_boost(&arena.cars, camera.watched_car()), expected);
        }

        camera.mode = CamMode::Director;
        arena.cars[1].1.is_demoed = true;
        camera.director_target(&arena, 0.0);
        assert_eq!(
            spectated_boost(&arena.cars, camera.watched_car()),
            Some(25.0)
        );
        assert_eq!(
            camera.manager.as_ref().unwrap().car_cam().unwrap().car_idx,
            0
        );

        arena.ball.pos.y = 0.0;
        camera.director_target(&arena, 0.0);
        assert_eq!(spectated_boost(&arena.cars, camera.watched_car()), None);
        arena.cars.clear();
        camera.director_target(&arena, 0.0);
        assert_eq!(spectated_boost(&arena.cars, camera.watched_car()), None);
    }

    #[test]
    fn director_wide_view_is_inside_arena_and_frames_kickoff() {
        use glam::Vec4Swizzles;
        use rocketsim::consts::car::spawn::get_kickoff_spawn_locations;

        let settings = ViewerSettings::default();
        let mut camera = CameraState::new(StartMode::Director);
        for mode in [GameMode::Soccar, GameMode::Hoops, GameMode::Dropshot] {
            let arena = ArenaState::new_empty(mode);
            let (eye, direction, fov, _) =
                camera.director_pose(DirectorTarget::Overview, &arena, &settings, 0.0);
            let bounds = rocketsim::consts::arena::get_aabb(mode);
            assert!(eye.cmpgt(bounds.min).all() && eye.cmplt(bounds.max).all());
            for aspect in [4.0 / 3.0, 16.0 / 9.0] {
                let (view, proj) =
                    view::view_proj(eye.into(), (eye + direction).into(), fov, aspect);
                let mvp = proj * view;
                let spawns = get_kickoff_spawn_locations(mode);
                let points = spawns.iter().flat_map(|spawn| {
                    let pos = Vec3A::new(spawn.x, spawn.y, 50.0);
                    [pos, Vec3A::new(-pos.x, -pos.y, pos.z)]
                });
                for point in points.chain([arena.ball.pos]) {
                    let clip = mvp * point.extend(1.0);
                    assert!(clip.w > 0.0, "{mode:?}: {point:?} is behind the camera");
                    let ndc = clip.xyz() / clip.w;
                    assert!(
                        ndc.abs().cmplt(glam::Vec3::ONE).all(),
                        "{mode:?}: {point:?} is outside the wide view: {ndc:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn follow_look_signs_match_resulting_camera_direction_and_accept_remapped_sources() {
        use crate::input::ActionValue;

        let mut values = HashMap::new();
        values.insert(
            Action::SwivelRight,
            ActionValue {
                value: 1.0,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        );
        let (yaw, pitch) = follow_look_targets(&values);
        let right = apply_follow_look(Vec3A::X, yaw, pitch);
        assert!(right.y > 0.0 && right.x < 1.0);

        values.clear();
        values.insert(
            Action::SwivelLeft,
            ActionValue {
                value: 1.0,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        );
        let (yaw, pitch) = follow_look_targets(&values);
        let left = apply_follow_look(Vec3A::X, yaw, pitch);
        assert!(left.y < 0.0 && left.x < 1.0);

        values.clear();
        values.insert(
            Action::SwivelUp,
            ActionValue {
                value: 1.0,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        );
        let (yaw, pitch) = follow_look_targets(&values);
        assert!(apply_follow_look(Vec3A::X, yaw, pitch).z > 0.0);
        values.clear();
        values.insert(
            Action::SwivelDown,
            ActionValue {
                value: 1.0,
                pressed: true,
                just_pressed: true,
                just_released: false,
            },
        );
        let (yaw, pitch) = follow_look_targets(&values);
        assert!(apply_follow_look(Vec3A::X, yaw, pitch).z < 0.0);
    }
}
