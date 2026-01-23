use crate::sim::collision_mesh_file::CollisionMeshFile;
use crate::vis::backend::{
    Color, ShaderSrc, ShaderSrcType, SharedVisRenderState, SharedWindowEvents, VisRenderLinePoint,
    VisRenderState, VisRenderer, WindowEvent,
};
use crate::vis::camera::{CameraConfig, CameraMan};
use crate::vis::ribbon_emitter::{RibbonConfig, RibbonEmitter};
use crate::vis::vis_asset_loader;
use crate::{ArenaState, CarBodyConfig, CarInfo, GameMode, Team, consts};
use glam::{Mat3A, Vec3A};
use miniquad::KeyCode;
use std::sync::RwLock;
use std::thread::JoinHandle;

pub struct VisInst {
    game_mode: GameMode,
    shared_state: SharedVisRenderState,
    camera_man: CameraMan,
    total_updates: usize,

    _renderer_handle: JoinHandle<()>,
    shared_window_events: SharedWindowEvents,

    boost_ribbons: Vec<RibbonEmitter>,
}

impl VisInst {
    pub fn new(game_mode: GameMode, arena_meshes: &[CollisionMeshFile]) -> Self {
        let models = vis_asset_loader::load_models(game_mode, arena_meshes);
        let textures = vis_asset_loader::load_textures();

        let shared_state = SharedVisRenderState::new(RwLock::new(VisRenderState::default()));
        let shader_srcs = vec![
            (
                "main",
                ShaderSrc::new(
                    ShaderSrcType::Model,
                    include_str!("shaders/main_vert.glsl"),
                    include_str!("shaders/main_frag.glsl"),
                ),
            ),
            (
                "arena",
                ShaderSrc::new(
                    ShaderSrcType::Model,
                    include_str!("shaders/arena_vert.glsl"),
                    include_str!("shaders/arena_frag.glsl"),
                ),
            ),
            (
                "line",
                ShaderSrc::new(
                    ShaderSrcType::Line,
                    include_str!("shaders/line_vert.glsl"),
                    include_str!("shaders/line_frag.glsl"),
                ),
            ),
        ];
        let (renderer_handle, shared_window_events) = VisRenderer::spawn_new(
            "RocketSim Visualizer",
            models,
            textures,
            shared_state.clone(),
            shader_srcs,
        );

        Self {
            game_mode,
            shared_state,

            // TODO: Allow user to specify config
            camera_man: CameraMan::new(game_mode, CameraConfig::default()),
            total_updates: 0,

            _renderer_handle: renderer_handle,
            shared_window_events,

            boost_ribbons: Vec::new(),
        }
    }

    pub fn update_camera(
        &mut self,
        new_render_state: &mut VisRenderState,
        arena_state: &ArenaState,
        dt: f32,
    ) {
        self.camera_man.update(arena_state, dt);
        new_render_state.camera_pos = self.camera_man.cur_pos();
        new_render_state.camera_look_target =
            new_render_state.camera_pos + self.camera_man.cur_dir() * 10.0;
        new_render_state.camera_fov_deg = self.camera_man.config().fov_degrees;
    }

    pub fn update(&mut self, astate: &ArenaState, dt: f32) {
        assert_eq!(astate.game_mode, self.game_mode);

        let arena_aabb = consts::arena::get_aabb(self.game_mode);
        let mut new_render_state = VisRenderState::default();

        self.update_camera(&mut new_render_state, astate, dt);

        fn get_car_model_name(car_info: &CarInfo) -> &str {
            if car_info.config == CarBodyConfig::DOMINUS {
                "car_dominus"
            } else if car_info.config == CarBodyConfig::BREAKOUT {
                "car_breakout"
            } else if car_info.config == CarBodyConfig::PLANK {
                "car_plank"
            } else if car_info.config == CarBodyConfig::HYBRID {
                "car_hybrid"
            } else if car_info.config == CarBodyConfig::MERC {
                "car_merc"
            } else {
                "car_octane"
            }
        }

        // Render arena mesh and planes
        {
            new_render_state.add_model_obj(
                "arena",
                Some("white"),
                Some("arena"),
                Vec3A::ZERO,
                Mat3A::IDENTITY,
            );
        }

        // Render ball
        new_render_state.add_model_obj(
            "ball",
            Some("ball"),
            None,
            astate.ball_state.pos,
            astate.ball_state.rot_mat,
        );

        // Render ball-to-ground line
        new_render_state.add_line_simple(
            astate.ball_state.pos,
            Vec3A::new(
                astate.ball_state.pos.x,
                astate.ball_state.pos.y,
                arena_aabb.min.z,
            ),
            Color::WHITE.with_alpha(0.15),
            2.0,
        );

        // Render cars
        for i in 0..astate.num_cars() {
            let car_info = &astate.car_infos[i];
            let car_state = &astate.car_states[i];

            if !car_state.is_demoed {
                let car_texture_name = if car_info.team == Team::Blue {
                    "car_blue"
                } else {
                    "car_orange"
                };
                let car_model = get_car_model_name(car_info);
                new_render_state.add_model_obj(
                    car_model,
                    Some(car_texture_name),
                    None,
                    car_state.pos,
                    car_state.rot_mat,
                );
            }

            // Update and boost ribbon
            {
                if self.boost_ribbons.len() < (i + 1) {
                    // TODO: Make this user-configurable
                    self.boost_ribbons.push(RibbonEmitter::new(RibbonConfig {
                        emit_rate: 30.0,
                        emit_lifetime: 0.5,

                        start_width: 20.0,
                        end_width: 0.0,

                        start_color: Color::new(1.0, 1.0, 0.5, 1.0),
                        end_color: Color::new(1.0, 0.4, 0.2, 0.0),

                        width_exponent: 0.7,
                        color_exponent: 1.0,
                    }))
                }

                let boost_ribbon = &mut self.boost_ribbons[i];
                let emit_dir = -car_state.get_forward_dir();
                let emit_pos = car_state.pos + (emit_dir * 60.0);
                let emit_vel = emit_dir * 300.0;

                boost_ribbon.update(car_state.is_boosting, emit_pos, emit_vel, dt);
                boost_ribbon.render(&mut new_render_state, emit_pos);
            }
        }

        // Render boost pads
        for i in 0..astate.num_boost_pads() {
            let pad_config = astate.boost_pad_configs[i];
            let pad_state = astate.boost_pad_states[i];

            let pad_model = if pad_config.is_big {
                if pad_state.is_active() {
                    "pad_big_on"
                } else {
                    "pad_big_off"
                }
            } else {
                if pad_state.is_active() {
                    "pad_small_on"
                } else {
                    "pad_small_off"
                }
            };

            new_render_state.add_model_obj(
                pad_model,
                Some("boost_pad"),
                None,
                pad_config.pos * Vec3A::new(1.0, 1.0, 0.0), // TODO: Temp Z-fix
                Mat3A::IDENTITY * 2.0,                      // TODO: Resize pad models
            );
        }

        // Handle window events
        {
            let window_events = self.shared_window_events.lock().unwrap().pop_all();
            for event in window_events {
                match event {
                    WindowEvent::KeyDown { key } => match key {
                        KeyCode::C => self.camera_man.cycle_target(astate),
                        KeyCode::Space => self.camera_man.try_toggle_ball_cam(),
                        _ => {}
                    },
                    _ => {}
                }
            }
        }

        *self.shared_state.write().unwrap() = new_render_state;
        self.total_updates += 1;
    }
}
