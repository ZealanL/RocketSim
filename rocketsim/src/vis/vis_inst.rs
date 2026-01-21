use crate::sim::collision_mesh_file::CollisionMeshFile;
use crate::vis::backend::{ShaderCode, SharedVisRenderState, VisRenderState, VisRenderer};
use crate::vis::vis_asset_loader;
use crate::{ArenaState, CarBodyConfig, CarInfo, GameMode, Team};
use glam::{Mat3A, Vec3A};
use std::sync::RwLock;
use std::thread::JoinHandle;

pub struct VisInst {
    game_mode: GameMode,
    renderer_handle: JoinHandle<()>,
    shared_state: SharedVisRenderState,
}

impl VisInst {
    pub fn new(game_mode: GameMode, arena_meshes: &[CollisionMeshFile]) -> Self {
        let models = vis_asset_loader::load_models(game_mode, arena_meshes);
        let textures = vis_asset_loader::load_textures();

        let shared_state = SharedVisRenderState::new(RwLock::new(VisRenderState::default()));
        let shader_srcs = vec![
            ("main", ShaderCode::new(include_str!("shaders/main_vert.glsl"), include_str!("shaders/main_frag.glsl"))),
            ("arena", ShaderCode::new(include_str!("shaders/arena_vert.glsl"), include_str!("shaders/arena_frag.glsl"))),
        ];
        let renderer_handle = VisRenderer::spawn_new(
            "RocketSim Visualizer",
            models, textures, shared_state.clone(),
            shader_srcs
        );

        Self {
            game_mode,
            shared_state,
            renderer_handle,
        }
    }

    pub fn update_camera(
        &mut self,
        new_render_state: &mut VisRenderState,
        arena_state: &ArenaState,
        dt: f32,
    ) {
        unimplemented!() // TODO: Implement
    }

    pub fn update(&mut self, astate: &ArenaState, dt: f32) {
        assert_eq!(astate.game_mode, self.game_mode);

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

        // Render cars
        for i in 0..astate.num_cars() {
            let car_info = &astate.car_infos[i];
            let car_state = &astate.car_states[i];
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

        *self.shared_state.write().unwrap() = new_render_state;
    }
}
