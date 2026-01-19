use crate::vis::{ModelSet, SharedVisRenderState, TextureSet};
use glam::{Mat4, Vec2, Vec3};
use miniquad::{Backend, Bindings, BufferLayout, BufferSource, BufferType, BufferUsage, EventHandler, Pipeline, PipelineParams, RawId, RenderingBackend, ShaderMeta, ShaderSource, TextureId, UniformBlockLayout, UniformDesc, UniformType, UniformsSource, VertexAttribute, VertexFormat, conf, window, PrimitiveType, gl};
use std::thread::JoinHandle;
use ahash::AHashMap;

#[repr(C)]
pub struct Uniforms {
    pub u_model: Mat4,
    pub u_view: Mat4,
    pub u_proj: Mat4,
}
impl Uniforms {
    pub fn get_shader_meta_layout() -> UniformBlockLayout {
        UniformBlockLayout {
            uniforms: vec![
                UniformDesc::new("u_model", UniformType::Mat4),
                UniformDesc::new("u_view", UniformType::Mat4),
                UniformDesc::new("u_proj", UniformType::Mat4),
            ],
        }
    }
}

impl Default for Uniforms {
    fn default() -> Self {
        Self {
            u_model: Mat4::IDENTITY,
            u_view: Mat4::IDENTITY,
            u_proj: Mat4::IDENTITY,
        }
    }
}

struct MiniQuadStage {
    pipelines: AHashMap<String, Pipeline>,
    bindings: Bindings,
    ctx: Box<dyn RenderingBackend>,
    texture_ids: Vec<TextureId>,

    cur_uniforms: Uniforms,
    cur_window_size: Vec2,
}

impl MiniQuadStage {
    pub fn update_uniforms(&mut self) {
        self.ctx
            .apply_uniforms(UniformsSource::table(&self.cur_uniforms));
    }

    pub fn set_texture(&mut self, texture_idx: Option<usize>) {
        if let Some(texture_idx) = texture_idx {
            self.bindings.images = vec![self.texture_ids[texture_idx]];
            self.ctx.apply_bindings(&self.bindings);
        } else {
            self.bindings.images = vec![TextureId::from_raw_id(RawId::OpenGl(0))];
            self.ctx.apply_bindings(&self.bindings);
        }
    }

    pub fn set_pipeline(&mut self, name: Option<&str>) {
        self.ctx.apply_pipeline(&self.pipelines[name.unwrap_or("main")]);
    }
}

pub struct VisRenderer {
    models: ModelSet,
    textures: TextureSet,
    mq_stage: MiniQuadStage,
    shared_render_state: SharedVisRenderState,
}

impl VisRenderer {
    fn new(
        models: ModelSet,
        textures: TextureSet,
        shared_render_state: SharedVisRenderState,
    ) -> VisRenderer {
        // Set up miniquad stuff
        let mq_stage = {
            let mut ctx: Box<dyn RenderingBackend> = window::new_rendering_backend();

            let vertex_buffers = models.build_mq_vertex_buffers(&mut ctx);

            // Dummy indices buffer (its just [0, 1, 2, ...])
            let dummy_indices = (0..models.num_verts())
                .map(|i| i as u32)
                .collect::<Vec<_>>();
            let index_buffer = ctx.new_buffer(
                BufferType::IndexBuffer,
                BufferUsage::Immutable,
                BufferSource::slice(dummy_indices.as_slice()),
            );

            let texture_ids = textures.build_mq_textures(&mut ctx);

            let bindings = Bindings {
                vertex_buffers,
                index_buffer,
                images: vec![TextureId::from_raw_id(RawId::OpenGl(0))],
            };

            let main_shader = ctx
                .new_shader(
                    match ctx.info().backend {
                        Backend::OpenGl => ShaderSource::Glsl {
                            vertex: include_str!("shaders/main_vert.glsl"),
                            fragment: include_str!("shaders/main_frag.glsl"),
                        },
                        Backend::Metal => unimplemented!(),
                    },
                    ShaderMeta {
                        images: vec!["u_texture".to_string()],
                        uniforms: Uniforms::get_shader_meta_layout(),
                    },
                ).unwrap();

            let arena_shader = ctx
                .new_shader(
                    match ctx.info().backend {
                        Backend::OpenGl => ShaderSource::Glsl {
                            vertex: include_str!("shaders/arena_vert.glsl"),
                            fragment: include_str!("shaders/arena_frag.glsl"),
                        },
                        Backend::Metal => unimplemented!(),
                    },
                    ShaderMeta {
                        images: vec!["u_texture".to_string()],
                        uniforms: Uniforms::get_shader_meta_layout(),
                    },
                ).unwrap();

            let buffer_layout = [
                BufferLayout::default(),
                BufferLayout::default(),
                BufferLayout::default(),
            ];
            let attributes = [
                VertexAttribute::with_buffer("vs_vert", VertexFormat::Float3, 0),
                VertexAttribute::with_buffer("vs_vert_normal", VertexFormat::Float3, 1),
                VertexAttribute::with_buffer("vs_vert_uv", VertexFormat::Float2, 2),
            ];
            let params = PipelineParams {
                depth_test: miniquad::Comparison::LessOrEqual,
                depth_write: true,
                cull_face: miniquad::CullFace::Back,
                primitive_type: PrimitiveType::Triangles,
                ..Default::default()
            };

            let main_pipeline = ctx.new_pipeline(&buffer_layout, &attributes, main_shader, params.clone());
            let arena_pipeline = ctx.new_pipeline(&buffer_layout, &attributes, arena_shader, params.clone());

            let pipelines = AHashMap::from(
                [
                    ("main".to_string(), main_pipeline.clone()),
                    ("arena".to_string(), arena_pipeline.clone()),
                ]
            );

            MiniQuadStage {
                pipelines,
                bindings,
                ctx,
                texture_ids,

                cur_uniforms: Uniforms::default(),
                cur_window_size: Vec2::from(window::screen_size()),
            }
        };

        VisRenderer {
            models,
            textures,
            mq_stage,
            shared_render_state,
        }
    }
}

impl EventHandler for VisRenderer {
    fn update(&mut self) {}

    fn resize_event(&mut self, width: f32, height: f32) {
        self.mq_stage.cur_window_size = glam::vec2(width, height);
    }

    fn draw(&mut self) {
        // Begin frame in macroquad
        {
            self.mq_stage
                .ctx
                .begin_default_pass(miniquad::PassAction::clear_color(0.1, 0.1, 0.1, 1.0));
            self.mq_stage.set_pipeline(None);
            self.mq_stage.ctx.apply_bindings(&self.mq_stage.bindings);
        }

        fn calc_camera_view_proj(
            pos: Vec3,
            look_target: Vec3,
            fov_degrees: f32,
            cur_window_size: Vec2,
        ) -> (Mat4, Mat4) {
            let view = Mat4::look_at_lh(pos, look_target, Vec3::Z);

            let aspect = cur_window_size.x / cur_window_size.y;

            const Z_RANGE_NEAR: f32 = 5.0;
            const Z_RANGE_FAR: f32 = 50_000.0;

            let proj = Mat4::perspective_lh(
                fov_degrees.to_radians(),
                aspect,
                Z_RANGE_NEAR,
                Z_RANGE_FAR,
            );

            (view, proj)
        }

        let state = self.shared_render_state.read().unwrap().clone();

        // Update camera
        {
            let (u_view, u_proj) = calc_camera_view_proj(
                state.camera_pos.to_vec3(),
                state.camera_look_target.to_vec3(),
                state.camera_fov_deg,
                self.mq_stage.cur_window_size,
            );
            self.mq_stage.cur_uniforms.u_view = u_view;
            self.mq_stage.cur_uniforms.u_proj = u_proj;
        }

        // Render objects
        {
            for render_obj in state.render_objects {
                if let Some(pipeline_name) = render_obj.pipeline_name {
                    self.mq_stage.set_pipeline(Some(&pipeline_name));
                } else {
                    self.mq_stage.set_pipeline(None);
                };

                let model_mat4 = Mat4::from_mat3_translation(
                    render_obj.model_rot_mat.into(),
                    render_obj.model_pos.into(),
                );

                self.mq_stage.cur_uniforms.u_model = model_mat4;
                self.mq_stage.cur_uniforms.u_model.y_axis *= -1.0; // Flip to left-handed
                self.mq_stage.update_uniforms();

                if let Some(texture_name) = render_obj.texture_name {
                    self.mq_stage
                        .set_texture(Some(self.textures.get_texture_idx(&texture_name)))
                } else {
                    self.mq_stage.set_texture(None);
                }

                let (draw_range_start, draw_range_end) =
                    self.models.get_model_draw_range(&render_obj.model_name);
                assert!(draw_range_start < draw_range_end);

                self.mq_stage.ctx.draw(
                    draw_range_start as i32,
                    (draw_range_end - draw_range_start) as i32,
                    1,
                );

                self.mq_stage.ctx.draw(
                    draw_range_start as i32,
                    (draw_range_end - draw_range_start) as i32,
                    1,
                );
            }
        }

        // End frame in macroquad
        {
            self.mq_stage.ctx.end_render_pass();
            self.mq_stage.ctx.commit_frame();
        }
    }
}

//////////////////

impl VisRenderer {
    pub fn spawn_new(
        model_set: ModelSet,
        texture_set: TextureSet,
        shared_render_state: SharedVisRenderState,
    ) -> JoinHandle<()> {
        let mut conf = conf::Conf::default();
        conf.window_title = "RocketSim Vis".to_string();
        conf.platform.apple_gfx_api = conf::AppleGfxApi::OpenGl; // Apple devices should still use OpenGL
        conf.sample_count = 8; // Heavy MSAA

        std::thread::spawn(|| {
            miniquad::start(conf, move || {
                Box::new(VisRenderer::new(
                    model_set,
                    texture_set,
                    shared_render_state.clone(),
                ))
            });
        })
    }
}
