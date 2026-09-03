use std::{sync::Mutex, thread::JoinHandle};

use egui::{Align, Align2, Pos2, Rect};
use egui_miniquad::EguiMq;
use glam::{
    Mat4, Vec2, Vec3, Vec4,
    camera::lh::{proj::directx, view::look_at_mat4},
};
use miniquad::{
    Bindings, BlendFactor, BlendState, BlendValue, BufferId, BufferLayout, BufferSource,
    BufferType, BufferUsage, Equation, EventHandler, KeyCode, KeyMods, MouseButton, Pipeline,
    PipelineParams, PrimitiveType, RawId, RenderingBackend, TextureId, UniformBlockLayout,
    UniformDesc, UniformType, UniformsSource, VertexAttribute, VertexFormat, VertexStep, conf,
    window,
};
use rustc_hash::FxHashMap;

use crate::backend::{
    Color, Elem2D, InfoLineType, ModelSet, ShaderSrc, ShaderSrcType, SharedVisRenderState,
    SharedWindowEvents, TextureSet, WindowEvent, WindowEventQueue,
};

#[repr(C)]
pub struct Uniforms {
    pub u_model: Mat4,
    pub u_view: Mat4,
    pub u_proj: Mat4,
    pub u_screen_size: Vec2,
}
impl Uniforms {
    pub fn get_shader_meta_layout() -> UniformBlockLayout {
        UniformBlockLayout {
            uniforms: vec![
                UniformDesc::new("u_model", UniformType::Mat4),
                UniformDesc::new("u_view", UniformType::Mat4),
                UniformDesc::new("u_proj", UniformType::Mat4),
                UniformDesc::new("u_screen_size", UniformType::Float2),
            ],
        }
    }
}

impl Uniforms {
    pub fn new_default(screen_size: Vec2) -> Self {
        Self {
            u_model: Mat4::IDENTITY,
            u_view: Mat4::IDENTITY,
            u_proj: Mat4::IDENTITY,
            u_screen_size: screen_size,
        }
    }
}

struct LineBuffers {
    pos_ab: [BufferId; 2],
    color_ab: [BufferId; 2],
    width_ab: [BufferId; 2],
}

impl LineBuffers {
    fn new(ctx: &mut Box<dyn RenderingBackend>, max_points: usize) -> Self {
        let mut pos_ab = Vec::new();
        let mut color_ab = Vec::new();
        let mut width_ab = Vec::new();

        for _ in 0..2 {
            pos_ab.push(ctx.new_buffer(
                BufferType::VertexBuffer,
                BufferUsage::Stream,
                BufferSource::slice(&vec![Vec3::ZERO; max_points]),
            ));
            color_ab.push(ctx.new_buffer(
                BufferType::VertexBuffer,
                BufferUsage::Stream,
                BufferSource::slice(&vec![0.0f32; max_points]),
            ));
            width_ab.push(ctx.new_buffer(
                BufferType::VertexBuffer,
                BufferUsage::Stream,
                BufferSource::slice(&vec![Vec4::ZERO; max_points]),
            ));
        }

        Self {
            pos_ab: pos_ab.try_into().unwrap(),
            color_ab: color_ab.try_into().unwrap(),
            width_ab: width_ab.try_into().unwrap(),
        }
    }

    fn update(
        &mut self,
        ctx: &mut Box<dyn RenderingBackend>,
        pos_set: &[Vec3],
        color_set: &[Color],
        width_set: &[f32],
    ) {
        let all_len = pos_set.len();
        assert_eq!(all_len, color_set.len());
        assert_eq!(all_len, width_set.len());
        assert!(all_len > 0);

        // Ensures proper field ordering
        let color_array_set: Vec<[f32; 4]> =
            color_set.iter().map(|color| color.to_array()).collect();

        fn split_even_odd<T: Clone>(slice: &[T]) -> [Vec<T>; 2] {
            assert_eq!(slice.len() % 2, 0, "Must have even length");
            let even: Vec<T> = slice.iter().step_by(2).cloned().collect();
            let odd: Vec<T> = slice.iter().skip(1).step_by(2).cloned().collect();
            [even, odd]
        }

        let pos_pair = split_even_odd(pos_set);
        let color_pair = split_even_odd(&color_array_set);
        let width_pair = split_even_odd(width_set);
        for i in 0..2 {
            ctx.buffer_update(self.pos_ab[i], BufferSource::slice(&pos_pair[i]));
            ctx.buffer_update(self.color_ab[i], BufferSource::slice(&color_pair[i]));
            ctx.buffer_update(self.width_ab[i], BufferSource::slice(&width_pair[i]));
        }
    }
}

pub struct VisRenderer {
    models: ModelSet,
    textures: TextureSet,

    mesh_pipelines: FxHashMap<String, Pipeline>,
    mesh_bindings: Bindings,
    mesh_texture_ids: Vec<TextureId>,

    line_pipeline: Pipeline,
    line_bindings: Bindings,
    line_buffers: LineBuffers,

    ctx: Box<dyn RenderingBackend>,

    cur_uniforms: Uniforms,
    cur_window_size: Vec2,

    shared_render_state: SharedVisRenderState,
    window_event_queue: SharedWindowEvents,

    egui_mq: EguiMq,
}

impl VisRenderer {
    pub const MAX_LINES: usize = 1024;
    pub const MAX_LINE_POINTS: usize = Self::MAX_LINES * 2;

    fn new(
        models: ModelSet,
        textures: TextureSet,
        shared_render_state: SharedVisRenderState,
        shader_srcs: Vec<(String, ShaderSrc)>,
        window_event_queue: SharedWindowEvents,
    ) -> VisRenderer {
        let mut ctx: Box<dyn RenderingBackend> = window::new_rendering_backend();

        let mut mesh_shaders = FxHashMap::default();
        let mut line_shaders = FxHashMap::default();
        for (shader_name, shader_src) in shader_srcs {
            match shader_src.src_type() {
                ShaderSrcType::Model => {
                    mesh_shaders.insert(shader_name, shader_src.build(&mut ctx))
                }
                ShaderSrcType::Line => line_shaders.insert(shader_name, shader_src.build(&mut ctx)),
            };
        }

        assert!(!line_shaders.is_empty(), "Missing line shaders");
        assert!(line_shaders.len() < 2, "Cannot have multiple line shaders");

        let color_blend = Some(BlendState::new(
            Equation::Add,
            BlendFactor::Value(BlendValue::SourceAlpha),
            BlendFactor::OneMinusValue(BlendValue::SourceAlpha),
        ));

        let (mesh_pipelines, mesh_bindings, mesh_texture_ids) = {
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

            //////////

            let texture_ids = textures.build_mq_textures(&mut ctx);

            let bindings = Bindings {
                vertex_buffers,
                index_buffer,
                images: vec![TextureId::from_raw_id(RawId::OpenGl(0))],
            };

            let mut pipelines = FxHashMap::default();
            for (shader_name, shader_id) in mesh_shaders {
                let params = PipelineParams {
                    depth_test: miniquad::Comparison::LessOrEqual,
                    depth_write: true,
                    cull_face: miniquad::CullFace::Back,
                    primitive_type: PrimitiveType::Triangles,
                    color_blend,
                    ..Default::default()
                };

                let pipeline = ctx.new_pipeline(&buffer_layout, &attributes, shader_id, params);
                pipelines.insert(shader_name, pipeline);
            }
            (pipelines, bindings, texture_ids)
        };

        let (line_pipeline, line_bindings, line_buffers) = {
            let vertices = [
                Vec3::new(-1.0, -1.0, 0.0),
                Vec3::new(1.0, -1.0, 0.0),
                Vec3::new(-1.0, 1.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
            ];
            let vertex_buffer = ctx.new_buffer(
                BufferType::VertexBuffer,
                BufferUsage::Immutable,
                BufferSource::slice(&vertices),
            );

            let indices = [0, 1, 2, 1, 3, 2];
            let index_buffer = ctx.new_buffer(
                BufferType::IndexBuffer,
                BufferUsage::Immutable,
                BufferSource::slice(&indices),
            );

            let line_buffers = LineBuffers::new(&mut ctx, Self::MAX_LINE_POINTS);

            let attributes = [
                VertexAttribute::with_buffer("vs_vert", VertexFormat::Float3, 0),
                VertexAttribute::with_buffer("vs_line_pos_a", VertexFormat::Float3, 1),
                VertexAttribute::with_buffer("vs_line_pos_b", VertexFormat::Float3, 2),
                VertexAttribute::with_buffer("vs_line_color_a", VertexFormat::Float4, 3),
                VertexAttribute::with_buffer("vs_line_color_b", VertexFormat::Float4, 4),
                VertexAttribute::with_buffer("vs_line_width_a", VertexFormat::Float1, 5),
                VertexAttribute::with_buffer("vs_line_width_b", VertexFormat::Float1, 6),
            ];

            //////////

            let shader_id = *line_shaders.values().nth(0).unwrap();

            let bindings = Bindings {
                vertex_buffers: vec![
                    vertex_buffer,
                    line_buffers.pos_ab[0],
                    line_buffers.pos_ab[1],
                    line_buffers.color_ab[0],
                    line_buffers.color_ab[1],
                    line_buffers.width_ab[0],
                    line_buffers.width_ab[1],
                ],
                index_buffer,
                images: vec![],
            };

            let params = PipelineParams {
                depth_test: miniquad::Comparison::LessOrEqual,
                depth_write: true,
                cull_face: miniquad::CullFace::Nothing,
                primitive_type: PrimitiveType::Triangles,
                color_blend,
                ..Default::default()
            };

            let instance_buffer_layout = BufferLayout {
                step_func: VertexStep::PerInstance, // Need this in order to instance the lines
                ..Default::default()
            };

            let buffer_layouts = [
                BufferLayout::default(),
                instance_buffer_layout.clone(),
                instance_buffer_layout.clone(),
                instance_buffer_layout.clone(),
                instance_buffer_layout.clone(),
                instance_buffer_layout.clone(),
                instance_buffer_layout.clone(),
            ];
            let pipeline = ctx.new_pipeline(&buffer_layouts, &attributes, shader_id, params);
            (pipeline, bindings, line_buffers)
        };

        let screen_size = Vec2::from(window::screen_size());
        let egui_mq = EguiMq::new(ctx.as_mut());
        VisRenderer {
            models,
            textures,

            mesh_pipelines,
            mesh_bindings,
            mesh_texture_ids,

            line_pipeline,
            line_bindings,
            line_buffers,

            ctx,

            cur_uniforms: Uniforms::new_default(screen_size),
            cur_window_size: Vec2::from(window::screen_size()),

            shared_render_state,
            window_event_queue,

            egui_mq,
        }
    }

    pub fn apply_uniforms(&mut self) {
        self.ctx
            .apply_uniforms(UniformsSource::table(&self.cur_uniforms));
    }

    pub fn set_mesh_texture(&mut self, texture_idx: Option<usize>) {
        if let Some(texture_idx) = texture_idx {
            self.mesh_bindings.images = vec![self.mesh_texture_ids[texture_idx]];
            self.ctx.apply_bindings(&self.mesh_bindings);
        } else {
            self.mesh_bindings.images = vec![TextureId::from_raw_id(RawId::OpenGl(0))];
            self.ctx.apply_bindings(&self.mesh_bindings);
        }
    }

    pub fn set_pipeline(&mut self, name: Option<&str>) {
        self.ctx
            .apply_pipeline(&self.mesh_pipelines[name.unwrap_or("main")]);
    }
}

impl EventHandler for VisRenderer {
    fn update(&mut self) {}

    fn draw(&mut self) {
        // Begin frame in macroquad
        {
            self.ctx
                .begin_default_pass(miniquad::PassAction::clear_color(0.0, 0.0, 0.0, 1.0));
            self.set_pipeline(None);
            self.ctx.apply_bindings(&self.mesh_bindings);
        }

        fn calc_camera_view_proj(
            pos: Vec3,
            look_target: Vec3,
            fov_degrees: f32,
            cur_window_size: Vec2,
        ) -> (Mat4, Mat4) {
            let view = look_at_mat4(pos, look_target, Vec3::Z);

            let aspect = cur_window_size.x / cur_window_size.y;

            const Z_RANGE_NEAR: f32 = 5.0;
            const Z_RANGE_FAR: f32 = 50_000.0;

            let proj =
                directx::perspective(fov_degrees.to_radians(), aspect, Z_RANGE_NEAR, Z_RANGE_FAR);

            (view, proj)
        }

        let state = self.shared_render_state.read().unwrap().clone();

        // Update camera
        {
            let (u_view, u_proj) = calc_camera_view_proj(
                state.camera_pos.to_vec3(),
                state.camera_look_target.to_vec3(),
                state.camera_fov_deg,
                self.cur_window_size,
            );
            self.cur_uniforms.u_view = u_view;
            self.cur_uniforms.u_proj = u_proj;
        }

        // Render objects
        {
            for render_obj in state.objects {
                if let Some(pipeline_name) = render_obj.pipeline_name {
                    self.set_pipeline(Some(&pipeline_name));
                } else {
                    self.set_pipeline(None);
                };

                let model_mat4 = Mat4::from_mat3_translation(
                    render_obj.model_rot_mat.into(),
                    render_obj.model_pos.into(),
                );

                self.cur_uniforms.u_model = model_mat4;
                self.cur_uniforms.u_model.y_axis *= -1.0; // Flip to left-handed

                if let Some(texture_name) = render_obj.texture_name {
                    self.set_mesh_texture(Some(self.textures.get_texture_idx(&texture_name)))
                } else {
                    self.set_mesh_texture(None);
                }

                let (draw_range_start, draw_range_end) =
                    self.models.get_model_draw_range(&render_obj.model_name);
                assert!(draw_range_start < draw_range_end);

                self.apply_uniforms();
                self.ctx.draw(
                    draw_range_start as i32,
                    (draw_range_end - draw_range_start) as i32,
                    1,
                );
            }
        }

        // Render lines
        {
            self.ctx.apply_pipeline(&self.line_pipeline);
            self.ctx.apply_bindings(&self.line_bindings);
            let mut num_lines = 0;
            if !state.lines.is_empty() {
                let mut pos_set = Vec::new();
                let mut color_set = Vec::new();
                let mut width_set = Vec::new();
                for render_line in state.lines {
                    for i in 1..render_line.len() {
                        let prev_point = &render_line[i - 1];
                        let cur_point = &render_line[i];
                        pos_set.push(prev_point.pos);
                        pos_set.push(cur_point.pos);

                        color_set.push(prev_point.color);
                        color_set.push(cur_point.color);

                        width_set.push(prev_point.width);
                        width_set.push(cur_point.width);

                        num_lines += 1;
                    }
                }

                // Map from Vec3A to Vec3
                let pos_set = pos_set.into_iter().map(|v| v.to_vec3()).collect::<Vec<_>>();

                self.line_buffers
                    .update(&mut self.ctx, &pos_set, &color_set, &width_set);

                self.apply_uniforms();
                self.ctx.draw(0, 6, num_lines);
            }
        }

        // Render EGUI
        {
            self.egui_mq.run(self.ctx.as_mut(), |_backend, ctx| {
                egui::Area::new(egui::Id::new("TextArea"))
                    .fixed_pos(egui::pos2(20.0, 20.0))
                    .show(ctx, |ui| {
                        ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);

                        // Draw debug lines
                        egui::Frame::canvas(ui.style())
                            .fill(egui::Color32::from_rgba_premultiplied(0, 0, 0, 100))
                            .inner_margin(8.0)
                            .show(ui, |ui| {
                                let title_text = egui::RichText::new(format!(
                                    "RocketSim Vis [{}]",
                                    env!("CARGO_PKG_VERSION")
                                ));
                                ui.colored_label(egui::Color32::LIGHT_BLUE, title_text);

                                for (line_type, line) in state.info_text_lines.clone() {
                                    let color = match line_type {
                                        InfoLineType::Normal => egui::Color32::WHITE,
                                        InfoLineType::Dim => egui::Color32::GRAY,
                                    };
                                    ui.colored_label(color, line);
                                }
                            });

                        let painter = ui.painter();
                        for shape in state.shapes_2d.clone() {
                            let total_area_size_x = painter.clip_rect().size().x;
                            let total_area_size_y = painter.clip_rect().size().y;
                            let fn_transform_pos = |pos: Vec2| {
                                pos / 2.0 * total_area_size_y
                                    + Vec2::new(total_area_size_x, total_area_size_y) / 2.0
                            };

                            let fn_transform_size = |size: f32| size / 2.0 * total_area_size_y;

                            match shape {
                                Elem2D::Circle { pos, radius, color } => {
                                    let pos = fn_transform_pos(pos);
                                    let radius = fn_transform_size(radius);
                                    painter.circle_filled(
                                        Pos2::new(pos.x, pos.y),
                                        radius,
                                        color.to_egui(),
                                    );
                                }
                                Elem2D::Rect {
                                    a,
                                    b,
                                    color,
                                    rounding,
                                } => {
                                    let a = fn_transform_pos(a);
                                    let b = fn_transform_pos(b);
                                    let rounding = fn_transform_size(rounding);
                                    painter.rect_filled(
                                        Rect::from_two_pos(
                                            Pos2::new(a.x, a.y),
                                            Pos2::new(b.x, b.y),
                                        ),
                                        rounding,
                                        color.to_egui(),
                                    );
                                }
                                Elem2D::Text {
                                    pos,
                                    centered,
                                    string,
                                    color,
                                } => {
                                    let pos = fn_transform_pos(pos);
                                    painter.text(
                                        Pos2::new(pos.x, pos.y),
                                        if centered {
                                            Align2([Align::Center, Align::Center])
                                        } else {
                                            Align2([Align::Min, Align::Min])
                                        },
                                        string,
                                        egui::FontId::proportional(14.0),
                                        color.to_egui(),
                                    );
                                }
                            }
                        }
                    });
            });
            self.egui_mq.draw(self.ctx.as_mut());
        }

        // End frame in macroquad
        {
            self.ctx.end_render_pass();
            self.ctx.commit_frame();
        }
    }

    fn resize_event(&mut self, width: f32, height: f32) {
        self.cur_window_size = glam::vec2(width, height);
        self.cur_uniforms.u_screen_size = self.cur_window_size;
        self.window_event_queue
            .lock()
            .unwrap()
            .push(WindowEvent::Resize { width, height })
    }

    fn mouse_button_down_event(&mut self, _button: MouseButton, _x: f32, _y: f32) {
        self.window_event_queue
            .lock()
            .unwrap()
            .push(WindowEvent::MouseButtonDown { button: _button })
    }

    fn key_down_event(&mut self, _keycode: KeyCode, _keymods: KeyMods, _repeat: bool) {
        self.window_event_queue
            .lock()
            .unwrap()
            .push(WindowEvent::KeyDown { key: _keycode })
    }
}

//////////////////

impl VisRenderer {
    pub fn spawn_new(
        window_title: &str,
        model_set: ModelSet,
        texture_set: TextureSet,
        shared_render_state: SharedVisRenderState,
        shader_srcs: Vec<(&str, ShaderSrc)>,
    ) -> (JoinHandle<()>, SharedWindowEvents) {
        let mut conf = conf::Conf {
            window_title: window_title.to_string(),
            window_width: 1280,
            window_height: 720,
            sample_count: 8, // Heavy MSAA
            ..Default::default()
        };

        conf.platform.apple_gfx_api = conf::AppleGfxApi::OpenGl; // Apple devices should still use OpenGL

        let shader_srcs_strings: Vec<(String, ShaderSrc)> = shader_srcs
            .iter()
            .map(|(name, code)| (name.to_string(), code.clone()))
            .collect();

        let shared_window_events = SharedWindowEvents::new(Mutex::new(WindowEventQueue::new()));
        let shared_window_events_clone = shared_window_events.clone();
        let handle = std::thread::spawn(|| {
            miniquad::start(conf, move || {
                window::show_mouse(false);

                Box::new(VisRenderer::new(
                    model_set,
                    texture_set,
                    shared_render_state.clone(),
                    shader_srcs_strings,
                    shared_window_events_clone,
                ))
            });
        });

        (handle, shared_window_events)
    }
}
