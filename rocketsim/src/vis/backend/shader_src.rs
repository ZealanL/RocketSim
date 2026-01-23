use crate::vis::backend::Uniforms;
use miniquad::{Backend, RenderingBackend, ShaderId, ShaderMeta, ShaderSource};

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ShaderSrcType {
    Model,
    Line,
}

#[derive(Debug, Clone)]
pub struct ShaderSrc {
    src_type: ShaderSrcType,
    vertex_src: String,
    fragment_src: String,
}

impl ShaderSrc {
    pub fn new(src_type: ShaderSrcType, vertex_src: &str, fragment_src: &str) -> Self {
        Self {
            src_type,
            vertex_src: vertex_src.to_string(),
            fragment_src: fragment_src.to_string(),
        }
    }

    pub fn src_type(&self) -> ShaderSrcType {
        self.src_type
    }

    pub fn build(&self, ctx: &mut Box<dyn RenderingBackend>) -> ShaderId {
        let images = match self.src_type {
            ShaderSrcType::Model => {
                vec!["u_texture".to_string()]
            }
            ShaderSrcType::Line => Vec::new(),
        };

        let uniforms = Uniforms::get_shader_meta_layout();

        let shader_result = ctx.new_shader(
            match ctx.info().backend {
                Backend::OpenGl => ShaderSource::Glsl {
                    vertex: &self.vertex_src,
                    fragment: &self.fragment_src,
                },
                Backend::Metal => unimplemented!(),
            },
            ShaderMeta { images, uniforms },
        );

        shader_result.expect("Failed to build/compile shader")
    }
}
