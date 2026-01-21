use miniquad::{Backend, RenderingBackend, ShaderId, ShaderMeta, ShaderSource};
use crate::vis::backend::Uniforms;

#[derive(Debug, Clone)]
pub struct ShaderCode {
    vertex_src: String,
    fragment_src: String,
}

impl ShaderCode {
    pub fn new(vertex_src: &str, fragment_src: &str) -> Self {
        Self {
            vertex_src: vertex_src.to_string(),
            fragment_src: fragment_src.to_string(),
        }
    }

    pub fn build(&self, ctx: &mut Box<dyn RenderingBackend>) -> ShaderId {
        ctx.new_shader(
                match ctx.info().backend {
                    Backend::OpenGl => ShaderSource::Glsl {
                        vertex: &self.vertex_src,
                        fragment: &self.fragment_src,
                    },
                    Backend::Metal => unimplemented!(),
                },
                ShaderMeta {
                    images: vec!["u_texture".to_string()],
                    uniforms: Uniforms::get_shader_meta_layout(),
                },
            )
            .unwrap()
    }
}