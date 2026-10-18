//! Instanced boost-pad pipeline: 4 draws cover all pads.
//!
//! Model matrix comes from per-instance `mat4` attributes (locations 3-6)
//! instead of `u_model`; fragment shader is `main_frag.glsl`.
//! Instance buffers stream per frame; empty draws skip.

use glam::Mat4;
use glow::HasContext;

use crate::{
    gl_util::{bytes_of, compile_program, uniform_location},
    main_gl::MAIN_FRAG_SRC,
    obj::ObjMesh,
    pads::MAX_PAD_INSTANCES,
    textures::TextureGl,
};

/// Instanced model-matrix vertex shader (locations mirror main_vert 0/1/2,
/// instance columns pinned 3-6 — the VAO hardcodes them, test-pinned below).
pub const PAD_VERT_SRC: &str = "#version 330 core

layout (location = 0) in vec3 vs_vert;
layout (location = 1) in vec3 vs_vert_normal;
layout (location = 2) in vec2 vs_vert_uv;
layout (location = 3) in vec4 vs_pad_m0;
layout (location = 4) in vec4 vs_pad_m1;
layout (location = 5) in vec4 vs_pad_m2;
layout (location = 6) in vec4 vs_pad_m3;

out vec2 fs_uv;
out vec3 fs_normal;

uniform mat4 u_view;
uniform mat4 u_proj;

void main() {
    mat4 inst = mat4(vs_pad_m0, vs_pad_m1, vs_pad_m2, vs_pad_m3);
    fs_uv = vs_vert_uv;
    fs_normal = normalize(mat3(inst) * vs_vert_normal);
    gl_Position = u_proj * u_view * inst * vec4(vs_vert, 1.0);
}
";

/// Boost-pad texture asset name (one texture for all four variants).
pub const PAD_TEXTURE: &str = "boost_pad";

/// White texture bytes for pads? No — pads sample the boost_pad PNG.
pub const PAD_PNG: &[u8] = include_bytes!("textures/boost_pad.png");

/// Bytes per instance matrix (mat4 of f32).
pub const INSTANCE_BYTES: usize = 64;

/// One pad variant: static mesh VAO + streaming instance buffer.
struct VariantGl {
    vao: glow::VertexArray,
    mesh_vbos: [glow::Buffer; 3],
    ibo: glow::Buffer,
    inst_vbo: glow::Buffer,
    n_indices: i32,
    n_instances: i32,
}

impl VariantGl {
    fn new(gl: &glow::Context, mesh: &ObjMesh) -> Self {
        assert_eq!(mesh.indices.len() % 3, 0);
        let (vao, mesh_vbos, ibo, inst_vbo) = unsafe {
            let vao = gl.create_vertex_array().expect("GL create pad VAO");
            gl.bind_vertex_array(Some(vao));
            let mesh_vbos = [
                gl.create_buffer().expect("GL create pad VBO"),
                gl.create_buffer().expect("GL create pad VBO"),
                gl.create_buffer().expect("GL create pad VBO"),
            ];
            let uploads: [&[u8]; 3] = [
                bytes_of(&mesh.verts),
                bytes_of(&mesh.normals),
                bytes_of(&mesh.uvs),
            ];
            for (i, (buf, bytes)) in mesh_vbos.iter().zip(uploads.iter()).enumerate() {
                gl.bind_buffer(glow::ARRAY_BUFFER, Some(*buf));
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes, glow::STATIC_DRAW);
                let size = if i == 2 { 2 } else { 3 };
                gl.vertex_attrib_pointer_f32(i as u32, size, glow::FLOAT, false, 0, 0);
                gl.enable_vertex_attrib_array(i as u32);
            }
            // Instance mat4 stream: 4 vec4 attribs (locs 3-6), divisor 1.
            let inst_vbo = gl.create_buffer().expect("GL create pad instance VBO");
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(inst_vbo));
            gl.buffer_data_u8_slice(
                glow::ARRAY_BUFFER,
                &[0u8; MAX_PAD_INSTANCES * INSTANCE_BYTES],
                glow::DYNAMIC_DRAW,
            );
            for col in 0..4 {
                let loc = 3 + col as u32;
                gl.vertex_attrib_pointer_f32(
                    loc,
                    4,
                    glow::FLOAT,
                    false,
                    INSTANCE_BYTES as i32,
                    col * 16,
                );
                gl.enable_vertex_attrib_array(loc);
                gl.vertex_attrib_divisor(loc, 1);
            }
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            let ibo = gl.create_buffer().expect("GL create pad IBO");
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(ibo));
            gl.buffer_data_u8_slice(
                glow::ELEMENT_ARRAY_BUFFER,
                bytes_of(&mesh.indices),
                glow::STATIC_DRAW,
            );
            gl.bind_vertex_array(None);
            (vao, mesh_vbos, ibo, inst_vbo)
        };
        Self {
            vao,
            mesh_vbos,
            ibo,
            inst_vbo,
            n_indices: mesh.indices.len() as i32,
            n_instances: 0,
        }
    }

    /// Streams instance matrices (truncates past the buffer — with the
    /// 34-pad cap the caller never exceeds it; truncation is still loud).
    /// Empty groups skip the upload (stale buffer contents are harmless:
    /// zero instances draw nothing) — a hidden pad layer costs no GL calls.
    fn upload(&mut self, gl: &glow::Context, matrices: &[Mat4]) {
        self.n_instances = matrices.len() as i32;
        if matrices.is_empty() {
            return;
        }
        assert!(
            matrices.len() <= MAX_PAD_INSTANCES,
            "pad instances exceed buffer: {}",
            matrices.len()
        );
        unsafe {
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.inst_vbo));
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes_of(matrices), glow::DYNAMIC_DRAW);
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
        }
    }

    fn destroy(&self, gl: &glow::Context) {
        unsafe {
            for b in self.mesh_vbos {
                gl.delete_buffer(b);
            }
            gl.delete_buffer(self.inst_vbo);
            gl.delete_buffer(self.ibo);
            gl.delete_vertex_array(self.vao);
        }
    }
}

/// Owned pad pipeline: program + 4 variant draws + shared texture.
pub struct PadsGl {
    prog: glow::Program,
    u_view: glow::UniformLocation,
    u_proj: glow::UniformLocation,
    texture: TextureGl,
    variants: [VariantGl; 4],
}

impl PadsGl {
    /// Compiles the instanced program and uploads the four variant meshes.
    pub fn new(gl: &glow::Context, meshes: &[ObjMesh; 4]) -> Self {
        let prog = compile_program(gl, "pads", PAD_VERT_SRC, MAIN_FRAG_SRC);
        let u_view = uniform_location(gl, prog, "pads", "u_view");
        let u_proj = uniform_location(gl, prog, "pads", "u_proj");
        unsafe {
            gl.use_program(Some(prog));
            gl.uniform_1_i32(gl.get_uniform_location(prog, "u_texture").as_ref(), 0);
            gl.use_program(None);
        }
        let texture = TextureGl::new(gl, &crate::textures::PngImage::decode(PAD_PNG));
        let variants = [
            VariantGl::new(gl, &meshes[0]),
            VariantGl::new(gl, &meshes[1]),
            VariantGl::new(gl, &meshes[2]),
            VariantGl::new(gl, &meshes[3]),
        ];
        Self {
            prog,
            u_view,
            u_proj,
            texture,
            variants,
        }
    }

    /// Streams per-variant matrices (see [`crate::pads::build_instances`]).
    pub fn upload(&mut self, gl: &glow::Context, instances: &[Vec<Mat4>; 4]) {
        for (variant, matrices) in self.variants.iter_mut().zip(instances.iter()) {
            variant.upload(gl, matrices);
        }
    }

    /// Draws non-empty variants with (`view`, `proj`). State reset matches
    /// the arena pass (shared egui context — every draw owns its state).
    /// With zero instances everywhere (pads hidden in settings) this issues
    /// no GL calls at all.
    pub fn draw(&self, gl: &glow::Context, view: &Mat4, proj: &Mat4) {
        if self.variants.iter().all(|v| v.n_instances == 0) {
            return;
        }
        unsafe {
            gl.disable(glow::CULL_FACE);
            gl.disable(glow::SCISSOR_TEST);
            gl.enable(glow::BLEND);
            // Keep the opaque background's alpha for window capture.
            gl.blend_func_separate(
                glow::SRC_ALPHA,
                glow::ONE_MINUS_SRC_ALPHA,
                glow::ONE,
                glow::ONE_MINUS_SRC_ALPHA,
            );
            gl.depth_mask(false);
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LEQUAL);
            gl.use_program(Some(self.prog));
            gl.uniform_matrix_4_f32_slice(Some(&self.u_view), false, &view.to_cols_array());
            gl.uniform_matrix_4_f32_slice(Some(&self.u_proj), false, &proj.to_cols_array());
            self.texture.bind(gl);
            for variant in &self.variants {
                if variant.n_instances == 0 {
                    continue;
                }
                gl.bind_vertex_array(Some(variant.vao));
                gl.draw_elements_instanced(
                    glow::TRIANGLES,
                    variant.n_indices,
                    glow::UNSIGNED_INT,
                    0,
                    variant.n_instances,
                );
            }
            gl.bind_vertex_array(None);
            gl.depth_mask(true);
            gl.disable(glow::BLEND);
            gl.use_program(None);
        }
    }

    /// Deletes every GL object. Call on the GL thread before dropping.
    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            for variant in &self.variants {
                variant.destroy(gl);
            }
            self.texture.destroy(gl);
            gl.delete_program(self.prog);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pad_shader_pins_locations() {
        // Instance mat4 must sit on 3-6 (divisor 1 each); mesh mirrors main.
        for loc in ["location = 0", "location = 1", "location = 2"] {
            assert!(PAD_VERT_SRC.contains(loc), "missing {loc}");
        }
        for loc in [
            "location = 3",
            "location = 4",
            "location = 5",
            "location = 6",
        ] {
            assert!(PAD_VERT_SRC.contains(loc), "missing instance {loc}");
        }
        assert!(PAD_VERT_SRC.contains("uniform mat4 u_view"));
        assert!(PAD_VERT_SRC.contains("uniform mat4 u_proj"));
        assert!(
            !PAD_VERT_SRC.contains("u_model"),
            "instancing must not use u_model"
        );
    }

    #[test]
    fn instance_stride() {
        assert_eq!(INSTANCE_BYTES, std::mem::size_of::<Mat4>());
        assert_eq!(MAX_PAD_INSTANCES * INSTANCE_BYTES, 34 * 64);
    }
}
