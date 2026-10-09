//! Stage 4: indexed arena draw, unlit flat color.
//!
//! One welded position VBO + one `u32` IBO (`STATIC_DRAW` — the arena never
//! changes for a mode), flat program (`u_mvp` + `u_color`, both cached at
//! init). No face culling: physics-hull winding is inconsistent, so both
//! faces draw. Textures/normals arrive with the textured pipeline (stage 5).

use glam::Mat4;
use glow::HasContext;

use crate::{
    gl_util::{bytes_of, compile_program, uniform_location},
    mesh::ArenaMesh,
};

/// Vertex shader: pinned `layout(location = 0)` (VAO hardcodes 0; test-pinned).
pub const FLAT_VERT_SRC: &str = "#version 330 core
layout(location = 0) in vec3 vs_pos;
uniform mat4 u_mvp;
void main() {
    gl_Position = u_mvp * vec4(vs_pos, 1.0);
}
";

/// Fragment shader: solid `u_color`.
pub const FLAT_FRAG_SRC: &str = "#version 330 core
uniform vec4 u_color;
out vec4 out_color;
void main() {
    out_color = u_color;
}
";

/// Flat slate colors: shell walls, lighter playing field, darker backdrop.
/// All unlit — the tones exist so the bowl reads as a stadium, not a void.
pub const SHELL_COLOR: [f32; 4] = [0.42, 0.47, 0.58, 1.0];
/// Flat playing-surface tone (the procedural field cap).
pub const FIELD_COLOR: [f32; 4] = [0.50, 0.55, 0.66, 1.0];
/// Flat dark backdrop for the procedural planes.
pub const PLANES_COLOR: [f32; 4] = [0.20, 0.24, 0.32, 1.0];

/// One uploaded index pool (VAO + position VBO + `u32` IBO).
struct PartGl {
    vao: glow::VertexArray,
    vbo_pos: glow::Buffer,
    ibo: glow::Buffer,
    n_indices: i32,
}

impl PartGl {
    fn new(gl: &glow::Context, verts: &[glam::Vec3], indices: &[u32]) -> Self {
        assert_eq!(indices.len() % 3, 0);
        let (vao, vbo_pos, ibo) = unsafe {
            let vao = gl.create_vertex_array().expect("GL create part VAO");
            gl.bind_vertex_array(Some(vao));
            let vbo_pos = gl.create_buffer().expect("GL create part VBO");
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo_pos));
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes_of(verts), glow::STATIC_DRAW);
            gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, 0, 0);
            gl.enable_vertex_attrib_array(0);
            let ibo = gl.create_buffer().expect("GL create part IBO");
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(ibo));
            gl.buffer_data_u8_slice(
                glow::ELEMENT_ARRAY_BUFFER,
                bytes_of(indices),
                glow::STATIC_DRAW,
            );
            gl.bind_vertex_array(None);
            (vao, vbo_pos, ibo)
        };
        Self {
            vao,
            vbo_pos,
            ibo,
            n_indices: indices.len() as i32,
        }
    }

    fn destroy(&self, gl: &glow::Context) {
        unsafe {
            gl.delete_buffer(self.vbo_pos);
            gl.delete_buffer(self.ibo);
            gl.delete_vertex_array(self.vao);
        }
    }
}

/// Owned GL objects for the arena: open bowl (shell) + field cap + low
/// backdrop planes. The ceiling lid + backdrop ceiling stay CPU-side — the
/// stage-4 cutaway hides them; stage 8 (interior cameras) adds their draws.
pub struct ArenaGl {
    prog: glow::Program,
    u_mvp: glow::UniformLocation,
    u_color: glow::UniformLocation,
    shell: PartGl,
    field: PartGl,
    planes_low: PartGl,
}

impl ArenaGl {
    /// Uploads shell + field + low planes (positions + indices per part);
    /// panics with logs on failure. Empty parts (TheVoid) build valid objects.
    pub fn new(gl: &glow::Context, mesh: &ArenaMesh) -> Self {
        let prog = compile_program(gl, "arena_flat", FLAT_VERT_SRC, FLAT_FRAG_SRC);
        let u_mvp = uniform_location(gl, prog, "arena_flat", "u_mvp");
        let u_color = uniform_location(gl, prog, "arena_flat", "u_color");
        Self {
            prog,
            u_mvp,
            u_color,
            shell: PartGl::new(gl, &mesh.shell.verts, &mesh.shell.indices),
            field: PartGl::new(gl, &mesh.field.verts, &mesh.field.indices),
            planes_low: PartGl::new(gl, &mesh.planes_low.verts, &mesh.planes_low.indices),
        }
    }

    /// Draws shell then planes (two flat tones). Viewport in GL pixels.
    ///
    /// Resets ALL relevant GL state first: the egui painter shares this
    /// context and leaves its own state behind. v1 never reset state here and
    /// chased ghosts for it — every draw owns its state, no exceptions.
    /// The list is load-bearing: a missing `depth_mask(true)` once made the
    /// backdrop paint over the arena (last-drawn-wins with no depth writes).
    pub fn draw(&self, gl: &glow::Context, viewport_px: (i32, i32, i32, i32), mvp: &Mat4) {
        unsafe {
            gl.disable(glow::CULL_FACE);
            gl.disable(glow::SCISSOR_TEST);
            gl.disable(glow::BLEND);
            gl.depth_mask(true);
            gl.viewport(viewport_px.0, viewport_px.1, viewport_px.2, viewport_px.3);
            gl.clear_color(0.0, 0.0, 0.0, 1.0);
            gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LEQUAL);
            gl.use_program(Some(self.prog));
            gl.uniform_matrix_4_f32_slice(Some(&self.u_mvp), false, &mvp.to_cols_array());
            // Order is irrelevant with a working depth buffer (front-to-back
            // for early-z); the one time order mattered, eframe had given us
            // a context with ZERO depth bits (see main) — load-bearing lesson.
            for (part, color) in [
                (&self.shell, SHELL_COLOR),
                (&self.field, FIELD_COLOR),
                (&self.planes_low, PLANES_COLOR),
            ] {
                gl.uniform_4_f32(Some(&self.u_color), color[0], color[1], color[2], color[3]);
                gl.bind_vertex_array(Some(part.vao));
                gl.draw_elements(glow::TRIANGLES, part.n_indices, glow::UNSIGNED_INT, 0);
            }
            gl.bind_vertex_array(None);
            gl.use_program(None);
        }
    }

    /// Deletes every GL object. Call on the GL thread before dropping.
    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            self.shell.destroy(gl);
            self.field.destroy(gl);
            self.planes_low.destroy(gl);
            gl.delete_program(self.prog);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_shader_pins_location_0() {
        assert!(FLAT_VERT_SRC.contains("layout(location = 0)"));
        assert!(FLAT_VERT_SRC.contains("uniform mat4 u_mvp"));
        assert!(FLAT_FRAG_SRC.contains("uniform vec4 u_color"));
    }
}
