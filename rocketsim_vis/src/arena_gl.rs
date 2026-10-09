//! Stage 4/5a: indexed arena draw with the OLD team-color/grid look.
//!
//! The shaders are the old `arena_vert/frag.glsl` bytes verbatim: team colors
//! by world-Y (blue +Y / orange −Y, gray mid, goal-aware), three octaves of
//! `fwidth`-antialiased grid lines, and normal-based brightness. Arena draws
//! sample the flat-white texture so the procedural colors pass through
//! untinted. Uniforms are cached at init; the sampler is bound to unit 0 once.

use glam::Mat4;
use glow::HasContext;

use crate::{
    gl_util::{bytes_of, compile_program, uniform_location},
    mesh::ArenaMesh,
    textures::{PngImage, TextureGl},
};

/// Old-look arena shaders (verbatim bytes, ZealanL).
pub const ARENA_VERT_SRC: &str = include_str!("shaders/arena_vert.glsl");
pub const ARENA_FRAG_SRC: &str = include_str!("shaders/arena_frag.glsl");

/// White texture bytes (flat — multiplies team colors neutrally).
pub const WHITE_PNG: &[u8] = include_bytes!("textures/white.png");

/// One uploaded index pool (position + normal + UV VBOs, `u32` IBO).
struct PartGl {
    vao: glow::VertexArray,
    vbos: [glow::Buffer; 3],
    ibo: glow::Buffer,
    n_indices: i32,
}

impl PartGl {
    fn new(
        gl: &glow::Context,
        verts: &[glam::Vec3],
        normals: &[glam::Vec3],
        uvs: &[glam::Vec2],
        indices: &[u32],
    ) -> Self {
        assert_eq!(indices.len() % 3, 0);
        assert_eq!(verts.len(), normals.len());
        assert_eq!(verts.len(), uvs.len());
        let (vao, vbos, ibo) = unsafe {
            let vao = gl.create_vertex_array().expect("GL create part VAO");
            gl.bind_vertex_array(Some(vao));
            let vbos = [
                gl.create_buffer().expect("GL create part VBO"),
                gl.create_buffer().expect("GL create part VBO"),
                gl.create_buffer().expect("GL create part VBO"),
            ];
            let uploads: [&[u8]; 3] = [bytes_of(verts), bytes_of(normals), bytes_of(uvs)];
            for (i, (buf, bytes)) in vbos.iter().zip(uploads.iter()).enumerate() {
                gl.bind_buffer(glow::ARRAY_BUFFER, Some(*buf));
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes, glow::STATIC_DRAW);
                let size = if i == 2 { 2 } else { 3 };
                gl.vertex_attrib_pointer_f32(i as u32, size, glow::FLOAT, false, 0, 0);
                gl.enable_vertex_attrib_array(i as u32);
            }
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            let ibo = gl.create_buffer().expect("GL create part IBO");
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(ibo));
            gl.buffer_data_u8_slice(
                glow::ELEMENT_ARRAY_BUFFER,
                bytes_of(indices),
                glow::STATIC_DRAW,
            );
            gl.bind_vertex_array(None);
            (vao, vbos, ibo)
        };
        Self {
            vao,
            vbos,
            ibo,
            n_indices: indices.len() as i32,
        }
    }

    fn destroy(&self, gl: &glow::Context) {
        unsafe {
            for b in self.vbos {
                gl.delete_buffer(b);
            }
            gl.delete_buffer(self.ibo);
            gl.delete_vertex_array(self.vao);
        }
    }
}

/// Owned GL objects for the arena: open bowl (shell) + field cap + low
/// backdrop planes, plus the ceiling lid + high backdrop for enclosed
/// (interior) views. The cutaway hides the roof; interior cameras need it.
pub struct ArenaGl {
    prog: glow::Program,
    u_model: glow::UniformLocation,
    u_view: glow::UniformLocation,
    u_proj: glow::UniformLocation,
    white: TextureGl,
    shell: PartGl,
    field: PartGl,
    planes_low: PartGl,
    lid: PartGl,
    planes_high: PartGl,
}

impl ArenaGl {
    /// Uploads all five parts; panics with logs on failure.
    /// Empty parts (TheVoid) build valid objects.
    pub fn new(gl: &glow::Context, mesh: &ArenaMesh) -> Self {
        let prog = compile_program(gl, "arena", ARENA_VERT_SRC, ARENA_FRAG_SRC);
        let u_model = uniform_location(gl, prog, "arena", "u_model");
        let u_view = uniform_location(gl, prog, "arena", "u_view");
        let u_proj = uniform_location(gl, prog, "arena", "u_proj");
        unsafe {
            gl.use_program(Some(prog));
            gl.uniform_1_i32(gl.get_uniform_location(prog, "u_texture").as_ref(), 0);
            gl.use_program(None);
        }
        let white = TextureGl::new(gl, &PngImage::decode(WHITE_PNG));
        Self {
            prog,
            u_model,
            u_view,
            u_proj,
            white,
            shell: PartGl::new(
                gl,
                &mesh.shell.verts,
                &mesh.shell.normals,
                &mesh.shell.uvs,
                &mesh.shell.indices,
            ),
            field: PartGl::new(
                gl,
                &mesh.field.verts,
                &mesh.field.normals,
                &mesh.field.uvs,
                &mesh.field.indices,
            ),
            planes_low: PartGl::new(
                gl,
                &mesh.planes_low.verts,
                &mesh.planes_low.normals,
                &mesh.planes_low.uvs,
                &mesh.planes_low.indices,
            ),
            lid: PartGl::new(
                gl,
                &mesh.lid.verts,
                &mesh.lid.normals,
                &mesh.lid.uvs,
                &mesh.lid.indices,
            ),
            planes_high: PartGl::new(
                gl,
                &mesh.planes_high.verts,
                &mesh.planes_high.normals,
                &mesh.planes_high.uvs,
                &mesh.planes_high.indices,
            ),
        }
    }

    /// Draws the cutaway (shell + field + low planes), plus the roof
    /// (lid + high backdrop) when `enclosed`. Viewport in GL pixels.
    /// Resets ALL shared-context GL state first (egui painter state leaks:
    /// blend/scissor/cull/mask — every draw owns its state, no exceptions).
    pub fn draw(
        &self,
        gl: &glow::Context,
        viewport_px: (i32, i32, i32, i32),
        model: &Mat4,
        view: &Mat4,
        proj: &Mat4,
        enclosed: bool,
    ) {
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
            gl.uniform_matrix_4_f32_slice(Some(&self.u_model), false, &model.to_cols_array());
            gl.uniform_matrix_4_f32_slice(Some(&self.u_view), false, &view.to_cols_array());
            gl.uniform_matrix_4_f32_slice(Some(&self.u_proj), false, &proj.to_cols_array());
            self.white.bind(gl);
            for part in [&self.shell, &self.field, &self.planes_low] {
                gl.bind_vertex_array(Some(part.vao));
                gl.draw_elements(glow::TRIANGLES, part.n_indices, glow::UNSIGNED_INT, 0);
            }
            if enclosed {
                for part in [&self.lid, &self.planes_high] {
                    gl.bind_vertex_array(Some(part.vao));
                    gl.draw_elements(glow::TRIANGLES, part.n_indices, glow::UNSIGNED_INT, 0);
                }
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
            self.lid.destroy(gl);
            self.planes_high.destroy(gl);
            self.white.destroy(gl);
            gl.delete_program(self.prog);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arena_shaders_pin_locations() {
        // VAOs hardcode 0/1/2; the other half of the contract lives here.
        assert!(ARENA_VERT_SRC.contains("location = 0"));
        assert!(ARENA_VERT_SRC.contains("location = 1"));
        assert!(ARENA_VERT_SRC.contains("location = 2"));
        assert!(ARENA_VERT_SRC.contains("uniform mat4 u_model"));
        assert!(ARENA_FRAG_SRC.contains("team_color_from_pos"));
        assert!(ARENA_FRAG_SRC.contains("get_grid_line_val"));
    }
}
