//! Indexed arena draw with the old team-color/grid look.
//!
//! Shaders are the old `arena_vert/frag.glsl` bytes verbatim: team colors
//! by world-Y, `fwidth`-antialiased grid lines, normal-based brightness.
//! Draws sample the flat-white texture so colors pass through untinted.
//! Uniforms are cached at init; the sampler binds unit 0 once.

use glam::{Mat4, Vec2, Vec3};
use glow::HasContext;

use crate::{
    gl_util::{bytes_of, compile_program, uniform_location},
    mesh::{ArenaMesh, ArenaPart},
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

/// Merges two always-together static parts into one index pool (one draw
/// covers both). Order is kept (all of `a`, then `b` rebased), so pixels
/// match two sequential draws under LEQUAL depth with blending off.
fn merge_two(a: &ArenaPart, b: &ArenaPart) -> (Vec<Vec3>, Vec<Vec3>, Vec<Vec2>, Vec<u32>) {
    let mut verts = Vec::with_capacity(a.verts.len() + b.verts.len());
    verts.extend_from_slice(&a.verts);
    verts.extend_from_slice(&b.verts);
    let mut normals = Vec::with_capacity(a.normals.len() + b.normals.len());
    normals.extend_from_slice(&a.normals);
    normals.extend_from_slice(&b.normals);
    let mut uvs = Vec::with_capacity(a.uvs.len() + b.uvs.len());
    uvs.extend_from_slice(&a.uvs);
    uvs.extend_from_slice(&b.uvs);
    let mut indices = Vec::with_capacity(a.indices.len() + b.indices.len());
    indices.extend_from_slice(&a.indices);
    let base = a.verts.len() as u32;
    indices.extend(b.indices.iter().map(|i| i + base));
    (verts, normals, uvs, indices)
}

/// Owned GL objects for the arena: the cutaway bowl (always drawn) plus
/// the roof (enclosed/interior views only). Shell + field + low planes
/// merge at init (1 draw); roof adds 1 more when enclosed.
pub struct ArenaGl {
    prog: glow::Program,
    u_model: glow::UniformLocation,
    u_view: glow::UniformLocation,
    u_proj: glow::UniformLocation,
    white: TextureGl,
    cut: PartGl,
    roof: PartGl,
}

impl ArenaGl {
    /// Uploads both merged parts; panics with logs on failure.
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
        // Cutaway triple (drawn in mesh order: shell, field, low planes);
        // roof pair (lid, then high backdrop).
        let (v, n, u, i) = merge_two(&mesh.shell, &mesh.field);
        let cut12 = ArenaPart {
            verts: v,
            normals: n,
            uvs: u,
            indices: i,
        };
        let (v, n, u, i) = merge_two(&cut12, &mesh.planes_low);
        let cut = PartGl::new(gl, &v, &n, &u, &i);
        let (v, n, u, i) = merge_two(&mesh.lid, &mesh.planes_high);
        let roof = PartGl::new(gl, &v, &n, &u, &i);
        Self {
            prog,
            u_model,
            u_view,
            u_proj,
            white,
            cut,
            roof,
        }
    }

    /// Draws the cutaway in ONE draw, plus the roof in ONE draw when
    /// `enclosed`. Viewport in GL pixels. Resets all shared GL state first
    /// (egui painter state leaks: every draw owns its state).
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
            gl.bind_vertex_array(Some(self.cut.vao));
            gl.draw_elements(glow::TRIANGLES, self.cut.n_indices, glow::UNSIGNED_INT, 0);
            if enclosed {
                gl.bind_vertex_array(Some(self.roof.vao));
                gl.draw_elements(glow::TRIANGLES, self.roof.n_indices, glow::UNSIGNED_INT, 0);
            }
            gl.bind_vertex_array(None);
            gl.use_program(None);
        }
    }

    /// Deletes every GL object. Call on the GL thread before dropping.
    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            self.cut.destroy(gl);
            self.roof.destroy(gl);
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

    #[test]
    fn merge_two_rebases() {
        use crate::mesh::ArenaPart;
        let a = ArenaPart {
            verts: vec![Vec3::ZERO, Vec3::X, Vec3::Y],
            normals: vec![Vec3::Z; 3],
            uvs: vec![Vec2::ZERO; 3],
            indices: vec![0, 1, 2],
        };
        let b = ArenaPart {
            verts: vec![Vec3::ONE, Vec3::NEG_ONE],
            normals: vec![Vec3::Z; 2],
            uvs: vec![Vec2::ONE; 2],
            indices: vec![0, 1, 0],
        };
        let (v, n, u, i) = merge_two(&a, &b);
        assert_eq!(v.len(), 5);
        assert_eq!(n.len(), 5);
        assert_eq!(u.len(), 5);
        // a-tris verbatim, then b-tris rebased past a's verts.
        assert_eq!(i, vec![0, 1, 2, 3, 4, 3]);
        assert_eq!(v[3], Vec3::ONE);
        // TheVoid builds empty parts; merging stays a valid no-op.
        let empty = ArenaPart {
            verts: vec![],
            normals: vec![],
            uvs: vec![],
            indices: vec![],
        };
        let (v, _, _, i) = merge_two(&empty, &empty);
        assert!(v.is_empty() && i.is_empty());
    }
}
