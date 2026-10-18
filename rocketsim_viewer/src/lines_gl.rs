//! Line quads: CPU-expanded, one dynamic VBO, one draw.
//!
//! [`expand_segment`](crate::lines::expand_segment) builds camera-facing
//! quads on CPU; the shaders only transform and discard-test. Fully
//! transparent segments skip pre-cap (bit-exact no-op).

use glam::Mat4;
use glow::HasContext;

use crate::{
    gl_util::{bytes_of, compile_program, uniform_location},
    lines::{Segment, expand_segment},
};

/// Line shaders: thin transform + segment-distance discard (CPU expands).
pub const LINE_VERT_SRC: &str = include_str!("shaders/line_vert.glsl");
pub const LINE_FRAG_SRC: &str = include_str!("shaders/line_frag.glsl");

/// Segment cap (draws beyond it are dropped; count reported for the HUD).
pub const MAX_LINES: usize = 1024;
/// Floats per corner: world(3) + pa(3) + pb(3) + color(4) + width(1).
const STRIDE_FLOATS: usize = 14;

/// Owned line pipeline. Create/upload/draw/destroy on the GL thread.
pub struct LinesGl {
    prog: glow::Program,
    u_view: glow::UniformLocation,
    u_proj: glow::UniformLocation,
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    ibo: glow::Buffer,
    /// Indices staged by the last [`upload`](Self::upload).
    n_drawn: i32,
    /// Segments dropped by the [`MAX_LINES`] cap (overlay diagnostics).
    pub truncated: usize,
}

impl LinesGl {
    /// Compiles the line program and builds the quad VAO + index pattern.
    pub fn new(gl: &glow::Context) -> Self {
        let prog = compile_program(gl, "lines", LINE_VERT_SRC, LINE_FRAG_SRC);
        let u_view = uniform_location(gl, prog, "lines", "u_view");
        let u_proj = uniform_location(gl, prog, "lines", "u_proj");
        let (vao, vbo, ibo) = unsafe {
            let vao = gl.create_vertex_array().expect("GL create line VAO");
            gl.bind_vertex_array(Some(vao));
            let vbo = gl.create_buffer().expect("GL create line VBO");
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
            gl.buffer_data_u8_slice(
                glow::ARRAY_BUFFER,
                &vec![0u8; MAX_LINES * 4 * STRIDE_FLOATS * 4],
                glow::DYNAMIC_DRAW,
            );
            let stride = (STRIDE_FLOATS * 4) as i32;
            let mut off = 0;
            for (loc, size) in [(0u32, 3), (1, 3), (2, 3), (3, 4), (4, 1)] {
                gl.vertex_attrib_pointer_f32(loc, size, glow::FLOAT, false, stride, off);
                gl.enable_vertex_attrib_array(loc);
                off += size * 4;
            }
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            // Static quad pattern: two tris per segment slot.
            let mut idx = Vec::with_capacity(MAX_LINES * 6);
            for k in 0..MAX_LINES as u32 {
                let v = k * 4;
                idx.extend_from_slice(&[v, v + 1, v + 2, v + 1, v + 3, v + 2]);
            }
            let ibo = gl.create_buffer().expect("GL create line IBO");
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(ibo));
            gl.buffer_data_u8_slice(
                glow::ELEMENT_ARRAY_BUFFER,
                bytes_of(&idx),
                glow::STATIC_DRAW,
            );
            gl.bind_vertex_array(None);
            (vao, vbo, ibo)
        };
        Self {
            prog,
            u_view,
            u_proj,
            vao,
            vbo,
            ibo,
            n_drawn: 0,
            truncated: 0,
        }
    }

    /// Expands + streams segments (skips invisible, truncates past
    /// [`MAX_LINES`]). `eye`/`view`/`proj11`/`screen_h` drive expansion.
    pub fn upload(
        &mut self,
        gl: &glow::Context,
        segs: &[Segment],
        eye: glam::Vec3A,
        view: &Mat4,
        proj11: f32,
        screen_h: f32,
    ) {
        let mut buf = Vec::new();
        let mut kept = 0usize;
        for seg in segs {
            if seg.is_invisible() {
                continue;
            }
            if kept >= MAX_LINES {
                break;
            }
            for corner in expand_segment(seg, eye, view, proj11, screen_h) {
                buf.extend_from_slice(&corner.world.to_array());
                buf.extend_from_slice(&seg.a.to_array());
                buf.extend_from_slice(&seg.b.to_array());
                buf.extend_from_slice(&corner.color.to_array());
                buf.push(corner.width);
            }
            kept += 1;
        }
        // Count cap-drops only (invisible skips are free, not truncated).
        let over = segs
            .iter()
            .filter(|s| !s.is_invisible())
            .count()
            .saturating_sub(kept);
        self.truncated = over;
        self.n_drawn = (kept * 6) as i32;
        if buf.is_empty() {
            // Nothing staged (all invisible or no segments): the draw skips
            // on `n_drawn == 0`, so stale buffer contents are harmless and
            // the upload costs no GL calls (common with toggles off).
            return;
        }
        unsafe {
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.vbo));
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes_of(&buf), glow::DYNAMIC_DRAW);
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
        }
    }

    /// Segments staged for (or by) the last draw.
    pub fn drawn(&self) -> usize {
        (self.n_drawn / 6) as usize
    }

    /// Draws staged quads (transparent — enables blending).
    /// State reset matches the opaque passes, plus blend for alpha.
    /// Draws nothing when no segments were uploaded.
    pub fn draw(&self, gl: &glow::Context, view: &Mat4, proj: &Mat4) {
        if self.n_drawn == 0 {
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
            gl.depth_mask(true);
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LEQUAL);
            gl.use_program(Some(self.prog));
            gl.uniform_matrix_4_f32_slice(Some(&self.u_view), false, &view.to_cols_array());
            gl.uniform_matrix_4_f32_slice(Some(&self.u_proj), false, &proj.to_cols_array());
            gl.bind_vertex_array(Some(self.vao));
            gl.draw_elements(glow::TRIANGLES, self.n_drawn, glow::UNSIGNED_INT, 0);
            gl.bind_vertex_array(None);
            gl.use_program(None);
        }
    }

    /// Deletes every GL object. Call on the GL thread before dropping.
    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            gl.delete_buffer(self.vbo);
            gl.delete_buffer(self.ibo);
            gl.delete_vertex_array(self.vao);
            gl.delete_program(self.prog);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pins_present_and_screen_gone() {
        // VAO hardcodes 0-4: valid only with pinned locations; u_screen_size
        // is gone (CPU applies the minimum width now).
        for loc in ["location = 0", "location = 1", "location = 4"] {
            assert!(LINE_VERT_SRC.contains(loc), "missing pin {loc}");
        }
        assert!(!LINE_VERT_SRC.contains("u_screen_size"), "stale uniform");
        assert!(!LINE_FRAG_SRC.contains("u_screen_size"), "stale uniform");
        assert!(LINE_FRAG_SRC.contains("point_segment_dist"));
    }

    #[test]
    fn stride_math() {
        assert_eq!(STRIDE_FLOATS, 3 + 3 + 3 + 4 + 1);
    }
}
