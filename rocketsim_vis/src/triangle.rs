//! Stage 3: one program, one VAO, one draw — hardcoded triangle in NDC.
//!
//! Establishes the draw path all later geometry reuses: pinned attrib
//! location 0 (the shader declares `layout(location = 0)`, pinned by the
//! `triangle_shader_pins_location_0` test), one cached uniform (`u_color`,
//! queried once), explicit viewport + clear, unbind after draw.

use glow::HasContext;

use crate::gl_util::{bytes_of, compile_program, uniform_location};

/// Vertex shader: `layout(location = 0)` pin is load-bearing — the VAO binds
/// position to location 0 without querying.
pub const TRI_VERT_SRC: &str = "#version 330 core
layout(location = 0) in vec2 vs_pos;
void main() {
    gl_Position = vec4(vs_pos, 0.0, 1.0);
}
";

/// Fragment shader: solid `u_color` (cached at init, set per draw).
pub const TRI_FRAG_SRC: &str = "#version 330 core
uniform vec4 u_color;
out vec4 out_color;
void main() {
    out_color = u_color;
}
";

/// Hardcoded triangle verts (NDC xy).
pub const TRI_VERTS: [[f32; 2]; 3] = [[-0.6, -0.5], [0.6, -0.5], [0.0, 0.6]];

/// Triangle orange (opaque).
pub const TRI_COLOR: [f32; 4] = [1.0, 0.45, 0.1, 1.0];

/// Owned GL objects for the triangle. Create/draw/destroy on the GL thread.
pub struct TriangleGl {
    prog: glow::Program,
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    u_color: glow::UniformLocation,
}

impl TriangleGl {
    /// Uploads the triangle; panics with logs on shader failure.
    pub fn new(gl: &glow::Context) -> Self {
        let prog = compile_program(gl, "triangle", TRI_VERT_SRC, TRI_FRAG_SRC);
        let u_color = uniform_location(gl, prog, "triangle", "u_color");
        let (vao, vbo) = unsafe {
            let vao = gl.create_vertex_array().expect("GL create triangle VAO");
            gl.bind_vertex_array(Some(vao));
            let vbo = gl.create_buffer().expect("GL create triangle VBO");
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
            let bytes: &[u8] = bytes_of(&TRI_VERTS);
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes, glow::STATIC_DRAW);
            // Location 0 is valid WITHOUT querying because TRI_VERT_SRC pins
            // `layout(location = 0)`.
            gl.vertex_attrib_pointer_f32(0, 2, glow::FLOAT, false, 0, 0);
            gl.enable_vertex_attrib_array(0);
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            gl.bind_vertex_array(None);
            (vao, vbo)
        };
        Self {
            prog,
            vao,
            vbo,
            u_color,
        }
    }

    /// Draws the triangle over a black clear. Viewport is in GL pixels
    /// (bottom-left origin).
    pub fn draw(&self, gl: &glow::Context, viewport_px: (i32, i32, i32, i32)) {
        unsafe {
            gl.viewport(viewport_px.0, viewport_px.1, viewport_px.2, viewport_px.3);
            gl.clear_color(0.0, 0.0, 0.0, 1.0);
            gl.clear(glow::COLOR_BUFFER_BIT);
            gl.use_program(Some(self.prog));
            gl.uniform_4_f32(
                Some(&self.u_color),
                TRI_COLOR[0],
                TRI_COLOR[1],
                TRI_COLOR[2],
                TRI_COLOR[3],
            );
            gl.bind_vertex_array(Some(self.vao));
            gl.draw_arrays(glow::TRIANGLES, 0, 3);
            gl.bind_vertex_array(None);
            gl.use_program(None);
        }
    }

    /// Deletes every GL object. Call on the GL thread before dropping.
    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            gl.delete_program(self.prog);
            gl.delete_buffer(self.vbo);
            gl.delete_vertex_array(self.vao);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn triangle_shader_pins_location_0() {
        // The VAO hardcodes location 0; this test pins the other half of the
        // contract so neither side can drift.
        assert!(
            TRI_VERT_SRC.contains("layout(location = 0)"),
            "triangle vs must pin location 0"
        );
        assert!(TRI_VERT_SRC.contains("#version 330 core"));
        assert!(TRI_FRAG_SRC.contains("uniform vec4 u_color"));
    }

    #[test]
    fn triangle_geometry_is_sane() {
        assert_eq!(TRI_VERTS.len(), 3);
        for [x, y] in TRI_VERTS {
            assert!(x.is_finite() && y.is_finite());
            assert!(x.abs() <= 1.0 && y.abs() <= 1.0, "verts must be in NDC");
        }
        // Non-degenerate: nonzero area.
        let ([ax, ay], [bx, by], [cx, cy]) = (TRI_VERTS[0], TRI_VERTS[1], TRI_VERTS[2]);
        let area = ((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)).abs();
        assert!(area > 0.1, "degenerate triangle");
    }
}
