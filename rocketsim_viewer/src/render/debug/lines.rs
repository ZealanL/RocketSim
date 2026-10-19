//! Instanced renderer for retained debug segments and camera-facing points.

use glam::Mat4;
use glow::HasContext;

use crate::render::gl_util::{bytes_of, compile_program, uniform_location};

pub const DEBUG_LINE_VERT_SRC: &str = include_str!("../../shaders/debug_line_vert.glsl");
pub const DEBUG_LINE_FRAG_SRC: &str = include_str!("../../shaders/debug_line_frag.glsl");
const INSTANCE_FLOATS: usize = 13;
const INSTANCE_STRIDE: i32 = (INSTANCE_FLOATS * std::mem::size_of::<f32>()) as i32;

/// One segment or point. `width` is pixels when `pixels` is true, otherwise world units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PackedLine {
    pub start: [f32; 3],
    pub end: [f32; 3],
    pub color: [f32; 4],
    pub width: f32,
    pub pixels: bool,
    pub point: bool,
}

impl PackedLine {
    fn pack(self, dst: &mut Vec<f32>) {
        dst.extend_from_slice(&self.start);
        dst.extend_from_slice(&self.end);
        dst.extend_from_slice(&self.color);
        dst.push(self.width);
        dst.push(if self.pixels { 1.0 } else { 0.0 });
        dst.push(if self.point { 1.0 } else { 0.0 });
    }
}

/// Static instance data and its VAO. Call `destroy` on the GL thread.
pub struct LineBatch {
    vao: glow::VertexArray,
    instance_vbo: glow::Buffer,
    /// Number of uploaded instances.
    pub count: usize,
    /// Number of bytes in the current instance data.
    pub upload_bytes: usize,
    capacity: usize,
}

impl LineBatch {
    /// Upload all supplied instances. Panics if the draw count exceeds GL's `i32` range.
    pub fn new(gl: &glow::Context, lines: &[PackedLine]) -> Self {
        assert!(
            lines.len() <= i32::MAX as usize,
            "too many debug line instances"
        );
        let mut packed = Vec::with_capacity(lines.len() * INSTANCE_FLOATS);
        for line in lines {
            line.pack(&mut packed);
        }
        let upload_bytes = packed.len() * std::mem::size_of::<f32>();
        let (vao, instance_vbo) = unsafe {
            let vao = gl.create_vertex_array().expect("GL create debug line VAO");
            let instance_vbo = gl
                .create_buffer()
                .expect("GL create debug line instance VBO");
            gl.bind_vertex_array(Some(vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(instance_vbo));
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes_of(&packed), glow::STATIC_DRAW);
            let attrs = [(0, 3), (1, 3), (2, 4), (3, 1), (4, 1), (5, 1)];
            let mut offset = 0;
            for (location, size) in attrs {
                gl.enable_vertex_attrib_array(location);
                gl.vertex_attrib_pointer_f32(
                    location,
                    size,
                    glow::FLOAT,
                    false,
                    INSTANCE_STRIDE,
                    offset,
                );
                gl.vertex_attrib_divisor(location, 1);
                offset += size * std::mem::size_of::<f32>() as i32;
            }
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            gl.bind_vertex_array(None);
            (vao, instance_vbo)
        };
        Self {
            vao,
            instance_vbo,
            count: lines.len(),
            upload_bytes,
            capacity: upload_bytes,
        }
    }

    /// Replace instance data while retaining the VAO and VBO.
    pub fn update(&mut self, gl: &glow::Context, lines: &[PackedLine]) -> usize {
        assert!(
            lines.len() <= i32::MAX as usize,
            "too many debug line instances"
        );
        let mut packed = Vec::with_capacity(lines.len() * INSTANCE_FLOATS);
        for line in lines {
            line.pack(&mut packed);
        }
        let bytes = bytes_of(&packed);
        unsafe {
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.instance_vbo));
            if bytes.len() > self.capacity {
                self.capacity = bytes.len().next_power_of_two();
                gl.buffer_data_size(glow::ARRAY_BUFFER, self.capacity as i32, glow::DYNAMIC_DRAW);
            }
            if !bytes.is_empty() {
                gl.buffer_sub_data_u8_slice(glow::ARRAY_BUFFER, 0, bytes);
            }
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
        }
        self.count = lines.len();
        self.upload_bytes = bytes.len();
        self.upload_bytes
    }

    /// Delete the VAO and instance buffer. Call on the context thread.
    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            gl.delete_buffer(self.instance_vbo);
            gl.delete_vertex_array(self.vao);
        }
    }
}

/// Shared shader program for instanced debug lines. Call `destroy` on the GL thread.
pub struct LineProgram {
    program: glow::Program,
    model: glow::UniformLocation,
    view: glow::UniformLocation,
    proj: glow::UniformLocation,
    viewport: glow::UniformLocation,
    alpha: glow::UniformLocation,
}

impl LineProgram {
    pub fn new(gl: &glow::Context) -> Self {
        let program = compile_program(gl, "debug lines", DEBUG_LINE_VERT_SRC, DEBUG_LINE_FRAG_SRC);
        Self {
            model: uniform_location(gl, program, "debug lines", "u_model"),
            view: uniform_location(gl, program, "debug lines", "u_view"),
            proj: uniform_location(gl, program, "debug lines", "u_proj"),
            viewport: uniform_location(gl, program, "debug lines", "u_viewport"),
            alpha: uniform_location(gl, program, "debug lines", "u_alpha"),
            program,
        }
    }

    /// Draw a batch. The caller controls depth and blend state.
    #[allow(clippy::too_many_arguments)]
    pub fn draw(
        &self,
        gl: &glow::Context,
        batch: &LineBatch,
        model: &Mat4,
        view: &Mat4,
        proj: &Mat4,
        viewport: [f32; 2],
        alpha: f32,
    ) {
        if batch.count == 0 {
            return;
        }
        unsafe {
            gl.use_program(Some(self.program));
            gl.uniform_matrix_4_f32_slice(Some(&self.model), false, &model.to_cols_array());
            gl.uniform_matrix_4_f32_slice(Some(&self.view), false, &view.to_cols_array());
            gl.uniform_matrix_4_f32_slice(Some(&self.proj), false, &proj.to_cols_array());
            gl.uniform_2_f32(Some(&self.viewport), viewport[0], viewport[1]);
            gl.uniform_1_f32(Some(&self.alpha), alpha);
            gl.bind_vertex_array(Some(batch.vao));
            gl.draw_arrays_instanced(glow::TRIANGLES, 0, 6, batch.count as i32);
            gl.bind_vertex_array(None);
            gl.use_program(None);
        }
    }

    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            gl.delete_program(self.program);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn instance_packing_has_fixed_float_stride_and_flags() {
        let line = PackedLine {
            start: [1.0, 2.0, 3.0],
            end: [4.0, 5.0, 6.0],
            color: [0.1, 0.2, 0.3, 0.4],
            width: 7.0,
            pixels: true,
            point: false,
        };
        let mut packed = Vec::new();
        line.pack(&mut packed);
        assert_eq!(packed.len(), INSTANCE_FLOATS);
        assert_eq!(&packed[11..], &[1.0, 0.0]);
    }
}
