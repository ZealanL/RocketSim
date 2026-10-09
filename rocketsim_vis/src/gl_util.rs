//! Minimal glow helpers with fail-fast rules (established stage 3, kept ever after).
//!
//! * [`compile_program`] asserts shader compile + link status with info logs
//!   and deletes the shaders after linking. A broken shader must crash with
//!   its log, never render black silently.
//! * Attribute locations are **hardcoded only when the shader pins them**
//!   with `layout(location = …)`; otherwise they must be queried with
//!   [`attrib_location`]. See `triangle` for the pinned example.
//! * Uniform locations are **cached once at init**, never queried per draw
//!   (per-frame `get_uniform_location` spam ate measurable CPU in v1).

use glow::HasContext;

/// Compiles + links a program, panicking with the info log on any failure.
pub fn compile_program(
    gl: &glow::Context,
    label: &str,
    vert_src: &str,
    frag_src: &str,
) -> glow::Program {
    unsafe {
        let vert = gl
            .create_shader(glow::VERTEX_SHADER)
            .expect("GL create vertex shader");
        gl.shader_source(vert, vert_src);
        gl.compile_shader(vert);
        assert!(
            gl.get_shader_compile_status(vert),
            "{label}: vertex compile failed: {}",
            gl.get_shader_info_log(vert)
        );
        let frag = gl
            .create_shader(glow::FRAGMENT_SHADER)
            .expect("GL create fragment shader");
        gl.shader_source(frag, frag_src);
        gl.compile_shader(frag);
        assert!(
            gl.get_shader_compile_status(frag),
            "{label}: fragment compile failed: {}",
            gl.get_shader_info_log(frag)
        );
        let prog = gl.create_program().expect("GL create program");
        gl.attach_shader(prog, vert);
        gl.attach_shader(prog, frag);
        gl.link_program(prog);
        assert!(
            gl.get_program_link_status(prog),
            "{label}: program link failed: {}",
            gl.get_program_info_log(prog)
        );
        gl.delete_shader(vert);
        gl.delete_shader(frag);
        prog
    }
}

/// Queries one uniform location at init; panics loudly if the shader lacks it.
///
/// A missing uniform is always a bug (typo or dead-code elimination changed
/// the shader) — crashing here beats rendering wrong values silently.
pub fn uniform_location(
    gl: &glow::Context,
    prog: glow::Program,
    label: &str,
    name: &str,
) -> glow::UniformLocation {
    unsafe {
        gl.get_uniform_location(prog, name).unwrap_or_else(|| {
            panic!("{label}: shader missing uniform '{name}'");
        })
    }
}

/// Reinterprets a slice as bytes for `buffer_data_u8_slice`.
pub fn bytes_of<T>(values: &[T]) -> &[u8] {
    unsafe {
        std::slice::from_raw_parts(values.as_ptr() as *const u8, std::mem::size_of_val(values))
    }
}
/// Queries one attribute location at init for shaders WITHOUT
/// `layout(location = …)` pins; panics loudly if missing.
pub fn attrib_location(gl: &glow::Context, prog: glow::Program, label: &str, name: &str) -> u32 {
    unsafe {
        gl.get_attrib_location(prog, name).unwrap_or_else(|| {
            panic!("{label}: shader missing attrib '{name}'");
        })
    }
}
