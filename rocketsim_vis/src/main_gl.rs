//! Textured + lit object pipeline (stage 5a): ball and cars in the old look.
//!
//! The shaders are the old `main_vert/frag.glsl` bytes verbatim (ambient +
//! sun lighting, transparent-texture bypass). Per-model VAOs bind position,
//! normal, UV to pinned locations 0/1/2; all uniforms are cached at init.
//! Model matrices are plain translation × rotation — no handedness flip
//! (the view carries the single global x-flip instead).

use glam::{Mat3, Mat4, Vec3};
use glow::HasContext;
use rocketsim::{CarBodyConfig, Team};

use crate::{
    gl_util::{bytes_of, compile_program, uniform_location},
    obj::ObjMesh,
    textures::TextureGl,
};

/// Old-look textured pipeline shaders (verbatim bytes, ZealanL).
pub const MAIN_VERT_SRC: &str = include_str!("shaders/main_vert.glsl");
pub const MAIN_FRAG_SRC: &str = include_str!("shaders/main_frag.glsl");

/// Instanced car vertex shader: `main_vert` with `u_model` fed from a
/// per-instance `mat4` (pinned locations 3-6, divisor 1) instead of a
/// uniform — the pads pattern. The body is verbatim (`normalize`d
/// `mat3(inst) * normal`, `u_proj * u_view * inst * v`), so pixels match
/// the per-draw path exactly. Pairs with [`MAIN_FRAG_SRC`].
pub const CAR_INST_VERT_SRC: &str = "#version 330 core

layout (location = 0) in vec3 vs_vert;
layout (location = 1) in vec3 vs_vert_normal;
layout (location = 2) in vec2 vs_vert_uv;
layout (location = 3) in vec4 vs_car_m0;
layout (location = 4) in vec4 vs_car_m1;
layout (location = 5) in vec4 vs_car_m2;
layout (location = 6) in vec4 vs_car_m3;

out vec2 fs_uv;
out vec3 fs_normal;

uniform mat4 u_view;
uniform mat4 u_proj;

void main() {
    mat4 inst = mat4(vs_car_m0, vs_car_m1, vs_car_m2, vs_car_m3);
    fs_uv = vs_vert_uv;
    fs_normal = normalize(mat3(inst) * vs_vert_normal);
    gl_Position = u_proj * u_view * inst * vec4(vs_vert, 1.0);
}
";

/// Bytes per instance matrix (mat4 of f32).
pub const CAR_INSTANCE_BYTES: usize = 64;

/// One instanced car draw: every car sharing a body mesh AND a team
/// texture. Groups are built by [`group_car_draws`].
pub struct CarDrawGroup {
    /// [`car_model_name`] output (`car_octane`, …).
    pub model: &'static str,
    /// [`car_texture_name`] output (`car_blue` / `car_orange`).
    pub texture: &'static str,
    /// Per-car model matrices for this draw.
    pub matrices: Vec<Mat4>,
}

/// Groups per-car placements into per-(body model, team texture) batches.
/// First-seen order is preserved (deterministic; opaque draws are
/// order-free anyway, so grouping never changes pixels).
pub fn group_car_draws(cars: &[(&'static str, &'static str, Mat4)]) -> Vec<CarDrawGroup> {
    let mut groups: Vec<CarDrawGroup> = Vec::with_capacity(cars.len());
    for &(model, texture, mat) in cars {
        match groups
            .iter_mut()
            .find(|g| g.model == model && g.texture == texture)
        {
            Some(g) => g.matrices.push(mat),
            None => groups.push(CarDrawGroup {
                model,
                texture,
                matrices: vec![mat],
            }),
        }
    }
    groups
}

/// Texture asset names (bundled PNGs).
pub const BALL_TEXTURE: &str = "ball";
/// Arena surface texture (flat white — multiplies the procedural team colors
/// neutrally; any tint here would tint the whole stadium, see texture test).
pub const ARENA_TEXTURE: &str = "white";

/// Model asset names (bundled OBJs).
pub const BALL_MODEL: &str = "ball";

/// Maps a body config to its bundled OBJ. Only four bodies have customs;
/// the rest fall back to octane (the old code named nonexistent
/// `car_plank`/`car_hybrid` models — fallback is strictly better).
pub fn car_model_name(config: &CarBodyConfig) -> &'static str {
    if *config == CarBodyConfig::DOMINUS {
        "car_dominus"
    } else if *config == CarBodyConfig::BREAKOUT {
        "car_breakout"
    } else if *config == CarBodyConfig::MERC {
        "car_merc"
    } else {
        "car_octane"
    }
}

/// Maps a team to its car texture, as the old vis did.
pub fn car_texture_name(team: Team) -> &'static str {
    if team == Team::Blue {
        "car_blue"
    } else {
        "car_orange"
    }
}

/// Object placement: rotation first, then translation. No mirror flip —
/// the view's global x-flip already puts screen-right at +X.
pub fn model_matrix(pos: Vec3, rot: Mat3) -> Mat4 {
    Mat4::from_translation(pos) * Mat4::from_mat3(rot)
}

/// Owned main-pipeline program. Create on the GL thread; sampler bound to
/// unit 0 once at init.
pub struct MainProgram {
    prog: glow::Program,
    u_model: glow::UniformLocation,
    u_view: glow::UniformLocation,
    u_proj: glow::UniformLocation,
}

impl MainProgram {
    /// Compiles the old main shaders; panics with logs on failure.
    pub fn new(gl: &glow::Context) -> Self {
        let prog = compile_program(gl, "main", MAIN_VERT_SRC, MAIN_FRAG_SRC);
        let u_model = uniform_location(gl, prog, "main", "u_model");
        let u_view = uniform_location(gl, prog, "main", "u_view");
        let u_proj = uniform_location(gl, prog, "main", "u_proj");
        unsafe {
            gl.use_program(Some(prog));
            gl.uniform_1_i32(gl.get_uniform_location(prog, "u_texture").as_ref(), 0);
            gl.use_program(None);
        }
        Self {
            prog,
            u_model,
            u_view,
            u_proj,
        }
    }

    /// Sets the shared view + projection for all main draws this frame.
    pub fn set_view_proj(&self, gl: &glow::Context, view: &Mat4, proj: &Mat4) {
        unsafe {
            gl.use_program(Some(self.prog));
            gl.uniform_matrix_4_f32_slice(Some(&self.u_view), false, &view.to_cols_array());
            gl.uniform_matrix_4_f32_slice(Some(&self.u_proj), false, &proj.to_cols_array());
        }
    }

    /// Draws one textured model with `model` transform. Leaves the program
    /// bound (batch draws share it); caller unbinds when done.
    pub fn draw(&self, gl: &glow::Context, model: &ModelGl, texture: &TextureGl, model_mat: &Mat4) {
        unsafe {
            gl.uniform_matrix_4_f32_slice(Some(&self.u_model), false, &model_mat.to_cols_array());
            texture.bind(gl);
            gl.bind_vertex_array(Some(model.vao));
            gl.draw_elements(glow::TRIANGLES, model.n_indices, glow::UNSIGNED_INT, 0);
            gl.bind_vertex_array(None);
        }
    }

    /// Unbinds the program. Call after a draw batch.
    pub fn unbind(&self, gl: &glow::Context) {
        unsafe {
            gl.use_program(None);
        }
    }

    /// Deletes the program. Call on the GL thread before dropping.
    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            gl.delete_program(self.prog);
        }
    }
}

/// Owned indexed model (pos + normal + uv VBOs, `u32` IBO).
pub struct ModelGl {
    vao: glow::VertexArray,
    vbos: [glow::Buffer; 3],
    ibo: glow::Buffer,
    n_indices: i32,
}

impl ModelGl {
    /// Uploads an OBJ mesh (`STATIC_DRAW` — customs never change).
    pub fn new(gl: &glow::Context, mesh: &ObjMesh) -> Self {
        assert_eq!(mesh.indices.len() % 3, 0);
        let (vao, vbos, ibo) = unsafe {
            let vao = gl.create_vertex_array().expect("GL create model VAO");
            gl.bind_vertex_array(Some(vao));
            let vbos = [
                gl.create_buffer().expect("GL create model VBO"),
                gl.create_buffer().expect("GL create model VBO"),
                gl.create_buffer().expect("GL create model VBO"),
            ];
            let uploads: [&[u8]; 3] = [
                bytes_of(&mesh.verts),
                bytes_of(&mesh.normals),
                bytes_of(&mesh.uvs),
            ];
            for (buf, bytes, loc, size) in [
                (vbos[0], uploads[0], 0, 3),
                (vbos[1], uploads[1], 1, 3),
                (vbos[2], uploads[2], 2, 2),
            ] {
                gl.bind_buffer(glow::ARRAY_BUFFER, Some(buf));
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes, glow::STATIC_DRAW);
                gl.vertex_attrib_pointer_f32(loc, size, glow::FLOAT, false, 0, 0);
                gl.enable_vertex_attrib_array(loc);
            }
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            let ibo = gl.create_buffer().expect("GL create model IBO");
            gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(ibo));
            gl.buffer_data_u8_slice(
                glow::ELEMENT_ARRAY_BUFFER,
                bytes_of(&mesh.indices),
                glow::STATIC_DRAW,
            );
            gl.bind_vertex_array(None);
            (vao, vbos, ibo)
        };
        Self {
            vao,
            vbos,
            ibo,
            n_indices: mesh.indices.len() as i32,
        }
    }

    /// Deletes every GL object. Call on the GL thread before dropping.
    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            for b in self.vbos {
                gl.delete_buffer(b);
            }
            gl.delete_buffer(self.ibo);
            gl.delete_vertex_array(self.vao);
        }
    }
}

/// Owned instanced car model: static mesh VAO + streaming per-frame
/// instance buffer (per-instance `mat4` on locations 3-6, divisor 1 —
/// the pads pattern). One [`InstancedModelGl`] per body mesh; each frame
/// the viewer uploads one [`CarDrawGroup`] per (body, team) and draws it
/// once instead of once per car.
pub struct InstancedModelGl {
    vao: glow::VertexArray,
    mesh_vbos: [glow::Buffer; 3],
    ibo: glow::Buffer,
    inst_vbo: glow::Buffer,
    n_indices: i32,
    n_instances: i32,
}

impl InstancedModelGl {
    /// Uploads an OBJ mesh (`STATIC_DRAW`); the instance buffer starts as
    /// one placeholder matrix and is reallocated by [`upload`](Self::upload)
    /// to fit each frame's group (groups are tiny — no cap needed).
    pub fn new(gl: &glow::Context, mesh: &ObjMesh) -> Self {
        assert_eq!(mesh.indices.len() % 3, 0);
        let (vao, mesh_vbos, ibo, inst_vbo) = unsafe {
            let vao = gl.create_vertex_array().expect("GL create car VAO");
            gl.bind_vertex_array(Some(vao));
            let mesh_vbos = [
                gl.create_buffer().expect("GL create car VBO"),
                gl.create_buffer().expect("GL create car VBO"),
                gl.create_buffer().expect("GL create car VBO"),
            ];
            let uploads: [&[u8]; 3] = [
                bytes_of(&mesh.verts),
                bytes_of(&mesh.normals),
                bytes_of(&mesh.uvs),
            ];
            for (buf, bytes, loc, size) in [
                (mesh_vbos[0], uploads[0], 0, 3),
                (mesh_vbos[1], uploads[1], 1, 3),
                (mesh_vbos[2], uploads[2], 2, 2),
            ] {
                gl.bind_buffer(glow::ARRAY_BUFFER, Some(buf));
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes, glow::STATIC_DRAW);
                gl.vertex_attrib_pointer_f32(loc, size, glow::FLOAT, false, 0, 0);
                gl.enable_vertex_attrib_array(loc);
            }
            let inst_vbo = gl.create_buffer().expect("GL create car instance VBO");
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(inst_vbo));
            gl.buffer_data_u8_slice(
                glow::ARRAY_BUFFER,
                &[0u8; CAR_INSTANCE_BYTES],
                glow::DYNAMIC_DRAW,
            );
            for col in 0..4 {
                let loc = 3 + col as u32;
                gl.vertex_attrib_pointer_f32(
                    loc,
                    4,
                    glow::FLOAT,
                    false,
                    CAR_INSTANCE_BYTES as i32,
                    col * 16,
                );
                gl.enable_vertex_attrib_array(loc);
                gl.vertex_attrib_divisor(loc, 1);
            }
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
            let ibo = gl.create_buffer().expect("GL create car IBO");
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

    /// Streams this frame's group matrices. Empty groups skip the upload
    /// (the draw then skips too — a hidden/culled group costs zero GL calls).
    pub fn upload(&mut self, gl: &glow::Context, matrices: &[Mat4]) {
        self.n_instances = matrices.len() as i32;
        if matrices.is_empty() {
            return;
        }
        unsafe {
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.inst_vbo));
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes_of(matrices), glow::DYNAMIC_DRAW);
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
        }
    }

    /// Instances staged by the last [`upload`](Self::upload).
    pub fn instance_count(&self) -> usize {
        self.n_instances.max(0) as usize
    }

    /// Deletes every GL object. Call on the GL thread before dropping.
    pub fn destroy(&self, gl: &glow::Context) {
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

/// Owned instanced-car program (`CAR_INST_VERT_SRC` + [`MAIN_FRAG_SRC`]).
/// Create on the GL thread; sampler bound to unit 0 once at init. Sets NO
/// GL state per draw — like [`MainProgram`], it inherits the arena pass
/// state (depth on, blend off), so group draws match per-car pixels.
pub struct MainInstProgram {
    prog: glow::Program,
    u_view: glow::UniformLocation,
    u_proj: glow::UniformLocation,
}

impl MainInstProgram {
    /// Compiles the instanced car shaders; panics with logs on failure.
    pub fn new(gl: &glow::Context) -> Self {
        let prog = compile_program(gl, "cars_inst", CAR_INST_VERT_SRC, MAIN_FRAG_SRC);
        let u_view = uniform_location(gl, prog, "cars_inst", "u_view");
        let u_proj = uniform_location(gl, prog, "cars_inst", "u_proj");
        unsafe {
            gl.use_program(Some(prog));
            gl.uniform_1_i32(gl.get_uniform_location(prog, "u_texture").as_ref(), 0);
            gl.use_program(None);
        }
        Self {
            prog,
            u_view,
            u_proj,
        }
    }

    /// Sets the shared view + projection for all group draws this frame.
    pub fn set_view_proj(&self, gl: &glow::Context, view: &Mat4, proj: &Mat4) {
        unsafe {
            gl.use_program(Some(self.prog));
            gl.uniform_matrix_4_f32_slice(Some(&self.u_view), false, &view.to_cols_array());
            gl.uniform_matrix_4_f32_slice(Some(&self.u_proj), false, &proj.to_cols_array());
        }
    }

    /// Draws one uploaded group with its team texture. Leaves the program
    /// bound (groups share it); caller unbinds when done. Groups with no
    /// instances draw nothing (no GL calls).
    pub fn draw_group(&self, gl: &glow::Context, model: &InstancedModelGl, texture: &TextureGl) {
        if model.n_instances <= 0 {
            return;
        }
        unsafe {
            texture.bind(gl);
            gl.bind_vertex_array(Some(model.vao));
            gl.draw_elements_instanced(
                glow::TRIANGLES,
                model.n_indices,
                glow::UNSIGNED_INT,
                0,
                model.n_instances,
            );
            gl.bind_vertex_array(None);
        }
    }

    /// Unbinds the program. Call after a draw batch.
    pub fn unbind(&self, gl: &glow::Context) {
        unsafe {
            gl.use_program(None);
        }
    }

    /// Deletes the program. Call on the GL thread before dropping.
    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            gl.delete_program(self.prog);
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec4Swizzles;

    use super::*;

    #[test]
    fn main_shaders_pin_locations() {
        // VAOs hardcode 0/1/2; the other half of the contract lives here.
        assert!(MAIN_VERT_SRC.contains("location = 0"));
        assert!(MAIN_VERT_SRC.contains("location = 1"));
        assert!(MAIN_VERT_SRC.contains("location = 2"));
        assert!(MAIN_VERT_SRC.contains("uniform mat4 u_model"));
        assert!(MAIN_FRAG_SRC.contains("uniform sampler2D u_texture"));
    }

    #[test]
    fn model_matrix_places_and_rotates() {
        let pos = Vec3::new(100.0, -200.0, 50.0);
        let m = model_matrix(pos, Mat3::IDENTITY);
        let p = m * Vec3::ZERO.extend(1.0);
        assert_eq!(p.xyz(), pos);
        // Yaw +90° about Z: nose (+X) points +Y.
        let rot = Mat3::from_rotation_z(std::f32::consts::FRAC_PI_2);
        let m = model_matrix(Vec3::ZERO, rot);
        let nose = (m * Vec3::X.extend(1.0)).xyz().round();
        assert_eq!(nose, Vec3::Y);
    }

    #[test]
    fn body_mapping_covers_all_configs() {
        // Every preset resolves to a BUNDLED model (no missing plank/hybrid).
        for config in [
            CarBodyConfig::OCTANE,
            CarBodyConfig::DOMINUS,
            CarBodyConfig::PLANK,
            CarBodyConfig::BREAKOUT,
            CarBodyConfig::HYBRID,
            CarBodyConfig::MERC,
            CarBodyConfig::PSYCLOPS,
        ] {
            let name = car_model_name(&config);
            assert!(
                ["car_octane", "car_dominus", "car_breakout", "car_merc"].contains(&name),
                "unbundled model {name}"
            );
        }
        assert_eq!(car_model_name(&CarBodyConfig::DOMINUS), "car_dominus");
        assert_eq!(car_model_name(&CarBodyConfig::BREAKOUT), "car_breakout");
        assert_eq!(car_model_name(&CarBodyConfig::MERC), "car_merc");
    }

    #[test]
    fn team_mapping() {
        assert_eq!(car_texture_name(Team::Blue), "car_blue");
        assert_eq!(car_texture_name(Team::Orange), "car_orange");
    }

    #[test]
    fn car_inst_shader_pins_locations() {
        // Mesh attribs mirror main (0/1/2); instance mat4 sits on 3-6.
        for loc in [
            "location = 0",
            "location = 1",
            "location = 2",
            "location = 3",
            "location = 4",
            "location = 5",
            "location = 6",
        ] {
            assert!(CAR_INST_VERT_SRC.contains(loc), "missing {loc}");
        }
        assert!(CAR_INST_VERT_SRC.contains("uniform mat4 u_view"));
        assert!(CAR_INST_VERT_SRC.contains("uniform mat4 u_proj"));
        assert!(
            !CAR_INST_VERT_SRC.contains("u_model"),
            "instancing must not use u_model"
        );
    }

    #[test]
    fn car_inst_stride() {
        assert_eq!(CAR_INSTANCE_BYTES, std::mem::size_of::<Mat4>());
    }

    #[test]
    fn group_car_draws_splits_body_and_team() {
        let m1 = Mat4::IDENTITY;
        let m2 = Mat4::from_translation(Vec3::new(1.0, 2.0, 3.0));
        let m3 = Mat4::from_scale(Vec3::splat(2.0));
        let cars = [
            ("car_octane", "car_blue", m1),
            ("car_octane", "car_orange", m2),
            ("car_octane", "car_blue", m3),
            ("car_dominus", "car_blue", m1),
        ];
        let groups = group_car_draws(&cars);
        // One draw per (body, team); first-seen order preserved.
        assert_eq!(groups.len(), 3);
        assert_eq!(
            groups
                .iter()
                .map(|g| (g.model, g.texture, g.matrices.len()))
                .collect::<Vec<_>>(),
            vec![
                ("car_octane", "car_blue", 2),
                ("car_octane", "car_orange", 1),
                ("car_dominus", "car_blue", 1),
            ]
        );
        assert_eq!(groups[0].matrices, vec![m1, m3]);
    }

    #[test]
    fn group_car_draws_empty_and_singleton() {
        assert!(group_car_draws(&[]).is_empty());
        let m = Mat4::IDENTITY;
        let groups = group_car_draws(&[("car_merc", "car_orange", m)]);
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].matrices, vec![m]);
    }
}
