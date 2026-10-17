//! Dynamic Dropshot tile surface geometry.

use glam::{Vec3, Vec4};
use glow::HasContext;

use crate::gl_util::{bytes_of, compile_program, uniform_location};

/// Damage level for one tile: 0 full, 1 damaged, 2 destroyed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TileDamage {
    Full,
    Damaged,
    Destroyed,
}

/// CPU mesh for the active Dropshot tiles.
#[derive(Clone, Debug, Default)]
pub struct DropshotTileMesh {
    /// Triangle vertices in world space.
    pub vertices: Vec<Vec3>,
    /// One team/damage color per vertex.
    pub colors: Vec<Vec4>,
}

impl DropshotTileMesh {
    /// Builds the simulation's two 70-tile teams from their exact tile states.
    pub fn from_states(states: &rocketsim::TileStates, z: f32) -> Self {
        let scale = rocketsim::consts::BT_TO_UU;
        let mut mesh = Self::default();
        let mut idx = 0usize;
        let mut y = rocketsim::consts::dropshot::TILE_OFFSET_Y;
        for row in 0..rocketsim::consts::dropshot::NUM_TILE_ROWS {
            let count = usize::from(rocketsim::consts::dropshot::TILES_IN_FIRST_ROW - row);
            let row_width = rocketsim::consts::dropshot::TILE_WIDTH_X * count as f32;
            let row_start = -row_width * 0.5 + rocketsim::consts::dropshot::TILE_WIDTH_X * 0.5;
            for col in 0..count {
                let center_x = row_start + rocketsim::consts::dropshot::TILE_WIDTH_X * col as f32;
                for team_idx in 0..2 {
                    let sign = if team_idx == 0 { -1.0 } else { 1.0 };
                    let center = Vec3::new(center_x * sign, y * sign, z);
                    let state = states.states[team_idx][idx];
                    if state != rocketsim::TileDamageState::Broken {
                        let team_color = if team_idx == 0 {
                            Vec4::new(0.12, 0.35, 0.95, 1.0)
                        } else {
                            Vec4::new(0.95, 0.30, 0.10, 1.0)
                        };
                        let color = match state {
                            rocketsim::TileDamageState::Full => team_color,
                            rocketsim::TileDamageState::Damaged => {
                                team_color.lerp(Vec4::splat(1.0), 0.45)
                            }
                            rocketsim::TileDamageState::Broken => unreachable!(),
                        };
                        let corners = rocketsim::consts::dropshot::TILE_HEXAGON_VERTS_BT.map(|v| {
                            let p = v * scale;
                            let edge_y = center.y + p.y;
                            let middle = rocketsim::consts::dropshot::TILE_OFFSET_Y;
                            // Match the collision shapes at the solid center strip.
                            let edge_y = if team_idx == 0 {
                                edge_y.min(-middle)
                            } else {
                                edge_y.max(middle)
                            };
                            Vec3::new(center.x + p.x, edge_y, center.z)
                        });
                        for i in 0..6 {
                            mesh.vertices.extend_from_slice(&[
                                center,
                                corners[i],
                                corners[(i + 1) % 6],
                            ]);
                            mesh.colors.extend_from_slice(&[color; 3]);
                        }
                    }
                }
                idx += 1;
            }
            y += rocketsim::consts::dropshot::ROW_OFFSET_Y;
        }
        mesh
    }

    /// Builds live tiles at caller-supplied centers. `centers` and `damage` use matching order.
    /// Use `from_states` for the game-mode tile grid and team colors.
    pub fn build(centers: &[Vec3], damage: &[TileDamage], z: f32) -> Self {
        assert_eq!(centers.len(), damage.len());
        let mut mesh = Self::default();
        for (center, state) in centers.iter().zip(damage) {
            if *state == TileDamage::Destroyed {
                continue;
            }
            let color = match state {
                TileDamage::Full => Vec4::new(0.25, 0.55, 0.95, 1.0),
                TileDamage::Damaged => Vec4::new(0.95, 0.62, 0.18, 1.0),
                TileDamage::Destroyed => unreachable!(),
            };
            let corners = rocketsim::consts::dropshot::TILE_HEXAGON_VERTS_BT.map(|v| {
                let offset = v * rocketsim::consts::BT_TO_UU;
                Vec3::new(center.x + offset.x, center.y + offset.y, z)
            });
            for i in 0..6 {
                mesh.vertices.extend_from_slice(&[
                    Vec3::new(center.x, center.y, z),
                    corners[i],
                    corners[(i + 1) % 6],
                ]);
                mesh.colors.extend_from_slice(&[color; 3]);
            }
        }
        mesh
    }
}

const VERT: &str = "#version 330 core\nlayout(location=0) in vec3 p; layout(location=1) in vec4 c; uniform mat4 u_view; uniform mat4 u_proj; out vec4 color; void main(){ color=c; gl_Position=u_proj*u_view*vec4(p,1.0); }";
const FRAG: &str =
    "#version 330 core\nin vec4 color; out vec4 out_color; void main(){out_color=color;}";

/// Reusable GL upload and draw for a changing Dropshot tile field.
pub struct DropshotTilesGl {
    program: glow::Program,
    view: glow::UniformLocation,
    proj: glow::UniformLocation,
    vao: glow::VertexArray,
    buffers: [glow::Buffer; 2],
    count: i32,
}

impl DropshotTilesGl {
    /// Creates empty renderer resources.
    pub fn new(gl: &glow::Context) -> Self {
        let program = compile_program(gl, "dropshot tiles", VERT, FRAG);
        let view = uniform_location(gl, program, "dropshot tiles", "u_view");
        let proj = uniform_location(gl, program, "dropshot tiles", "u_proj");
        unsafe {
            let vao = gl.create_vertex_array().expect("GL create tile VAO");
            gl.bind_vertex_array(Some(vao));
            let buffers = [
                gl.create_buffer().expect("GL create tile buffer"),
                gl.create_buffer().expect("GL create tile buffer"),
            ];
            for (i, buffer) in buffers.iter().enumerate() {
                gl.bind_buffer(glow::ARRAY_BUFFER, Some(*buffer));
                gl.vertex_attrib_pointer_f32(
                    i as u32,
                    if i == 0 { 3 } else { 4 },
                    glow::FLOAT,
                    false,
                    0,
                    0,
                );
                gl.enable_vertex_attrib_array(i as u32);
            }
            gl.bind_vertex_array(None);
            Self {
                program,
                view,
                proj,
                vao,
                buffers,
                count: 0,
            }
        }
    }

    /// Replaces the streamed CPU mesh. Empty geometry removes all tiles.
    pub fn upload(&mut self, gl: &glow::Context, mesh: &DropshotTileMesh) {
        assert_eq!(mesh.vertices.len(), mesh.colors.len());
        self.count = mesh.vertices.len() as i32;
        unsafe {
            for (buffer, bytes) in self
                .buffers
                .iter()
                .zip([bytes_of(&mesh.vertices), bytes_of(&mesh.colors)])
            {
                gl.bind_buffer(glow::ARRAY_BUFFER, Some(*buffer));
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytes, glow::DYNAMIC_DRAW);
            }
            gl.bind_buffer(glow::ARRAY_BUFFER, None);
        }
    }

    /// Draws live tile triangles after the opaque arena pass.
    pub fn draw(&self, gl: &glow::Context, view: &glam::Mat4, proj: &glam::Mat4) {
        if self.count == 0 {
            return;
        }
        unsafe {
            gl.disable(glow::CULL_FACE);
            gl.enable(glow::DEPTH_TEST);
            gl.depth_mask(true);
            gl.use_program(Some(self.program));
            gl.uniform_matrix_4_f32_slice(Some(&self.view), false, &view.to_cols_array());
            gl.uniform_matrix_4_f32_slice(Some(&self.proj), false, &proj.to_cols_array());
            gl.bind_vertex_array(Some(self.vao));
            gl.draw_arrays(glow::TRIANGLES, 0, self.count);
            gl.bind_vertex_array(None);
            gl.use_program(None);
        }
    }

    /// Deletes all GL objects.
    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            for buffer in self.buffers {
                gl.delete_buffer(buffer);
            }
            gl.delete_vertex_array(self.vao);
            gl.delete_program(self.program);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simulation_grid_has_team_colors_and_destroyed_holes() {
        let mut states = rocketsim::TileStates::DEFAULT;
        states.states[0][0] = rocketsim::TileDamageState::Damaged;
        states.states[1][0] = rocketsim::TileDamageState::Broken;
        let mesh = DropshotTileMesh::from_states(&states, 0.5);
        let full_tile_verts = 6 * 3;
        assert_eq!(mesh.vertices.len(), (140 - 1) * full_tile_verts);
        assert_eq!(mesh.colors.len(), mesh.vertices.len());
        assert!(mesh.colors.iter().any(|color| color.x > color.z));
        assert!(mesh.colors.iter().any(|color| color.z > color.x));
    }

    #[test]
    fn team_tiles_do_not_overlap_the_center_strip() {
        let mesh = DropshotTileMesh::from_states(&rocketsim::TileStates::DEFAULT, 0.5);
        let middle = rocketsim::consts::dropshot::TILE_OFFSET_Y;
        for (vertex, color) in mesh.vertices.iter().zip(&mesh.colors) {
            if color.z > color.x {
                assert!(vertex.y <= -middle);
            } else {
                assert!(vertex.y >= middle);
            }
        }
    }

    #[test]
    fn generic_mesh_omits_destroyed_tiles() {
        let centers = [Vec3::ZERO, Vec3::X * 1000.0];
        let mesh =
            DropshotTileMesh::build(&centers, &[TileDamage::Full, TileDamage::Destroyed], 0.5);
        assert_eq!(mesh.vertices.len(), 18);
        assert!(mesh.vertices.iter().all(|vertex| vertex.x < 500.0));
    }
}
