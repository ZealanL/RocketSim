//! Dynamic Dropshot tile surface geometry.

use std::sync::Arc;

use glam::{Vec3, Vec4};
use glow::HasContext;
use rocketsim::{
    TileDamageState, TileStates,
    consts::{BT_TO_UU, dropshot},
};

use crate::gl_util::{bytes_of, compile_program, uniform_location};

/// Height of the visible tile surface in world units.
pub const TILE_SURFACE_Z: f32 = 0.5;

const BLUE: Vec4 = Vec4::new(0.08, 0.42, 1.0, 1.0);
const ORANGE: Vec4 = Vec4::new(1.0, 0.32, 0.06, 1.0);
const SEAM: Vec4 = Vec4::new(0.015, 0.02, 0.03, 1.0);

/// Damage level for one tile: intact, activated, or open.
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
    /// Builds all 140 tile polygons with the simulation's center-strip clipping.
    pub fn from_states(states: &TileStates, z: f32) -> Self {
        let mut mesh = Self::default();
        let mut index = 0;
        let mut y = dropshot::TILE_OFFSET_Y;
        for row in 0..dropshot::NUM_TILE_ROWS {
            let count = usize::from(dropshot::TILES_IN_FIRST_ROW - row);
            let start_x = dropshot::TILE_WIDTH_X * (1.0 - count as f32) * 0.5;
            for col in 0..count {
                let x = start_x + dropshot::TILE_WIDTH_X * col as f32;
                for (team, color) in [BLUE, ORANGE].into_iter().enumerate() {
                    let sign = if team == 0 { -1.0 } else { 1.0 };
                    let center = Vec3::new(x * sign, y * sign, z);
                    let corners = dropshot::TILE_HEXAGON_VERTS_BT.map(|v| {
                        let mut point = center + Vec3::from(v * BT_TO_UU);
                        point.y = if team == 0 {
                            point.y.min(-dropshot::TILE_OFFSET_Y)
                        } else {
                            point.y.max(dropshot::TILE_OFFSET_Y)
                        };
                        point
                    });
                    mesh.push_tile(corners, states.states[team][index], color);
                }
                index += 1;
            }
            y += dropshot::ROW_OFFSET_Y;
        }
        debug_assert_eq!(index, dropshot::NUM_TILES_PER_TEAM);
        mesh
    }

    /// Builds blue-team tiles at caller-supplied centers, without center clipping.
    /// Open tiles retain only their edge rim, not a surface across the hole.
    pub fn build(centers: &[Vec3], damage: &[TileDamage], z: f32) -> Self {
        assert_eq!(centers.len(), damage.len());
        let mut mesh = Self::default();
        for (center, state) in centers.iter().zip(damage) {
            let corners = dropshot::TILE_HEXAGON_VERTS_BT.map(|v| {
                let offset = Vec3::from(v * BT_TO_UU);
                Vec3::new(center.x + offset.x, center.y + offset.y, z)
            });
            let state = match state {
                TileDamage::Full => TileDamageState::Full,
                TileDamage::Damaged => TileDamageState::Damaged,
                TileDamage::Destroyed => TileDamageState::Broken,
            };
            mesh.push_tile(corners, state, BLUE);
        }
        mesh
    }

    fn triangle(&mut self, points: [Vec3; 3], color: Vec4) {
        // Clipping can leave consecutive collinear corners at the center strip.
        if (points[1] - points[0])
            .cross(points[2] - points[0])
            .length_squared()
            < 1e-8
        {
            return;
        }
        self.vertices.extend_from_slice(&points);
        self.colors.extend_from_slice(&[color; 3]);
    }

    fn quad(&mut self, points: [Vec3; 4], color: Vec4) {
        self.triangle([points[0], points[1], points[2]], color);
        self.triangle([points[0], points[2], points[3]], color);
    }

    fn ring(&mut self, corners: [Vec3; 6], center: Vec3, inner: f32, outer: f32, color: Vec4) {
        for edge in 0..6 {
            let next = (edge + 1) % 6;
            self.quad(
                [
                    center.lerp(corners[edge], inner),
                    center.lerp(corners[edge], outer),
                    center.lerp(corners[next], outer),
                    center.lerp(corners[next], inner),
                ],
                color,
            );
        }
    }

    fn push_tile(&mut self, corners: [Vec3; 6], state: TileDamageState, team: Vec4) {
        let center = corners.into_iter().sum::<Vec3>() / 6.0;
        let metal = Vec4::new(0.06, 0.075, 0.095, 1.0).lerp(team, 0.12);
        if state == TileDamageState::Broken {
            self.ring(corners, center, 0.965, 0.99, team);
            self.ring(corners, center, 0.99, 1.0, SEAM);
            return;
        }

        let activated = state == TileDamageState::Damaged;
        let light = if activated {
            team.lerp(Vec4::ONE, 0.75)
        } else {
            team * Vec4::new(0.35, 0.35, 0.35, 1.0)
        };
        for edge in 0..6 {
            self.triangle(
                [
                    center,
                    center.lerp(corners[edge], 0.14),
                    center.lerp(corners[(edge + 1) % 6], 0.14),
                ],
                light,
            );
        }
        for (inner, outer, color) in [
            (0.14, 0.18, SEAM),
            (0.18, 0.34, metal),
            (0.34, 0.36, light),
            (0.36, 0.62, metal),
            (0.76, 0.79, light),
            (0.79, 0.94, metal),
            (0.94, 0.965, team * Vec4::new(0.55, 0.55, 0.55, 1.0)),
            (0.965, 1.0, SEAM),
        ] {
            self.ring(corners, center, inner, outer, color);
        }
        // Segmented hexagonal lights distinguish an activated tile from an intact panel.
        for edge in 0..6 {
            let next = (edge + 1) % 6;
            for segment in 0..12 {
                let start = corners[edge].lerp(corners[next], segment as f32 / 12.0);
                let end = corners[edge].lerp(corners[next], (segment + 1) as f32 / 12.0);
                self.quad(
                    [
                        center.lerp(start, 0.62),
                        center.lerp(start, 0.76),
                        center.lerp(end, 0.76),
                        center.lerp(end, 0.62),
                    ],
                    if segment % 2 == 0 { light } else { metal },
                );
            }
        }
    }
}

/// Reuses tile geometry until a streamed damage state changes.
#[derive(Default)]
pub struct DropshotTileCache {
    states: Option<TileStates>,
    mesh: Arc<DropshotTileMesh>,
}

impl DropshotTileCache {
    pub fn get(&mut self, states: Option<&TileStates>) -> Arc<DropshotTileMesh> {
        if self.states.as_ref() != states {
            self.states = states.copied();
            self.mesh = Arc::new(states.map_or_else(DropshotTileMesh::default, |states| {
                DropshotTileMesh::from_states(states, TILE_SURFACE_Z)
            }));
        }
        self.mesh.clone()
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
    uploaded: Option<Arc<DropshotTileMesh>>,
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
                uploaded: None,
            }
        }
    }

    /// Replaces the streamed CPU mesh. Empty geometry removes all tiles.
    pub fn upload(&mut self, gl: &glow::Context, mesh: &DropshotTileMesh) {
        self.uploaded = None;
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

    /// Uploads a cached mesh only when its shared allocation changes.
    pub fn upload_shared(&mut self, gl: &glow::Context, mesh: &Arc<DropshotTileMesh>) {
        if self
            .uploaded
            .as_ref()
            .is_some_and(|previous| Arc::ptr_eq(previous, mesh))
        {
            return;
        }
        self.upload(gl, mesh);
        self.uploaded = Some(mesh.clone());
    }

    /// Draws live tile triangles after the opaque arena pass.
    pub fn draw(&self, gl: &glow::Context, view: &glam::Mat4, proj: &glam::Mat4) {
        if self.count == 0 {
            return;
        }
        unsafe {
            gl.disable(glow::CULL_FACE);
            gl.disable(glow::BLEND);
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LEQUAL);
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

    fn tile(damage: TileDamage) -> DropshotTileMesh {
        DropshotTileMesh::build(&[Vec3::ZERO], &[damage], TILE_SURFACE_Z)
    }

    fn area(mesh: &DropshotTileMesh) -> f32 {
        mesh.vertices
            .as_chunks::<3>()
            .0
            .iter()
            .map(|tri| (tri[1] - tri[0]).cross(tri[2] - tri[0]).length() * 0.5)
            .sum()
    }

    fn covers(mesh: &DropshotTileMesh, point: Vec3) -> bool {
        mesh.vertices.as_chunks::<3>().0.iter().any(|tri| {
            let sides = [0, 1, 2].map(|i| (tri[(i + 1) % 3] - tri[i]).cross(point - tri[i]).z);
            sides.iter().all(|&v| v >= -0.001) || sides.iter().all(|&v| v <= 0.001)
        })
    }

    #[test]
    fn simulation_grid_has_team_colors_and_destroyed_holes() {
        let mut states = TileStates::DEFAULT;
        states.states[0][0] = TileDamageState::Damaged;
        states.states[1][0] = TileDamageState::Broken;
        let mesh = DropshotTileMesh::from_states(&states, TILE_SURFACE_Z);
        let full = tile(TileDamage::Full).vertices.len();
        let broken = tile(TileDamage::Destroyed).vertices.len();
        assert_eq!(mesh.vertices.len(), 139 * full + broken);
        assert_eq!(mesh.colors.len(), mesh.vertices.len());
        assert!(mesh.colors.iter().any(|color| color.x > color.z));
        assert!(mesh.colors.iter().any(|color| color.z > color.x));
    }

    #[test]
    fn team_tiles_do_not_overlap_the_center_strip() {
        let mesh = DropshotTileMesh::from_states(&TileStates::DEFAULT, TILE_SURFACE_Z);
        let middle = dropshot::TILE_OFFSET_Y;
        for triangle in mesh.vertices.as_chunks::<3>().0 {
            assert!(
                triangle.iter().all(|v| v.y <= -middle + 0.001)
                    || triangle.iter().all(|v| v.y >= middle - 0.001)
            );
        }
    }

    #[test]
    fn intact_and_activated_panels_keep_the_collision_footprint() {
        let full = tile(TileDamage::Full);
        let damaged = tile(TileDamage::Damaged);
        assert_eq!(full.vertices, damaged.vertices);
        assert_ne!(full.colors, damaged.colors);
        let expected_area =
            3.0 * (dropshot::TILE_WIDTH_X * 0.5) * dropshot::TILE_HEXAGON_VERTS_BT[3].y * BT_TO_UU;
        assert!((area(&full) - expected_area).abs() < 1.0);
        for corner in dropshot::TILE_HEXAGON_VERTS_BT {
            let expected = Vec3::new(corner.x * BT_TO_UU, corner.y * BT_TO_UU, TILE_SURFACE_Z);
            assert!(full.vertices.iter().any(|v| v.distance(expected) < 0.001));
        }
        let brightness = |mesh: &DropshotTileMesh| {
            mesh.colors
                .iter()
                .map(|c| c.x + c.y + c.z)
                .fold(0.0_f32, f32::max)
        };
        assert!(brightness(&damaged) > brightness(&full) * 2.0);
        assert!(full.colors.contains(&SEAM));
    }

    #[test]
    fn destroyed_tile_has_a_rim_but_no_surface_over_the_hole() {
        let broken = tile(TileDamage::Destroyed);
        assert!(!broken.vertices.is_empty());
        assert!(!covers(&broken, Vec3::new(0.0, 0.0, TILE_SURFACE_Z)));
        assert!(area(&broken) < area(&tile(TileDamage::Full)) * 0.1);
    }

    #[test]
    fn streamed_damage_opens_the_correct_individual_tile() {
        let mut states = TileStates::DEFAULT;
        // Row 2 starts at tile 25. Tile 27 is its third column.
        let center = Vec3::new(
            dropshot::TILE_WIDTH_X * 3.0,
            -(dropshot::TILE_OFFSET_Y + dropshot::ROW_OFFSET_Y * 2.0),
            TILE_SURFACE_Z,
        );
        assert!(covers(
            &DropshotTileMesh::from_states(&states, TILE_SURFACE_Z),
            center
        ));
        states.states[0][27] = TileDamageState::Broken;
        assert!(!covers(
            &DropshotTileMesh::from_states(&states, TILE_SURFACE_Z),
            center
        ));
        assert!(covers(
            &DropshotTileMesh::from_states(&states, TILE_SURFACE_Z),
            Vec3::new(-center.x, -center.y, center.z)
        ));
    }

    #[test]
    fn cached_geometry_changes_only_with_tile_state() {
        let mut cache = DropshotTileCache::default();
        let mut states = TileStates::DEFAULT;
        let full = cache.get(Some(&states));
        assert!(Arc::ptr_eq(&full, &cache.get(Some(&states))));
        states.states[1][40] = TileDamageState::Damaged;
        let damaged = cache.get(Some(&states));
        assert!(!Arc::ptr_eq(&full, &damaged));
        assert!(Arc::ptr_eq(&damaged, &cache.get(Some(&states))));
        states.states[1][40] = TileDamageState::Broken;
        assert!(!Arc::ptr_eq(&damaged, &cache.get(Some(&states))));
        let empty = cache.get(None);
        assert!(empty.vertices.is_empty());
        assert!(Arc::ptr_eq(&empty, &cache.get(None)));
    }
}
