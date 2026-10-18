//! Triangle-soup meshes and exact-bit welding.
//!
//! The arena visual is built from the physics collision meshes (what you
//! see is what collides) plus procedural backdrop planes. Soup corners
//! weld on exact float-bit `(position, uv)` keys: hulls share exact
//! corner values, so no epsilon is needed.

use glam::{EulerRot, Mat3, Mat4, Vec2, Vec3};
use rocketsim::{Arena, CollisionMeshFile, GameMode, consts::BT_TO_UU};
use rustc_hash::FxHashMap;

use crate::obj::smooth_normals;

/// Triangle soup with per-vertex UVs, in Unreal units (uu).
#[derive(Debug, Clone)]
pub struct SoupMesh {
    /// Every 3 verts form one triangle.
    pub verts: Vec<Vec3>,
    /// One UV per vert.
    pub uvs: Vec<Vec2>,
}

impl SoupMesh {
    /// Expands one collision-mesh component to soup (mid UVs — physics has
    /// no UVs).
    fn from_mesh(mesh: &CollisionMeshFile) -> Self {
        let mut verts = Vec::new();
        let mut uvs = Vec::new();
        let m_verts = mesh.get_vertices();
        for tri in mesh.get_indices().chunks(3) {
            for &i in tri {
                verts.push(Vec3::from(m_verts[i]) * BT_TO_UU);
                uvs.push(Vec2::splat(0.5));
            }
        }
        Self { verts, uvs }
    }

    /// Expands collision-mesh tris to soup (mid UVs — physics has no UVs).
    pub fn from_arena_meshes(meshes: &[CollisionMeshFile]) -> Self {
        let mut verts = Vec::new();
        let mut uvs = Vec::new();
        for mesh in meshes {
            let soup = Self::from_mesh(mesh);
            verts.extend_from_slice(&soup.verts);
            uvs.extend_from_slice(&soup.uvs);
        }
        Self { verts, uvs }
    }

    /// Single ±10000uu quad in the XY plane (normal +Z, UVs 0..1).
    pub fn big_plane() -> Self {
        Self::quad(-10_000.0, 10_000.0, -10_000.0, 10_000.0, 0.0)
    }

    /// Axis-aligned playing-surface quad at height `z`. The physics hull
    /// is open at the bottom (floor collision is analytic), so this cap
    /// sits just under the wall/goal bases.
    pub fn field_cap(x0: f32, x1: f32, y0: f32, y1: f32, z: f32) -> Self {
        Self::quad(x0, x1, y0, y1, z)
    }

    /// Axis-aligned quad in the XY plane at height `z` (normal +Z, UVs 0..1).
    fn quad(x0: f32, x1: f32, y0: f32, y1: f32, z: f32) -> Self {
        let corners = [
            Vec3::new(x0, y0, z),
            Vec3::new(x1, y0, z),
            Vec3::new(x1, y1, z),
            Vec3::new(x0, y1, z),
        ];
        let corner_uvs = [
            Vec2::new(0.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(0.0, 1.0),
        ];
        let mut verts = Vec::with_capacity(6);
        let mut uvs = Vec::with_capacity(6);
        for i in [0, 1, 2, 0, 2, 3] {
            verts.push(corners[i]);
            uvs.push(corner_uvs[i]);
        }
        Self { verts, uvs }
    }

    /// Rigid transform (rotation + offset); UVs pass through.
    pub fn transformed(&self, offset: Vec3, basis: Mat3) -> Self {
        Self {
            verts: self.verts.iter().map(|v| basis * *v + offset).collect(),
            uvs: self.uvs.clone(),
        }
    }

    /// Concatenates soups (arena hull + backdrop planes).
    pub fn concat(parts: &[&Self]) -> Self {
        let mut verts = Vec::new();
        let mut uvs = Vec::new();
        for part in parts {
            verts.extend_from_slice(&part.verts);
            uvs.extend_from_slice(&part.uvs);
        }
        Self { verts, uvs }
    }

    /// Welds soup into unique `(position, uv)` corners plus indices.
    /// Exact float-bit keys: only bit-identical corners merge.
    pub fn weld(&self) -> (Vec<Vec3>, Vec<Vec2>, Vec<u32>) {
        assert_eq!(self.verts.len(), self.uvs.len());
        let mut uniq_verts = Vec::new();
        let mut uniq_uvs = Vec::new();
        let mut vert_index = FxHashMap::default();
        let mut indices = Vec::with_capacity(self.verts.len());
        for (pos, uv) in self.verts.iter().zip(self.uvs.iter()) {
            let key = (
                pos.to_array().map(f32::to_bits),
                uv.to_array().map(f32::to_bits),
            );
            if let Some(&idx) = vert_index.get(&key) {
                indices.push(idx);
            } else {
                assert!(
                    uniq_verts.len() < u32::MAX as usize,
                    "welded vertex count exceeds u32 range"
                );
                let idx = uniq_verts.len() as u32;
                uniq_verts.push(*pos);
                uniq_uvs.push(*uv);
                vert_index.insert(key, idx);
                indices.push(idx);
            }
        }
        (uniq_verts, uniq_uvs, indices)
    }
}

/// One welded index pool (positions + triangles over them).
pub struct ArenaPart {
    /// Welded positions (uu).
    pub verts: Vec<Vec3>,
    /// Welded UVs (mid-UV for hull, 0..1 for procedural).
    pub uvs: Vec<Vec2>,
    /// Smooth per-vertex normals (unit, or zero on degenerate tris).
    pub normals: Vec<Vec3>,
    /// Triangles over `verts`.
    pub indices: Vec<u32>,
}

impl ArenaPart {
    /// Welds a soup subset (see [`SoupMesh::weld`]).
    fn from_soup(soup: &SoupMesh) -> Self {
        let (verts, uvs, indices) = soup.weld();
        let normals = smooth_normals(&verts, &indices);
        Self {
            verts,
            uvs,
            normals,
            indices,
        }
    }
}

/// Indexed arena visual for a game mode: open bowl (shell), ceiling lid,
/// playing-surface cap, and backdrop planes weld (and draw) separately.
/// Side and roof backdrops sit 5uu outside the mesh extent. Hoops end walls
/// sit just outside the physics bounds so the rim does not move the walls.
pub struct ArenaMesh {
    /// Mode this mesh was built for (drives scene rebuilds on switches).
    pub mode: GameMode,
    /// Hull without ceiling (walls, goals, curbs).
    pub shell: ArenaPart,
    /// Hull ceiling (hidden for open-bowl shots, shown from inside).
    pub lid: ArenaPart,
    /// Hoops net (ball-only collision): drawn translucent, never occludes.
    /// Empty in every other mode.
    pub net: ArenaPart,
    /// Procedural field cap (the visible playing surface).
    pub field: ArenaPart,
    /// Backdrop floor and side walls. Dropshot has no backdrop floor.
    pub planes_low: ArenaPart,
    /// Opaque Hoops end walls. Outside faces are hidden for cutaway views.
    pub planes_end: ArenaPart,
    /// Backdrop ceiling (interior views only).
    pub planes_high: ArenaPart,
    /// Pre-weld soup vertex count (weld-win denominator).
    pub soup_verts: usize,
}

impl ArenaMesh {
    /// Builds an arena visual from streamed world-space mesh data.
    pub fn from_meshes(game_mode: GameMode, meshes: &[rocketsim_vis::proto::MeshData]) -> Self {
        let mut hull_soup = SoupMesh {
            verts: Vec::new(),
            uvs: Vec::new(),
        };
        let mut net_soup = SoupMesh {
            verts: Vec::new(),
            uvs: Vec::new(),
        };
        for mesh in meshes {
            assert_eq!(
                mesh.indices.len() % 3,
                0,
                "metadata mesh indices must be triangles"
            );
            let dest = if game_mode == GameMode::Hoops
                && mesh.indices.len() / 3 == Arena::HOOPS_NET_FACE_COUNT
            {
                &mut net_soup
            } else {
                &mut hull_soup
            };
            for &idx in &mesh.indices {
                let p = mesh.vertices[idx as usize];
                dest.verts.push(Vec3::from_array(p));
                dest.uvs.push(Vec2::splat(0.5));
            }
        }
        Self::from_hull_soups(game_mode, hull_soup, net_soup)
    }

    /// Builds an empty arena mesh for startup before simulation metadata arrives.
    pub fn placeholder(mode: GameMode) -> Self {
        let empty = || ArenaPart {
            verts: Vec::new(),
            uvs: Vec::new(),
            normals: Vec::new(),
            indices: Vec::new(),
        };
        Self {
            mode,
            shell: empty(),
            lid: empty(),
            net: empty(),
            field: empty(),
            planes_low: empty(),
            planes_end: empty(),
            planes_high: empty(),
            soup_verts: 0,
        }
    }

    /// Ceiling test: face normal mostly vertical AND high up (normal alone
    /// would also grab the flat floor).
    const LID_MIN_Z: f32 = 1500.0;

    /// Builds hull soup, splits the ceiling, adds backdrop planes, and welds each
    /// part. Needs [`crate::photo::init_meshes`] first (same as the sim).
    /// TheVoid has no hull — shell/lid stay empty, planes still build.
    pub fn build(game_mode: GameMode) -> Self {
        // The Hoops net collides with the ball only (cars pass through;
        // see `Arena::HOOPS_NET_FACE_COUNT`): it welds/draws as its own
        // part so the viewer can render it translucent. Everything else
        // is hull.
        let mut hull_soup = SoupMesh {
            verts: Vec::new(),
            uvs: Vec::new(),
        };
        let mut net_soup = SoupMesh {
            verts: Vec::new(),
            uvs: Vec::new(),
        };
        for mesh in &rocketsim::get_arena_collision_mesh_files(game_mode) {
            let is_net = game_mode == GameMode::Hoops
                && mesh.get_indices().len() / 3 == Arena::HOOPS_NET_FACE_COUNT;
            let source = SoupMesh::from_mesh(mesh);
            let dest = if is_net {
                &mut net_soup
            } else {
                &mut hull_soup
            };
            dest.verts.extend_from_slice(&source.verts);
            dest.uvs.extend_from_slice(&source.uvs);
        }
        Self::from_hull_soups(game_mode, hull_soup, net_soup)
    }

    fn from_hull_soups(game_mode: GameMode, hull_soup: SoupMesh, net_soup: SoupMesh) -> Self {
        let mut shell_soup = SoupMesh {
            verts: Vec::new(),
            uvs: Vec::new(),
        };
        let mut lid_soup = SoupMesh {
            verts: Vec::new(),
            uvs: Vec::new(),
        };
        for (tri, uvs) in hull_soup.verts.chunks(3).zip(hull_soup.uvs.chunks(3)) {
            let n = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
            let z_min = tri[0].z.min(tri[1].z).min(tri[2].z);
            let dest = if z_min > Self::LID_MIN_Z && n.z.abs() > n.x.abs().max(n.y.abs()) {
                &mut lid_soup
            } else {
                &mut shell_soup
            };
            dest.verts.extend_from_slice(tri);
            dest.uvs.extend_from_slice(uvs);
        }
        let aabb = rocketsim::consts::arena::get_aabb(game_mode);
        // Side and roof backdrops sit outside the curved collision mesh.
        let gap = 5.0;
        let (mesh_min_x, mesh_max_x, mesh_min_y, mesh_max_y, mesh_max_z) = {
            let (mut mnx, mut mxx, mut mny, mut mxy, mut mxz) =
                (f32::MAX, f32::MIN, f32::MAX, f32::MIN, f32::MIN);
            for v in shell_soup.verts.iter().chain(lid_soup.verts.iter()) {
                mnx = mnx.min(v.x);
                mxx = mxx.max(v.x);
                mny = mny.min(v.y);
                mxy = mxy.max(v.y);
                mxz = mxz.max(v.z);
            }
            // TheVoid hull is empty: fall back to the logic AABB.
            if mnx > mxx {
                (aabb.min.x, aabb.max.x, aabb.min.y, aabb.max.y, aabb.max.z)
            } else {
                (mnx, mxx, mny, mxy, mxz)
            }
        };
        let plane = SoupMesh::big_plane();
        let floor = if game_mode == GameMode::Dropshot {
            SoupMesh {
                verts: Vec::new(),
                uvs: Vec::new(),
            }
        } else {
            plane.transformed(
                Vec3::new(0.0, 0.0, aabb.min.z - gap),
                Mat3::from_euler(EulerRot::ZYX, 0.0, 0.0, 0.0),
            )
        };
        let ceiling = plane.transformed(
            Vec3::new(0.0, 0.0, mesh_max_z + gap),
            Mat3::from_euler(EulerRot::ZYX, 0.0, 0.0, std::f32::consts::PI),
        );
        let wall_neg_x = plane.transformed(
            Vec3::new(mesh_min_x - gap, 0.0, aabb.center().z),
            Mat3::from_euler(EulerRot::ZYX, 0.0, std::f32::consts::FRAC_PI_2, 0.0),
        );
        let wall_pos_x = plane.transformed(
            Vec3::new(mesh_max_x + gap, 0.0, aabb.center().z),
            Mat3::from_euler(EulerRot::ZYX, 0.0, -std::f32::consts::FRAC_PI_2, 0.0),
        );
        // Cap footprint covers the bowl; edges hide inside the walls.
        let field_soup = if game_mode == GameMode::Dropshot {
            SoupMesh {
                verts: Vec::new(),
                uvs: Vec::new(),
            }
        } else {
            SoupMesh::field_cap(
                mesh_min_x + gap,
                mesh_max_x - gap,
                mesh_min_y + gap,
                mesh_max_y - gap,
                0.5,
            )
        };
        let planes_low_soup = SoupMesh::concat(&[&floor, &wall_neg_x, &wall_pos_x]);
        let planes_end_soup = if game_mode == GameMode::Hoops {
            // The rim extends behind the physics end walls. Use the physics
            // bounds to hide those parts and meet the curved wall mesh.
            let end_gap = 0.5;
            let wall_neg_y = plane.transformed(
                Vec3::new(0.0, aabb.min.y - end_gap, aabb.center().z),
                Mat3::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            );
            let wall_pos_y = plane.transformed(
                Vec3::new(0.0, aabb.max.y + end_gap, aabb.center().z),
                Mat3::from_rotation_x(std::f32::consts::FRAC_PI_2),
            );
            SoupMesh::concat(&[&wall_neg_y, &wall_pos_y])
        } else {
            SoupMesh {
                verts: Vec::new(),
                uvs: Vec::new(),
            }
        };
        let planes_high_soup = SoupMesh::concat(&[&ceiling]);
        let soup_verts = shell_soup.verts.len()
            + lid_soup.verts.len()
            + net_soup.verts.len()
            + field_soup.verts.len()
            + planes_low_soup.verts.len()
            + planes_end_soup.verts.len()
            + planes_high_soup.verts.len();
        Self {
            mode: game_mode,
            shell: ArenaPart::from_soup(&shell_soup),
            lid: ArenaPart::from_soup(&lid_soup),
            net: ArenaPart::from_soup(&net_soup),
            field: ArenaPart::from_soup(&field_soup),
            planes_low: ArenaPart::from_soup(&planes_low_soup),
            planes_end: ArenaPart::from_soup(&planes_end_soup),
            planes_high: ArenaPart::from_soup(&planes_high_soup),
            soup_verts,
        }
    }

    /// Welded vertex total (all parts).
    pub fn welded_verts(&self) -> usize {
        self.shell.verts.len()
            + self.lid.verts.len()
            + self.net.verts.len()
            + self.field.verts.len()
            + self.planes_low.verts.len()
            + self.planes_end.verts.len()
            + self.planes_high.verts.len()
    }

    /// Index total (all parts).
    pub fn index_count(&self) -> usize {
        self.shell.indices.len()
            + self.lid.indices.len()
            + self.net.indices.len()
            + self.field.indices.len()
            + self.planes_low.indices.len()
            + self.planes_end.indices.len()
            + self.planes_high.indices.len()
    }

    /// Model matrix for arena draws: identity (already in world units).
    pub fn model_matrix() -> Mat4 {
        Mat4::IDENTITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn metadata_mesh_builds_without_sim_initialization() {
        let input = [rocketsim_vis::proto::MeshData {
            vertices: vec![[1.0, 2.0, 3.0], [4.0, 2.0, 3.0], [1.0, 6.0, 3.0]],
            indices: vec![0, 1, 2],
        }];
        let mesh = ArenaMesh::from_meshes(GameMode::Soccar, &input);
        assert_eq!(mesh.shell.verts.len(), 3);
        assert_eq!(mesh.shell.indices, vec![0, 1, 2]);
        assert_eq!(mesh.shell.verts[1], Vec3::new(4.0, 2.0, 3.0));
        assert_eq!(mesh.mode, GameMode::Soccar);
        assert_eq!(mesh.field.indices.len(), 6);
        assert_eq!(mesh.planes_low.indices.len(), 18);
        assert_eq!(mesh.planes_high.indices.len(), 6);
        assert_eq!(mesh.index_count(), 33);
        assert!(mesh.field.uvs.iter().all(|uv| *uv != Vec2::splat(0.5)));
    }

    #[test]
    fn streamed_dropshot_preserves_neutral_center_strip() {
        let strip = [
            [-4892.0, -128.0, 0.0],
            [4876.0, -128.0, 0.0],
            [4876.0, 128.0, 0.0],
        ];
        let wall = [
            [100.0, 200.0, 300.0],
            [100.0, 200.0, 500.0],
            [100.0, 400.0, 500.0],
        ];
        let input = [rocketsim_vis::proto::MeshData {
            vertices: strip.into_iter().chain(wall).collect(),
            indices: vec![0, 1, 2, 3, 4, 5],
        }];
        let dropshot = ArenaMesh::from_meshes(GameMode::Dropshot, &input);
        assert_eq!(dropshot.shell.indices.len(), 6);
        assert!(dropshot.field.indices.is_empty());
        assert_eq!(
            dropshot.planes_low.indices.len(),
            12,
            "only backdrop walls remain"
        );
        assert_eq!(
            dropshot.planes_high.indices.len(),
            6,
            "backdrop ceiling remains"
        );
        assert!(dropshot.shell.verts.iter().any(|p| p.x == -4892.0));
        assert!(dropshot.shell.verts.iter().any(|p| p.z == 500.0));

        for mode in [GameMode::Soccar, GameMode::Hoops] {
            let mesh = ArenaMesh::from_meshes(mode, &input);
            assert_eq!(mesh.shell.indices.len(), 6, "{mode:?} stream changed");
        }
    }

    #[test]
    fn streamed_empty_void_builds_backdrop_without_sim_initialization() {
        let mesh = ArenaMesh::from_meshes(GameMode::TheVoid, &[]);
        assert!(mesh.shell.indices.is_empty());
        assert!(mesh.lid.indices.is_empty());
        assert_eq!(mesh.field.indices.len(), 6);
        assert_eq!(mesh.planes_low.indices.len(), 18);
        assert_eq!(mesh.planes_high.indices.len(), 6);
    }

    #[test]
    fn only_dropshot_omits_procedural_floor_surfaces() {
        for mode in [
            GameMode::Soccar,
            GameMode::Hoops,
            GameMode::Snowday,
            GameMode::Heatseeker,
            GameMode::Dropshot,
            GameMode::TheVoid,
        ] {
            let mesh = ArenaMesh::from_meshes(mode, &[]);
            let dropshot = mode == GameMode::Dropshot;
            assert_eq!(mesh.field.indices.is_empty(), dropshot, "{mode:?}");
            assert_eq!(
                mesh.planes_low.indices.len(),
                if dropshot { 12 } else { 18 },
                "{mode:?}"
            );
            assert_eq!(mesh.planes_high.indices.len(), 6, "{mode:?}");
            assert_eq!(
                mesh.planes_end.indices.len(),
                if mode == GameMode::Hoops { 12 } else { 0 },
                "{mode:?}"
            );
        }
    }

    #[test]
    fn placeholder_has_no_geometry() {
        let mesh = ArenaMesh::placeholder(GameMode::Dropshot);
        assert_eq!(mesh.mode, GameMode::Dropshot);
        assert_eq!(mesh.index_count(), 0);
        assert_eq!(mesh.welded_verts(), 0);
    }

    fn check_part(part: &ArenaPart, label: &str) {
        assert_eq!(part.indices.len() % 3, 0, "{label}: index count");
        for &i in &part.indices {
            assert!(
                (i as usize) < part.verts.len(),
                "{label}: index out of range"
            );
        }
        for v in &part.verts {
            assert!(v.is_finite(), "{label}: non-finite vert");
        }
        assert_eq!(
            part.verts.len(),
            part.uvs.len(),
            "{label}: vert/uv mismatch"
        );
        assert_eq!(
            part.verts.len(),
            part.normals.len(),
            "{label}: vert/normal mismatch"
        );
        for n in &part.normals {
            let len = n.length();
            assert!(
                len < 1e-6 || (len - 1.0).abs() < 1e-4,
                "{label}: normal not unit-or-zero ({len})"
            );
        }
    }

    fn check_sanity(mesh: &ArenaMesh, label: &str) {
        // Lid is Soccar-only asserted (Dropshot is open-top); everything else
        // must exist in every mode.
        assert!(!mesh.shell.verts.is_empty(), "{label}: no shell verts");
        assert!(!mesh.planes_low.verts.is_empty(), "{label}: no plane verts");
        assert!(
            !mesh.planes_high.verts.is_empty(),
            "{label}: no ceiling plane"
        );
        check_part(&mesh.shell, &format!("{label} shell"));
        check_part(&mesh.lid, &format!("{label} lid"));
        check_part(&mesh.net, &format!("{label} net"));
        check_part(&mesh.field, &format!("{label} field"));
        check_part(&mesh.planes_low, &format!("{label} planes-low"));
        check_part(&mesh.planes_end, &format!("{label} planes-end"));
        check_part(&mesh.planes_high, &format!("{label} planes-high"));
        // Partition accounting (every hull soup tri lands in shell xor lid)
        // is structural: weld preserves tri counts, and `lid_is_only_roof`
        // pins the classification boundary.
    }

    #[test]
    fn mesh_remembers_its_mode() {
        crate::photo::init_meshes();
        // The GL scene rebuilds off this tag on stream-mode switches.
        for mode in [
            GameMode::Soccar,
            GameMode::Hoops,
            GameMode::Dropshot,
            GameMode::TheVoid,
        ] {
            assert_eq!(ArenaMesh::build(mode).mode, mode);
        }
    }

    #[test]
    fn soccar_weld_ratio() {
        crate::photo::init_meshes();
        let mesh = ArenaMesh::build(GameMode::Soccar);
        check_sanity(&mesh, "soccar");
        assert!(!mesh.lid.verts.is_empty(), "soccar must have a ceiling lid");
        let ratio = mesh.soup_verts as f64 / mesh.welded_verts() as f64;
        // Measured ~5.4x; 4x is generous headroom against hull changes.
        assert!(ratio >= 4.0, "weld win collapsed: {ratio:.2}x");
    }

    #[test]
    fn streamed_hoops_splits_only_net_components() {
        let net = rocketsim_vis::proto::MeshData {
            vertices: vec![
                [-50.0, -500.0, 500.0],
                [50.0, -500.0, 500.0],
                [0.0, -400.0, 600.0],
            ],
            indices: [0, 1, 2].repeat(Arena::HOOPS_NET_FACE_COUNT),
        };
        let mut other_net = net.clone();
        for vertex in &mut other_net.vertices {
            vertex[1] = -vertex[1];
        }
        let wall = rocketsim_vis::proto::MeshData {
            vertices: vec![
                [-100.0, 20.0, 0.0],
                [100.0, 20.0, 0.0],
                [-100.0, 100.0, 100.0],
            ],
            indices: vec![0, 1, 2],
        };
        let input = [net, other_net, wall];
        let hoops = ArenaMesh::from_meshes(GameMode::Hoops, &input);
        assert_eq!(hoops.net.indices.len(), 2 * Arena::HOOPS_NET_FACE_COUNT * 3);
        assert_eq!(hoops.shell.indices.len(), 3);
        assert!(hoops.shell.verts.iter().all(|vertex| vertex.y >= 20.0));
        assert!(hoops.net.verts.iter().any(|vertex| vertex.y < 0.0));
        assert!(hoops.net.verts.iter().any(|vertex| vertex.y > 0.0));
        for mode in [
            GameMode::Soccar,
            GameMode::Heatseeker,
            GameMode::Snowday,
            GameMode::Dropshot,
            GameMode::TheVoid,
        ] {
            let mesh = ArenaMesh::from_meshes(mode, &input);
            assert!(mesh.net.indices.is_empty(), "{mode:?} must stay opaque");
            assert_eq!(
                mesh.shell.indices.len(),
                2 * Arena::HOOPS_NET_FACE_COUNT * 3 + 3
            );
        }
    }

    #[test]
    fn streamed_hoops_matches_local_mesh_partitions() {
        crate::photo::init_meshes();
        let arena = Arena::new(GameMode::Hoops);
        let scene = rocketsim_vis::proto::SceneMetadata::from_arena(&arena);
        let streamed = ArenaMesh::from_meshes(scene.game_mode, &scene.meshes);
        let local = ArenaMesh::build(GameMode::Hoops);
        assert_eq!(
            streamed.net.indices.len(),
            2 * Arena::HOOPS_NET_FACE_COUNT * 3
        );
        for (streamed, local) in [
            (&streamed.shell, &local.shell),
            (&streamed.lid, &local.lid),
            (&streamed.net, &local.net),
            (&streamed.field, &local.field),
            (&streamed.planes_low, &local.planes_low),
            (&streamed.planes_end, &local.planes_end),
            (&streamed.planes_high, &local.planes_high),
        ] {
            assert_eq!(streamed.verts, local.verts);
            assert_eq!(streamed.normals, local.normals);
            assert_eq!(streamed.uvs, local.uvs);
            assert_eq!(streamed.indices, local.indices);
        }
    }

    #[test]
    fn hoops_end_wall_planes_face_inward() {
        let hoops = ArenaMesh::from_meshes(GameMode::Hoops, &[]);
        let bounds = rocketsim::consts::arena::get_aabb(GameMode::Hoops);
        assert!(hoops.net.indices.is_empty());
        assert_eq!(hoops.planes_end.indices.len(), 12);
        for (y, normal) in [
            (bounds.min.y - 0.5, Vec3::Y),
            (bounds.max.y + 0.5, Vec3::NEG_Y),
        ] {
            let faces: Vec<_> = hoops
                .planes_end
                .indices
                .as_chunks::<3>()
                .0
                .iter()
                .filter(|indices| {
                    indices
                        .iter()
                        .all(|&index| (hoops.planes_end.verts[index as usize].y - y).abs() < 0.01)
                })
                .collect();
            assert_eq!(faces.len(), 2, "missing opaque end wall at y={y}");
            for indices in faces {
                assert!(
                    indices
                        .iter()
                        .all(|&index| hoops.planes_end.normals[index as usize].dot(normal) > 0.999)
                );
            }
        }
    }

    #[test]
    fn hoops_end_walls_follow_physics_not_rim_extent() {
        crate::photo::init_meshes();
        let hoops = ArenaMesh::build(GameMode::Hoops);
        let bounds = rocketsim::consts::arena::get_aabb(GameMode::Hoops);
        assert_eq!(hoops.planes_end.indices.len(), 12);
        for (y, sign) in [(bounds.min.y - 0.5, -1.0), (bounds.max.y + 0.5, 1.0)] {
            let wall: Vec<_> = hoops
                .planes_end
                .verts
                .iter()
                .filter(|vertex| vertex.y * sign > 0.0)
                .collect();
            assert_eq!(wall.len(), 4);
            assert!(wall.iter().all(|vertex| (vertex.y - y).abs() < 0.01));
            assert!(
                hoops
                    .shell
                    .verts
                    .iter()
                    .any(|vertex| (vertex.y - y) * sign > 100.0),
                "rim geometry must not push the end-wall backdrop away from the physics plane"
            );
        }
    }

    #[test]
    fn hoops_net_splits_translucent() {
        crate::photo::init_meshes();
        // The 798-face net collides with the ball only: it must leave the
        // opaque shell and land whole in the translucent net part (weld
        // preserves tri counts, so the split is exact).
        let hoops = ArenaMesh::build(GameMode::Hoops);
        check_sanity(&hoops, "hoops");
        // Whole 798-face components only (two rims → two net components);
        // mirrors the sim's per-component HoopsNet rule.
        let net_tris = hoops.net.indices.len() / 3;
        assert_ne!(net_tris, 0, "hoops net must be non-empty");
        assert_eq!(
            net_tris % Arena::HOOPS_NET_FACE_COUNT,
            0,
            "hoops net must hold whole net components (got {net_tris} tris)"
        );
        // No other mode has a net part.
        for mode in [
            GameMode::Soccar,
            GameMode::Heatseeker,
            GameMode::Snowday,
            GameMode::Dropshot,
            GameMode::TheVoid,
        ] {
            let mesh = ArenaMesh::build(mode);
            assert!(
                mesh.net.indices.is_empty(),
                "{mode:?} must have no net part"
            );
        }
    }

    #[test]
    fn dropshot_has_no_default_field_or_backdrop_floor() {
        crate::photo::init_meshes();
        let mesh = ArenaMesh::build(GameMode::Dropshot);
        assert!(mesh.field.indices.is_empty());
        // Only the two vertical backdrop walls remain.
        assert_eq!(mesh.planes_low.indices.len(), 12);
        assert!(mesh.planes_low.normals.iter().all(|n| n.z.abs() < 0.01));
    }

    #[test]
    fn field_cap_shape() {
        crate::photo::init_meshes();
        let mesh = ArenaMesh::build(GameMode::Soccar);
        // One welded quad: 4 corners, 2 tris, flat at cap height.
        assert_eq!(mesh.field.verts.len(), 4);
        assert_eq!(mesh.field.indices.len(), 6);
        for v in &mesh.field.verts {
            assert!((v.z - 0.5).abs() < 1e-6, "cap must be flat at z=0.5");
            assert!(v.x.abs() < 10_000.0 && v.y.abs() < 10_000.0);
        }
        // Cap covers the bowl (past the goals) but stays inside the apron.
        let max_y = mesh
            .field
            .verts
            .iter()
            .map(|v| v.y.abs())
            .fold(0.0_f32, f32::max);
        assert!(max_y > 5900.0 && max_y < 10_000.0, "cap y extent {max_y}");
    }

    #[test]
    fn lid_is_only_roof() {
        crate::photo::init_meshes();
        let mesh = ArenaMesh::build(GameMode::Soccar);
        let min_z = |part: &ArenaPart| part.verts.iter().map(|v| v.z).fold(f32::MAX, f32::min);
        assert!(min_z(&mesh.shell) < 20.0, "shell must reach the floor");
        assert!(
            min_z(&mesh.lid) > ArenaMesh::LID_MIN_Z,
            "lid must stay high, min_z={}",
            min_z(&mesh.lid)
        );
    }

    #[test]
    fn all_modes_build_sane() {
        crate::photo::init_meshes();
        for mode in [
            GameMode::Soccar,
            GameMode::Hoops,
            GameMode::Heatseeker,
            GameMode::Snowday,
            GameMode::Dropshot,
        ] {
            let mesh = ArenaMesh::build(mode);
            check_sanity(&mesh, &format!("{mode:?}"));
        }
        // TheVoid has no hull and must not panic — shell/lid stay empty.
        let mesh = ArenaMesh::build(GameMode::TheVoid);
        check_part(&mesh.planes_low, "void planes-low");
        check_part(&mesh.planes_high, "void planes-high");
        assert!(mesh.shell.verts.is_empty());
        assert!(mesh.lid.verts.is_empty());
    }

    #[test]
    fn weld_basics() {
        let plane = SoupMesh::big_plane();
        assert_eq!(plane.verts.len(), 6);
        let (verts, _, indices) = plane.weld();
        assert_eq!(verts.len(), 4, "quad corners must weld to 4");
        assert_eq!(indices.len(), 6);
        // Two tris sharing an edge: 6 soup verts -> 4 unique.
        let soup = SoupMesh {
            verts: vec![
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ],
            uvs: vec![Vec2::splat(0.5); 6],
        };
        let (verts, _, indices) = soup.weld();
        assert_eq!(verts.len(), 4);
        assert_eq!(indices.len(), 6);
    }
}
