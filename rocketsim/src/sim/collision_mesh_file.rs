use std::{
    io::{Cursor, Result as IoResult},
    num::Wrapping,
};

use byteorder::{LittleEndian, ReadBytesExt};
use glam::Vec3A;
use log::info;

use crate::bullet::collision::shapes::triangle_mesh::TriangleMesh;

/// Default folder searched by [`crate::init_from_default`] (`./collision_meshes/`).
pub const COLLISION_MESH_BASE_PATH: &str = "./collision_meshes/";
/// Mesh file extension (`.cmf`) scanned by [`crate::init`].
pub const COLLISION_MESH_FILE_EXTENSION: &str = "cmf";

/// Recovered north/south goal component translation in Bullet units.
/// Target runtime proves GJK input transB is (0, +102.4, 0) for north-goal
/// triangles and symmetric (0, -102.4, 0) south. 102.4 BT equals the
/// 5120 UU back-wall plane (BT_TO_UU is 50). Only goal components use a
/// non-zero translation. All other components stay at identity.
pub const GOAL_COMPONENT_TRANSLATION_BT: f32 = 102.4;

trait FromCursor {
    fn from_cursor(bytes: &mut Cursor<&[u8]>) -> IoResult<Self>
    where
        Self: Sized;
}

impl FromCursor for Vec3A {
    fn from_cursor(bytes: &mut Cursor<&[u8]>) -> IoResult<Self> {
        Ok(Self::new(
            bytes.read_f32::<LittleEndian>()?,
            bytes.read_f32::<LittleEndian>()?,
            bytes.read_f32::<LittleEndian>()?,
        ))
    }
}

/// One parsed `.cmf` arena component (triangles + vertices in Bullet units).
///
/// Obtain via [`crate::get_arena_collision_mesh_files`] after [`crate::init`].
/// `get_vertices`/`get_indices` stay in world coordinates;
/// `component_translation`/`make_bullet_mesh_local` expose the goal-component
/// fixup used internally at init.
#[derive(Debug, Clone)]
pub struct CollisionMeshFile {
    indices: Vec<usize>,
    vertices: Vec<Vec3A>,
    hash: u32,
}

impl CollisionMeshFile {
    /// Hash identifying which known arena component this is.
    ///
    /// [`crate::init`] matches it against the expected per-mode hashes and
    /// warns/skips unknown or duplicate meshes.
    #[inline]
    pub const fn get_hash(&self) -> u32 {
        self.hash
    }

    /// From: <https://stackoverflow.com/questions/20511347/a-good-hash-function-for-a-vector/72073933#72073933>
    #[allow(clippy::cast_sign_loss)]
    #[allow(clippy::cast_possible_truncation)]
    fn calculate_hash(indices: &Vec<usize>, vertices: &[Vec3A]) -> u32 {
        const HASH_VAL_MUELLER: Wrapping<u32> = Wrapping(0x45D_9F3B);
        const HASH_VAL_SHIFT: Wrapping<u32> = Wrapping(0x9E37_79B9);

        let mut hash = Wrapping((vertices.len() + (indices.len() / 3 * vertices.len())) as u32);

        for &vert_idx in indices {
            for pos in vertices[vert_idx].to_array() {
                let mut cur_val = Wrapping(pos as i32 as u32);
                cur_val = ((cur_val >> 16) ^ cur_val) * HASH_VAL_MUELLER;
                cur_val = ((cur_val >> 16) ^ cur_val) * HASH_VAL_MUELLER;
                cur_val = (cur_val >> 16) ^ cur_val;
                hash ^= cur_val + HASH_VAL_SHIFT + (hash << 6) + (hash >> 2);
            }
        }

        hash.0
    }

    /// Parses the `.cmf` binary format (`u32 tri_count, u32 vert_count`,
    /// then packed triangles and `f32 xyz` vertices, little-endian).
    ///
    /// # Errors
    ///
    /// Returns an I/O error on truncated data; panics (debug) or mis-hashes
    /// on out-of-range indices / empty / oversized (>1M) headers.
    pub fn read_from_bytes(bytes: &[u8]) -> IoResult<Self> {
        const MAX_VERT_OR_TRI_COUNT: usize = 1_000_000;

        let mut bytes = Cursor::new(bytes);
        let num_tris = bytes.read_u32::<LittleEndian>()? as usize;
        let num_indices = num_tris * 3;
        let num_vertices = bytes.read_u32::<LittleEndian>()? as usize;

        assert!(
            num_tris.min(num_vertices) != 0 && num_tris.max(num_vertices) <= MAX_VERT_OR_TRI_COUNT,
            "Invalid collision mesh file (bad triangle/vertex count: [{num_tris}/{num_vertices}])"
        );

        let mut indices = Vec::with_capacity(num_indices);
        for _ in 0..num_indices {
            indices.push(bytes.read_u32::<LittleEndian>()? as usize);
        }

        let mut vertices = Vec::with_capacity(num_vertices);
        for _ in 0..num_vertices {
            vertices.push(Vec3A::from_cursor(&mut bytes)?);
        }

        #[cfg(debug_assertions)]
        {
            // Verify that the triangle data is correct
            for &vert_idx in &indices {
                assert!(
                    vert_idx < num_vertices,
                    "Invalid collision mesh file (bad triangle vertex index)"
                );
            }
        }

        let hash = Self::calculate_hash(&indices, &vertices);

        info!("\tLoaded {num_vertices} verts and {num_tris} tris, hash: {hash:#x}");

        Ok(Self {
            indices,
            vertices,
            hash,
        })
    }

    /// Triangle mesh in stored (world) coordinates.
    pub fn make_bullet_mesh(&self) -> TriangleMesh {
        TriangleMesh::new(&self.vertices, &self.indices)
    }

    /// Geometric goal-component test. Use the world-vertex AABB center on
    /// the Y axis. North/south goal halves center at |y| ~ 103.4 BT, past
    /// the 102.4 BT back-wall plane. All other Soccar components center at
    /// |y| <= 84.5 BT. Select only by geometry, never by scenario, tick,
    /// or triangle ID. Start with north/south only because only their
    /// transforms are proven.
    pub fn component_translation(&self) -> Vec3A {
        let mut min = Vec3A::splat(f32::MAX);
        let mut max = Vec3A::splat(f32::MIN);
        for v in &self.vertices {
            min = min.min(*v);
            max = max.max(*v);
        }
        let center_y = (min.y + max.y) * 0.5;
        if center_y > GOAL_COMPONENT_TRANSLATION_BT {
            Vec3A::new(0.0, GOAL_COMPONENT_TRANSLATION_BT, 0.0)
        } else if center_y < -GOAL_COMPONENT_TRANSLATION_BT {
            Vec3A::new(0.0, -GOAL_COMPONENT_TRANSLATION_BT, 0.0)
        } else {
            Vec3A::ZERO
        }
    }

    /// Build the Bullet triangle mesh in component-local coordinates.
    /// World vertices and hashes stay unchanged. Local equals world minus
    /// the component translation. For non-goal components this equals the
    /// world mesh. The rigid body must carry the translation so world
    /// geometry stays equal.
    pub fn make_bullet_mesh_local(&self) -> TriangleMesh {
        let translation = self.component_translation();
        if translation == Vec3A::ZERO {
            return self.make_bullet_mesh();
        }
        let local: Vec<Vec3A> = self.vertices.iter().map(|v| *v - translation).collect();
        TriangleMesh::new(&local, &self.indices)
    }

    /// World-space vertices in Bullet units (see `BT_TO_UU` = 50 for uu).
    pub fn get_vertices(&self) -> &[Vec3A] {
        &self.vertices
    }

    /// Triangle indices into [`CollisionMeshFile::get_vertices`] (3 per tri).
    pub fn get_indices(&self) -> &[usize] {
        &self.indices
    }
}
