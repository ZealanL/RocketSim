use std::{
    io::{Cursor, Result as IoResult},
    num::Wrapping,
};

use byteorder::{LittleEndian, ReadBytesExt};
use glam::Vec3A;
use log::info;

use crate::bullet::collision::shapes::triangle_mesh::TriangleMesh;

pub const COLLISION_MESH_BASE_PATH: &str = "./collision_meshes/";
pub const COLLISION_MESH_FILE_EXTENSION: &str = "cmf";

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

pub struct CollisionMeshFile {
    indices: Vec<usize>,
    vertices: Vec<Vec3A>,
    hash: u32,
}

impl CollisionMeshFile {
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

    pub fn make_bullet_mesh(self) -> TriangleMesh {
        TriangleMesh::new(&self.vertices, &self.indices)
    }
}
