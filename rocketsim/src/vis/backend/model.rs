use crate::consts::BT_TO_UU;
use crate::shared::rsmath;
use crate::sim::collision_mesh_file::CollisionMeshFile;
use glam::{Affine3A, Mat3, UVec3, Vec2, Vec3, Vec3A};

#[derive(Debug, Clone)]
pub struct Model {
    pub verts: Vec<Vec3>,
    pub vert_uvs: Vec<Vec2>,
}

impl Model {
    pub const fn empty() -> Self {
        Self {
            verts: Vec::new(),
            vert_uvs: Vec::new(),
        }
    }

    pub fn from_arena_meshes(meshes: &[CollisionMeshFile]) -> Self {
        let mut verts = Vec::new();
        let mut vert_uvs = Vec::new();
        for mesh in meshes {
            let m_verts = mesh.get_vertices();
            for tri in mesh.get_indices().chunks(3) {
                let tri_verts = [m_verts[tri[0]], m_verts[tri[1]], m_verts[tri[2]]];
                for vert in tri_verts {
                    verts.push(vert.to_vec3() * BT_TO_UU);
                    vert_uvs.push(Vec2::splat(0.5)); // Middle UVs
                }
            }
        }

        Self { verts, vert_uvs }
    }

    pub fn load_obj(obj_str: &str) -> Self {
        let mut verts = Vec::new();
        let mut vert_normals = Vec::new();
        let mut vert_uvs = Vec::new();
        let mut tris = Vec::new();

        // Reads a string like "34/102/83" into the indices [33, 101, 82]
        fn read_tri_indices(s: &str) -> UVec3 {
            let split: Vec<&str> = s.split('/').collect();
            assert_eq!(split.len(), 3, "Invalid tri indices: \"{s}\"");
            let vals: Vec<u32> = split
                .iter()
                .map(|part| part.parse::<u32>().unwrap())
                .collect();
            UVec3::from_array(vals.try_into().unwrap()) - 1 // NOTE: OBJ indices start at 1
        }

        for line in obj_str.lines() {
            let parts = line.split_whitespace().collect::<Vec<&str>>();
            if parts.is_empty() {
                continue;
            }
            let prefix = parts[0];
            let num_after = parts.len() - 1;
            match prefix {
                "v" => {
                    assert_eq!(num_after, 3, "Invalid vertex: \"{line}\"");
                    verts.push(Vec3::new(
                        parts[1].parse().unwrap(),
                        parts[2].parse().unwrap(),
                        parts[3].parse().unwrap(),
                    ));
                }
                "vn" => {
                    assert_eq!(num_after, 3, "Invalid vertex normal: \"{line}\"");
                    vert_normals.push(Vec3::new(
                        parts[1].parse().unwrap(),
                        parts[2].parse().unwrap(),
                        parts[3].parse().unwrap(),
                    ));
                }
                "vt" => {
                    assert_eq!(
                        num_after, 2,
                        "Invalid vertex texture coordinate: \"{line}\""
                    );
                    let u: f32 = parts[1].parse().unwrap();
                    let v: f32 = parts[2].parse().unwrap();
                    // NOTE: Obj files invert the vertical axis >:(
                    vert_uvs.push(Vec2::new(u, 1.0 - v));
                }
                "f" => {
                    assert_eq!(num_after, 3, "Invalid face: \"{line}\"");
                    let idc_a = read_tri_indices(parts[1]);
                    let idc_b = read_tri_indices(parts[2]);
                    let idc_c = read_tri_indices(parts[3]);
                    tris.push((
                        UVec3::new(idc_a[0], idc_b[0], idc_c[0]),
                        UVec3::new(idc_a[1], idc_b[1], idc_c[1]),
                        UVec3::new(idc_a[2], idc_b[2], idc_c[2]),
                    ))
                }
                _ => {}
            }
        }

        assert!(!verts.is_empty());
        assert!(!vert_normals.is_empty());
        assert!(!vert_uvs.is_empty());
        assert!(!tris.is_empty());

        let mut flat_verts = Vec::new();
        let mut flat_vert_uvs = Vec::new();
        for (vert_idc, vert_uv_idc, _vert_norm_idc) in tris {
            for i in 0..3 {
                flat_verts.push(verts[vert_idc[i] as usize]);
                flat_vert_uvs.push(vert_uvs[vert_uv_idc[i] as usize]);
            }
        }

        assert_eq!(flat_verts.len(), flat_vert_uvs.len());

        Self {
            verts: flat_verts,
            vert_uvs: flat_vert_uvs,
        }
    }

    pub fn num_verts(&self) -> usize {
        self.verts.len()
    }

    pub fn concat(models: &[Self]) -> Self {
        assert!(!models.is_empty());
        let mut result = models[0].clone();
        for model in models.iter().skip(1) {
            result.verts.append(&mut model.verts.clone());
            result.vert_uvs.append(&mut model.vert_uvs.clone());
        }
        result
    }

    pub fn recompute_normals(&mut self) {
        let mut new_vert_normals = Vec::with_capacity(self.verts.len());
        for tri_points in self.verts.chunks(3) {
            let tri_normal = rsmath::try_calc_tri_normal(&[
                tri_points[0].to_vec3a(),
                tri_points[1].to_vec3a(),
                tri_points[2].to_vec3a(),
            ])
            .unwrap_or(Vec3A::ZERO)
            .to_vec3();

            for _ in 0..3 {
                new_vert_normals.push(tri_normal);
            }
        }
    }

    /// NOTE: Order of application is the opposite of the argument order
    pub fn transform(&self, translation: Vec3, rotation: Mat3) -> Self {
        let mut result = self.clone();
        let affine = Affine3A::from_mat3_translation(rotation, translation);
        for vert in &mut result.verts {
            *vert = affine.transform_point3(*vert);
        }

        result
    }
}
