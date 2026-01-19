use crate::consts::BT_TO_UU;
use crate::shared::geo_math;
use crate::sim::collision_mesh_file::CollisionMeshFile;
use glam::{Affine3A, Mat3, UVec3, Vec2, Vec3};

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
                for i in 0..3 {
                    verts.push(tri_verts[i].to_vec3() * BT_TO_UU);
                    vert_uvs.push(Vec2::splat(0.5)); // Middle UVs
                }
            }
        }

        Self {
            verts,
            vert_uvs,
        }
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
        for (vert_idc, vert_uv_idc, vert_norm_idc) in tris {
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

    /// Returns an outer and an inner mesh
    pub fn make_wireframe_split(&self, width: f32) -> (Self, Self) {
        assert!(width > 0.0);

        let width_sq = width * width;

        let mut result_outer = Model::empty();
        let mut result_inner = Model::empty();

        for (tri_verts, tri_vert_uvs) in self.verts.chunks(3).zip(self.vert_uvs.chunks(3)) {
            let (a, b, c) = (tri_verts[0], tri_verts[1], tri_verts[2]);

            let tri_center = (a + b + c) / 3.0;
            let center_dists_sq = [
                a.distance_squared(tri_center),
                b.distance_squared(tri_center),
                c.distance_squared(tri_center),
            ];
            let min_center_dist_sq = center_dists_sq[0]
                .min(center_dists_sq[1])
                .min(center_dists_sq[2]);
            if min_center_dist_sq <= width_sq {
                // Too small to be split up
                // Just add the normal triangle
                for i in 0..3 {
                    result_outer.verts.push(tri_verts[i]);
                    result_outer.vert_uvs.push(tri_vert_uvs[i]);
                }
            } else {
                // The inner tri points are the original 3 points, but moved towards the center
                //  by a distance equal to the wireframe width.
                let mut inner_tri_verts = [a, b, c];
                for i in 0..3 {
                    let center_dist = center_dists_sq[i].sqrt();

                    // By what fraction/portion (0 to 1) we should move this point towards the center,
                    //  such that the distance from the original point is equal to the wireframe width.
                    let centering_frac = width / center_dist;
                    inner_tri_verts[i] += (tri_center - inner_tri_verts[i]).normalize() * width;
                }

                // Make the inner triangles
                for i in 0..3 {
                    result_inner.verts.push(inner_tri_verts[i]);
                    result_inner.vert_uvs.push(tri_vert_uvs[i]);
                }

                // Make the new outer triangles
                // We gotta make 6 because we need 3 quads for each triangle side
                for i in 0..3 {
                    let ni = (i + 1) % 3;

                    // TRIANGLES:
                    // [i-out, (i+1)-out, i-in]
                    // [i-in, (i+1)-out, (i+1)-in]
                    // ^ Clockwise rotating order too :3

                    result_outer.verts.push(tri_verts[i]);
                    result_outer.verts.push(tri_verts[ni]);
                    result_outer.verts.push(inner_tri_verts[i]);

                    result_outer.verts.push(inner_tri_verts[i]);
                    result_outer.verts.push(tri_verts[ni]);
                    result_outer.verts.push(inner_tri_verts[ni]);


                    for _ in 0..6 {
                        // TODO: Properly interpolate UVs (just pushing zeros for now)
                        result_outer.vert_uvs.push(Vec2::ZERO);
                    }
                }
            }
        }

        (result_outer, result_inner)
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
            let tri_normal = geo_math::calc_tri_normal(&[
                tri_points[0].to_vec3a(),
                tri_points[1].to_vec3a(),
                tri_points[2].to_vec3a(),
            ]).to_vec3();

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
