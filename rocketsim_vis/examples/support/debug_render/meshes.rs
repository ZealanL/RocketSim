use std::time::Instant;

use glam::{Mat3A, Vec2, Vec3A, Vec3Swizzles};
use rocketsim_vis::debug::{Color, DebugRenderer, DrawList, GroupOptions, MeshResource, palette};

use super::obj::{Corner, ObjData};

pub(super) fn make_mesh(data: &ObjData) -> Result<MeshResource, Box<dyn std::error::Error>> {
    let mut positions = Vec::new();
    let mut colors = Vec::new();
    let mut normals = Vec::new();
    let mut uvs = Vec::new();
    let mut indices = Vec::new();
    let mut remap = std::collections::HashMap::<(Corner, [u8; 4]), u32>::new();
    for face in &data.faces {
        let mut face_ids = Vec::with_capacity(face.corners.len());
        let a = data.positions[face.corners[0].v];
        let b = data.positions[face.corners[1].v];
        let c = data.positions[face.corners[2].v];
        let face_normal = normal_or_z((b - a).cross(c - a));
        for c in &face.corners {
            let key = (*c, face.color.to_array().map(|v| (v * 255.0).round() as u8));
            let id = *remap.entry(key).or_insert_with(|| {
                let id = positions.len() as u32;
                positions.push(data.positions[c.v].to_array());
                colors.push(face.color);
                normals.push(
                    c.vn.map(|n| normal_or_z(data.normals[n].zxy()))
                        .unwrap_or(face_normal)
                        .to_array(),
                );
                uvs.push(c.vt.map(|t| data.uvs[t]).unwrap_or(Vec2::ZERO).to_array());
                id
            });
            face_ids.push(id);
        }
        for i in 1..face_ids.len() - 1 {
            indices.extend([face_ids[0], face_ids[i], face_ids[i + 1]]);
        }
    }
    Ok(MeshResource::new(
        "imported_obj",
        positions,
        colors,
        Some(normals),
        Some(uvs),
        indices,
    )?)
}
// Keep the importer's existing cutoff for degenerate normals.
pub(super) fn normal_or_z(normal: Vec3A) -> Vec3A {
    let length = normal.length();
    if length > 1e-6 {
        normal / length
    } else {
        Vec3A::Z
    }
}

pub(super) fn insert(
    renderer: &DebugRenderer,
    name: &str,
    list: DrawList,
) -> Result<(), Box<dyn std::error::Error>> {
    renderer.replace_group(name, list, GroupOptions::default())?;
    Ok(())
}

pub fn rotation_z(angle: f32) -> [[f32; 3]; 3] {
    Mat3A::from_rotation_z(angle).transpose().to_cols_array_2d()
}

pub(super) fn procedural_mesh(
    id: &str,
    positions: Vec<Vec3A>,
    colors: Vec<Color>,
    normals: Vec<Vec3A>,
    uvs: Vec<Vec2>,
    indices: Vec<u32>,
) -> Result<MeshResource, Box<dyn std::error::Error>> {
    Ok(MeshResource::new(
        id,
        positions.into_iter().map(|v| v.to_array()).collect(),
        colors,
        Some(normals.into_iter().map(|v| v.to_array()).collect()),
        Some(uvs.into_iter().map(|v| v.to_array()).collect()),
        indices,
    )?)
}

pub(super) fn submit_mesh(
    renderer: &DebugRenderer,
    mesh: MeshResource,
    elapsed_ms: &mut f64,
) -> Result<(), Box<dyn std::error::Error>> {
    let start = Instant::now();
    renderer.insert_mesh(mesh)?;
    *elapsed_ms += start.elapsed().as_secs_f64() * 1000.0;
    Ok(())
}

pub(super) fn planet_mesh(
    id: &str,
    color: Color,
) -> Result<MeshResource, Box<dyn std::error::Error>> {
    let t = (1.0 + 5.0_f32.sqrt()) * 0.5;
    let raw = [
        [-1.0, t, 0.0],
        [1.0, t, 0.0],
        [-1.0, -t, 0.0],
        [1.0, -t, 0.0],
        [0.0, -1.0, t],
        [0.0, 1.0, t],
        [0.0, -1.0, -t],
        [0.0, 1.0, -t],
        [t, 0.0, -1.0],
        [t, 0.0, 1.0],
        [-t, 0.0, -1.0],
        [-t, 0.0, 1.0],
    ];
    let faces: [[u32; 3]; 20] = [
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    let positions: Vec<_> = raw
        .into_iter()
        .map(|v| normal_or_z(Vec3A::from_array(v)) * 105.0)
        .collect();
    let normals: Vec<_> = positions.iter().map(|&p| normal_or_z(p)).collect();
    let colors = normals
        .iter()
        .map(|normal| {
            let light = (0.55 + 0.45 * normal.z.max(0.0)).clamp(0.0, 1.0);
            Color::rgba(color.r * light, color.g * light, color.b * light, 1.0)
        })
        .collect();
    let uvs = positions
        .iter()
        .map(|p| p.truncate() / 210.0 + Vec2::splat(0.5))
        .collect();
    let indices = faces.into_iter().flatten().collect();
    procedural_mesh(id, positions, colors, normals, uvs, indices)
}

pub(super) fn torus_knot_mesh() -> Result<MeshResource, Box<dyn std::error::Error>> {
    let around = 96usize;
    let sides = 12usize;
    let mut positions = Vec::with_capacity(around * sides);
    let mut colors = Vec::with_capacity(around * sides);
    let mut normals = Vec::with_capacity(around * sides);
    let mut uvs = Vec::with_capacity(around * sides);
    let mut indices = Vec::with_capacity(around * sides * 6);
    for i in 0..around {
        let u = i as f32 / around as f32 * std::f32::consts::TAU;
        let (sin_2u, cos_2u) = (2.0 * u).sin_cos();
        let (sin_3u, cos_3u) = (3.0 * u).sin_cos();
        let center = Vec3A::new(
            520.0 * cos_2u + 130.0 * cos_3u,
            520.0 * sin_2u - 130.0 * sin_3u,
            560.0 + 150.0 * sin_3u,
        );
        let tangent = normal_or_z(Vec3A::new(
            -1040.0 * sin_2u - 390.0 * sin_3u,
            1040.0 * cos_2u - 390.0 * cos_3u,
            450.0 * cos_3u,
        ));
        let side = normal_or_z(Vec3A::Z.cross(tangent));
        let up = normal_or_z(tangent.cross(side));
        for j in 0..sides {
            let v = j as f32 / sides as f32 * std::f32::consts::TAU;
            let (sin_v, cos_v) = v.sin_cos();
            let n = normal_or_z(side * cos_v + up * sin_v);
            positions.push(center + n * 52.0);
            normals.push(n);
            colors.push(palette::rainbow(i as f32 / around as f32));
            uvs.push(Vec2::new(i as f32 / around as f32, j as f32 / sides as f32));
            let a = (i * sides + j) as u32;
            let b = (i * sides + (j + 1) % sides) as u32;
            let c = (((i + 1) % around) * sides + j) as u32;
            let d = (((i + 1) % around) * sides + (j + 1) % sides) as u32;
            indices.extend([a, b, c, b, d, c]);
        }
    }
    procedural_mesh("knot_mesh", positions, colors, normals, uvs, indices)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn procedural_meshes_keep_topology_unit_normals_and_fallback_rules() {
        assert_eq!(normal_or_z(Vec3A::ZERO), Vec3A::Z);
        assert_eq!(normal_or_z(Vec3A::X * 0.5e-6), Vec3A::Z);
        assert_eq!(normal_or_z(Vec3A::X * 2.0e-6), Vec3A::X);
        assert!(
            normal_or_z(Vec3A::new(0.0, 3.0, 4.0)).abs_diff_eq(Vec3A::new(0.0, 0.6, 0.8), 1e-6)
        );
        for (mesh, vertices, indices) in [
            (planet_mesh("planet", Color::WHITE).unwrap(), 12, 60),
            (torus_knot_mesh().unwrap(), 96 * 12, 96 * 12 * 6),
        ] {
            mesh.validate().unwrap();
            assert_eq!(mesh.positions.len(), vertices);
            assert_eq!(mesh.indices.len(), indices);
            for normal in mesh.normals.unwrap() {
                assert!((Vec3A::from_array(normal).length() - 1.0).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn row_major_rotation_turns_x_toward_y() {
        let rows = rotation_z(std::f32::consts::FRAC_PI_2);
        assert!(Vec3A::from_array(rows[0]).abs_diff_eq(Vec3A::NEG_Y, 1e-6));
        let rotation = Mat3A::from_cols_array_2d(&rows).transpose();
        assert!((rotation * Vec3A::X).abs_diff_eq(Vec3A::Y, 1e-6));
    }
}
