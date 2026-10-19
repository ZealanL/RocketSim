use std::{
    collections::{BTreeMap, HashSet},
    fs,
    path::Path,
};

use glam::{Mat3A, Vec2, Vec3A, Vec3Swizzles};
use rocketsim_vis::debug::{Color, DebugRenderer, DepthMode, DrawList, TextAlign, Width};

#[derive(Clone, Debug)]
pub(super) struct ObjFace {
    pub(super) corners: Vec<Corner>,
    pub(super) color: Color,
}
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub(super) struct Corner {
    pub(super) v: usize,
    pub(super) vt: Option<usize>,
    pub(super) vn: Option<usize>,
}
#[derive(Clone, Debug)]
pub(super) struct ObjData {
    pub(super) positions: Vec<Vec3A>,
    pub(super) uvs: Vec<Vec2>,
    pub(super) normals: Vec<Vec3A>,
    pub(super) faces: Vec<ObjFace>,
}

fn obj_index(s: &str, len: usize) -> Result<usize, String> {
    let i: i64 = s.parse().map_err(|_| format!("invalid OBJ index '{s}'"))?;
    let resolved = if i > 0 {
        i - 1
    } else if i < 0 {
        len as i64 + i
    } else {
        return Err("OBJ indices cannot be zero".into());
    };
    if resolved < 0 || resolved as usize >= len {
        return Err(format!("OBJ index {i} is out of range for {len} values"));
    }
    Ok(resolved as usize)
}

pub(super) fn parse_obj(src: &str, original_placement: bool) -> Result<ObjData, String> {
    let mut out = ObjData {
        positions: vec![],
        uvs: vec![],
        normals: vec![],
        faces: vec![],
    };
    let mut color = Color::WHITE;
    for (line_no, line) in src.lines().enumerate() {
        let mut fields = line.split_whitespace();
        let Some(tag) = fields.next() else { continue };
        let vals: Vec<_> = fields.collect();
        let err = |e: String| format!("OBJ line {}: {e}", line_no + 1);
        match tag {
            "v" => {
                if vals.len() < 3 {
                    return Err(err("vertex needs 3 coordinates".into()));
                }
                let x = parse_f32(vals[0]).map_err(err)?;
                let y = parse_f32(vals[1]).map_err(err)?;
                let z = parse_f32(vals[2]).map_err(err)?;
                let mut position = Vec3A::new(x, y, z).zxy() * 70.0;
                if original_placement {
                    position.x -= 3500.0;
                }
                out.positions.push(position);
            }
            "vt" => {
                if vals.len() < 2 {
                    return Err(err("texture coordinate needs 2 values".into()));
                }
                out.uvs.push(Vec2::new(
                    parse_f32(vals[0]).map_err(err)?,
                    parse_f32(vals[1]).map_err(err)?,
                ));
            }
            "vn" => {
                if vals.len() < 3 {
                    return Err(err("normal needs 3 values".into()));
                }
                out.normals.push(Vec3A::new(
                    parse_f32(vals[0]).map_err(err)?,
                    parse_f32(vals[1]).map_err(err)?,
                    parse_f32(vals[2]).map_err(err)?,
                ));
            }
            "o" | "g" => {
                if let Some(name) = vals.first() {
                    color = color_from_name(name);
                }
            }
            "f" => {
                if vals.len() < 3 {
                    return Err(err("face needs at least 3 corners".into()));
                }
                let mut corners = Vec::with_capacity(vals.len());
                for item in vals {
                    let mut parts = item.split('/');
                    let v =
                        obj_index(parts.next().unwrap_or(""), out.positions.len()).map_err(err)?;
                    let vt = parts
                        .next()
                        .filter(|s| !s.is_empty())
                        .map(|s| obj_index(s, out.uvs.len()))
                        .transpose()
                        .map_err(err)?;
                    let vn = parts
                        .next()
                        .filter(|s| !s.is_empty())
                        .map(|s| obj_index(s, out.normals.len()))
                        .transpose()
                        .map_err(err)?;
                    if parts.next().is_some() {
                        return Err(err("invalid face corner".into()));
                    }
                    corners.push(Corner { v, vt, vn });
                }
                out.faces.push(ObjFace { corners, color });
            }
            _ => {}
        }
    }
    if out.positions.is_empty() || out.faces.is_empty() {
        return Err("OBJ has no vertices or faces".into());
    }
    if !original_placement {
        let mut min = Vec3A::splat(f32::INFINITY);
        let mut max = Vec3A::splat(f32::NEG_INFINITY);
        for &position in &out.positions {
            min = min.min(position);
            max = max.max(position);
        }
        let center = (min + max) * 0.5;
        let origin = Vec3A::new(center.x, center.y, min.z);
        for position in &mut out.positions {
            *position -= origin;
        }
    }
    Ok(out)
}
fn parse_f32(s: &str) -> Result<f32, String> {
    let n: f32 = s.parse().map_err(|_| format!("invalid number '{s}'"))?;
    if n.is_finite() {
        Ok(n)
    } else {
        Err("non-finite number".into())
    }
}
fn color_from_name(name: &str) -> Color {
    let Some((_, hex)) = name.rsplit_once('_') else {
        return Color::WHITE;
    };
    if hex.len() != 6 {
        return Color::WHITE;
    }
    let Ok(rgb) = u32::from_str_radix(hex, 16) else {
        return Color::WHITE;
    };
    Color::rgb(
        ((rgb >> 16) & 255) as f32 / 255.0,
        ((rgb >> 8) & 255) as f32 / 255.0,
        (rgb & 255) as f32 / 255.0,
    )
}

pub fn render_obj(
    renderer: &DebugRenderer,
    path: &Path,
    solid: bool,
    original_placement: bool,
    mesh_submit_ms: &mut f64,
) -> Result<(), Box<dyn std::error::Error>> {
    let text = fs::read_to_string(path)?;
    let data = parse_obj(&text, original_placement)?;
    if solid {
        let mesh = super::meshes::make_mesh(&data)?;
        let bytes = mesh.estimated_bytes();
        super::meshes::submit_mesh(renderer, mesh, mesh_submit_ms)?;
        let mut list = DrawList::new();
        list.instance(
            "imported_obj",
            [0.0, 0.0, 0.0],
            Mat3A::IDENTITY.to_cols_array_2d(),
            [1.0; 3],
            Color::WHITE,
            None,
            DepthMode::Test,
        );
        super::meshes::insert(renderer, "obj_solid", list)?;
        eprintln!(
            "OBJ solid: {} positions, {} faces, resource {:.2} MiB",
            data.positions.len(),
            data.faces.len(),
            bytes as f64 / 1048576.0
        );
    } else {
        let mut edges = BTreeMap::<[u8; 4], (Color, HashSet<(usize, usize)>)>::new();
        for face in &data.faces {
            let key = face
                .color
                .to_array()
                .map(|value| (value * 255.0).round() as u8);
            let entry = edges
                .entry(key)
                .or_insert_with(|| (face.color, HashSet::new()));
            for i in 0..face.corners.len() {
                let a = face.corners[i].v;
                let b = face.corners[(i + 1) % face.corners.len()].v;
                entry.1.insert((a.min(b), a.max(b)));
            }
        }
        let mut group_id = 0usize;
        for (_, (color, edges)) in edges {
            let mut sorted_edges: Vec<_> = edges.into_iter().collect();
            sorted_edges.sort_unstable();
            let mut list = DrawList::with_capacity(4096);
            for (a, b) in sorted_edges {
                if list.len() == 4096 {
                    super::meshes::insert(renderer, &format!("obj_wire_{group_id}"), list)?;
                    group_id += 1;
                    list = DrawList::with_capacity(4096);
                }
                list.line(
                    data.positions[a].to_array(),
                    data.positions[b].to_array(),
                    color,
                    Width::pixels(1.2),
                    DepthMode::Always,
                );
            }
            if !list.is_empty() {
                super::meshes::insert(renderer, &format!("obj_wire_{group_id}"), list)?;
                group_id += 1;
            }
        }
        eprintln!(
            "OBJ wire: {} positions, {} polygon boundaries",
            data.positions.len(),
            data.faces.len()
        );
    }
    let mut guides = DrawList::new();
    guides.grid(
        [0.0, 0.0, 2.0],
        1600.0,
        400.0,
        Color::rgba(0.45, 0.52, 0.62, 0.5),
        Width::pixels(1.0),
        DepthMode::Test,
    );
    guides.label_screen(
        "OBJ DEBUG RENDER",
        [0.5, 0.04],
        20.0,
        TextAlign::Center,
        Color::WHITE,
        Some(Color::BLACK.with_alpha(0.65)),
    );
    super::meshes::insert(renderer, "obj_guides", guides)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_obj_parses_mixed_corners_colors_and_triangulates() {
        let src = "o trim_12aBcD\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvn 0 0 1\nf 1/1/1 2/2/1 3//1 4/1/1\n";
        let data = parse_obj(src, false).unwrap();
        assert_eq!(data.faces.len(), 1);
        assert_eq!(data.faces[0].corners.len(), 4);
        assert_eq!(
            data.faces[0].color,
            Color::rgb(
                0x12 as f32 / 255.0,
                0xab as f32 / 255.0,
                0xcd as f32 / 255.0
            )
        );
        let mesh = super::super::meshes::make_mesh(&data).unwrap();
        assert_eq!(mesh.indices.len(), 6);
        assert_eq!(mesh.positions.len(), 4);
    }

    #[test]
    fn obj_positions_recenter_or_keep_the_source_placement() {
        let src = "v 10 0 0\nv 11 0 0\nv 10 1 0\nf -3 -2 -1\n";
        let centered = parse_obj(src, false).unwrap();
        assert!((centered.positions.iter().map(|p| p.x).sum::<f32>() / 3.0).abs() < 0.001);
        assert_eq!(
            centered
                .positions
                .iter()
                .map(|p| p.z)
                .fold(f32::INFINITY, f32::min),
            0.0
        );
        let original = parse_obj("v 1 2 3\nv 2 2 3\nv 1 3 3\nf -3 -2 -1\n", true).unwrap();
        assert_eq!(original.positions[0], Vec3A::new(-3290.0, 70.0, 140.0));
    }

    #[test]
    fn imported_and_computed_normals_share_the_axis_mapping() {
        let vertices = "v 0 0 0\nv 1 0 0\nv 0 1 0\n";
        for faces in ["f 1 2 3\n", "vn 0 0 1\nf 1//1 2//1 3//1\n"] {
            let data = parse_obj(&format!("{vertices}{faces}"), false).unwrap();
            let mesh = super::super::meshes::make_mesh(&data).unwrap();
            for normal in mesh.normals.unwrap() {
                assert_eq!(Vec3A::from_array(normal), Vec3A::X);
            }
        }
    }

    #[test]
    fn malformed_obj_indices_and_numbers_return_errors() {
        for source in [
            "v 0 0 0\nf 1 2 3\n",
            "v 0 0 0\nv NaN 0 0\nf 1 1 1\n",
            "v 0 0 0\nf 1/99 1 1\n",
            "v 0 0 0\nf -2 -1 -1\n",
        ] {
            assert!(
                parse_obj(source, false).is_err(),
                "accepted invalid OBJ: {source}"
            );
        }
    }
}
