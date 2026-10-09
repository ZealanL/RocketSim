//! Wavefront OBJ loading + smooth normals.
//!
//! Parses the bundled tri-only models, V-flips UVs, welds on exact
//! `(position, uv)` corners, and recomputes smooth normals (zero on
//! degenerate tris). File normals are ignored.

use glam::{UVec3, Vec2, Vec3};
use rustc_hash::FxHashMap;

/// Indexed OBJ mesh: welded corners + triangles + smooth normals.
pub struct ObjMesh {
    /// Welded positions (obj units = uu for our customs).
    pub verts: Vec<Vec3>,
    /// Welded UVs (V-flipped at parse).
    pub uvs: Vec<Vec2>,
    /// Triangles over `verts`.
    pub indices: Vec<u32>,
    /// Smooth per-vertex normals (unit, or zero on degenerate tris).
    pub normals: Vec<Vec3>,
    /// Pre-weld soup corner count (weld-win denominator).
    pub soup_corners: usize,
}

impl ObjMesh {
    /// Parses + welds + smooths. Panics on malformed input (assets are
    /// fixed at build time: fail fast, never half-load).
    pub fn load(obj_str: &str) -> Self {
        let mut positions = Vec::new();
        let mut uvs = Vec::new();
        // (pos_idx, uv_idx) soup corners; normal indices are ignored.
        let mut soup: Vec<(u32, u32)> = Vec::new();

        // Reads "34/102/83" into 0-based (pos, uv, normal) indices.
        fn read_tri_indices(s: &str) -> UVec3 {
            let split: Vec<&str> = s.split('/').collect();
            assert_eq!(split.len(), 3, "invalid tri indices: \"{s}\"");
            let vals: Vec<u32> = split
                .iter()
                .map(|part| part.parse::<u32>().expect("non-numeric index"))
                .collect();
            UVec3::from_array(vals.try_into().unwrap()) - 1 // OBJ is 1-based
        }

        for line in obj_str.lines() {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.is_empty() {
                continue;
            }
            match parts[0] {
                "v" => {
                    assert_eq!(parts.len(), 4, "invalid vertex: \"{line}\"");
                    positions.push(Vec3::new(
                        parts[1].parse().expect("bad float"),
                        parts[2].parse().expect("bad float"),
                        parts[3].parse().expect("bad float"),
                    ));
                }
                "vt" => {
                    assert_eq!(parts.len(), 3, "invalid uv: \"{line}\"");
                    let u: f32 = parts[1].parse().expect("bad float");
                    let v: f32 = parts[2].parse().expect("bad float");
                    // OBJ V points up; GL textures point down.
                    uvs.push(Vec2::new(u, 1.0 - v));
                }
                "vn" => {} // ignored: normals are recomputed smooth
                "f" => {
                    assert_eq!(parts.len(), 4, "non-triangle face: \"{line}\"");
                    for part in &parts[1..] {
                        let idx = read_tri_indices(part);
                        soup.push((idx[0], idx[1]));
                    }
                }
                _ => {}
            }
        }
        assert!(!positions.is_empty(), "obj has no verts");
        assert!(!uvs.is_empty(), "obj has no uvs");
        assert!(!soup.is_empty(), "obj has no faces");

        let soup_corners = soup.len();
        let mut uniq_verts = Vec::new();
        let mut uniq_uvs = Vec::new();
        let mut vert_index = FxHashMap::default();
        let mut indices = Vec::with_capacity(soup.len());
        for (pi, ti) in soup {
            let key = (pi, ti);
            if let Some(&idx) = vert_index.get(&key) {
                indices.push(idx);
            } else {
                assert!(
                    uniq_verts.len() < u32::MAX as usize,
                    "obj vertex count exceeds u32 range"
                );
                let idx = uniq_verts.len() as u32;
                uniq_verts.push(positions[pi as usize]);
                uniq_uvs.push(uvs[ti as usize]);
                vert_index.insert(key, idx);
                indices.push(idx);
            }
        }

        let normals = smooth_normals(&uniq_verts, &indices);
        Self {
            verts: uniq_verts,
            uvs: uniq_uvs,
            indices,
            normals,
            soup_corners,
        }
    }
}

/// Smooth per-vertex normals: face normals summed per corner, then
/// normalized (zero on degenerate tris).
pub fn smooth_normals(verts: &[Vec3], indices: &[u32]) -> Vec<Vec3> {
    assert_eq!(indices.len() % 3, 0);
    let mut acc = vec![Vec3::ZERO; verts.len()];
    for tri in indices.chunks(3) {
        let (ia, ib, ic) = (tri[0] as usize, tri[1] as usize, tri[2] as usize);
        let (a, b, c) = (verts[ia], verts[ib], verts[ic]);
        let n = (b - a).cross(c - a);
        let n = if n.length_squared() > 1e-12 {
            n.normalize()
        } else {
            Vec3::ZERO
        };
        acc[ia] += n;
        acc[ib] += n;
        acc[ic] += n;
    }
    acc.into_iter()
        .map(|n| {
            if n.length_squared() > 1e-12 {
                n.normalize()
            } else {
                Vec3::ZERO
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check_sanity(mesh: &ObjMesh, label: &str) {
        assert_eq!(mesh.indices.len() % 3, 0, "{label}: index count");
        assert!(!mesh.verts.is_empty(), "{label}: no verts");
        assert_eq!(mesh.verts.len(), mesh.uvs.len(), "{label}: vert/uv");
        assert_eq!(mesh.verts.len(), mesh.normals.len(), "{label}: vert/normal");
        for &i in &mesh.indices {
            assert!((i as usize) < mesh.verts.len(), "{label}: index range");
        }
        for (v, n) in mesh.verts.iter().zip(mesh.normals.iter()) {
            assert!(v.is_finite(), "{label}: non-finite vert");
            let len = n.length();
            assert!(
                len < 1e-6 || (len - 1.0).abs() < 1e-4,
                "{label}: normal not unit-or-zero ({len})"
            );
        }
        assert!(
            mesh.soup_corners >= mesh.verts.len(),
            "{label}: weld grew verts?"
        );
    }

    #[test]
    fn bundled_models_load() {
        let ball = ObjMesh::load(include_str!("models/ball.obj"));
        check_sanity(&ball, "ball");
        // Ball is a smooth closed solid: weld must merge shared corners and
        // nearly every normal must be unit (no degenerate tris).
        assert!(
            ball.verts.len() < ball.soup_corners,
            "ball weld did nothing"
        );
        let unit = ball.normals.iter().filter(|n| n.length() > 0.5).count();
        assert!(unit * 100 / ball.normals.len() > 95);
        for (label, obj) in [
            ("octane", include_str!("models/car_octane.obj")),
            ("dominus", include_str!("models/car_dominus.obj")),
            ("breakout", include_str!("models/car_breakout.obj")),
            ("merc", include_str!("models/car_merc.obj")),
        ] {
            check_sanity(&ObjMesh::load(obj), label);
        }
    }

    #[test]
    fn uv_v_is_flipped() {
        let mesh = ObjMesh::load("v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0.25 0.25\nf 1/1/1 2/1/1 3/1/1\n");
        assert_eq!(mesh.uvs[0], Vec2::new(0.25, 0.75));
    }

    #[test]
    fn weld_shares_corners() {
        // Two tris sharing an edge: 6 soup corners -> 4 verts.
        let mesh = ObjMesh::load(
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0\nvt 0 0\n\
             f 1/1/1 2/1/1 3/1/1\nf 2/1/1 4/1/1 3/1/1\n",
        );
        assert_eq!(mesh.verts.len(), 4);
        assert_eq!(mesh.indices.len(), 6);
        // Flat quad in XY: all normals ±Z unit.
        for n in &mesh.normals {
            assert!((n.length() - 1.0).abs() < 1e-5);
            assert!(n.z.abs() > 0.99);
        }
    }

    #[test]
    fn degenerate_tri_gives_zero_normal() {
        let normals = smooth_normals(&[Vec3::ZERO, Vec3::X, Vec3::ZERO], &[0, 1, 2]);
        assert!(normals.iter().all(|n| *n == Vec3::ZERO));
    }

    #[test]
    #[should_panic(expected = "non-triangle face")]
    fn quad_face_rejected() {
        ObjMesh::load("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvt 0 0\nf 1/1/1 2/1/1 3/1/1 4/1/1\n");
    }
}
