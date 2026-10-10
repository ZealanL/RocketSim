//! Boost-pad instancing data: variant mapping, instance matrices, bounds,
//! and frustum-filtered instance lists.
//!
//! 4 instanced draws (big/small x on/off) with per-instance matrices.
//! Pad XY comes from config, Z is dropped to the field, uniform 2x scale.

use glam::{Mat4, Vec3, Vec3A};
use rocketsim::{BoostPadConfig, BoostPadState};

use crate::{frustum::sphere_visible, obj::ObjMesh};

/// Variant order = draw order (matches the model names).
pub const PAD_VARIANTS: [&str; 4] = ["pad_big_on", "pad_big_off", "pad_small_on", "pad_small_off"];
/// Max pads in Soccar (the instance buffer size).
pub const MAX_PAD_INSTANCES: usize = 34;

/// Maps (big, active) to a variant index. The small on/off contents are
/// CROSSED (`pad_small_on` <- `small_off.obj`): the
/// small files are misnamed, and the elaborate-ON test below pins this.
pub fn variant_index(is_big: bool, is_active: bool) -> usize {
    match (is_big, is_active) {
        (true, true) => 0,
        (true, false) => 1,
        (false, true) => 2,
        (false, false) => 3,
    }
}

/// Loads the four pad meshes in [`PAD_VARIANTS`] order, with the
/// small-pair cross (files are misnamed; code compensates).
pub fn load_pad_meshes() -> [ObjMesh; 4] {
    [
        ObjMesh::load(include_str!("models/boost_pad_big_on.obj")),
        ObjMesh::load(include_str!("models/boost_pad_big_off.obj")),
        // Crossed: `small_off.obj` holds the elaborate ON look.
        ObjMesh::load(include_str!("models/boost_pad_small_off.obj")),
        // Crossed: `small_on.obj` holds the plain OFF look.
        ObjMesh::load(include_str!("models/boost_pad_small_on.obj")),
    ]
}

/// Instance transform for a pad at `pos`: translate to (x, y, 0) at 2x
/// scale (Z dropped: pads sit on the field plane).
pub fn instance_matrix(pos: Vec3A) -> Mat4 {
    Mat4::from_translation(Vec3::new(pos.x, pos.y, 0.0)) * Mat4::from_scale(Vec3::splat(2.0))
}

/// World-space cull radius for a variant mesh: mesh radius × draw scale + margin.
pub fn variant_radius(mesh: &ObjMesh) -> f32 {
    let r = mesh
        .verts
        .iter()
        .map(|v| v.length())
        .fold(0.0_f32, f32::max);
    r * 2.0 + 10.0
}

/// Per-variant instance matrices for live pad states, frustum-filtered by
/// each variant's `radii` ([`variant_radius`]). Returns one `Vec` per
/// [`PAD_VARIANTS`] entry (possibly empty) plus the culled count (overlay).
pub fn build_instances(
    pads: &[(BoostPadConfig, BoostPadState)],
    planes: &[glam::Vec4; 6],
    radii: &[f32; 4],
) -> ([Vec<Mat4>; 4], usize) {
    let mut out: [Vec<Mat4>; 4] = Default::default();
    let mut culled = 0usize;
    for (config, state) in pads {
        let v = variant_index(config.is_big, state.is_active());
        let mat = instance_matrix(config.pos);
        let center = Vec3::new(config.pos.x, config.pos.y, 0.0);
        if sphere_visible(planes, center, radii[v]) {
            out[v].push(mat);
        } else {
            culled += 1;
        }
    }
    (out, culled)
}

#[cfg(test)]
mod tests {
    use glam::Vec4Swizzles;

    use super::*;
    use crate::{frustum::frustum_planes, view::arena_pose_view_proj};

    fn radii() -> [f32; 4] {
        let meshes = load_pad_meshes();
        [
            variant_radius(&meshes[0]),
            variant_radius(&meshes[1]),
            variant_radius(&meshes[2]),
            variant_radius(&meshes[3]),
        ]
    }

    #[test]
    fn variant_mapping() {
        assert_eq!(variant_index(true, true), 0);
        assert_eq!(variant_index(true, false), 1);
        assert_eq!(variant_index(false, true), 2);
        assert_eq!(variant_index(false, false), 3);
        assert_eq!(PAD_VARIANTS[variant_index(true, true)], "pad_big_on");
        assert_eq!(PAD_VARIANTS[variant_index(false, false)], "pad_small_off");
        assert_eq!(MAX_PAD_INSTANCES, 34);
    }

    #[test]
    fn instance_matrix_drops_z_and_doubles() {
        let m = instance_matrix(Vec3A::new(100.0, -200.0, 70.0));
        // Origin lands on the field plane (z=0, config z dropped).
        let p = m * Vec3::ZERO.extend(1.0);
        assert_eq!(p.xyz(), Vec3::new(100.0, -200.0, 0.0));
        // Unit axes double (uniform 2x).
        let x = (m * Vec3::X.extend(1.0)).xyz() - Vec3::new(100.0, -200.0, 0.0);
        assert_eq!(x, Vec3::new(2.0, 0.0, 0.0));
    }

    #[test]
    fn pad_meshes_load() {
        let meshes = load_pad_meshes();
        for (i, mesh) in meshes.iter().enumerate() {
            assert!(!mesh.verts.is_empty(), "{}: empty", PAD_VARIANTS[i]);
            assert_eq!(mesh.indices.len() % 3, 0);
        }
        // ON looks are the elaborate ones, both sizes. This pins the
        // small-pair cross (files are misnamed) — flip it and this fails.
        assert!(
            meshes[0].verts.len() > meshes[1].verts.len(),
            "big on/off inversion"
        );
        assert!(
            meshes[2].verts.len() > meshes[3].verts.len(),
            "small on/off inversion (cross broken?)"
        );
        // Cull radii: pads are ~100-200uu across at 2x, so low hundreds.
        for (i, mesh) in meshes.iter().enumerate() {
            let r = variant_radius(mesh);
            assert!(r > 50.0 && r < 1000.0, "{}: radius {r}", PAD_VARIANTS[i]);
        }
    }

    #[test]
    fn frustum_culling() {
        crate::photo::init_meshes();
        let arena = crate::photo::build_arena();
        assert_eq!(arena.get_arena_state().boost_pads.len(), 34);
        let (view, proj) = arena_pose_view_proj(16.0 / 9.0);
        let planes = frustum_planes(proj * view);
        let (instances, culled) =
            build_instances(&arena.get_arena_state().boost_pads, &planes, &radii());
        let total: usize = instances.iter().map(|v| v.len()).sum();
        // Edge pads legitimately leave this framing — but only a few.
        assert!(total >= 30, "too many pads culled: {total}/34");
        assert!(culled <= 4, "culled {culled}, expected edge pads only");
        // All active at kickoff → only ON variants filled.
        assert!(instances[1].is_empty() && instances[3].is_empty());
        assert_eq!(total, instances[0].len() + instances[2].len());
        // Midfield small pads (±1024, 0) must survive (translation check).
        let has_pad = |x: f32, y: f32| {
            instances[2].iter().any(|m| {
                let t = m.w_axis.xyz();
                (t.x - x).abs() < 1.0 && (t.y - y).abs() < 1.0 && t.z.abs() < 1e-6
            })
        };
        assert!(has_pad(-1024.0, 0.0), "midfield pad missing");
        assert!(has_pad(1024.0, 0.0), "midfield pad missing");

        // Behind the camera: everything culled.
        let (view, proj) = crate::view::view_proj(Vec3::ZERO, Vec3::X, 90.0, 16.0 / 9.0);
        let planes = frustum_planes(proj * view);
        let pads = [(
            BoostPadConfig {
                pos: Vec3A::new(-5000.0, 0.0, 70.0),
                is_big: false,
            },
            BoostPadState::default(),
        )];
        let (instances, culled) = build_instances(&pads, &planes, &radii());
        assert_eq!(culled, 1);
        assert!(instances.iter().all(|v| v.is_empty()));
    }
}
