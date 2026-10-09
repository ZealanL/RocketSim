//! View/projection scaffolding (stage 4).
//!
//! Fixed-pose helper for the arena beauty shot. The full camera system
//! (birds-eye / car-cam / director, RL-authentic spec) lands in stage 8 and
//! replaces [`arena_pose_mvp`]; the [`hfov_to_vfov_deg`] conversion and the
//! Z-up LH view + DirectX-depth projection conventions are final and tested
//! here day one.

use glam::{
    Mat4, Vec3,
    camera::lh::{proj::directx, view::look_at_mat4},
};

/// RL convention: FOV settings are horizontal. Converts to vertical degrees
/// for the projection matrix given the viewport aspect.
pub fn hfov_to_vfov_deg(hfov_deg: f32, aspect: f32) -> f32 {
    let aspect = aspect.max(1e-3);
    let half_tan = (0.5 * hfov_deg.to_radians()).tan() / aspect;
    2.0 * half_tan.atan().to_degrees()
}

/// Z-up look-at view + DirectX-depth (0..1) perspective, near 50 / far 60000.
/// `fov_deg` is horizontal (RL convention). Near is 50 (not 5): a 5..50000
/// ratio wastes depth precision and z-fights the 5uu hull/backdrop gap at
/// range; 50 still clips nothing (closest geometry is meters away).
///
/// The X-flip is load-bearing: glam's LH `look_at` with a Z-up `up` maps
/// world +X to screen LEFT (mirrored vs physical intuition — stand facing
/// +Y with head +Z, your right hand is +X). v1 compensated per MODEL
/// (`y_axis *= -1`) and still got team-color sides wrong. v2 flips the VIEW
/// once, globally: screen-right is +X everywhere, no per-model fixes, and
/// culling stays OFF so the winding inversion is harmless.
pub fn view_proj(eye: Vec3, target: Vec3, hfov_deg: f32, aspect: f32) -> (Mat4, Mat4) {
    let view = Mat4::from_scale(Vec3::new(-1.0, 1.0, 1.0)) * look_at_mat4(eye, target, Vec3::Z);
    let vfov = hfov_to_vfov_deg(hfov_deg, aspect);
    let proj = directx::perspective(vfov.to_radians(), aspect.max(1e-3), 50.0, 60_000.0);
    (view, proj)
}

/// Fixed open-bowl beauty shot: high at the -Y end, looking across the field.
/// The ceiling lid is hidden for this view (see [`crate::mesh::ArenaMesh`]),
/// so the bowl reads as a stadium from above. Stage-4 scaffolding (stage 8
/// owns cameras).
pub fn arena_pose() -> (Vec3, Vec3, f32) {
    (
        Vec3::new(0.0, -7000.0, 5000.0),
        Vec3::new(0.0, 500.0, 0.0),
        80.0,
    )
}

/// View + projection for the beauty-shot pose.
pub fn arena_pose_view_proj(aspect: f32) -> (Mat4, Mat4) {
    let (eye, target, hfov) = arena_pose();
    view_proj(eye, target, hfov, aspect)
}

/// Combined MVP for the beauty-shot pose.
pub fn arena_pose_mvp(aspect: f32) -> Mat4 {
    let (view, proj) = arena_pose_view_proj(aspect);
    proj * view
}

#[cfg(test)]
mod tests {
    use glam::Vec4Swizzles;

    use super::*;

    #[test]
    fn hfov_converts_sane() {
        // 90° horizontal at 16:9 -> ~58.66° vertical.
        let vfov = hfov_to_vfov_deg(90.0, 16.0 / 9.0);
        assert!((vfov - 58.66).abs() < 0.1, "vfov={vfov}");
        // Wider aspect shows more horizontally -> smaller vertical slice.
        assert!(vfov < 90.0);
        assert!(hfov_to_vfov_deg(90.0, 1.0) > vfov);
    }

    #[test]
    fn look_at_puts_eye_at_origin() {
        let eye = Vec3::new(100.0, -200.0, 300.0);
        let (view, _) = view_proj(eye, Vec3::ZERO, 90.0, 16.0 / 9.0);
        let mapped = view * eye.extend(1.0);
        assert!(mapped.xyz().length() < 1e-2, "eye must map to origin");
    }

    #[test]
    fn screen_right_is_plus_x() {
        // Physical grounding: stand at -Y facing +Y (north), head +Z —
        // your right hand is +X (east). The view must agree.
        let eye = Vec3::new(0.0, -5000.0, 1000.0);
        let (view, proj) = view_proj(eye, Vec3::ZERO, 90.0, 16.0 / 9.0);
        let mvp = proj * view;
        let left_wall = mvp * Vec3::new(-4096.0, 0.0, 1000.0).extend(1.0);
        let right_wall = mvp * Vec3::new(4096.0, 0.0, 1000.0).extend(1.0);
        let (ln, rn) = (
            left_wall.xyz() / left_wall.w,
            right_wall.xyz() / right_wall.w,
        );
        assert!(ln.x < 0.0, "-X wall must be screen-left, got {ln:?}");
        assert!(rn.x > 0.0, "+X wall must be screen-right, got {rn:?}");
    }

    #[test]
    fn arena_pose_frames_field() {
        // Kickoff center, far goal, and the photo-scene actors: all in front
        // (w > 0) and inside NDC. (The near end falls outside this closer
        // framing — the wide stage-4 pose covered the whole bowl.)
        let mvp = arena_pose_mvp(16.0 / 9.0);
        for corner in [
            Vec3::new(0.0, 0.0, 93.0),
            Vec3::new(0.0, 5120.0, 500.0),
            Vec3::new(256.0, 2184.0, 17.0),
        ] {
            let clip = mvp * corner.extend(1.0);
            assert!(clip.w > 0.0, "corner {corner:?} behind camera");
            let ndc = clip.xyz() / clip.w;
            assert!(
                ndc.x.abs() < 1.0 && ndc.y.abs() < 1.0,
                "corner {corner:?} outside: {ndc:?}"
            );
            assert!((0.0..1.0).contains(&ndc.z), "depth outside 0..1: {ndc:?}");
        }
    }
}
