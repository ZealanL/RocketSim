//! Frustum culling for instanced passes.
//!
//! Planes fit OpenGL `[-1, 1]` depth (near is `row3 + row2`).
//! All planes are normalized; touching counts as visible.

use glam::{Mat4, Vec3, Vec4};

/// Six normalized frustum planes (left, right, bottom, top, near, far) from
/// a combined `proj * view` matrix.
pub fn frustum_planes(view_proj: Mat4) -> [Vec4; 6] {
    // glam Mat4 is column-major: row i = (x[i], y[i], z[i], w[i]).
    let row = |i: usize| {
        Vec4::new(
            view_proj.x_axis[i],
            view_proj.y_axis[i],
            view_proj.z_axis[i],
            view_proj.w_axis[i],
        )
    };
    let (r0, r1, r2, r3) = (row(0), row(1), row(2), row(3));
    let mut planes = [
        r3 + r0, // left
        r3 - r0, // right
        r3 + r1, // bottom
        r3 - r1, // top
        r3 + r2, // near (OpenGL [-1,1] depth)
        r3 - r2, // far
    ];
    for p in &mut planes {
        let inv_len = 1.0 / p.truncate().length().max(1e-9);
        *p *= inv_len;
    }
    planes
}

/// Whether the sphere at `center` with `radius` intersects the frustum
/// (touching counts as visible).
pub fn sphere_visible(planes: &[Vec4; 6], center: Vec3, radius: f32) -> bool {
    planes
        .iter()
        .all(|p| p.truncate().dot(center) + p.w >= -radius - 1e-4)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::view::view_proj;
    /// Camera at origin looking down +X (Z-up), 90° horizontal FOV.
    fn test_planes() -> [Vec4; 6] {
        let (view, _) = view_proj(Vec3::ZERO, Vec3::X, 90.0, 16.0 / 9.0);
        let proj = glam::camera::lh::proj::opengl::perspective(
            90.0_f32.to_radians(),
            16.0 / 9.0,
            50.0,
            60_000.0,
        );
        frustum_planes(proj * view)
    }

    #[test]
    fn opengl_near_plane_is_row3_plus_row2() {
        let planes = test_planes();
        let near = planes[4];
        assert!((near.truncate().length() - 1.0).abs() < 1e-6);
        assert!(near.truncate().dot(Vec3::new(50.0, 0.0, 0.0)) + near.w > -1e-4);
        assert!(near.truncate().dot(Vec3::new(49.0, 0.0, 0.0)) + near.w < 0.0);
    }

    #[test]
    fn ahead_visible_behind_culled() {
        let planes = test_planes();
        assert!(sphere_visible(&planes, Vec3::new(1000.0, 0.0, 0.0), 10.0));
        assert!(!sphere_visible(&planes, Vec3::new(-1000.0, 0.0, 0.0), 10.0));
    }

    #[test]
    fn touching_counts_as_visible() {
        let planes = test_planes();
        // Near plane sits at x=50 (view depth): dead-center tangent is in,
        // a hair behind it is out.
        assert!(sphere_visible(&planes, Vec3::new(50.0, 0.0, 0.0), 0.0));
        assert!(!sphere_visible(&planes, Vec3::new(49.9, 0.0, 0.0), 0.0));
        // Side graze: far off-axis but huge radius still touches.
        assert!(sphere_visible(&planes, Vec3::new(1000.0, 0.0, 0.0), 5000.0));
        assert!(!sphere_visible(
            &planes,
            Vec3::new(1000.0, 9000.0, 0.0),
            10.0
        ));
    }

    #[test]
    fn sides_culled() {
        let planes = test_planes();
        // Well above/below the ±34° vertical half-FOV at 1000 out.
        assert!(!sphere_visible(
            &planes,
            Vec3::new(1000.0, 0.0, 5000.0),
            10.0
        ));
        assert!(!sphere_visible(
            &planes,
            Vec3::new(1000.0, 0.0, -5000.0),
            10.0
        ));
        // Just inside stays.
        assert!(sphere_visible(&planes, Vec3::new(1000.0, 0.0, 500.0), 10.0));
    }

    #[test]
    fn planes_are_normalized() {
        for p in test_planes() {
            assert!((p.truncate().length() - 1.0).abs() < 1e-5);
        }
    }
}
