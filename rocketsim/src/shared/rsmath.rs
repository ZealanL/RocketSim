use glam::Vec3A;

pub fn calc_tri_normal(points: &[Vec3A; 3]) -> Vec3A {
    (points[1] - points[0])
        .cross(points[2] - points[0])
        .try_normalize()
        .unwrap()
}

/// Projects a point onto a line, given two points on said line
pub fn project_point_on_inf_line(point: Vec3A, line_a: Vec3A, line_b: Vec3A) -> Vec3A {
    let line_vec = line_b - line_a;
    let length_sq = line_vec.length_squared();
    if length_sq >= f32::EPSILON {
        let t = (point - line_a).dot(line_vec) / length_sq;
        line_a + line_vec * t
    } else {
        // It's basically just a point
        line_a
    }
}

/// Finds the closest point on a finite segment to a target point.
pub fn closest_point_on_segment(target: Vec3A, seg_a: Vec3A, seg_b: Vec3A) -> Vec3A {
    let seg_dim = seg_b - seg_a;
    let length_sq = seg_dim.length_squared();
    if length_sq >= f32::EPSILON {
        let t = (target - seg_a).dot(seg_dim) / length_sq;
        seg_a + seg_dim * t.clamp(0.0, 1.0)
    } else {
        seg_a
    }
}

/// Finds a point closest to two line segments
///
/// **Ref**: https://zalo.github.io/blog/closest-point-between-segments/
pub fn closest_points_between_segments(
    seg_a: Vec3A,
    seg_b: Vec3A,
    seg_c: Vec3A,
    seg_d: Vec3A,
) -> Vec3A {
    let in_plane_a = project_point_on_inf_line(seg_a, seg_c, seg_d);
    let in_plane_b = project_point_on_inf_line(seg_b, seg_c, seg_d);
    let in_plane_ba = in_plane_b - in_plane_a;

    let denom = in_plane_ba.length_squared();
    let t = if denom > f32::EPSILON {
        (seg_c - in_plane_a).dot(in_plane_ba) / denom
    } else {
        0.0 // Segments are basically parallel
    };

    let point_on_ab_closest_to_line_cd = seg_a.lerp(seg_b, t.clamp(0.0, 1.0));

    let point_on_cd_closest_to_ab =
        closest_point_on_segment(point_on_ab_closest_to_line_cd, seg_c, seg_d);
    let point_on_ab_closest_to_cd =
        closest_point_on_segment(point_on_cd_closest_to_ab, seg_a, seg_b);

    (point_on_ab_closest_to_cd + point_on_cd_closest_to_ab) / 2.0
}

pub fn to_2d(v: Vec3A) -> Vec3A {
    v.truncate().extend(0.0).to_vec3a()
}
