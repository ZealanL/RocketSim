use glam::Vec3A;

/// Maximum number of conservative-advancement steps.
pub const MAX_ITERATIONS: usize = 64;

/// Distance below which the cast stops, in Bullet units.
pub const STOP_RADIUS: f32 = 0.001;

/// Result of a point convex cast.
#[derive(Clone, Copy, Debug)]
pub struct PointCastResult {
    /// Outward shape normal at the hit point, in shape space.
    pub normal: Vec3A,
    /// Fraction of ray travel before the hit, in `0.0..=1.0`.
    pub fraction: f32,
}

/// Casts a zero-radius point from `from` to `to` against a convex shape.
///
/// `closest` returns the distance from the point to the shape surface and the
/// outward normal at the closest point.
pub fn calc_point_time_of_impact(
    from: Vec3A,
    to: Vec3A,
    mut closest: impl FnMut(Vec3A) -> Option<(f32, Vec3A)>,
) -> Option<PointCastResult> {
    let delta = to - from;

    let (mut dist, mut normal) = closest(from)?;

    let mut lambda = 0.0f32;
    let mut last_lambda = 0.0f32;
    let mut iteration = 0usize;

    while dist > STOP_RADIUS {
        iteration += 1;
        if iteration > MAX_ITERATIONS {
            return None;
        }

        lambda -= dist / delta.dot(normal);
        if lambda > 1.0 || lambda < 0.0 || lambda <= last_lambda {
            return None;
        }
        last_lambda = lambda;

        let point = from * (1.0 - lambda) + to * lambda;
        let (next_dist, next_normal) = closest(point)?;
        if next_dist < 0.0 {
            return Some(PointCastResult {
                normal: next_normal,
                fraction: last_lambda,
            });
        }

        dist = next_dist;
        normal = next_normal;
    }

    if normal.dot(delta) >= 0.0 {
        return None;
    }

    Some(PointCastResult {
        normal,
        fraction: lambda,
    })
}
