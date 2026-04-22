use glam::Vec3A;

use super::solver::VoronoiSimplexSolver;
use crate::bullet::collision::shapes::convex_hull_shape::ConvexHullShape;

const MAX_CONVEX_CAST_ITERATIONS: usize = 32;
const MAX_CONVEX_CAST_EPSILON: f32 = 0.0001;

pub struct SubSimplexCastResult {
    pub normal: Vec3A,
    pub fraction: f32,
}

pub fn calc_time_of_impact(
    convex_b: &ConvexHullShape,
    from_a: Vec3A,
    to_a: Vec3A,
) -> Option<SubSimplexCastResult> {
    let mut simplex_solver = VoronoiSimplexSolver::new();
    let mut lambda = 0.0;
    let mut interpolated_trans_a = from_a;

    let sup_vertex_b = convex_b.local_get_supporting_vertex(to_a - from_a);
    let mut v = from_a - sup_vertex_b;

    let mut n = Vec3A::ZERO;
    for _ in 0..MAX_CONVEX_CAST_ITERATIONS {
        let w = interpolated_trans_a - sup_vertex_b;

        let v_dot_w = v.dot(w);

        if lambda > 1.0 {
            return None;
        }

        if v_dot_w > 0.0 {
            let v_dot_r = v.dot(to_a - from_a);

            if v_dot_r >= -(f32::EPSILON * f32::EPSILON) {
                return None;
            }

            lambda -= v_dot_w / v_dot_r;

            interpolated_trans_a = from_a + (to_a - from_a) * lambda;

            n = v;
        }

        if !simplex_solver.in_simplex(w) {
            simplex_solver.add_vertex(w, interpolated_trans_a, sup_vertex_b);
        }

        let dist2 = if simplex_solver.closest(&mut v) {
            v.length_squared()
        } else {
            0.0
        };

        if dist2 <= MAX_CONVEX_CAST_EPSILON {
            break;
        }
    }

    let normal = n.try_normalize()?;
    if normal.dot(to_a - from_a).is_sign_negative() {
        Some(SubSimplexCastResult {
            normal,
            fraction: lambda,
        })
    } else {
        None
    }
}
