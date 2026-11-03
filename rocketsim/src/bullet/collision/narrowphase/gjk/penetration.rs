use glam::{Affine3A, Vec3A};

use super::solver::{Epa2, EpaStatus, Gjk, GjkStatus, MinkowskiDiff};
use crate::bullet::collision::shapes::collision_shape::CollisionShapes;

const GJK_MIN_DISTANCE: f32 = 1.0e-4;

pub struct GjkEpa2Result {
    pub witnesses: [Vec3A; 2],
    pub normal: Vec3A,
    pub penetrating: bool,
}

fn penetration(
    shape_a: &CollisionShapes,
    trans_a: &Affine3A,
    shape_b: &CollisionShapes,
    trans_b: &Affine3A,
    guess: Vec3A,
) -> Option<GjkEpa2Result> {
    penetration_with_status(shape_a, trans_a, shape_b, trans_b, guess).map(|(result, _)| result)
}

fn penetration_with_status(
    shape_a: &CollisionShapes,
    trans_a: &Affine3A,
    shape_b: &CollisionShapes,
    trans_b: &Affine3A,
    guess: Vec3A,
) -> Option<(GjkEpa2Result, EpaStatus)> {
    let shape = MinkowskiDiff::new(shape_a, *trans_a, shape_b, *trans_b);

    let mut gjk = Gjk::new(&shape);
    match gjk.evaluate::<true>(-guess) {
        GjkStatus::Inside => {}
        GjkStatus::Valid | GjkStatus::Failed => return None,
    }

    let mut epa = Epa2::new();
    let status = epa.evaluate::<true>(gjk, -guess);
    if status == EpaStatus::Failed {
        return None;
    }

    // `Penetration` reconstructs the witness from the marginless support plus
    // the margin along the stored direction, in shape A local space.
    let margin_a = shape.margin_a();
    let mut w0 = Vec3A::ZERO;
    for i in 0..epa.result.rank {
        let vertex = epa.sv_store[epa.result.c[i]];

        let mut dir = vertex.d;
        if dir.length_squared() < f32::EPSILON * f32::EPSILON {
            dir = Vec3A::NEG_ONE;
        }
        dir = dir * (1.0 / dir.length());

        let support0 = shape.support0_without_margin(dir) + margin_a * dir;
        w0 += support0 * epa.result.p[i];
    }

    let witness0 = trans_a.transform_point3a(w0);
    let witness1 = trans_a.transform_point3a(w0 - epa.normal * epa.depth);

    let normal = trans_a.matrix3 * -epa.normal;

    Some((
        GjkEpa2Result {
            witnesses: [witness0, witness1],
            normal,
            penetrating: true,
        },
        status,
    ))
}

fn distance(
    shape_a: &CollisionShapes,
    trans_a: &Affine3A,
    shape_b: &CollisionShapes,
    trans_b: &Affine3A,
    guess: Vec3A,
) -> Option<GjkEpa2Result> {
    let shape = MinkowskiDiff::new(shape_a, *trans_a, shape_b, *trans_b);

    let mut gjk = Gjk::new(&shape);
    match gjk.evaluate::<false>(guess) {
        GjkStatus::Valid => {}
        GjkStatus::Inside | GjkStatus::Failed => return None,
    }

    let simplex = *gjk.simplex();
    let mut w0 = Vec3A::ZERO;
    let mut w1 = Vec3A::ZERO;

    for (i, [_w, sv_d]) in gjk.simplex_w_d().enumerate() {
        let weight = simplex.p[i];
        w0 += shape.support0_without_margin(sv_d) * weight;
        w1 += shape.support1_without_margin(-sv_d) * weight;
    }

    let witness0 = trans_a.transform_point3a(w0);
    let witness1 = trans_a.transform_point3a(w1);

    let normal_local = w0 - w1;
    let distance = normal_local.length();
    let normal_scale = 1.0
        / if distance > GJK_MIN_DISTANCE {
            distance
        } else {
            1.0
        };
    let normal_local = normal_local * normal_scale;
    let normal = trans_a.matrix3 * normal_local;

    Some(GjkEpa2Result {
        witnesses: [witness0, witness1],
        normal,
        penetrating: false,
    })
}

pub fn default_guess(trans_a: &Affine3A, trans_b: &Affine3A) -> Vec3A {
    let delta = trans_b.translation - trans_a.translation;
    let len2 = delta.length_squared();
    if len2 > 1.0e-20 {
        delta * (1.0 / len2.sqrt())
    } else {
        Vec3A::ZERO
    }
}

/// Penetration first, then distance, with one guess.
pub fn calc_pen_depth(
    shape_a: &CollisionShapes,
    shape_b: &CollisionShapes,
    trans_a: &Affine3A,
    trans_b: &Affine3A,
) -> Option<GjkEpa2Result> {
    calc_pen_depth_with_guess(
        shape_a,
        shape_b,
        trans_a,
        trans_b,
        default_guess(trans_a, trans_b),
    )
}

/// Explicit-guess form. No current caller uses it. It keeps the one-guess
/// contract available without changing the `calc_pen_depth` signature.
pub fn calc_pen_depth_with_guess(
    shape_a: &CollisionShapes,
    shape_b: &CollisionShapes,
    trans_a: &Affine3A,
    trans_b: &Affine3A,
    guess: Vec3A,
) -> Option<GjkEpa2Result> {
    if let Some(result) = penetration(shape_a, trans_a, shape_b, trans_b, guess) {
        return Some(result);
    }

    distance(shape_a, trans_a, shape_b, trans_b, guess)
}
