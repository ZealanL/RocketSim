use std::debug_assert_matches;

use glam::{Affine3A, Vec3A};

use super::solver::{Epa2, EpaStatus, Gjk, GjkStatus, MinkowskiDiff};
use crate::bullet::collision::shapes::collision_shape::CollisionShapes;

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
    let shape = MinkowskiDiff::new(shape_a, *trans_a, shape_b, *trans_b);

    let mut gjk = Gjk::new(&shape);
    match gjk.evaluate::<true>(-guess) {
        GjkStatus::Failed | GjkStatus::Valid => return None,
        GjkStatus::Inside => {}
    }

    let mut epa = Epa2::new();
    let status = epa.evaluate::<true>(gjk, guess);
    debug_assert_matches!(status, EpaStatus::Valid | EpaStatus::AccuracyReached);

    let mut w0 = Vec3A::ZERO;
    for i in 0..epa.result.rank {
        w0 += shape.support0::<true>(epa.sv_store[epa.result.c[i]].d) * epa.result.p[i];
    }

    Some(GjkEpa2Result {
        witnesses: [
            trans_a.transform_point3a(w0),
            trans_a.transform_point3a(w0 - epa.normal * epa.depth),
        ],
        normal: -epa.normal,
        penetrating: true,
    })
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
        GjkStatus::Inside | GjkStatus::Failed => return None,
        GjkStatus::Valid => {}
    }

    let simplex = gjk.simplex();
    if simplex.rank == 0 {
        return None;
    }

    let mut w0 = Vec3A::ZERO;
    let mut w1 = Vec3A::ZERO;

    for (i, sv_d) in gjk.simplex_d().enumerate() {
        let weight = simplex.p[i];
        if weight != 0.0 {
            w0 += shape.support0::<false>(sv_d) * weight;
            w1 += shape.support1::<false>(-sv_d) * weight;
        }
    }

    Some(GjkEpa2Result {
        witnesses: [w0, w1],
        normal: (w1 - w0).normalize_or_zero(),
        penetrating: false,
    })
}

pub fn calc_pen_depth(
    shape_a: &CollisionShapes,
    shape_b: &CollisionShapes,
    trans_a: &Affine3A,
    trans_b: &Affine3A,
) -> Option<GjkEpa2Result> {
    let guess_vectors = [
        (trans_b.translation - trans_a.translation).normalize_or_zero(),
        (trans_a.translation - trans_b.translation).normalize_or_zero(),
        Vec3A::Z,
        Vec3A::Y,
        Vec3A::X,
        Vec3A::ONE.with_z(0.0),
        Vec3A::ONE,
        Vec3A::ONE.with_x(0.0),
        Vec3A::ONE.with_y(0.0),
    ];

    for guess in guess_vectors {
        let results = penetration(shape_a, trans_a, shape_b, trans_b, guess);
        if results.is_some() {
            return results;
        }

        let results = distance(shape_a, trans_a, shape_b, trans_b, guess);
        if results.is_some() {
            return results;
        }
    }

    None
}
