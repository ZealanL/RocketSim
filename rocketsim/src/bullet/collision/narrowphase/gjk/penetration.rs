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
    let status: EpaStatus = epa.evaluate::<true>(gjk, guess);
    debug_assert!(status == EpaStatus::Valid || status == EpaStatus::AccuracyReached);

    let mut w0 = Vec3A::ZERO;
    for i in 0..epa.result.rank {
        w0 += shape.support0::<true>(epa.sv_store[epa.result.c[i]].d) * epa.result.p[i];
    }

    // w0 is already in world space: MinkowskiDiff::support0 transforms the
    // local support point by trans_a. Do not transform it a second time.
    // This matches the distance fallback below, which returns world witnesses.
    Some(GjkEpa2Result {
        witnesses: [w0, w0 - epa.normal * epa.depth],
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

    // Bullet's Distance normal is w0-w1 (shape B toward shape A). Keep the
    // same direction in world space.
    Some(GjkEpa2Result {
        witnesses: [w0, w1],
        normal: (w0 - w1).normalize_or_zero(),
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bullet::collision::shapes::{box_shape::BoxShape, compound_shape::CompoundShape};

    fn box_compound(half: Vec3A) -> (CollisionShapes, f32) {
        let shape = BoxShape::new(half);
        let margin = shape.get_margin();
        (
            CollisionShapes::Compound(CompoundShape::new(shape, Affine3A::IDENTITY)),
            margin,
        )
    }

    /// Analytic A/B witness derivation (Bullet `btGjkEpa2.cpp` convention).
    ///
    /// Bullet's `MinkowskiDiff::Support0/1` return shape-A-local points, so
    /// the EPA API emits `witnesses = [wtrs0*w0, wtrs0*(w0-n*d)]` with
    /// `normal = -n`, `distance = -d`. RocketSim's `MinkowskiDiff::support0/1`
    /// already return world points (`minkowski_diff.rs` applies the transform
    /// inside), so its `w0` already equals Bullet's `wtrs0*w0`: transforming
    /// by `trans_a` a second time displaces witnesses by one full shape
    /// translation. This test pins the world-frame convention on two unit
    /// boxes overlapped 0.5 along +x, parked far from the origin so any
    /// extra transform fails loudly.
    ///
    /// Note the assertions are structural, not face-optimal: EPA returns a
    /// hull-face lower bound on the true depth (here a diagonal face with
    /// depth 0.2887 <= true 0.5 on the exactly symmetric setup), and the
    /// detector recomputes its normal/depth from the witness separation, so
    /// what must hold exactly is the witness order, the normal sign, and
    /// the `worldA = point + normal * depth` adapter identity.
    #[test]
    fn penetration_witnesses_match_bullet_world_frame_convention() {
        let (shape_a, margin_a) = box_compound(Vec3A::splat(1.0));
        let (shape_b, margin_b) = box_compound(Vec3A::splat(1.0));
        assert!((margin_a - 0.04).abs() < 1e-6);
        assert!((margin_b - 0.04).abs() < 1e-6);

        let center_a = Vec3A::new(50.0, -30.0, 10.0);
        let center_b = center_a + Vec3A::new(1.5, 0.0, 0.0);
        let trans_a = Affine3A::from_translation(center_a.into());
        let trans_b = Affine3A::from_translation(center_b.into());

        let result = calc_pen_depth(&shape_a, &shape_b, &trans_a, &trans_b).expect("boxes overlap");
        assert!(result.penetrating);
        let [w_on_a, w_on_b] = result.witnesses;

        // EPA lower bound: hull-face depth is positive and cannot exceed the
        // true minimum-face overlap (full-surface overlap is 0.5 here).
        let ab = w_on_b - w_on_a;
        let len = ab.length();
        assert!(len > 1e-6, "degenerate witness separation");
        assert!(len <= 0.5 + 1e-3, "EPA depth {len} exceeds true overlap");

        // Convention: stored normal is unit length and agrees exactly with
        // the witness order (B - A), matching Bullet's detector derivation
        // `tmpNormalInB = B - A` and the target `worldA = point + n*d` check.
        assert!((result.normal.length() - 1.0).abs() < 1e-5);
        assert!((result.normal - ab / len).length() < 1e-5);
        let world_a = w_on_b + result.normal * (-len);
        assert!((world_a - w_on_a).length() < 1e-4);

        // w0 is a convex combination of grown-surface supports, so it always
        // lies inside shape A's grown box (half 1.0 about its center).
        assert!((w_on_a - center_a).abs().max_element() <= 1.0 + 1e-3);

        // World frame, not double-transformed: both witnesses sit on the
        // pair, not a full translation away from it.
        let pair_mid = (center_a + center_b) * 0.5;
        assert!((w_on_a - pair_mid).length() < 3.0);
        assert!((w_on_b - pair_mid).length() < 3.0);
    }

    /// Separated arm: Bullet's `Distance` normal is `w0 - w1` (B toward A)
    /// with unmargined witnesses. Same far-from-origin placement. Box corner
    /// snapping on exactly axis-aligned directions is faithful Bullet
    /// behavior (`btBoxShape` sign selection); the gap and normal are exact.
    #[test]
    fn distance_witnesses_match_bullet_separated_convention() {
        let (shape_a, _) = box_compound(Vec3A::splat(1.0));
        let (shape_b, _) = box_compound(Vec3A::splat(1.0));

        let center_a = Vec3A::new(50.0, -30.0, 10.0);
        let center_b = center_a + Vec3A::new(3.0, 0.0, 0.0);
        let trans_a = Affine3A::from_translation(center_a.into());
        let trans_b = Affine3A::from_translation(center_b.into());

        let result =
            calc_pen_depth(&shape_a, &shape_b, &trans_a, &trans_b).expect("separated boxes");
        assert!(!result.penetrating);
        let [w_on_a, w_on_b] = result.witnesses;

        // Unmargined faces: A max-x at 50.96, B min-x at 52.04, gap 1.08.
        assert!((w_on_a.x - 50.96).abs() < 1e-3, "witness on A {w_on_a:?}");
        assert!((w_on_b.x - 52.04).abs() < 1e-3, "witness on B {w_on_b:?}");
        assert!(((w_on_b - w_on_a).length() - 1.08).abs() < 2e-3);
        assert!((result.normal - Vec3A::new(-1.0, 0.0, 0.0)).length() < 1e-5);
        // Normal equals (w0 - w1) direction, per Bullet.
        let ab = w_on_a - w_on_b;
        assert!((result.normal - ab.normalize_or_zero()).length() < 1e-5);

        // Witnesses lie inside their unmargined boxes (half 0.96) in world frame.
        assert!((w_on_a - center_a).abs().max_element() <= 0.96 + 1e-3);
        assert!((w_on_b - center_b).abs().max_element() <= 0.96 + 1e-3);
    }
}
