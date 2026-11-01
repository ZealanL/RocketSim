use glam::{IVec3, Mat3A, Quat, Vec3A};

use crate::{
    bullet::dynamics::rigid_body::{CollisionFlags, RigidBody},
    sim::consts::{BT_TO_UU, UU_TO_BT, quantize},
};

/// An extension of `Vec3A::signum` that keeps zero-components as zero (regardless of sign bit).
#[must_use]
fn vec3a_sign_3(v: Vec3A) -> Vec3A {
    let signum = v.signum();
    Vec3A::select(v.cmpeq(Vec3A::ZERO), Vec3A::ZERO, signum)
}

enum VecQuantizeMode {
    Position,
    Velocity,
}

/// UE3-networking-style quantization of vectors.
#[must_use]
fn quantize_vec_ue3(vec: Vec3A, scale: f32, quantize_mode: VecQuantizeMode) -> Vec3A {
    match quantize_mode {
        VecQuantizeMode::Position => (vec * scale + 0.5).floor() * (1.0 / scale),
        VecQuantizeMode::Velocity => {
            let inv_scale = 1.0 / scale;
            let scaled = vec * scale;
            let scaled_values = scaled.to_array();
            let mut rounded = Vec3A::ZERO;
            for i in 0..3 {
                let i_val = scaled_values[i] as i32;
                rounded[i] = (i_val as f32) * inv_scale;
            }

            const OFFSET_CORRECT_FRAC: f32 = 0.1;
            let offset_mag = OFFSET_CORRECT_FRAC * inv_scale;

            rounded + (vec3a_sign_3(rounded) * offset_mag)
        }
    }
}

// Recorded-state model of the replicated-rigid-body quaternion
// representation. These are the scalar Bullet forms. This model reproduces
// the recorded representation; it is not a recovered native routine.
fn game_set_rotation_scalar(q: Quat) -> Mat3A {
    let d = ((q.x * q.x + q.y * q.y) + q.z * q.z) + q.w * q.w;
    let s = 2.0 / d;
    let xs = q.x * s;
    let ys = q.y * s;
    let zs = q.z * s;
    let wx = q.w * xs;
    let wy = q.w * ys;
    let wz = q.w * zs;
    let xx = q.x * xs;
    let xy = q.x * ys;
    let xz = q.x * zs;
    let yy = q.y * ys;
    let yz = q.y * zs;
    let zz = q.z * zs;
    Mat3A::from_cols(
        Vec3A::new(1.0 - (yy + zz), xy + wz, xz - wy),
        Vec3A::new(xy - wz, 1.0 - (xx + zz), yz + wx),
        Vec3A::new(xz + wy, yz - wx, 1.0 - (xx + yy)),
    )
}

fn game_quat_normalize_scalar(q: Quat, min_len_sq: f32) -> Quat {
    let len_sq = ((q.x * q.x + q.y * q.y) + q.z * q.z) + q.w * q.w;
    if len_sq.partial_cmp(&min_len_sq) != Some(std::cmp::Ordering::Greater) {
        return Quat::IDENTITY;
    }
    let s = 1.0 / len_sq.sqrt();
    Quat::from_xyzw(q.x * s, q.y * s, q.z * s, q.w * s)
}

fn game_quat_quantize_component(c: f32) -> f32 {
    const MAX: i32 = (1 << 18) - 1;
    let max_f = MAX as f32;
    let mut v = ((c * std::f32::consts::SQRT_2) * 0.5 + 0.5) * max_f;
    v = (v + v) + 0.5;
    let k = ((v.round_ties_even() as i32) >> 1).clamp(0, MAX);
    let r = k as f32 / max_f - 0.5;
    (r + r) * 0.707_106_77
}

fn game_rb_state_quantize_quat(q_in: Quat) -> Quat {
    let q = game_quat_normalize_scalar(q_in, 1e-8);
    let c = [q.x, q.y, q.z, q.w];
    let a = [c[0].abs(), c[1].abs(), c[2].abs(), c[3].abs()];
    let i01 = if a[0] < a[1] { 1 } else { 0 };
    let i23 = if a[2] < a[3] { 3 } else { 2 };
    let big = if a[i01] >= a[i23] { i01 } else { i23 };
    let sign = if c[big] < 0.0 { -1.0 } else { 1.0 };
    let mut d = [0.0; 3];
    let mut n = 0;
    for (i, &value) in c.iter().enumerate() {
        if i != big {
            d[n] = game_quat_quantize_component(value * sign);
            n += 1;
        }
    }
    let rest = ((1.0 - d[0] * d[0]) - d[1] * d[1]) - d[2] * d[2];
    debug_assert!(
        rest >= 0.0,
        "largest-component decode must have non-negative rest, got {rest}"
    );
    let dropped = rest.sqrt();
    let mut o = [0.0; 4];
    n = 0;
    for (i, slot) in o.iter_mut().enumerate() {
        *slot = if i == big {
            dropped
        } else {
            let value = d[n];
            n += 1;
            value
        };
    }
    game_quat_normalize_scalar(Quat::from_xyzw(o[0], o[1], o[2], o[3]), 1e-8)
}

/// Quantize position, orientation, linear velocity, and angular velocity.
/// Leave orientation unchanged when angular motion is disabled.
pub fn quantize(body: &mut RigidBody) {
    let new_pos = quantize_vec_ue3(
        body.get_world_pos() * BT_TO_UU,
        quantize::POS_SCALE,
        VecQuantizeMode::Position,
    ) * UU_TO_BT;
    let new_vel = quantize_vec_ue3(
        body.lin_vel * BT_TO_UU,
        quantize::VEL_SCALE,
        VecQuantizeMode::Velocity,
    ) * UU_TO_BT;
    let new_ang_vel = quantize_vec_ue3(
        body.ang_vel,
        quantize::ANG_VEL_SCALE,
        VecQuantizeMode::Velocity,
    );

    body.set_world_pos(new_pos);
    body.set_lin_vel(new_vel);
    body.set_ang_vel(new_ang_vel);

    if body.collision_flags & CollisionFlags::NoAngularMotion != 0 {
        return;
    }

    let rot = game_rb_state_quantize_quat(body.get_world_rot());
    let mut trans = *body.get_world_trans();
    trans.matrix3 = game_set_rotation_scalar(rot);
    body.set_world_trans_and_rot(trans, rot);
}

/// Asymmetric 8-bit input quantization: negatives scale by 128, positives by 127.
pub fn quantize_axis_inputs(ctrls: Vec3A) -> Vec3A {
    const UPPER_BOUND: Vec3A = Vec3A::splat(128.0);
    const LOWER_BOUND: Vec3A = Vec3A::splat(127.0);

    let clamped = ctrls.clamp(Vec3A::NEG_ONE, Vec3A::ONE);
    let scale = Vec3A::select(clamped.cmplt(Vec3A::ZERO), UPPER_BOUND, LOWER_BOUND);
    let biased = clamped * scale + UPPER_BOUND;
    let w = biased + biased + Vec3A::splat(0.5);
    let byte = ((w.round().as_ivec3() >> 1i32) & IVec3::splat(0xFF)).as_vec3a();
    let s = byte - UPPER_BOUND;

    Vec3A::select(
        s.cmplt(Vec3A::ZERO),
        s * (1.0 / UPPER_BOUND),
        s / LOWER_BOUND,
    )
}

/// Single-axis form of [`quantize_axis_inputs`].
#[must_use]
pub fn quantize_axis_input(x: f32) -> f32 {
    let clamped = x.clamp(-1.0, 1.0);
    let y = if clamped < 0.0 {
        (clamped * 128.0).max(-128.0)
    } else {
        (clamped * 127.0).min(127.0)
    };
    let w = ((y + 128.0) + (y + 128.0)) + 0.5;
    let eax = w.round_ties_even() as i32;
    let byte = ((eax >> 1) & 0xFF) as u8;
    let s = (byte as f32) - 128.0;
    if byte < 0x80 {
        s * (1.0 / 128.0)
    } else {
        s / 127.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bullet::{
        collision::shapes::{collision_shape::CollisionShapes, sphere_shape::SphereShape},
        dynamics::rigid_body::RigidBodyConstructionInfo,
    };

    #[test]
    fn quantize_uses_cached_rotation_after_transform_updates() {
        let mut body = RigidBody::new(RigidBodyConstructionInfo::new(
            1.0,
            CollisionShapes::Sphere(SphereShape::new(1.0)),
        ));
        body.set_ang_vel(Vec3A::new(1.2, -2.3, 0.7));

        // Case 102 crosses a codec boundary if the matrix is converted again.
        for i in [0, 1, 102] {
            let angle = i as f32;
            let mut trans = *body.get_world_trans();
            trans.matrix3 = glam::Mat3A::from_quat(Quat::from_euler(
                glam::EulerRot::XYZ,
                angle * 0.31,
                angle * 0.17,
                angle * -0.23,
            ));
            body.set_world_trans(trans);

            for integrate in [false, true] {
                if integrate {
                    body.set_center_of_mass_trans(body.predict_integration_trans(1.0 / 120.0));
                }
                let expected = game_rb_state_quantize_quat(body.get_world_rot());
                quantize(&mut body);
                assert_eq!(
                    body.get_world_rot().to_array().map(f32::to_bits),
                    expected.to_array().map(f32::to_bits),
                    "orientation {i}, integrate {integrate}",
                );
                let expected_matrix = glam::Mat3A::from_quat(expected.normalize());
                assert!(
                    body.get_world_trans()
                        .matrix3
                        .abs_diff_eq(expected_matrix, 5e-7)
                );
            }
        }
    }

    /// The largest component is x and w with a tie. The model drops the
    /// lower-index x, keeps its positive root, and flips the three negated
    /// inputs. This pins the tie and sign rules.
    #[test]
    fn quat_codec_tie_and_sign_are_canonical() {
        let q = game_rb_state_quantize_quat(Quat::from_xyzw(-0.5, 0.5, 0.5, 0.5));
        assert!(q.x > 0.0, "{q:?}");
        assert!(q.y < 0.0 && q.z < 0.0 && q.w < 0.0, "{q:?}");
        let again = game_rb_state_quantize_quat(q);
        assert_eq!(
            again.to_array().map(f32::to_bits),
            q.to_array().map(f32::to_bits),
            "model must be a fixed point"
        );
    }

    #[test]
    fn quat_component_quantize_is_idempotent_on_its_grid() {
        for c in [-1.0, -0.5, -0.1, 0.0, 0.1, 0.5, 0.9, 1.0] {
            let q = game_quat_quantize_component(c);
            assert_eq!(
                game_quat_quantize_component(q).to_bits(),
                q.to_bits(),
                "component {c} decode must be a grid fixed point"
            );
        }
    }
}
