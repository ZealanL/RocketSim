use std::f32::consts::FRAC_PI_4;

use glam::{Affine3A, Mat3A, Quat, Vec3A};

use super::util::bullet_length_squared;

const ANGULAR_MOTION_THRESHOLD: f32 = FRAC_PI_4;

/// Match btMatrix3x3::getRotation, including its row/column convention and
/// operation ordering.  Rust stores the basis in Mat3A columns; Bullet's
/// `m_el` is addressed as rows, so the entries are transposed on read.
pub(crate) fn bullet_quat_from_mat3(m: Mat3A) -> Quat {
    let m00 = m.x_axis.x;
    let m01 = m.y_axis.x;
    let m02 = m.z_axis.x;
    let m10 = m.x_axis.y;
    let m11 = m.y_axis.y;
    let m12 = m.z_axis.y;
    let m20 = m.x_axis.z;
    let m21 = m.y_axis.z;
    let m22 = m.z_axis.z;

    let trace = m00 + m11 + m22;
    let (x, y, z, w, root) = if trace > 0.0 {
        (m21 - m12, m02 - m20, m10 - m01, trace + 1.0, trace + 1.0)
    } else {
        let (i, j, k) = if m00 < m11 {
            if m11 < m22 { (2, 0, 1) } else { (1, 2, 0) }
        } else if m00 < m22 {
            (2, 0, 1)
        } else {
            (0, 1, 2)
        };
        let diag = [[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]];
        let root = diag[i][i] - diag[j][j] - diag[k][k] + 1.0;
        let mut t = [0.0f32; 4];
        t[3] = diag[k][j] - diag[j][k];
        t[j] = diag[j][i] + diag[i][j];
        t[k] = diag[k][i] + diag[i][k];
        t[i] = root;
        (t[0], t[1], t[2], t[3], root)
    };

    let s = 0.5 / root.sqrt();
    Quat::from_xyzw(x * s, y * s, z * s, w * s)
}

/// Bullet's SSE quaternion product operation order.  The grouping mirrors
/// btQuaternion's SIMD path (AB0/AB3 and AB1/AB2 are formed separately before
/// the final add), which is observable at low angular velocities.
#[inline]
fn bullet_quat_mul(q1: Quat, q2: Quat) -> Quat {
    let a1 = [
        q1.x * q2.w + q1.y * q2.z,
        q1.y * q2.w + q1.z * q2.x,
        q1.z * q2.w + q1.x * q2.y,
        q1.x * q2.x + q1.y * q2.y,
    ];
    let a0 = [
        q1.w * q2.x - q1.z * q2.y,
        q1.w * q2.y - q1.x * q2.z,
        q1.w * q2.z - q1.y * q2.x,
        q1.w * q2.w - q1.z * q2.z,
    ];
    Quat::from_xyzw(a0[0] + a1[0], a0[1] + a1[1], a0[2] + a1[2], a0[3] - a1[3])
}

/// Normalize a quaternion using the same horizontal reduction as Bullet's
/// `btQuaternion::normalize()` SSE path.  glam's `Quat::normalize()` routes
/// through its generic `Vec4` dot helper; that helper is algebraically
/// equivalent but can round the horizontal sum in a different order.  The
/// difference is visible as a one-ulp basis change after low-speed wheel
/// contacts and then gets amplified by later collision callbacks.
#[inline]
fn bullet_quat_normalize(q: Quat) -> Quat {
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    {
        use core::arch::x86_64::*;

        unsafe {
            let qv = _mm_set_ps(q.w, q.z, q.y, q.x);
            let mut vd = _mm_mul_ps(qv, qv);
            // Bullet: (x*x + z*z) + (y*y + w*w), via movehl/add/shuffle/addss.
            let t = _mm_movehl_ps(vd, vd);
            vd = _mm_add_ps(vd, t);
            let t = _mm_shuffle_ps(vd, vd, 0x55);
            vd = _mm_add_ss(vd, t);
            vd = _mm_sqrt_ss(vd);
            vd = _mm_div_ss(_mm_set_ss(1.0), vd);
            vd = _mm_shuffle_ps(vd, vd, 0x00);
            let out = _mm_mul_ps(qv, vd);
            let mut values = [0.0f32; 4];
            _mm_storeu_ps(values.as_mut_ptr(), out);
            Quat::from_xyzw(values[0], values[1], values[2], values[3])
        }
    }

    #[cfg(not(any(target_arch = "x86", target_arch = "x86_64")))]
    {
        q / q.length()
    }
}

/// Bullet's quaternion-to-basis conversion (the scalar formula, with the
/// same `s = 2 / length2` normalization compensation).  `Mat3A` is column
/// major, while Bullet stores basis rows, so the values are emitted as the
/// transposed columns.
#[inline]
pub(crate) fn bullet_mat3_from_quat(q: Quat) -> Mat3A {
    // On the supported desktop target, use the same SSE instruction sequence
    // as btMatrix3x3::setRotation.  This avoids the last-ulp differences that
    // remain even when the scalar expression has the same algebraic grouping.
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    {
        use core::arch::x86_64::*;

        unsafe {
            let qv = _mm_set_ps(q.w, q.z, q.y, q.x);
            let nq = _mm_xor_ps(qv, _mm_set1_ps(-0.0));

            let vd = _mm_mul_ps(qv, qv);
            let t = _mm_movehl_ps(vd, vd);
            let vd = _mm_add_ps(vd, t);
            let t = _mm_shuffle_ps(vd, vd, 0x55);
            let vd = _mm_add_ss(vd, t);
            let d = _mm_cvtss_f32(vd);
            let s = 2.0 / d;

            let mut v1 = _mm_shuffle_ps(qv, qv, 0xE1); // Y X Z W
            let mut v2 = _mm_shuffle_ps(nq, qv, 0xD0); // -X -X Y W
            let mut v3 = _mm_shuffle_ps(qv, qv, 0xC6); // Z Y X W
            v1 = _mm_xor_ps(v1, _mm_set_ps(0.0, 0.0, 0.0, -0.0));

            let v11 = _mm_shuffle_ps(qv, qv, 0xC5); // Y Y X W
            let mut v21 = _mm_unpackhi_ps(qv, qv); // Z Z W W
            let mut v31 = _mm_shuffle_ps(qv, nq, 0xC8); // X Z -X -W

            v2 = _mm_mul_ps(v2, v1);
            v1 = _mm_mul_ps(v1, v11);
            v3 = _mm_mul_ps(v3, v31);

            let v11b = _mm_shuffle_ps(nq, qv, 0xDE); // -Z -W Y W
            let v11b = _mm_mul_ps(v11b, v21);
            v21 = _mm_xor_ps(v21, _mm_set_ps(0.0, 0.0, 0.0, -0.0));
            v31 = _mm_shuffle_ps(qv, nq, 0xDF); // W W -Y -W
            v31 = _mm_xor_ps(v31, _mm_set_ps(0.0, 0.0, 0.0, -0.0));
            let y = _mm_shuffle_ps(nq, nq, 0xCB); // -W -Z -X -W
            let z = _mm_shuffle_ps(qv, qv, 0xD1); // Y X Y W

            v21 = _mm_mul_ps(v21, y);
            v31 = _mm_mul_ps(v31, z);

            v1 = _mm_add_ps(v1, v11b);
            v2 = _mm_add_ps(v2, v21);
            v3 = _mm_add_ps(v3, v31);

            let vs = _mm_set1_ps(s);
            v1 = _mm_add_ps(_mm_mul_ps(v1, vs), _mm_set_ps(0.0, 0.0, 0.0, 1.0));
            v2 = _mm_add_ps(_mm_mul_ps(v2, vs), _mm_set_ps(0.0, 0.0, 1.0, 0.0));
            v3 = _mm_add_ps(_mm_mul_ps(v3, vs), _mm_set_ps(0.0, 1.0, 0.0, 0.0));

            let mut r1 = [0.0f32; 4];
            let mut r2 = [0.0f32; 4];
            let mut r3 = [0.0f32; 4];
            _mm_storeu_ps(r1.as_mut_ptr(), v1);
            _mm_storeu_ps(r2.as_mut_ptr(), v2);
            _mm_storeu_ps(r3.as_mut_ptr(), v3);
            return Mat3A::from_cols(
                Vec3A::new(r1[0], r2[0], r3[0]),
                Vec3A::new(r1[1], r2[1], r3[1]),
                Vec3A::new(r1[2], r2[2], r3[2]),
            );
        }
    }

    #[cfg(not(any(target_arch = "x86", target_arch = "x86_64")))]
    {
        // Bullet's SIMD dot product reduces as (x*x + z*z) + (y*y + w*w).
        // Keep that grouping instead of glam's generic length_squared helper.
        let x2 = q.x * q.x;
        let y2 = q.y * q.y;
        let z2 = q.z * q.z;
        let w2 = q.w * q.w;
        let d = (x2 + z2) + (y2 + w2);
        let s = 2.0 / d;

        // btMatrix3x3's SSE path forms the products first, adds the signed
        // terms, and only then multiplies each row by s.  The scalar-looking
        // formula (q.x * (q.y * s), etc.) rounds differently and accumulates a
        // visible orientation drift over a replay.
        let r00 = 1.0 + ((-y2 - z2) * s);
        let r01 = ((q.x * q.y) + (-(q.w * q.z))) * s;
        let r02 = ((q.x * q.z) + (q.w * q.y)) * s;
        let r10 = ((q.x * q.y) + (q.w * q.z)) * s;
        let r11 = 1.0 + ((-x2 - z2) * s);
        let r12 = ((q.y * q.z) + (-(q.w * q.x))) * s;
        let r20 = ((q.x * q.z) + (-(q.w * q.y))) * s;
        let r21 = ((q.y * q.z) + (q.w * q.x)) * s;
        let r22 = 1.0 + ((-x2 - y2) * s);

        return Mat3A::from_cols(
            Vec3A::new(r00, r10, r20),
            Vec3A::new(r01, r11, r21),
            Vec3A::new(r02, r12, r22),
        );
    }
}

#[inline]
pub fn integrate_trans_no_rot(cur_trans: &mut Vec3A, lin_vel: Vec3A, time_step: f32) {
    *cur_trans += lin_vel * time_step;
}

pub fn integrate_trans(
    cur_trans: &mut Affine3A,
    cur_rot: &mut Quat,
    lin_vel: Vec3A,
    ang_vel: Vec3A,
    time_step: f32,
) {
    integrate_trans_no_rot(&mut cur_trans.translation, lin_vel, time_step);

    // Bullet's exponential-map path first checks fAngle2 against
    // SIMD_EPSILON and only then takes the square root.  In particular, the
    // tiny angular velocities generated by resting wheel contacts become an
    // exact zero here; using ang_vel.length() directly leaves a small Taylor
    // correction that accumulates into replay drift.
    let f_angle2 = bullet_length_squared(ang_vel);
    let mut angle = if f_angle2 > f32::EPSILON {
        f_angle2.sqrt()
    } else {
        0.0
    };

    if angle * time_step > ANGULAR_MOTION_THRESHOLD {
        angle = ANGULAR_MOTION_THRESHOLD / time_step;
    }

    let axis = if angle < 0.001 {
        ang_vel
            * (0.5 * time_step
                - time_step * time_step * time_step * 0.020_833_333_333 * angle * angle)
    } else {
        ang_vel * ((0.5 * angle * time_step).sin() / angle)
    };

    let dorn = Quat::from_xyzw(axis.x, axis.y, axis.z, (angle * time_step * 0.5).cos());
    *cur_rot = bullet_quat_normalize(bullet_quat_mul(dorn, *cur_rot));
    cur_trans.matrix3 = bullet_mat3_from_quat(*cur_rot);
}
