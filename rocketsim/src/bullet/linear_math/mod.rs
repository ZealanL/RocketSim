use std::f32::consts::FRAC_1_SQRT_2;

use glam::{Affine3A, Mat3A, Quat, Vec3A, Vec4, Vec4Swizzles};
pub mod angle;
pub mod obb;
pub mod transform_util;

pub const LARGE_FLOAT: f32 = 1e18;

pub trait AffineExt {
    fn transpose(&self) -> Self;
    fn inv_x_form(&self, in_vec: Vec3A) -> Vec3A;
}

impl AffineExt for Affine3A {
    fn transpose(&self) -> Self {
        let matrix3 = self.matrix3.transpose();

        Self {
            matrix3,
            translation: matrix3 * -self.translation,
        }
    }

    fn inv_x_form(&self, in_vec: Vec3A) -> Vec3A {
        self.matrix3.mul_transpose_vec3a(in_vec - self.translation)
    }
}

pub trait Mat3AExt {
    fn cofac(&self, r1: usize, c1: usize, r2: usize, c2: usize) -> f32;
    fn bullet_inverse(&self) -> Self;
    fn bullet_from_quat(q: Quat) -> Self;
}

impl Mat3AExt for Mat3A {
    fn cofac(&self, r1: usize, c1: usize, r2: usize, c2: usize) -> f32 {
        self.col(r1)[c1] * self.col(r2)[c2] - self.col(r1)[c2] * self.col(r2)[c1]
    }

    fn bullet_inverse(&self) -> Self {
        let co = Vec3A::new(
            self.cofac(1, 1, 2, 2),
            self.cofac(1, 2, 2, 0),
            self.cofac(1, 0, 2, 1),
        );
        let det = self.x_axis.dot(co);
        debug_assert_ne!(det, 0.0);
        let s = Vec3A::splat(det.recip());

        Self::from_cols(
            co * s,
            Vec3A::new(
                self.cofac(0, 2, 2, 1),
                self.cofac(0, 0, 2, 2),
                self.cofac(0, 1, 2, 0),
            ) * s,
            Vec3A::new(
                self.cofac(0, 1, 1, 2),
                self.cofac(0, 2, 1, 0),
                self.cofac(0, 0, 1, 1),
            ) * s,
        )
    }

    fn bullet_from_quat(q: Quat) -> Self {
        let d = q.length_squared();
        let s = 2.0 / d;

        let q: Vec4 = q.into();
        let nq = -q;

        let mut v1 = q.yxzw() * Vec4::new(-1.0, 1.0, 1.0, 1.0);
        let mut v2 = q.xxyw() * Vec4::new(-1.0, -1.0, 1.0, 1.0);
        let mut v3 = q.zyxw();

        let v11 = q.yyxw();
        let mut v21 = q.zzww();
        let v31 = q.xzxw() * Vec4::new(1.0, 1.0, -1.0, -1.0);

        v2 *= v1;
        v1 *= v11;
        v3 *= v31;

        let v11 = q.zwyw() * Vec4::new(-1.0, -1.0, 1.0, 1.0) * v21;
        v21.x *= -1.0;
        let mut v31 = q.wwyw() * Vec4::new(-1.0, 1.0, -1.0, -1.0);
        let y = nq.wzxw();
        let z = q.yxyw();

        v21 *= y;
        v31 *= z;

        v1 += v11;
        v2 += v21;
        v3 += v31;

        let vs = Vec4::new(s, s, s, 0.0);
        v1 *= vs;
        v2 *= vs;
        v3 *= vs;

        v1.x += 1.0;
        v2.y += 1.0;
        v3.z += 1.0;

        Self::from_cols(
            Vec3A::from_vec4(v1),
            Vec3A::from_vec4(v2),
            Vec3A::from_vec4(v3),
        )
        .transpose()
    }
}

pub trait QuatExt {
    fn bullet_mul_quat(self, q2: Self) -> Self;
    fn bullet_normalize(self) -> Self;
    fn from_axis_angle_simd(axis: Vec3A, angle: f32) -> Self;
}

impl QuatExt for Quat {
    fn bullet_mul_quat(self, q2: Self) -> Self {
        const NEG_W: Vec4 = Vec4::new(1.0, 1.0, 1.0, -1.0);

        let q1: Vec4 = self.into();
        let q2: Vec4 = q2.into();

        let a2 = q1.yzxy() * q2.zxyy();
        let a1 = q1.xyzx() * q2.wwwx() + a2;

        let b1 = q1.zxyz() * q2.yzxz();
        let a0 = q1.wwww() * q2 - b1;

        let q = a0 + a1 * NEG_W;
        Self::from_vec4(q)
    }

    fn bullet_normalize(self) -> Self {
        let vec: Vec4 = self.into();
        let q = vec * vec.length().recip();
        Self::from_vec4(q)
    }

    #[inline]
    /// An implementation of `Quat::from_axis_angle` that leverages simd
    fn from_axis_angle_simd(axis: Vec3A, angle: f32) -> Self {
        debug_assert!(axis.is_normalized());
        let (s, c) = f32::sin_cos(angle * 0.5);
        let v = axis * s;
        Self::from_xyzw(v.x, v.y, v.z, c)
    }
}

#[inline]
pub fn interpolate_3(v0: Vec3A, v1: Vec3A, rt: f32) -> Vec3A {
    let s = 1.0 - rt;
    s * v0 + rt * v1
}

pub fn plane_space_2(n: Vec3A) -> (Vec3A, Vec3A) {
    if n.z.abs() > FRAC_1_SQRT_2 {
        // choose p in y-z plane
        let a = n.y * n.y + n.z * n.z;
        let k = a.sqrt().recip();
        let p = Vec3A::new(0., -n.z * k, n.y * k);
        (p, Vec3A::new(a * k, -n.x * p.z, n.x * p.y))
    } else {
        // choose p in x-y plane
        let a = n.x * n.x + n.y * n.y;
        let k = a.sqrt().recip();
        let p = Vec3A::new(-n.y * k, n.x * k, 0.);
        (p, Vec3A::new(-n.z * p.y, n.z * p.x, a * k))
    }
}

pub fn plane_space_1(n: Vec3A) -> Vec3A {
    if n.z.abs() > FRAC_1_SQRT_2 {
        // choose p in y-z plane
        let a = n.y * n.y + n.z * n.z;
        let k = a.sqrt().recip();
        Vec3A::new(0., -n.z * k, n.y * k)
    } else {
        // choose p in x-y plane
        let a = n.x * n.x + n.y * n.y;
        let k = a.sqrt().recip();
        Vec3A::new(-n.y * k, n.x * k, 0.)
    }
}

pub fn max_dot(simd_points: &[[Vec4; 3]], points: &[Vec3A], direction: Vec3A) -> Vec3A {
    let mut max_dot = f32::NEG_INFINITY;
    let mut support_vertex = Vec3A::ZERO;

    let dir_x = Vec4::splat(direction.x);
    let dir_y = Vec4::splat(direction.y);
    let dir_z = Vec4::splat(direction.z);

    for (i, pts) in simd_points.iter().enumerate() {
        let dots = pts[0] * dir_x + pts[1] * dir_y + pts[2] * dir_z;

        let this_max_dot = dots.max_element();
        if this_max_dot > max_dot {
            max_dot = this_max_dot;
            support_vertex = points[i * 4 + dots.max_position()];
        }
    }

    for &point in &points[simd_points.len() * 4..] {
        let dot = direction.dot(point);
        if dot > max_dot {
            support_vertex = point;
            max_dot = dot;
        }
    }

    support_vertex
}
