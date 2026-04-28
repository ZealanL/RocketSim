use std::f32::consts::FRAC_1_SQRT_2;

use glam::{Affine3A, Quat, Vec3A, Vec4};
pub mod angle;
pub mod obb;
pub mod transform_util;

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

pub trait QuatExt {
    fn from_axis_angle_simd(axis: Vec3A, angle: f32) -> Self;
}

impl QuatExt for Quat {
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
