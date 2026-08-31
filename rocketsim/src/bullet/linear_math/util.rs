use std::f32::consts::FRAC_1_SQRT_2;

use glam::{Vec3A, Vec4};

/// Match Bullet's btVector3::normalize() on the x86 SIMD build: an SSE
/// reciprocal-square-root followed by one Newton-Raphson refinement.
#[inline]
pub fn bullet_dot(a: Vec3A, b: Vec3A) -> f32 {
    #[cfg(target_arch = "x86_64")]
    unsafe {
        use std::arch::x86_64::{
            _mm_add_ss, _mm_cvtss_f32, _mm_movehl_ps, _mm_mul_ps, _mm_setr_ps, _mm_shuffle_ps,
        };

        // btVector3::dot() uses the SSE lanes in this order (x + y) + z.
        let av = _mm_setr_ps(a.x, a.y, a.z, 0.0);
        let bv = _mm_setr_ps(b.x, b.y, b.z, 0.0);
        let products = _mm_mul_ps(av, bv);
        let z = _mm_movehl_ps(products, products);
        let y = _mm_shuffle_ps(products, products, 0x55);
        let sum = _mm_add_ss(_mm_add_ss(products, y), z);
        _mm_cvtss_f32(sum)
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        let products = a * b;
        products.x + (products.y + products.z)
    }
}

#[inline]
pub fn bullet_length_squared(v: Vec3A) -> f32 {
    bullet_dot(v, v)
}

/// Match Bullet's `btVector3::safeNormalize()` used by the V2 gameplay
/// car-ball impulse.  This is deliberately different from Bullet's
/// `normalize()` SSE fast path: `safeNormalize()` computes a scalar sqrt and
/// then scales the vector by its reciprocal.
#[inline]
pub fn bullet_safe_normalize(v: Vec3A) -> Vec3A {
    let len_sq = bullet_length_squared(v);
    if len_sq >= f32::EPSILON * f32::EPSILON {
        v * (1.0 / len_sq.sqrt())
    } else {
        Vec3A::X
    }
}

/// Match Bullet's btVector3::normalize() on the x86 SIMD build: an SSE
/// reciprocal-square-root followed by one Newton-Raphson refinement.
#[inline]
pub fn bullet_normalize(v: Vec3A) -> Vec3A {
    #[cfg(target_arch = "x86_64")]
    unsafe {
        use std::arch::x86_64::{
            _mm_add_ss, _mm_cvtss_f32, _mm_movehl_ps, _mm_mul_ps, _mm_mul_ss, _mm_rsqrt_ss,
            _mm_set_ss, _mm_setr_ps, _mm_shuffle_ps, _mm_sub_ss,
        };

        // Bullet's SSE path forms the dot product in a very specific order:
        // multiply all lanes, add Y into X, then add Z.  Keep that sequence
        // instead of relying on scalar/FMA reassociation, because the normal
        // is fed directly into GJK and contact impulses.
        let vv = _mm_setr_ps(v.x, v.y, v.z, 0.0);
        let vd = _mm_mul_ps(vv, vv);
        let z = _mm_movehl_ps(vd, vd);
        let y = _mm_shuffle_ps(vd, vd, 0x55);
        let vd = _mm_add_ss(vd, y);
        let vd = _mm_add_ss(vd, z);
        let len_sq = _mm_cvtss_f32(vd);
        if len_sq == 0.0 {
            return Vec3A::ZERO;
        }
        let len_sq = _mm_set_ss(len_sq);
        let mut inv_len = _mm_rsqrt_ss(len_sq);
        let half_len_sq = _mm_mul_ss(_mm_mul_ss(len_sq, _mm_set_ss(0.5)), inv_len);
        let correction = _mm_mul_ss(half_len_sq, inv_len);
        inv_len = _mm_mul_ss(inv_len, _mm_sub_ss(_mm_set_ss(1.5), correction));

        v * _mm_cvtss_f32(inv_len)
    }

    #[cfg(not(target_arch = "x86_64"))]
    {
        v.normalize_or_zero()
    }
}

#[inline]
pub fn interpolate_3(v0: Vec3A, v1: Vec3A, rt: f32) -> Vec3A {
    let s = 1.0 - rt;
    s * v0 + rt * v1
}

pub fn plane_space_1(n: Vec3A) -> Vec3A {
    if n.z.abs() > FRAC_1_SQRT_2 {
        let a = n.y * n.y + n.z * n.z;
        let k = a.sqrt().recip();
        Vec3A::new(0., -n.z * k, n.y * k)
    } else {
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
