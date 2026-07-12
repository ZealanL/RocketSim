use glam::{Quat, Vec3A};

pub trait QuatExt {
    fn from_axis_angle_simd(axis: Vec3A, angle: f32) -> Self;
}

impl QuatExt for Quat {
    /// An implementation of `Quat::from_axis_angle` that leverages simd
    #[inline]
    fn from_axis_angle_simd(axis: Vec3A, angle: f32) -> Self {
        debug_assert!(axis.is_normalized());
        let (s, c) = f32::sin_cos(angle * 0.5);
        let v = axis * s;
        Self::from_xyzw(v.x, v.y, v.z, c)
    }
}
