use glam::{Quat, Vec3A};

pub trait QuatExt {
    fn from_axis_angle_simd(axis: Vec3A, angle: f32) -> Self;
    /// Construct an axis-angle quaternion using Bullet's btQuaternion(axis,
    /// angle) sequence. Bullet divides by the axis length before multiplying
    /// the sine term; glam's constructor assumes a unit axis and therefore
    /// can differ by an ulp when a live chassis basis has drifted slightly.
    fn from_axis_angle_bullet(axis: Vec3A, angle: f32) -> Self;
}

impl QuatExt for Quat {
    /// An implementation of `Quat::from_axis_angle` that leverages SIMD.
    #[inline]
    fn from_axis_angle_simd(axis: Vec3A, angle: f32) -> Self {
        debug_assert!(axis.is_normalized());
        let (s, c) = f32::sin_cos(angle * 0.5);
        let v = axis * s;
        Self::from_xyzw(v.x, v.y, v.z, c)
    }

    #[inline]
    fn from_axis_angle_bullet(axis: Vec3A, angle: f32) -> Self {
        // btQuaternion::setRotation uses btSin/btCos independently and
        // divides by d = axis.length(). Keep the same operation order.
        let half = angle * 0.5;
        let d = axis.length();
        let s = half.sin() / d;
        Self::from_xyzw(axis.x * s, axis.y * s, axis.z * s, half.cos())
    }
}
