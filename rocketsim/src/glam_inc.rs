//! Re-exported [`glam`] vector/matrix types used throughout the API.
//!
//! Positions/velocities are [`Vec3A`]; rotations are [`Mat3A`]. Analog
//! control triples (`pitch/yaw/roll`) use [`Vec3`].
pub use glam::{
    BVec2, BVec3, BVec4, IVec2, IVec3, IVec4, Mat2, Mat3, Mat3A, Mat4, Vec2, Vec2Swizzles, Vec3,
    Vec3A, Vec3Swizzles, Vec4,
};
