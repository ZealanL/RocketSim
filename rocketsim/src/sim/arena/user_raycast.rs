use glam::Vec3A;

use crate::UserInfoType;

/// One segment query for [`crate::Arena::cast_rays`], in Unreal units.
///
/// `from` is the ray start, `to` is the ray end (not a direction).
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RaycastQuery {
    pub from: Vec3A,
    pub to: Vec3A,
    /// Whether the ray should hit non-static objects (i.e. cars and ball)
    pub include_dynamics: bool,
}

/// Closest hit along a [`RaycastQuery`] segment, in Unreal units.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RaycastHitInfo {
    /// World position of the hit (uu).
    pub hit_point: Vec3A,
    /// Surface normal at the hit.
    pub hit_normal: Vec3A,
    /// Fraction of ray travel (0 to 1) before hitting
    pub hit_fraction: f32,
    /// What kind of object was hit.
    pub user_info: UserInfoType,
}
