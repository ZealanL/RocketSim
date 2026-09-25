use glam::Vec3A;

/// One segment query for [`crate::Arena::cast_rays`].
///
/// NOTE: as currently implemented the query coordinates are passed straight
/// to the Bullet world, which works in Bullet units (meters, `uu * UU_TO_BT`),
/// and hits come back in the same space — multiply/divide by 50 (`BT_TO_UU` /
/// `UU_TO_BT`) to convert. This is almost certainly a bug (everything else in
/// the public API is Unreal units); see `Arena::cast_rays`.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RaycastQuery {
    pub from: Vec3A,
    pub to: Vec3A,

    /// Whether the ray should hit non-static objects (i.e. cars and ball)
    pub hit_dynamic: bool,
}

/// Closest hit along a [`RaycastQuery`] segment.
///
/// See [`RaycastQuery`] for the units caveat (currently Bullet units).
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RaycastHitInfo {
    /// World position of the hit (uu).
    pub hit_point: Vec3A,
    /// Surface normal at the hit.
    pub hit_normal: Vec3A,

    /// Fraction of ray travel (0 to 1) before hitting
    pub hit_fraction: f32,
}

/// Result for one [`RaycastQuery`]: `hit_info` is `None` on a miss.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RaycastResult {
    pub hit_info: Option<RaycastHitInfo>,
}
impl RaycastResult {
    /// Returns `true` when the ray hit something.
    pub fn hit(&self) -> bool {
        self.hit_info.is_some()
    }
}
