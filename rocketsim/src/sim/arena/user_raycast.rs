use glam::Vec3A;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RaycastQuery {
    pub from: Vec3A,
    pub to: Vec3A,

    /// Whether the ray should hit non-static objects (i.e. cars and ball)
    pub hit_dynamic: bool,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RaycastHitInfo {
    pub hit_point: Vec3A,
    pub hit_normal: Vec3A,

    /// Fraction of ray travel (0 to 1) before hitting
    pub hit_fraction: f32,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RaycastResult {
    pub hit_info: Option<RaycastHitInfo>,
}
impl RaycastResult {
    pub fn hit(&self) -> bool {
        self.hit_info.is_some()
    }
}
