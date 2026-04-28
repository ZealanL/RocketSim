use glam::USizeVec3;

use crate::shared::Aabb;

pub enum CollisionFilterGroups {
    Default = 1,
    Static = (1 << 1),
    HoopsNet = (1 << 2),
    DropshotTile = (1 << 3),
    DropshotFloor = (1 << 4),
    All = -1,
}

pub struct BroadphaseProxy {
    /// The index of the client `RigidBody` in `CollisionWorld`
    pub client_obj_idx: usize,
    pub collision_filter_group: u8,
    pub collision_filter_mask: u8,
    pub unique_id: u32,
    pub aabb: Aabb,
    pub cell_idx: usize,
    pub indices: USizeVec3,
}

pub struct BroadphasePair {
    pub proxy0: usize,
    pub proxy1: usize,
}

pub trait BroadphaseAabbCallback {
    fn process(&mut self, proxy: &BroadphaseProxy) -> bool;
}
