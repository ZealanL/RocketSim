use crate::shared::Aabb;

pub enum CollisionFilterGroups {
    Default = 1,
    Static = (1 << 1),
    All = -1,
}

pub struct BroadphaseProxy {
    /// The index of the client `RigidBody` in `CollisionWorld`
    pub client_object_idx: usize,
    pub collision_filter_group: u8,
    pub collision_filter_mask: u8,
    pub unique_id: u32,
    pub is_static: bool,
    pub aabb: Aabb,
}

pub struct BroadphasePair {
    pub proxy0: usize,
    pub proxy1: usize,
}

pub trait BroadphaseAabbCallback {
    fn process(&mut self, proxy: &BroadphaseProxy) -> bool;
}
