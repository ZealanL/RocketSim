use std::ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, BitXor};

use crate::shared::Aabb;

pub enum CollisionFilterGroups {
    Default = 1,
    Static = (1 << 1),
    HoopsNet = (1 << 2),
    DropshotTile = (1 << 3),
    DropshotFloor = (1 << 4),
}

impl CollisionFilterGroups {
    pub const ALL: u8 = Self::Default as u8
        | Self::Static as u8
        | Self::HoopsNet as u8
        | Self::DropshotTile as u8
        | Self::DropshotFloor as u8;
}

impl BitOrAssign<CollisionFilterGroups> for u8 {
    fn bitor_assign(&mut self, rhs: CollisionFilterGroups) {
        *self |= rhs as u8;
    }
}

impl BitAndAssign<CollisionFilterGroups> for u8 {
    fn bitand_assign(&mut self, rhs: CollisionFilterGroups) {
        *self &= rhs as u8;
    }
}

impl BitAnd<CollisionFilterGroups> for u8 {
    type Output = u8;

    fn bitand(self, rhs: CollisionFilterGroups) -> Self::Output {
        self & (rhs as u8)
    }
}

impl BitOr<CollisionFilterGroups> for u8 {
    type Output = u8;

    fn bitor(self, rhs: CollisionFilterGroups) -> Self::Output {
        self | (rhs as u8)
    }
}

impl BitXor<CollisionFilterGroups> for u8 {
    type Output = u8;

    fn bitxor(self, rhs: CollisionFilterGroups) -> Self::Output {
        self ^ (rhs as u8)
    }
}

impl BitXor for CollisionFilterGroups {
    type Output = u8;

    fn bitxor(self, rhs: Self) -> Self::Output {
        self as u8 ^ rhs as u8
    }
}

impl BitOr for CollisionFilterGroups {
    type Output = u8;

    fn bitor(self, rhs: Self) -> Self::Output {
        self as u8 | rhs as u8
    }
}

pub struct BroadphaseProxy {
    /// The index of the client `RigidBody` in `CollisionWorld`
    pub client_obj_idx: u32,
    pub collision_filter_group: u8,
    pub collision_filter_mask: u8,
    pub unique_id: u32,
    pub aabb: Aabb,
    pub cell_idx: u32,
    pub indices: [u32; 3],
}

pub struct BroadphasePair {
    pub proxy0: usize,
    pub proxy1: usize,
}

pub trait BroadphaseAabbCallback {
    fn process(&mut self, proxy: &BroadphaseProxy) -> bool;
}

#[cfg(test)]
mod layout_tests {
    use std::mem::size_of;

    use super::BroadphaseProxy;
    use crate::bullet::collision::{
        narrowphase::{manifold_point::ManifoldPoint, persistent_manifold::PersistentManifold},
        shapes::triangle_shape::TriangleShape,
    };

    #[test]
    fn compact_layouts() {
        assert_eq!(size_of::<BroadphaseProxy>(), 64);
        assert_eq!(size_of::<TriangleShape>(), 80);
        assert_eq!(size_of::<ManifoldPoint>(), 128);
        assert_eq!(size_of::<PersistentManifold>(), 560);
    }
}
