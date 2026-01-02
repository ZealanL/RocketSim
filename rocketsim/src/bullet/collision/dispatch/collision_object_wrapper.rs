use glam::Affine3A;

use crate::bullet::collision::dispatch::collision_object::CollisionObject;

pub struct CollisionObjectWrapper<'a> {
    pub object: &'a CollisionObject,
    pub world_trans: Affine3A,
}
