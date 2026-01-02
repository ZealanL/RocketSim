use glam::Affine3A;

use crate::bullet::dynamics::rigid_body::RigidBody;

pub struct RigidBodyWrapper<'a> {
    pub object: &'a RigidBody,
    pub world_trans: Affine3A,
}
