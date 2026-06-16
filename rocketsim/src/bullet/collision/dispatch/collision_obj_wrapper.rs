use glam::Affine3A;

use crate::{
    bullet::{
        collision::shapes::{box_shape::BoxShape, collision_shape::CollisionShapes},
        dynamics::rigid_body::RigidBody,
    },
    shared::Aabb,
};

pub struct RigidBodyWrapper<'a> {
    pub obj: &'a RigidBody,
    pub world_trans: Affine3A,
    pub child_shape_override: Option<&'a BoxShape>,
}

impl RigidBodyWrapper<'_> {
    pub fn get_collision_shape(&self) -> &CollisionShapes {
        self.obj.get_collision_shape()
    }

    pub fn get_aabb(&self) -> Aabb {
        if let Some(child_shape) = self.child_shape_override {
            child_shape.get_aabb(&self.world_trans)
        } else {
            self.get_collision_shape().get_aabb(&self.world_trans)
        }
    }

    pub fn local_get_supporting_vertex(&self, vec: glam::Vec3A) -> glam::Vec3A {
        if let Some(child_shape) = self.child_shape_override {
            child_shape.local_get_supporting_vertex(vec)
        } else {
            self.get_collision_shape().local_get_supporting_vertex(vec)
        }
    }
}
