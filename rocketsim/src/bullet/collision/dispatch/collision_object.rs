use glam::{Affine3A, Vec3A};

use crate::{
    bullet::{
        collision::shapes::collision_shape::CollisionShapes,
        dynamics::rigid_body::RigidBodyConstructionInfo,
    },
    sim::UserInfoTypes,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActivationState {
    Active,
    Sleeping,
    DisableSimulation,
}

pub enum CollisionFlags {
    StaticObject = 1,
    NoContactResponse = (1 << 1),
    CustomMaterialCallback = (1 << 2),
}

pub struct CollisionObject {
    world_trans: Affine3A,
    shape: CollisionShapes,
    activation: ActivationState,

    pub interp_world_trans: Affine3A,
    pub interp_linear_velocity: Vec3A,
    pub interp_angular_velocity: Vec3A,
    pub contact_processing_threshold: f32,
    broadphase_handle: Option<usize>,

    pub flags: u8,
    pub companion_id: Option<usize>,
    /// The index of this object in `CollisionWorld`
    pub world_array_idx: usize,
    pub can_sleep: bool,
    pub deactivation_time: f32,
    pub friction: f32,
    pub restitution: f32,
    pub no_rot: bool,
    pub user_pointer: usize,
    pub user_idx: UserInfoTypes,
}

impl From<RigidBodyConstructionInfo> for CollisionObject {
    fn from(value: RigidBodyConstructionInfo) -> Self {
        Self {
            world_trans: value.start_world_trans,
            interp_world_trans: value.start_world_trans,
            interp_linear_velocity: Vec3A::ZERO,
            interp_angular_velocity: Vec3A::ZERO,
            contact_processing_threshold: f32::MAX,
            broadphase_handle: None,
            shape: value.collision_shape,
            flags: if value.mass == 0.0 {
                CollisionFlags::StaticObject as u8
            } else {
                0
            },
            companion_id: None,
            world_array_idx: 0,
            activation: ActivationState::Active,
            deactivation_time: 0.0,
            friction: value.friction,
            restitution: value.restitution,
            no_rot: false,
            user_pointer: 0,
            user_idx: UserInfoTypes::default(),
            can_sleep: value.can_sleep,
        }
    }
}

impl CollisionObject {
    pub const fn set_world_trans(&mut self, world_trans: Affine3A) {
        self.world_trans = world_trans;
    }

    #[must_use]
    pub const fn get_world_trans(&self) -> &Affine3A {
        &self.world_trans
    }

    #[must_use]
    pub const fn get_collision_shape(&self) -> &CollisionShapes {
        &self.shape
    }

    #[must_use]
    pub const fn is_static_object(&self) -> bool {
        self.flags & CollisionFlags::StaticObject as u8 != 0
    }

    #[must_use]
    pub const fn is_active(&self) -> bool {
        !matches!(
            self.activation,
            ActivationState::Sleeping | ActivationState::DisableSimulation
        )
    }

    #[must_use]
    pub const fn has_contact_response(&self) -> bool {
        self.flags & CollisionFlags::NoContactResponse as u8 == 0
    }

    #[inline]
    #[must_use]
    #[allow(unused)]
    pub const fn get_activation_state(&self) -> ActivationState {
        self.activation
    }

    pub fn set_activation_state(&mut self, new_state: ActivationState) {
        if self.activation != ActivationState::DisableSimulation {
            self.activation = new_state;
        }
    }

    pub const fn force_activate(&mut self) {
        self.activation = ActivationState::Active;
        self.deactivation_time = 0.0;
    }

    pub const fn set_broadphase_handle(&mut self, handle: usize) {
        self.broadphase_handle = Some(handle);
    }

    #[must_use]
    pub const fn get_broadphase_handle(&self) -> Option<usize> {
        self.broadphase_handle
    }
}
