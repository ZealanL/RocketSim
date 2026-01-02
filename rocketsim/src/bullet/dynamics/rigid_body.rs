use glam::{Affine3A, Mat3A, Vec3A};

use crate::bullet::{
    collision::shapes::collision_shape::CollisionShapes,
    linear_math::transform_util::{integrate_trans, integrate_trans_no_rot},
};
use crate::sim::UserInfoTypes;

pub enum RigidBodyFlags {
    DisableWorldGravity = 1,
}

pub struct RigidBodyConstructionInfo {
    pub mass: f32,
    pub start_world_trans: Affine3A,
    pub collision_shape: CollisionShapes,
    pub local_inertia: Vec3A,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub friction: f32,
    pub restitution: f32,
    pub linear_sleeping_threshold: f32,
    pub angular_sleeping_threshold: f32,
    pub can_sleep: bool,
}

impl RigidBodyConstructionInfo {
    #[must_use]
    pub const fn new(mass: f32, collision_shape: CollisionShapes, can_sleep: bool) -> Self {
        Self {
            mass,
            collision_shape,
            local_inertia: Vec3A::ZERO,
            linear_damping: 0.0,
            angular_damping: 0.0,
            friction: 0.5,
            restitution: 0.0,
            linear_sleeping_threshold: 0.0,
            angular_sleeping_threshold: 1.0,
            start_world_trans: Affine3A::IDENTITY,
            can_sleep,
        }
    }
}

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

pub struct RigidBody {
    world_trans: Affine3A,
    shape: CollisionShapes,
    activation: ActivationState,
    rb_flags: u8,

    pub interp_world_trans: Affine3A,
    pub interp_linear_velocity: Vec3A,
    pub interp_angular_velocity: Vec3A,
    pub contact_processing_threshold: f32,
    broadphase_handle: Option<usize>,

    pub collision_flags: u8,
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

    pub inertia_tensor_world: Mat3A,
    pub inv_inertia_tensor_world: Mat3A,
    pub linear_velocity: Vec3A,
    pub angular_velocity: Vec3A,
    pub inverse_mass: f32,
    pub gravity: Vec3A,
    pub gravity_acceleration: Vec3A,
    pub inv_inertia_local: Vec3A,
    pub total_force: Vec3A,
    pub total_torque: Vec3A,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub linear_sleeping_threshold: f32,
    pub angular_sleeping_threshold: f32,
    pub inv_mass: Vec3A,
}

impl RigidBody {
    #[must_use]
    pub fn new(info: RigidBodyConstructionInfo) -> Self {
        let inverse_mass = if info.mass == 0.0 {
            0.0
        } else {
            1.0 / info.mass
        };

        let linear_damping = info.linear_damping.clamp(0.0, 1.0);
        let angular_damping = info.angular_damping.clamp(0.0, 1.0);
        let linear_sleeping_threshold = info.linear_sleeping_threshold;
        let angular_sleeping_threshold = info.angular_sleeping_threshold;

        let inv_inertia_local = Vec3A::select(
            info.local_inertia.cmpeq(Vec3A::ZERO),
            Vec3A::ZERO,
            1.0 / info.local_inertia,
        );

        let inv_inertia_tensor_world =
            Self::get_inertia_tensor(info.start_world_trans.matrix3, inv_inertia_local);

        Self {
            world_trans: info.start_world_trans,
            interp_world_trans: info.start_world_trans,
            interp_linear_velocity: Vec3A::ZERO,
            interp_angular_velocity: Vec3A::ZERO,
            contact_processing_threshold: f32::MAX,
            broadphase_handle: None,
            shape: info.collision_shape,
            collision_flags: if info.mass == 0.0 {
                CollisionFlags::StaticObject as u8
            } else {
                0
            },
            companion_id: None,
            world_array_idx: 0,
            activation: ActivationState::Active,
            deactivation_time: 0.0,
            friction: info.friction,
            restitution: info.restitution,
            no_rot: false,
            user_pointer: 0,
            user_idx: UserInfoTypes::default(),
            can_sleep: info.can_sleep,

            inv_inertia_tensor_world,
            inertia_tensor_world: inv_inertia_tensor_world.transpose(),
            linear_velocity: Vec3A::ZERO,
            angular_velocity: Vec3A::ZERO,
            inverse_mass,
            gravity: Vec3A::ZERO,
            gravity_acceleration: Vec3A::ZERO,
            inv_inertia_local,
            total_force: Vec3A::ZERO,
            total_torque: Vec3A::ZERO,
            linear_damping,
            angular_damping,
            linear_sleeping_threshold,
            angular_sleeping_threshold,
            inv_mass: Vec3A::splat(inverse_mass),
            rb_flags: 0,
        }
    }

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
        self.collision_flags & CollisionFlags::StaticObject as u8 != 0
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
        self.collision_flags & CollisionFlags::NoContactResponse as u8 == 0
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

    #[must_use]
    pub const fn get_rb_flags(&self) -> u8 {
        self.rb_flags
    }

    pub fn set_gravity(&mut self, acceleration: Vec3A) {
        if self.inverse_mass != 0.0 {
            self.gravity = acceleration * (1.0 / self.inverse_mass);
        }

        self.gravity_acceleration = acceleration;
    }

    pub fn set_linear_velocity(&mut self, lin_vel: Vec3A) {
        debug_assert!(!lin_vel.is_nan());
        self.linear_velocity = lin_vel;
    }

    pub fn set_angular_velocity(&mut self, ang_vel: Vec3A) {
        debug_assert!(!ang_vel.is_nan());
        self.angular_velocity = ang_vel;
    }

    fn get_inertia_tensor(world_mat: Mat3A, inv_inertia_local: Vec3A) -> Mat3A {
        let mut scaled_mat = world_mat.transpose();
        scaled_mat.x_axis *= inv_inertia_local;
        scaled_mat.y_axis *= inv_inertia_local;
        scaled_mat.z_axis *= inv_inertia_local;

        world_mat * scaled_mat
    }

    pub fn update_inertia_tensor(&mut self) {
        self.inv_inertia_tensor_world =
            Self::get_inertia_tensor(self.get_world_trans().matrix3, self.inv_inertia_local);
        self.inertia_tensor_world = self.inv_inertia_tensor_world.transpose();
    }

    pub fn apply_torque_impulse(&mut self, torque: Vec3A) {
        debug_assert!(!torque.is_nan());
        self.angular_velocity += self.inv_inertia_tensor_world * torque;
    }

    pub fn apply_impulse(&mut self, impulse: Vec3A, rel_pos: Vec3A) {
        debug_assert_ne!(self.inverse_mass, 0.0);
        self.apply_central_impulse(impulse);
        self.apply_torque_impulse(rel_pos.cross(impulse));
    }

    pub fn apply_torque(&mut self, torque: Vec3A) {
        debug_assert!(!torque.is_nan());
        self.total_torque += torque;
    }

    pub fn apply_central_impulse(&mut self, impulse: Vec3A) {
        debug_assert!(!impulse.is_nan());
        self.linear_velocity += impulse * self.inverse_mass;
    }

    pub fn apply_central_force(&mut self, force: Vec3A) {
        debug_assert!(!force.is_nan());
        self.total_force += force;
    }

    pub fn apply_gravity(&mut self) {
        debug_assert!(!self.is_static_object());
        self.apply_central_force(self.gravity);
    }

    pub fn apply_damping(&mut self, time_step: f32) {
        self.linear_velocity *= (1.0 - self.linear_damping).powf(time_step);
        self.angular_velocity *= (1.0 - self.angular_damping).powf(time_step);
    }

    #[must_use]
    pub fn predict_integration_trans(&self, time_step: f32) -> Affine3A {
        let mut trans = *self.get_world_trans();

        if self.no_rot {
            integrate_trans_no_rot(&mut trans, self.linear_velocity, time_step);
        } else {
            integrate_trans(
                &mut trans,
                self.linear_velocity,
                self.angular_velocity,
                time_step,
            );
        }

        trans
    }

    pub fn set_center_of_mass_trans(&mut self, xform: Affine3A) {
        self.interp_world_trans = xform;
        self.interp_linear_velocity = self.linear_velocity;
        self.interp_angular_velocity = self.angular_velocity;
        self.set_world_trans(xform);

        self.update_inertia_tensor();
    }

    #[must_use]
    pub fn get_velocity_in_local_point(&self, rel_pos: Vec3A) -> Vec3A {
        self.linear_velocity + self.angular_velocity.cross(rel_pos)
    }

    pub fn update_activation_state(&mut self, _time_step: f32) {
        if !self.can_sleep {
            self.set_activation_state(ActivationState::Active);
            return;
        }

        let thresh_lin_sq = self.linear_sleeping_threshold.powi(2);
        let thresh_ang_sq = self.angular_sleeping_threshold.powi(2);
        let within_sleep_thresh = (self.linear_velocity.length_squared() < thresh_lin_sq)
            && (self.angular_velocity.length_squared() < thresh_ang_sq);

        if within_sleep_thresh {
            self.set_activation_state(ActivationState::Sleeping);
        } else {
            self.set_activation_state(ActivationState::Active);
        }
    }

    pub const fn clear_forces(&mut self) {
        self.total_force = Vec3A::ZERO;
        self.total_torque = Vec3A::ZERO;
    }

    pub fn get_mass(&self) -> f32 {
        if self.inverse_mass == 0.0 {
            0.0
        } else {
            1.0 / self.inverse_mass
        }
    }

    pub fn compute_impulse_denominator(&self, pos: Vec3A, normal: Vec3A) -> f32 {
        let r0 = pos - self.get_world_trans().translation;
        let c0 = r0.cross(normal);
        let vec = (self.inv_inertia_tensor_world.transpose() * c0).cross(r0);

        self.inverse_mass + normal.dot(vec)
    }

    pub const fn get_up_vector(&self) -> Vec3A {
        self.get_world_trans().matrix3.z_axis
    }

    pub const fn get_forward_vector(&self) -> Vec3A {
        self.get_world_trans().matrix3.x_axis
    }

    pub fn get_forward_speed(&self) -> f32 {
        self.linear_velocity.dot(self.get_forward_vector())
    }

    pub fn get_world_pos(&self) -> Vec3A {
        self.get_world_trans().translation
    }
}
