use std::ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, BitXor, BitXorAssign, Not};

use glam::{Affine3A, Mat3A, Quat, Vec3A};
#[cfg(debug_assertions)]
use indexmap::IndexMap;

use crate::{
    bullet::{
        collision::{
            narrowphase::persistent_manifold::CONTACT_BREAKING_THRESHOLD,
            shapes::collision_shape::CollisionShapes,
        },
        linear_math::{integrate_trans, integrate_trans_no_rot},
    },
    sim::UserInfoTypes,
};

pub struct RigidBodyConstructionInfo {
    pub mass: f32,
    pub start_world_trans: Affine3A,
    pub collision_shape: CollisionShapes,
    pub local_inertia: Vec3A,
    pub linear_damping: f32,
    pub friction: f32,
    pub restitution: f32,
    pub linear_sleeping_threshold: f32,
    pub angular_sleeping_threshold: f32,
}

impl RigidBodyConstructionInfo {
    pub const fn new(mass: f32, collision_shape: CollisionShapes) -> Self {
        Self {
            mass,
            collision_shape,
            local_inertia: Vec3A::ZERO,
            linear_damping: 0.0,
            friction: 0.5,
            restitution: 0.0,
            linear_sleeping_threshold: 0.0,
            angular_sleeping_threshold: 1.0,
            start_world_trans: Affine3A::IDENTITY,
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
    CustomMaterialCallback = (1 << 1),
    CanSleep = (1 << 2),
    NoContactResponse = (1 << 3),
    NoWorldGravity = (1 << 4),
    NoAngularMotion = (1 << 5),
}

impl BitOrAssign<CollisionFlags> for u8 {
    fn bitor_assign(&mut self, rhs: CollisionFlags) {
        *self |= rhs as u8;
    }
}

impl BitAndAssign<CollisionFlags> for u8 {
    fn bitand_assign(&mut self, rhs: CollisionFlags) {
        *self &= rhs as u8;
    }
}

impl BitAnd<CollisionFlags> for u8 {
    type Output = u8;

    fn bitand(self, rhs: CollisionFlags) -> Self::Output {
        self & (rhs as u8)
    }
}

impl BitOr<CollisionFlags> for u8 {
    type Output = u8;

    fn bitor(self, rhs: CollisionFlags) -> Self::Output {
        self | (rhs as u8)
    }
}

impl BitXor<CollisionFlags> for u8 {
    type Output = u8;

    fn bitxor(self, rhs: CollisionFlags) -> Self::Output {
        self ^ (rhs as u8)
    }
}

impl BitOr for CollisionFlags {
    type Output = u8;

    fn bitor(self, rhs: Self) -> Self::Output {
        self as u8 | rhs as u8
    }
}

impl BitXorAssign<CollisionFlags> for u8 {
    fn bitxor_assign(&mut self, rhs: CollisionFlags) {
        *self ^= rhs as u8;
    }
}

impl Not for CollisionFlags {
    type Output = u8;

    fn not(self) -> Self::Output {
        !(self as u8)
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Impulse {
    /// (lin_impulse)
    Linear(Vec3A),
    /// (lin_impulse, rel_pos_offset)
    LinearRelPos(Vec3A, Vec3A),
    /// (ang_impulse)
    Angular(Vec3A),
}

pub struct RigidBody {
    world_trans: Affine3A,
    world_quat: Quat,
    shape: CollisionShapes,
    activation: ActivationState,
    broadphase_handle: usize,

    pub interp_world_trans: Affine3A,
    pub contact_processing_threshold: f32,
    /// Cached shape breaking threshold (`angular_disc * 0.02`).
    /// Shapes never change after construction, so cache the disc math here
    /// instead of recomputing two square roots per manifold creation.
    /// Read it via [`get_contact_breaking_threshold`](Self::get_contact_breaking_threshold).
    contact_breaking_threshold: f32,

    pub collision_flags: u8,
    pub companion_id: Option<usize>,
    /// The index of this object in `CollisionWorld`
    pub world_array_idx: usize,
    pub deactivation_time: f32,
    pub friction: f32,
    pub restitution: f32,
    pub user_pointer: usize,
    pub user_idx: UserInfoTypes,

    pub inv_inertia_tensor_world: Mat3A,
    pub lin_vel: Vec3A,
    pub ang_vel: Vec3A,
    pub mass: f32,
    pub inv_mass: f32,
    pub inv_inertia_local: Vec3A,

    pub accum_lin_vel: Vec3A,
    pub accum_ang_vel: Vec3A,

    pub linear_damping: f32,
    pub linear_sleeping_threshold: f32,
    pub angular_sleeping_threshold: f32,
    pub inv_mass_splat: Vec3A,

    /// For debugging physics, this tracks every impulse applied during a tick
    #[cfg(debug_assertions)]
    pub dbg_tick_impulse_history: IndexMap<(&'static str, bool), (Vec3A, Vec3A)>,
}

impl RigidBody {
    pub fn new(info: RigidBodyConstructionInfo) -> Self {
        let (inverse_mass, collision_flags) = if info.mass == 0.0 {
            (0.0, CollisionFlags::StaticObject as u8)
        } else {
            (1.0 / info.mass, 0)
        };

        let linear_damping = info.linear_damping.clamp(0.0, 1.0);
        let linear_sleeping_threshold = info.linear_sleeping_threshold;
        let angular_sleeping_threshold = info.angular_sleeping_threshold;

        let inv_inertia_local = Vec3A::select(
            info.local_inertia.cmpeq(Vec3A::ZERO),
            Vec3A::ZERO,
            1.0 / info.local_inertia,
        );

        let inv_inertia_tensor_world =
            Self::get_inertia_tensor(info.start_world_trans.matrix3, inv_inertia_local);

        // Shapes are immutable after construction, so the angular-disc
        // threshold never changes for this body. Cache it once.
        let contact_breaking_threshold = info
            .collision_shape
            .get_contact_breaking_threshold(CONTACT_BREAKING_THRESHOLD);

        Self {
            world_trans: info.start_world_trans,
            world_quat: Quat::from_mat3a(&info.start_world_trans.matrix3),
            interp_world_trans: info.start_world_trans,
            contact_processing_threshold: f32::MAX,
            contact_breaking_threshold,
            broadphase_handle: 0,
            shape: info.collision_shape,
            collision_flags,
            companion_id: None,
            world_array_idx: 0,
            activation: ActivationState::Active,
            deactivation_time: 0.0,
            friction: info.friction,
            restitution: info.restitution,
            user_pointer: 0,
            user_idx: UserInfoTypes::default(),
            inv_inertia_tensor_world,
            lin_vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
            mass: info.mass,
            inv_mass: inverse_mass,
            inv_inertia_local,
            accum_lin_vel: Vec3A::ZERO,
            accum_ang_vel: Vec3A::ZERO,
            linear_damping,
            linear_sleeping_threshold,
            angular_sleeping_threshold,
            inv_mass_splat: Vec3A::splat(inverse_mass),

            #[cfg(debug_assertions)]
            dbg_tick_impulse_history: IndexMap::new(),
        }
    }

    /// Cached breaking threshold for this body's shape (see field docs).
    /// Shapes are immutable, so this never changes after construction.
    #[inline]
    pub const fn get_contact_breaking_threshold(&self) -> f32 {
        self.contact_breaking_threshold
    }

    pub fn set_world_trans(&mut self, world_trans: Affine3A) {
        if (self.collision_flags & CollisionFlags::NoAngularMotion) == 0 {
            self.world_quat = Quat::from_mat3a(&world_trans.matrix3);
        }

        self.world_trans = world_trans;
    }

    pub const fn get_world_trans(&self) -> &Affine3A {
        &self.world_trans
    }

    pub const fn get_world_rot(&self) -> Quat {
        self.world_quat
    }

    pub const fn get_collision_shape(&self) -> &CollisionShapes {
        &self.shape
    }

    pub fn is_static_obj(&self) -> bool {
        (self.collision_flags & CollisionFlags::StaticObject) != 0
    }

    pub const fn is_active(&self) -> bool {
        !matches!(
            self.activation,
            ActivationState::Sleeping | ActivationState::DisableSimulation
        )
    }

    pub fn has_contact_response(&self) -> bool {
        self.collision_flags & CollisionFlags::NoContactResponse == 0
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
        self.broadphase_handle = handle;
    }

    pub const fn get_broadphase_handle(&self) -> usize {
        self.broadphase_handle
    }

    pub fn set_lin_vel(&mut self, lin_vel: Vec3A) {
        debug_assert!(!lin_vel.is_nan());
        self.lin_vel = lin_vel;
    }

    pub fn set_ang_vel(&mut self, ang_vel: Vec3A) {
        debug_assert!(!ang_vel.is_nan());
        self.ang_vel = ang_vel;
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
    }

    /// Add an impulse of a given type
    ///
    /// `massed`: Scale down by `self.inv_mass`
    ///
    /// `accum`: Accumulate this impulse to be applied while
    /// stepping the simulation (instead of immediately)
    #[inline(always)] // Should assure const evaluation
    pub fn add_impulse(
        &mut self,
        _name: Option<&'static str>,
        impulse: Impulse,
        massed: bool,
        accum: bool,
    ) {
        let mut lin_impulse = Vec3A::ZERO;
        let mut ang_impulse = Vec3A::ZERO;

        let massed_scaler = if massed { self.inv_mass } else { 1.0 };
        match impulse {
            Impulse::Linear(v) => {
                lin_impulse = v * massed_scaler;
            }
            Impulse::LinearRelPos(v, rel_pos) => {
                lin_impulse = v * massed_scaler;
                ang_impulse = self.inv_inertia_tensor_world * rel_pos.cross(v);
                if !massed {
                    ang_impulse *= self.mass; // Have to undo the effects of inv inertia tensor
                }
            }
            Impulse::Angular(av) => {
                ang_impulse = av * massed_scaler;
            }
        };

        if accum {
            self.accum_lin_vel += lin_impulse;
            self.accum_ang_vel += ang_impulse;
        } else {
            self.lin_vel += lin_impulse;
            self.ang_vel += ang_impulse;
        }

        #[cfg(debug_assertions)]
        if let Some(name) = _name {
            let map = &mut self.dbg_tick_impulse_history;
            if let Some((lin, ang)) = map.get_mut(&(name, accum)) {
                *lin += lin_impulse;
                *ang += ang_impulse;
            } else {
                map.insert((name, accum), (lin_impulse, ang_impulse));
            }
        }
    }

    pub fn apply_damping(&mut self, time_step: f32) {
        if self.linear_damping != 0.0 {
            self.lin_vel *= (1.0 - self.linear_damping).powf(time_step);
        }
    }

    #[inline]
    pub fn predict_integration_trans(&self, time_step: f32) -> Affine3A {
        let mut trans = self.world_trans;
        let mut quat = self.world_quat;

        if (self.collision_flags & CollisionFlags::NoAngularMotion) != 0 {
            integrate_trans_no_rot(&mut trans.translation, self.lin_vel, time_step);
        } else {
            integrate_trans(&mut trans, &mut quat, self.lin_vel, self.ang_vel, time_step);
        }

        trans
    }

    pub fn set_center_of_mass_trans(&mut self, xform: Affine3A) {
        self.interp_world_trans = xform;
        self.set_world_trans(xform);

        self.update_inertia_tensor();
    }

    pub fn get_vel_in_local_point(&self, rel_pos: Vec3A) -> Vec3A {
        self.lin_vel + self.ang_vel.cross(rel_pos)
    }

    pub fn update_activation_state(&mut self, _time_step: f32) {
        if self.collision_flags & CollisionFlags::CanSleep == 0 {
            self.set_activation_state(ActivationState::Active);
            return;
        }

        let thresh_lin_sq = self.linear_sleeping_threshold.powi(2);
        let thresh_ang_sq = self.angular_sleeping_threshold.powi(2);
        let within_sleep_thresh = (self.lin_vel.length_squared() < thresh_lin_sq)
            && (self.ang_vel.length_squared() < thresh_ang_sq);

        if within_sleep_thresh {
            self.set_activation_state(ActivationState::Sleeping);
        } else {
            self.set_activation_state(ActivationState::Active);
        }
    }

    pub fn clear_accum_vels(&mut self) {
        self.accum_lin_vel = Vec3A::ZERO;
        self.accum_ang_vel = Vec3A::ZERO;
        #[cfg(debug_assertions)]
        self.dbg_tick_impulse_history.clear();
    }

    pub fn get_mass(&self) -> f32 {
        if self.inv_mass == 0.0 {
            0.0
        } else {
            1.0 / self.inv_mass
        }
    }

    pub fn compute_impulse_denominator(&self, pos: Vec3A, normal: Vec3A) -> f32 {
        let r0 = pos - self.get_world_trans().translation;
        let c0 = r0.cross(normal);
        let vec = self
            .inv_inertia_tensor_world
            .mul_transpose_vec3a(c0)
            .cross(r0);

        self.inv_mass + normal.dot(vec)
    }

    pub const fn get_up_vector(&self) -> Vec3A {
        self.get_world_trans().matrix3.z_axis
    }

    pub const fn get_forward_vector(&self) -> Vec3A {
        self.get_world_trans().matrix3.x_axis
    }

    pub fn get_forward_speed(&self) -> f32 {
        self.lin_vel.dot(self.get_forward_vector())
    }

    pub const fn get_world_pos(&self) -> Vec3A {
        self.get_world_trans().translation
    }

    pub fn set_world_pos(&mut self, pos: Vec3A) {
        self.world_trans.translation = pos;
    }

    pub fn limit_vels(&mut self, max_lin_speed: f32, max_ang_speed: f32) {
        if self.lin_vel.length_squared() > max_lin_speed.powi(2) {
            self.lin_vel = self.lin_vel.normalize_or_zero() * max_lin_speed;
        }

        if self.ang_vel.length_squared() > max_ang_speed.powi(2) {
            self.ang_vel = self.ang_vel.normalize_or_zero() * max_ang_speed;
        }
    }
}
