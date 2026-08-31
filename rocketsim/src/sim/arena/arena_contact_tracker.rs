use std::mem;

use glam::Vec3A;

use crate::{
    bullet::{
        collision::{
            dispatch::internal_edge_utility::adjust_internal_edge_contacts,
            narrowphase::{
                manifold_point::ManifoldPoint, persistent_manifold::ContactAddedCallback,
            },
        },
        dynamics::rigid_body::RigidBody,
    },
    consts,
    sim::UserInfoTypes,
};

// An instance of a contact event
#[derive(Debug, Copy, Clone)]
pub(crate) struct ContactRecord {
    pub is_swap: bool,
    pub rb_idx_a: usize,
    pub rb_idx_b: usize,
    pub manifold_point: ManifoldPoint,
    /// RocketSim V2 invokes semantic contact handling before Bullet integrates
    /// transforms. Rust drains contact records after the step, so retain the
    /// pre-solver body values for equivalent hit calculations.
    pub pre_pos_a: Vec3A,
    pub pre_vel_a: Vec3A,
    pub pre_pos_b: Vec3A,
    pub pre_vel_b: Vec3A,
}

// A struct to be accessed through the bullet contact callbacks
pub(crate) struct ArenaContactTracker {
    collision_records: Vec<ContactRecord>,
}

impl ArenaContactTracker {
    pub fn new() -> Self {
        Self {
            collision_records: Vec::with_capacity(4), // Rarely exceeded
        }
    }

    pub const fn num_records(&self) -> usize {
        self.collision_records.len()
    }

    pub fn get_record(&self, idx: usize) -> &ContactRecord {
        &self.collision_records[idx]
    }

    pub fn clear_records(&mut self) {
        self.collision_records.clear();
    }
}

impl ContactAddedCallback for ArenaContactTracker {
    fn callback<'a>(
        &mut self,
        manifold_point: &mut ManifoldPoint,
        mut body_a: &'a RigidBody,
        mut body_b: &'a RigidBody,
        idx: Option<usize>,
    ) {
        debug_assert!(body_a.has_contact_response() || body_b.has_contact_response());

        let should_swap =
            if body_a.user_idx != UserInfoTypes::None && body_b.user_idx != UserInfoTypes::None {
                body_a.user_idx > body_b.user_idx
            } else {
                body_b.user_idx != UserInfoTypes::None
            };

        if should_swap {
            mem::swap(&mut body_a, &mut body_b);
        }

        let user_idx_a = body_a.user_idx;
        let user_idx_b = body_b.user_idx;

        if user_idx_a == UserInfoTypes::Car {
            let hit_coefs = match user_idx_b {
                UserInfoTypes::Ball => consts::car::HIT_BALL_COEFS,
                UserInfoTypes::Car => consts::car::HIT_CAR_COEFS,
                _ => consts::car::HIT_WORLD_COEFS,
            };
            manifold_point.combined_friction = hit_coefs.friction;
            manifold_point.combined_restitution = hit_coefs.restitution;
        } else if user_idx_a == UserInfoTypes::Ball && user_idx_b == UserInfoTypes::None {
            manifold_point.is_special = true;
        }

        // NOTE: Push *before* the manifold is mutated by adjust_internal_edge_contacts()
        self.collision_records.push(ContactRecord {
            is_swap: should_swap,
            rb_idx_a: body_a.world_array_idx,
            rb_idx_b: body_b.world_array_idx,
            manifold_point: *manifold_point,
            pre_pos_a: body_a.get_world_trans().translation,
            pre_vel_a: body_a.lin_vel,
            pre_pos_b: body_b.get_world_trans().translation,
            pre_vel_b: body_b.lin_vel,
        });

        if std::env::var("RSIM_DEBUG_CONTACT").is_ok() {
            println!(
                "rust_contact,a={},b={},swap={},idx={:?},special={},point_a={:?},point_b={:?},normal={:?},distance={},friction={},restitution={},pre_a={:?},pre_b={:?}",
                body_a.world_array_idx,
                body_b.world_array_idx,
                should_swap,
                idx,
                manifold_point.is_special,
                manifold_point.pos_world_on_a,
                manifold_point.pos_world_on_b,
                manifold_point.normal_world_on_b,
                manifold_point.distance_1,
                manifold_point.combined_friction,
                manifold_point.combined_restitution,
                body_a.get_world_trans().translation,
                body_b.get_world_trans().translation,
            );
        }

        if let Some(idx) = idx {
            adjust_internal_edge_contacts(manifold_point, body_b, idx);
        }
    }
}
