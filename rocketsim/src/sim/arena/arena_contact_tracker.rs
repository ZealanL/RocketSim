use std::mem;

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

// Store one contact event.
#[derive(Debug, Copy, Clone)]
pub(crate) struct ContactRecord {
    pub is_swap: bool,
    pub rb_idx_a: usize,
    pub rb_idx_b: usize,
    pub manifold_point: ManifoldPoint,
}

#[derive(Debug, Copy, Clone)]
pub(crate) struct BallWorldContactRecord {
    pub rb_idx: usize,
    pub contact_point: glam::Vec3A,
    pub contact_normal: glam::Vec3A,
}

// Track contacts reported by Bullet callbacks.
pub(crate) struct ArenaContactTracker {
    collision_records: Vec<ContactRecord>,
    ball_world_records: Vec<BallWorldContactRecord>,
    ball_only: bool,
}

impl ArenaContactTracker {
    pub fn new() -> Self {
        Self {
            collision_records: Vec::with_capacity(4), // Reserve space for common contact counts.
            ball_world_records: Vec::with_capacity(4),
            ball_only: false,
        }
    }

    pub fn set_ball_only(&mut self, ball_only: bool) {
        self.ball_only = ball_only;
    }

    pub const fn num_records(&self) -> usize {
        self.collision_records.len()
    }

    pub fn get_record(&self, idx: usize) -> &ContactRecord {
        &self.collision_records[idx]
    }

    pub const fn num_ball_world_records(&self) -> usize {
        self.ball_world_records.len()
    }

    pub fn get_ball_world_record(&self, idx: usize) -> &BallWorldContactRecord {
        &self.ball_world_records[idx]
    }

    pub fn clear_records(&mut self) {
        self.collision_records.clear();
        self.ball_world_records.clear();
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
        } else if user_idx_a == UserInfoTypes::Ball
            && user_idx_b == UserInfoTypes::None
            && body_b.is_static_obj()
        {
            manifold_point.is_special = true;
        }

        // Record contact data before edge adjustment changes the manifold.
        if self.ball_only
            && user_idx_a == UserInfoTypes::Ball
            && user_idx_b == UserInfoTypes::None
            && body_b.is_static_obj()
        {
            self.ball_world_records.push(BallWorldContactRecord {
                rb_idx: body_a.world_array_idx,
                contact_point: manifold_point.pos_world_on_b,
                contact_normal: manifold_point.normal_world_on_b,
            });
        } else {
            self.collision_records.push(ContactRecord {
                is_swap: should_swap,
                rb_idx_a: body_a.world_array_idx,
                rb_idx_b: body_b.world_array_idx,
                manifold_point: *manifold_point,
            });
        }

        if let Some(idx) = idx {
            adjust_internal_edge_contacts(manifold_point, body_b, idx);
        }
    }
}
