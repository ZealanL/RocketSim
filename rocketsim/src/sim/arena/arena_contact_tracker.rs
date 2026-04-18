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

// An instance of a contact event
#[derive(Debug, Copy, Clone)]
pub(crate) struct ContactRecord {
    pub is_swap: bool,
    pub rb_idx_a: usize,
    pub rb_idx_b: usize,
    pub user_idx_a: UserInfoTypes,
    pub user_idx_b: UserInfoTypes,
    pub manifold_point: ManifoldPoint,
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

    pub fn num_records(&self) -> usize {
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
        } else if user_idx_a == UserInfoTypes::Ball {
            if user_idx_b == UserInfoTypes::DropshotTile {
                todo!()
            } else if user_idx_b == UserInfoTypes::None {
                manifold_point.is_special = true;
            }
        }

        // NOTE: Push *before* the manifold is mutated by adjust_internal_edge_contacts()
        self.collision_records.push(ContactRecord {
            is_swap: should_swap,
            rb_idx_a: body_a.world_array_idx,
            rb_idx_b: body_b.world_array_idx,
            user_idx_a,
            user_idx_b,
            manifold_point: *manifold_point,
        });

        adjust_internal_edge_contacts(
            manifold_point,
            body_b,
            if should_swap {
                manifold_point.idx_0
            } else {
                manifold_point.idx_1
            } as usize,
        );
    }
}
