use crate::bullet::collision::dispatch::collision_object::CollisionObject;
use crate::bullet::collision::dispatch::internal_edge_utility::adjust_internal_edge_contacts;
use crate::bullet::collision::narrowphase::manifold_point::ManifoldPoint;
use crate::bullet::collision::narrowphase::persistent_manifold::ContactAddedCallback;
use crate::consts;
use crate::sim::UserInfoTypes;
use std::mem;

// An instance of a contact event
#[derive(Debug, Copy, Clone)]
pub(crate) struct ContactRecord {
    pub is_swap: bool,
    pub rb_index_a: usize,
    pub rb_index_b: usize,
    pub user_index_a: UserInfoTypes,
    pub user_index_b: UserInfoTypes,
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

    pub fn drain_records(&mut self) -> Vec<ContactRecord> {
        self.collision_records.drain(..).collect()
    }
}

impl ContactAddedCallback for ArenaContactTracker {
    fn callback<'a>(
        &mut self,
        manifold_point: &mut ManifoldPoint,
        mut body_a: &'a CollisionObject,
        mut body_b: &'a CollisionObject,
    ) {
        debug_assert!(body_a.has_contact_response() || body_b.has_contact_response());

        let should_swap = if body_a.user_index != UserInfoTypes::None
            && body_b.user_index != UserInfoTypes::None
        {
            body_a.user_index > body_b.user_index
        } else {
            body_b.user_index != UserInfoTypes::None
        };

        if should_swap {
            mem::swap(&mut body_a, &mut body_b);
        }

        let user_index_a = body_a.user_index;
        let user_index_b = body_b.user_index;

        if user_index_a == UserInfoTypes::Car {
            let hit_coefs = match user_index_b {
                UserInfoTypes::Ball => consts::car::HIT_BALL_COEFS,
                UserInfoTypes::Car => consts::car::HIT_CAR_COEFS,
                _ => consts::car::HIT_WORLD_COEFS,
            };
            manifold_point.combined_friction = hit_coefs.friction;
            manifold_point.combined_restitution = hit_coefs.restitution;
        } else if user_index_a == UserInfoTypes::Ball {
            if user_index_b == UserInfoTypes::DropshotTile {
                todo!()
            } else if user_index_b == UserInfoTypes::None {
                manifold_point.is_special = true;
            }
        }

        // NOTE: Push *before* the manifold is mutated by adjust_internal_edge_contacts()
        self.collision_records.push(ContactRecord {
            is_swap: should_swap,
            rb_index_a: body_a.world_array_index,
            rb_index_b: body_b.world_array_index,
            user_index_a,
            user_index_b,
            manifold_point: *manifold_point,
        });

        adjust_internal_edge_contacts(
            manifold_point,
            body_a,
            if should_swap {
                manifold_point.index_0
            } else {
                manifold_point.index_1
            } as usize,
        );
    }
}
