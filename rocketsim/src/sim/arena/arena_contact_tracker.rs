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

// Store one contact event.
#[derive(Debug, Copy, Clone)]
pub(crate) struct ContactRecord {
    pub is_swap: bool,
    pub rb_idx_a: usize,
    pub rb_idx_b: usize,
    pub manifold_point: ManifoldPoint,
}

// Track contacts reported by Bullet callbacks.
pub(crate) struct ArenaContactTracker {
    collision_records: Vec<ContactRecord>,
    /// Deferred car-ball extra impulses armed for this tick, indexed by car
    /// index: `(ball rigid body index, impulse in UU)`.
    armed_ball_impulses: Vec<Option<(usize, Vec3A)>>,
    /// The armed impulse released by an actual car-ball manifold contact
    /// during narrowphase this tick: `(car index, ball rigid body index,
    /// impulse in UU)`. The real game applies at most one extra hit impulse
    /// per tick, so only the first released impulse is kept.
    released_ball_impulses: Vec<(usize, usize, Vec3A)>,
    /// The car whose released impulse was applied to the ball by
    /// [`ContactAddedCallback::post_detection_hook`] this tick. Read by the
    /// arena after the step so other cars' impulses stay suppressed for the
    /// tick even when their contact records are processed first.
    applied_ball_impulse_car: Option<usize>,
}

impl ArenaContactTracker {
    pub fn new() -> Self {
        Self {
            collision_records: Vec::with_capacity(4), // Reserve space for common contact counts.
            armed_ball_impulses: Vec::with_capacity(4),
            released_ball_impulses: Vec::with_capacity(1),
            applied_ball_impulse_car: None,
        }
    }

    /// Arm a deferred extra hit impulse for `car_idx`; it is released onto
    /// the ball before the solver runs if the pair produces a manifold
    /// contact this tick.
    pub fn arm_ball_impulse(&mut self, car_idx: usize, ball_rb_idx: usize, impulse_uu: Vec3A) {
        if self.armed_ball_impulses.len() <= car_idx {
            self.armed_ball_impulses.resize(car_idx + 1, None);
        }
        self.armed_ball_impulses[car_idx] = Some((ball_rb_idx, impulse_uu));
    }

    /// Resets the per-tick deferred-impulse bookkeeping; called once per
    /// tick before the arena arms this tick's impulses.
    pub fn clear_armed_ball_impulses(&mut self) {
        self.armed_ball_impulses.iter_mut().for_each(|a| *a = None);
        self.applied_ball_impulse_car = None;
    }

    /// The car whose deferred impulse was applied to the ball pre-solve
    /// this tick, if any.
    pub const fn applied_ball_impulse_car(&self) -> Option<usize> {
        self.applied_ball_impulse_car
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

            if user_idx_b == UserInfoTypes::Ball {
                // At most one extra hit impulse applies per tick: only the
                // first armed car to produce a manifold contact releases
                // (contacts arrive in detection order, so this matches the
                // post-step record order used by the cooldown). Later armed
                // impulses stay armed until the per-tick clear.
                if self.released_ball_impulses.is_empty()
                    && let Some(armed) =
                        self.armed_ball_impulses.get_mut(body_a.user_pointer)
                    && let Some((ball_rb_idx, impulse_uu)) = armed.take()
                {
                    self.released_ball_impulses
                        .push((body_a.user_pointer, ball_rb_idx, impulse_uu));
                }
            }
        } else if user_idx_a == UserInfoTypes::Ball
            && user_idx_b == UserInfoTypes::None
            && body_b.is_static_obj()
        {
            manifold_point.is_special = true;
        }

        // Record contact data before edge adjustment changes the manifold.
        if manifold_point.is_special {
            // Save the raw normal for special-contact aggregation.
            manifold_point.raw_normal_world_on_b = manifold_point.normal_world_on_b;
        }

        self.collision_records.push(ContactRecord {
            is_swap: should_swap,
            rb_idx_a: body_a.world_array_idx,
            rb_idx_b: body_b.world_array_idx,
            manifold_point: *manifold_point,
        });

        if let Some(idx) = idx {
            adjust_internal_edge_contacts(manifold_point, body_b, idx);
        }
    }

    /// Applies the deferred extra impulse released by this tick's manifold
    /// contacts to the ball, before the solver sees the pair. The real
    /// engine decides a deferred extra impulse on an earlier tick and
    /// applies it while the contact is processed — ahead of the constraint
    /// solve — so a ball that was already separating picks up no solver
    /// friction at all.
    fn post_detection_hook(&mut self, bodies: &mut [RigidBody]) {
        for (car_idx, ball_rb_idx, impulse_uu) in self.released_ball_impulses.drain(..) {
            bodies[ball_rb_idx].lin_vel += impulse_uu * consts::UU_TO_BT;
            self.applied_ball_impulse_car = Some(car_idx);
        }
    }
}
