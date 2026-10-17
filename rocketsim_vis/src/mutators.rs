//! Live demo mutators for the drive demo.
//!
//! [`MutatorConfig`](rocketsim::MutatorConfig) cannot change mid-match:
//! [`Arena`](rocketsim::Arena) keeps it in a private field with only a
//! shared-reference getter, so the drive demo emulates the live-capable
//! subset per tick instead. Before each [`step_tick`](rocketsim::Arena::step_tick),
//! call [`Mutators::apply_pre_tick`] before [`Arena::step_tick`]. Boost uses
//! the targeted setter. Flip flags use the full-state editor to preserve tuning
//! and contact history.
//!
//! Live-capable set (each verified against the sim source):
//!
//! * `unlimited_boost` — refills the tank every tick. Same net effect as
//!   `boost_used_per_second = 0` (Heatseeker setting).
//! * `boost_recharge` — passive regen at [`Mutators::recharge_per_second`]
//!   once [`time_since_boosted`](rocketsim::CarState::time_since_boosted)
//!   passes the RL recharge delay. Same net effect as
//!   `recharge_boost_enabled = true` (Dropshot setting).
//! * `unlimited_flips` / `unlimited_double_jumps` — clear the spent
//!   flip/double-jump flags while airborne. Same net effect as the matching
//!   [`MutatorConfig`](rocketsim::MutatorConfig) flags.
//!
//! Not live (excluded, one line each):
//!
//! * gravity — baked into `DiscreteDynamicsWorld` at build, no setter, and
//!   `Arena.bullet_world` is crate-private.
//! * ball bounciness/restitution — baked into the rigid body at `Ball::new`.
//! * ball scale — baked into the collision shape at `Ball::new`.
//! * car/ball mass — baked into rigid-body inertia at build.
//! * jump/dodge/boost force magnitudes — read per tick from the private
//!   config with no public setter and no state-write emulation.
//! * tick-rate slow-mo — not a sim mutator; pacing lives in the demo loop.
//! * demo rules — read per tick from the private config in car-car contact
//!   handling; no public override.
//! * boost-pad amounts/cooldowns — baked into `BoostPadGrid` at build.
//! * `GameMode`/arena geometry — meshes and goals built once at construction.

use rocketsim::consts;

use crate::proto::{
    MUT_BOOST_RECHARGE, MUT_KNOWN_MASK, MUT_UNLIMITED_BOOST, MUT_UNLIMITED_DOUBLE_JUMPS,
    MUT_UNLIMITED_FLIPS,
};

/// Default passive regen rate (boost/s, RL default).
pub const DEFAULT_RECHARGE_PER_SECOND: f32 = consts::car::boost::RECHARGE_PER_SECOND;
/// Regen starts only after this long without boosting (s, RL default).
pub const RECHARGE_DELAY: f32 = consts::car::boost::RECHARGE_DELAY;
/// Full boost tank (RL default).
pub const MAX_BOOST: f32 = consts::car::boost::MAX;
/// Upper clamp for [`Mutators::recharge_per_second`].
pub const MAX_RECHARGE_PER_SECOND: f32 = 100.0;

fn default_recharge_per_second() -> f32 {
    DEFAULT_RECHARGE_PER_SECOND
}

/// Live mutators for one driven car. All off = unmodded Soccar behavior.
///
/// Wire form is a [`u8`] level-state bitmask
/// ([`Mutators::to_mask`]/[`from_mask`](Mutators::from_mask)); floats stay
/// at local defaults and are not sent.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Mutators {
    /// Refill the tank every tick (no boost drain).
    #[serde(default)]
    pub unlimited_boost: bool,
    /// Passive regen at [`recharge_per_second`](Self::recharge_per_second).
    #[serde(default)]
    pub boost_recharge: bool,
    /// Regen rate in boost/s (clamped `0..=100`).
    #[serde(default = "default_recharge_per_second")]
    pub recharge_per_second: f32,
    /// Re-arm flips while airborne.
    #[serde(default)]
    pub unlimited_flips: bool,
    /// Re-arm double jumps while airborne.
    #[serde(default)]
    pub unlimited_double_jumps: bool,
}

impl Default for Mutators {
    fn default() -> Self {
        Self {
            unlimited_boost: false,
            boost_recharge: false,
            recharge_per_second: DEFAULT_RECHARGE_PER_SECOND,
            unlimited_flips: false,
            unlimited_double_jumps: false,
        }
    }
}

impl Mutators {
    /// Clamps [`recharge_per_second`](Self::recharge_per_second) to `0..=100`.
    pub fn clamp(&mut self) {
        self.recharge_per_second = self.recharge_per_second.clamp(0.0, MAX_RECHARGE_PER_SECOND);
    }

    /// Copy with [`recharge_per_second`](Self::recharge_per_second) clamped.
    #[must_use]
    pub fn clamped(mut self) -> Self {
        self.clamp();
        self
    }

    /// Packs the bool flags into the wire bitmask (floats are not sent).
    #[must_use]
    pub fn to_mask(self) -> u8 {
        let mut mask = 0u8;
        if self.unlimited_boost {
            mask |= MUT_UNLIMITED_BOOST;
        }
        if self.boost_recharge {
            mask |= MUT_BOOST_RECHARGE;
        }
        if self.unlimited_flips {
            mask |= MUT_UNLIMITED_FLIPS;
        }
        if self.unlimited_double_jumps {
            mask |= MUT_UNLIMITED_DOUBLE_JUMPS;
        }
        mask
    }

    /// Unpacks a wire bitmask. Reserved bits are ignored (forward compatible).
    #[must_use]
    pub fn from_mask(mask: u8) -> Self {
        Self {
            unlimited_boost: mask & MUT_UNLIMITED_BOOST != 0,
            boost_recharge: mask & MUT_BOOST_RECHARGE != 0,
            recharge_per_second: DEFAULT_RECHARGE_PER_SECOND,
            unlimited_flips: mask & MUT_UNLIMITED_FLIPS != 0,
            unlimited_double_jumps: mask & MUT_UNLIMITED_DOUBLE_JUMPS != 0,
        }
    }

    /// True when no bit in `mask` is outside the known set.
    #[must_use]
    pub const fn is_known_mask(mask: u8) -> bool {
        mask & !MUT_KNOWN_MASK == 0
    }

    /// Applies active mutators to one car before the tick, then call `step_tick`.
    /// Uses targeted boost updates and preserves tuning and contact history.
    pub fn apply_pre_tick(&self, arena: &mut rocketsim::Arena, car_idx: usize) {
        let car = arena.get_car_state(car_idx);
        let mut boost = car.boost;
        let should_rearm_flips = !car.is_flipping && self.unlimited_flips && car.has_flipped;
        let should_rearm_double_jumps = !car.is_flipping
            && self.unlimited_double_jumps
            && (car.has_double_jumped || car.air_time_since_jump != 0.0);
        if self.unlimited_boost {
            boost = MAX_BOOST;
        } else if self.boost_recharge
            && car.time_since_boosted >= RECHARGE_DELAY
            && car.boost < MAX_BOOST
        {
            let rate = self.recharge_per_second.clamp(0.0, MAX_RECHARGE_PER_SECOND);
            boost = (boost + rate * consts::TICK_TIME).min(MAX_BOOST);
        }
        if boost != car.boost {
            arena.set_car_boost(car_idx, boost);
        }
        if should_rearm_flips || should_rearm_double_jumps {
            arena.edit_car_state_full(car_idx, |state| {
                if should_rearm_flips {
                    state.has_flipped = false;
                }
                if should_rearm_double_jumps {
                    state.has_double_jumped = false;
                    state.air_time_since_jump = 0.0;
                }
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use rocketsim::{Arena, CarBodyConfig, CarControls, CarState, GameMode, Team};

    use super::*;

    fn arena_with_car() -> (Arena, usize) {
        let mut arena = Arena::new(GameMode::TheVoid);
        let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        (arena, car_idx)
    }

    fn state_with_history() -> CarState {
        let mut state = CarState::DEFAULT;
        state.handbrake_val = 0.37;
        state.is_supersonic = true;
        state.supersonic_grace_timer = 0.12;
        state.time_since_boosted = 2.5;
        state.boosting_time = 0.4;
        state.is_boosting = true;
        state.air_time = 1.25;
        state.set_contact_flags(true, [true, false, true, false]);
        state.set_prev_controls(CarControls {
            jump: true,
            throttle: 0.5,
            ..CarControls::default()
        });
        state
    }

    #[test]
    fn boost_refill_preserves_tuning_contact_cache_and_core() {
        let (mut arena, car_idx) = arena_with_car();
        let mut state = state_with_history();
        state.boost = 12.0;
        arena.set_car_state_full(car_idx, state);

        Mutators {
            unlimited_boost: true,
            ..Mutators::default()
        }
        .apply_pre_tick(&mut arena, car_idx);

        let after = arena.get_car_state(car_idx);
        assert_eq!(after.boost, MAX_BOOST);
        assert_eq!(after.handbrake_val, state.handbrake_val);
        assert_eq!(after.time_since_boosted, state.time_since_boosted);
        assert_eq!(after.is_supersonic, state.is_supersonic);
        assert_eq!(after.core().has_jumped, state.core().has_jumped);
        assert_eq!(after.is_on_ground(), state.is_on_ground());
        assert_eq!(after.wheels_with_contact(), state.wheels_with_contact());
        assert_eq!(after.prev_controls(), state.prev_controls());
    }

    #[test]
    fn flip_rearm_preserves_tuning_and_contact_cache() {
        let (mut arena, car_idx) = arena_with_car();
        let mut state = state_with_history();
        state.has_flipped = true;
        state.has_double_jumped = true;
        state.air_time_since_jump = 0.8;
        arena.set_car_state_full(car_idx, state);

        Mutators {
            unlimited_flips: true,
            unlimited_double_jumps: true,
            ..Mutators::default()
        }
        .apply_pre_tick(&mut arena, car_idx);

        let after = arena.get_car_state(car_idx);
        assert!(!after.has_flipped);
        assert!(!after.has_double_jumped);
        assert_eq!(after.air_time_since_jump, 0.0);
        assert_eq!(after.handbrake_val, state.handbrake_val);
        assert_eq!(after.time_since_boosted, state.time_since_boosted);
        assert_eq!(after.is_supersonic, state.is_supersonic);
        assert_eq!(after.is_on_ground(), state.is_on_ground());
        assert_eq!(after.wheels_with_contact(), state.wheels_with_contact());
        assert_eq!(after.prev_controls(), state.prev_controls());
    }

    #[test]
    fn unlimited_double_jump_reopens_an_expired_jump_window() {
        let (mut arena, car_idx) = arena_with_car();
        let mut state = state_with_history();
        state.has_jumped = true;
        state.air_time_since_jump = consts::car::jump::DOUBLEJUMP_MAX_DELAY;
        state.set_contact_flags(false, [false; 4]);
        arena.set_car_state_full(car_idx, state);
        assert!(!arena.get_car_state(car_idx).has_flip_or_jump());

        Mutators {
            unlimited_double_jumps: true,
            ..Mutators::default()
        }
        .apply_pre_tick(&mut arena, car_idx);

        let after = arena.get_car_state(car_idx);
        assert!(after.has_flip_or_jump());
        assert_eq!(after.air_time_since_jump, 0.0);
        assert_eq!(after.tuning().air_time, state.tuning().air_time);
        assert_eq!(after.prev_controls(), state.prev_controls());
        assert_eq!(after.wheels_with_contact(), state.wheels_with_contact());
    }

    #[test]
    fn spent_flip_flags_do_not_edit_state_when_nothing_needs_rearming() {
        let (mut arena, car_idx) = arena_with_car();
        let state = state_with_history();
        arena.set_car_state_full(car_idx, state);

        Mutators {
            unlimited_flips: true,
            unlimited_double_jumps: true,
            ..Mutators::default()
        }
        .apply_pre_tick(&mut arena, car_idx);

        let after = arena.get_car_state(car_idx);
        assert_eq!(after.handbrake_val, state.handbrake_val);
        assert_eq!(after.time_since_boosted, state.time_since_boosted);
        assert_eq!(after.is_supersonic, state.is_supersonic);
        assert_eq!(after.is_on_ground(), state.is_on_ground());
        assert_eq!(after.wheels_with_contact(), state.wheels_with_contact());
        assert_eq!(after.prev_controls(), state.prev_controls());
    }
}
