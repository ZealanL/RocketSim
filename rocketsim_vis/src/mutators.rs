//! Apply the drive example's supported live mutators to one car.
//!
//! [`rocketsim::MutatorConfig`] is fixed when an arena is created. This module
//! implements four options through car-state updates before each physics tick:
//!
//! - Unlimited boost refills the car's boost before the tick.
//! - Boost recharge adds boost after the normal recharge delay.
//! - Unlimited flips clear the used-flip flag when the car is not flipping.
//! - Unlimited double jumps clear the used-double-jump state under the same
//!   condition.
//!
//! Call [`Mutators::apply_pre_tick`] before [`rocketsim::Arena::step_tick`].
//! These updates do not replace the arena's configuration. They do not change
//! gravity, masses, ball size, arena geometry, or boost-pad rules. Use
//! [`rocketsim::ArenaConfig`] to set those values before arena creation.
//!
//! [`crate::proto::ControlMsg::mutators`] carries only boolean flags. The
//! recharge rate remains local to the application. Your application decides
//! whether to accept any flags received from the viewer.
//!
//! ```
//! use rocketsim::{Arena, CarBodyConfig, GameMode, Team};
//! use rocketsim_vis::{mutators::Mutators, proto::MUT_UNLIMITED_BOOST};
//!
//! let mut arena = Arena::new(GameMode::TheVoid);
//! let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
//! arena.set_car_boost(car, 10.0);
//! let mutators = Mutators::from_mask(MUT_UNLIMITED_BOOST);
//! mutators.apply_pre_tick(&mut arena, car);
//! assert_eq!(arena.get_car_state(car).boost, 100.0);
//! arena.step_tick();
//! ```

use rocketsim::consts;

use crate::proto::{
    MUT_BOOST_RECHARGE, MUT_KNOWN_MASK, MUT_UNLIMITED_BOOST, MUT_UNLIMITED_DOUBLE_JUMPS,
    MUT_UNLIMITED_FLIPS,
};

/// Default recharge rate in boost units per second.
pub const DEFAULT_RECHARGE_PER_SECOND: f32 = consts::car::boost::RECHARGE_PER_SECOND;
/// Delay in seconds without boosting before recharge can start.
pub const RECHARGE_DELAY: f32 = consts::car::boost::RECHARGE_DELAY;
/// Maximum boost amount: `100.0`.
pub const MAX_BOOST: f32 = consts::car::boost::MAX;
/// Maximum configured recharge rate in boost units per second.
pub const MAX_RECHARGE_PER_SECOND: f32 = 100.0;

fn default_recharge_per_second() -> f32 {
    DEFAULT_RECHARGE_PER_SECOND
}

/// Options applied to one car before each simulation tick.
///
/// The default disables all flags and uses [`DEFAULT_RECHARGE_PER_SECOND`].
/// Convert flags to and from [`crate::proto::ControlMsg::mutators`] with
/// [`Self::to_mask`] and [`Self::from_mask`]. The bit mask does not include
/// `recharge_per_second`. See the [module example](self) for application.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Mutators {
    /// Refill boost before each tick. This takes priority over recharge.
    #[serde(default)]
    pub unlimited_boost: bool,
    /// Recharge boost at [`Self::recharge_per_second`] after [`RECHARGE_DELAY`].
    #[serde(default)]
    pub boost_recharge: bool,
    /// Recharge rate in boost units per second. Finite values use `0..=100`.
    #[serde(default = "default_recharge_per_second")]
    pub recharge_per_second: f32,
    /// Clear a used flip when the car is not currently flipping.
    #[serde(default)]
    pub unlimited_flips: bool,
    /// Clear used double-jump state when the car is not currently flipping.
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
    /// Restrict the recharge rate to `0..=100` boost units per second.
    ///
    /// This does not change the boolean flags. As with [`f32::clamp`], a
    /// `NaN` rate remains `NaN`. Supply a finite rate before applying it.
    pub fn clamp(&mut self) {
        self.recharge_per_second = self.recharge_per_second.clamp(0.0, MAX_RECHARGE_PER_SECOND);
    }

    /// Return these options with the recharge rate restricted by [`Self::clamp`].
    #[must_use]
    pub fn clamped(mut self) -> Self {
        self.clamp();
        self
    }

    /// Encode the four boolean flags as a protocol bit mask.
    ///
    /// Do not include the recharge rate. The bit positions are the
    /// `MUT_*` constants in [`crate::proto`].
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

    /// Decode the known mutator bits and ignore reserved bits.
    ///
    /// Set the recharge rate to [`DEFAULT_RECHARGE_PER_SECOND`]. This method
    /// does not preserve a previously configured custom rate.
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

    /// Return `true` if `mask` contains no reserved mutator bits.
    ///
    /// A zero mask is valid. [`Self::from_mask`] ignores reserved bits even
    /// when this method returns `false`.
    #[must_use]
    pub const fn is_known_mask(mask: u8) -> bool {
        mask & !MUT_KNOWN_MASK == 0
    }

    /// Apply these options to `car_idx` before the next physics tick.
    ///
    /// Add recharge for one `1/120`-second tick. Restrict finite recharge
    /// rates to `0..=100`. Unlimited boost takes priority over recharge.
    /// Preserve unrelated state, tuning, and collision history. This method
    /// does not step the arena. Call [`rocketsim::Arena::step_tick`] afterward.
    ///
    /// # Panics
    ///
    /// Panic if `car_idx` does not identify an existing car.
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
