//! Live demo mutators for the drive demo.
//!
//! [`MutatorConfig`](rocketsim::MutatorConfig) cannot change mid-match:
//! [`Arena`](rocketsim::Arena) keeps it in a private field with only a
//! shared-reference getter, so the drive demo emulates the live-capable
//! subset per tick instead. Before each [`step_tick`](rocketsim::Arena::step_tick),
//! call [`Mutators::apply_pre_tick`] on the driven car state, then write it
//! back with [`set_car_state`](rocketsim::Arena::set_car_state).
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

use rocketsim::{CarState, consts};

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

    /// Applies active mutators to `car` before the tick. Write back with
    /// `set_car_state`, then `step_tick`. No-op when all flags are off.
    pub fn apply_pre_tick(&self, car: &mut CarState) {
        if self.unlimited_boost {
            car.boost = MAX_BOOST;
        } else if self.boost_recharge
            && car.time_since_boosted >= RECHARGE_DELAY
            && car.boost < MAX_BOOST
        {
            let rate = self.recharge_per_second.clamp(0.0, MAX_RECHARGE_PER_SECOND);
            car.boost = (car.boost + rate * consts::TICK_TIME).min(MAX_BOOST);
        }
        // Skip mid-flip: clearing `has_flipped` early would cut the flip
        // torque phase short. The sim re-arms both flags on landing anyway.
        if !car.is_flipping {
            if self.unlimited_flips {
                car.has_flipped = false;
            }
            if self.unlimited_double_jumps {
                car.has_double_jumped = false;
                car.air_time_since_jump = 0.0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use rocketsim::{GameMode, MutatorConfig};

    use super::*;

    #[test]
    fn defaults_match_unmodded_soccar() {
        let m = Mutators::default();
        assert!(!m.unlimited_boost && !m.boost_recharge);
        assert!(!m.unlimited_flips && !m.unlimited_double_jumps);
        assert_eq!(m.recharge_per_second, DEFAULT_RECHARGE_PER_SECOND);

        // The sim defaults these mutators confirm: normal drain (Heatseeker
        // uses 0.0), no passive regen, no free flips.
        let cfg = MutatorConfig::new(GameMode::Soccar);
        assert_eq!(cfg.boost_used_per_second, 33.3);
        assert!(!cfg.recharge_boost_enabled);
        assert!(!cfg.unlimited_flips && !cfg.unlimited_double_jumps);

        // Normal per-tick drain for reference: 33.3/s, not 33/tick.
        assert!((cfg.boost_used_per_second * consts::TICK_TIME - 0.2775).abs() < 1e-4);

        // Defaults leave a fresh spawn state untouched.
        let mut car = CarState::DEFAULT;
        let before = car.boost;
        m.apply_pre_tick(&mut car);
        assert_eq!(car.boost, before);
        assert!(!car.has_flipped && !car.has_double_jumped);
    }

    #[test]
    fn unlimited_boost_pins_tank() {
        let m = Mutators {
            unlimited_boost: true,
            ..Mutators::default()
        };
        let mut car = CarState::DEFAULT;
        car.boost = 12.0;
        m.apply_pre_tick(&mut car);
        assert_eq!(car.boost, MAX_BOOST);
    }

    #[test]
    fn recharge_regen_and_delay_gate() {
        let m = Mutators {
            boost_recharge: true,
            ..Mutators::default()
        };
        // Past the delay: regen one tick at 10/s.
        let mut car = CarState::DEFAULT;
        car.boost = 50.0;
        car.time_since_boosted = 1.0;
        m.apply_pre_tick(&mut car);
        assert!((car.boost - (50.0 + 10.0 * consts::TICK_TIME)).abs() < 1e-6);

        // Inside the delay window: no regen (mirrors the sim gate).
        let mut car = CarState::DEFAULT;
        car.boost = 50.0;
        car.time_since_boosted = 0.0;
        m.apply_pre_tick(&mut car);
        assert_eq!(car.boost, 50.0);

        // Clamped at a full tank.
        let mut car = CarState::DEFAULT;
        car.boost = 99.99;
        car.time_since_boosted = 1.0;
        m.apply_pre_tick(&mut car);
        assert_eq!(car.boost, MAX_BOOST);
    }

    #[test]
    fn flip_flags_rearm_only_between_flips() {
        let m = Mutators {
            unlimited_flips: true,
            unlimited_double_jumps: true,
            ..Mutators::default()
        };
        let mut car = CarState::DEFAULT;
        car.has_flipped = true;
        car.has_double_jumped = true;
        car.air_time_since_jump = 0.5;
        car.is_flipping = false;
        m.apply_pre_tick(&mut car);
        assert!(!car.has_flipped && !car.has_double_jumped);
        assert_eq!(car.air_time_since_jump, 0.0);

        // Mid-flip: hands off so the torque phase can finish.
        let mut car = CarState::DEFAULT;
        car.has_flipped = true;
        car.is_flipping = true;
        m.apply_pre_tick(&mut car);
        assert!(car.has_flipped);
    }

    #[test]
    fn mask_round_trip_and_reserved_bits() {
        let m = Mutators {
            unlimited_boost: true,
            boost_recharge: true,
            recharge_per_second: DEFAULT_RECHARGE_PER_SECOND,
            unlimited_flips: true,
            unlimited_double_jumps: true,
        };
        assert_eq!(m.to_mask(), MUT_KNOWN_MASK);
        assert_eq!(Mutators::from_mask(m.to_mask()), m);
        assert!(Mutators::is_known_mask(m.to_mask()));
        assert!(!Mutators::is_known_mask(0b1111_0000));

        // Reserved bits are ignored on receive.
        let back = Mutators::from_mask(0xFF);
        assert!(back.unlimited_boost && back.unlimited_double_jumps);
        assert_eq!(back.to_mask(), MUT_KNOWN_MASK);

        // Each flag maps to its own bit.
        assert_eq!(
            Mutators {
                unlimited_boost: true,
                ..Mutators::default()
            }
            .to_mask(),
            MUT_UNLIMITED_BOOST
        );
        assert_eq!(
            Mutators {
                boost_recharge: true,
                ..Mutators::default()
            }
            .to_mask(),
            MUT_BOOST_RECHARGE
        );
    }

    #[test]
    fn toml_round_trip() {
        let m = Mutators {
            unlimited_boost: true,
            boost_recharge: true,
            recharge_per_second: 12.5,
            unlimited_flips: false,
            unlimited_double_jumps: true,
        };
        let text = toml::to_string(&m).unwrap();
        let back: Mutators = toml::from_str(&text).unwrap();
        assert_eq!(back.unlimited_boost, m.unlimited_boost);
        assert_eq!(back.boost_recharge, m.boost_recharge);
        assert_eq!(back.unlimited_flips, m.unlimited_flips);
        assert_eq!(back.unlimited_double_jumps, m.unlimited_double_jumps);
        assert!((back.recharge_per_second - 12.5).abs() < 1e-6);
    }

    #[test]
    fn sparse_toml_uses_defaults() {
        // Old file without mutator keys: all off, RL regen rate.
        let back: Mutators = toml::from_str("unlimited_boost = true\n").unwrap();
        assert!(back.unlimited_boost);
        assert!(!back.boost_recharge);
        assert_eq!(back.recharge_per_second, DEFAULT_RECHARGE_PER_SECOND);
    }

    #[test]
    fn clamping() {
        let mut m = Mutators {
            recharge_per_second: -5.0,
            ..Mutators::default()
        };
        m.clamp();
        assert_eq!(m.recharge_per_second, 0.0);

        let m = Mutators {
            recharge_per_second: 1.0e9,
            boost_recharge: true,
            ..Mutators::default()
        }
        .clamped();
        assert_eq!(m.recharge_per_second, MAX_RECHARGE_PER_SECOND);

        // Clamped view of a wild rate still regens sanely.
        let mut car = CarState::DEFAULT;
        car.boost = 10.0;
        car.time_since_boosted = 1.0;
        m.apply_pre_tick(&mut car);
        assert!((car.boost - (10.0 + MAX_RECHARGE_PER_SECOND * consts::TICK_TIME)).abs() < 1e-4);
    }
}
