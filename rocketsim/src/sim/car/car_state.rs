use std::ops::{Deref, DerefMut};

use glam::{Mat3A, Vec3A};

use crate::{CarControls, PhysState, consts};

/// Full mutable car simulation state.
///
/// Derefs to [`crate::PhysState`] so `car_state.pos/vel/...` work directly.
/// Read with `Arena::get_car_state`, write with `Arena::set_car_state`.
/// Times are seconds, ticks use [`crate::consts::TICK_TIME`] (`1/120` s).
/// `DEFAULT` spawns at rest with spawn boost.
#[derive(Clone, Copy, Debug)]
pub struct CarState {
    pub phys: PhysState,
    /// Controls to simulate the car with
    pub controls: CarControls,
    /// Controls from the last time this car was simulated (equals `controls` after step)
    pub prev_controls: CarControls,
    /// True if 3 or more wheels have contact
    pub is_on_ground: bool,
    /// Whether each of the 4 wheels have contact
    /// First two are front
    /// If your car has 3 wheels, the 4th bool will always be false
    pub wheels_with_contact: [bool; 4],
    /// Whether we jumped to get into the air
    ///
    /// Can be false while airborne, if we left the ground with a flip reset
    pub has_jumped: bool,
    /// True if we have double jumped and are still in the air
    pub has_double_jumped: bool,
    /// True if we are in the air, and (have flipped or are currently flipping)
    pub has_flipped: bool,
    /// Relative torque direction of the flip
    ///
    /// Forward flip will have positive Y
    pub flip_rel_torque: Vec3A,
    /// Integer-exact tick counter driving [`Self::jump_time()`].
    ///
    /// Counts ticks since the current jump phase began; reset to 0 on jump
    /// activation and to 1 on the tick the jump ends.
    pub jump_ticks: u32,
    /// When currently flipping, the time since we started flipping, else 0
    pub flip_time: f32,
    /// True during a flip (not an auto-flip, and not after a flip)
    pub is_flipping: bool,
    /// True during a jump
    pub is_jumping: bool,
    /// Total time spent in the air
    pub air_time: f32,
    /// Time spent in the air once `!is_jumping`
    ///
    /// If we never jumped, it is 0
    pub air_time_since_jump: f32,
    /// Goes from 0 to 100
    pub boost: f32,
    /// Seconds since boost was last held (drives recharge delay).
    pub time_since_boosted: f32,
    /// True if we boosted that tick
    ///
    /// There exists a minimum boosting time, thus why we must track boosting time
    pub is_boosting: bool,
    /// Seconds spent continuously boosting (see `boost::MIN_TIME` latch).
    pub boosting_time: f32,
    /// True above supersonic speed (with 1 s grace, see `supersonic` consts).
    pub is_supersonic: bool,
    /// Time since the car's speed dropped below `START_SPEED` while still supersonic,
    /// used for the supersonic maintain grace period
    pub supersonic_grace_timer: f32,
    /// Smoothed handbrake `0..1` (rise/fall rates, drives steering curves).
    pub handbrake_val: f32,
    /// True while the auto-flip recovery is playing.
    pub is_auto_flipping: bool,
    /// Counts down when auto-flipping
    pub auto_flip_timer: f32,
    /// Roll direction sign for the auto-flip torque.
    pub auto_flip_torque_scale: f32,
    /// Seconds until this car can bump/demo again.
    pub bump_cooldown_timer: f32,
    /// Last arena tick when this car applied an extra ball-hit impulse.
    pub last_extra_hit_tick: Option<u64>,
    /// If in contact with a static mesh/body, this is the collision normal of that contact on said body
    pub world_contact_normal: Option<Vec3A>,
    /// True while demolished (physics disabled until the respawn timer ends).
    pub is_demoed: bool,
    /// Seconds until respawn when demoed.
    pub demo_respawn_timer: f32,
}

impl Default for CarState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl CarState {
    pub const DEFAULT: Self = Self {
        phys: PhysState {
            pos: Vec3A::new(0.0, 0.0, consts::car::spawn::REST_Z),
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
        },
        controls: CarControls::DEFAULT,
        prev_controls: CarControls::DEFAULT,
        is_on_ground: true,
        wheels_with_contact: [false; 4],
        has_jumped: false,
        has_double_jumped: false,
        has_flipped: false,
        flip_rel_torque: Vec3A::ZERO,
        jump_ticks: 0,
        flip_time: 0.0,
        is_flipping: false,
        is_jumping: false,
        air_time: 0.0,
        air_time_since_jump: 0.0,
        boost: consts::car::boost::SPAWN_AMOUNT,
        time_since_boosted: 0.0,
        is_boosting: false,
        boosting_time: 0.0,
        is_supersonic: false,
        supersonic_grace_timer: 0.0,
        handbrake_val: 0.0,
        is_auto_flipping: false,
        auto_flip_timer: 0.0,
        auto_flip_torque_scale: 0.0,
        bump_cooldown_timer: 0.0,
        last_extra_hit_tick: None,
        world_contact_normal: None,
        is_demoed: false,
        demo_respawn_timer: 0.0,
    };

    /// True on ground, or airborne with a flip/double-jump still available
    /// (within `DOUBLEJUMP_MAX_DELAY`). Gate jump/flip inputs on this.
    #[must_use]
    pub const fn has_flip_or_jump(&self) -> bool {
        self.is_on_ground
            || (!self.has_flipped
                && !self.has_double_jumped
                && self.air_time_since_jump < consts::car::jump::DOUBLEJUMP_MAX_DELAY)
    }

    /// True when airborne via wheels (not a jump) with a flip available —
    /// i.e. a flip reset was just picked up.
    #[must_use]
    pub const fn has_flip_reset(&self) -> bool {
        !self.is_on_ground && self.has_flip_or_jump() && !self.has_jumped
    }

    /// True when airborne and never jumped (wheels-only launch state).
    #[must_use]
    pub const fn got_flip_reset(&self) -> bool {
        !self.is_on_ground && !self.has_jumped
    }

    /// Count of wheels (`0..4`) currently touching something.
    #[must_use]
    pub fn num_wheels_in_contact(&self) -> usize {
        let mut result = 0;
        for b in self.wheels_with_contact {
            if b {
                result += 1;
            }
        }
        result
    }

    /// Elapsed jump-hold time in seconds (`jump_ticks * TICK_TIME`).
    #[must_use]
    pub const fn jump_time(&self) -> f32 {
        self.jump_ticks as f32 * consts::TICK_TIME
    }
}

impl Deref for CarState {
    type Target = PhysState;
    fn deref(&self) -> &Self::Target {
        &self.phys
    }
}

impl DerefMut for CarState {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.phys
    }
}
