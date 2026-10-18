use std::ops::{Deref, DerefMut};

use glam::{Mat3A, Vec3A};

use crate::{CarControls, PhysState, RaycastHitInfo, consts};

/// Core fields used to build or replace a car snapshot.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CarCore {
    pub phys: PhysState,
    /// Controls to simulate the car with.
    pub controls: CarControls,
    /// Whether the car jumped to get into the air.
    pub has_jumped: bool,
    /// Whether the car double jumped and is still in the air.
    pub has_double_jumped: bool,
    /// Whether the car has flipped in the air.
    pub has_flipped: bool,
    /// Relative torque direction of the flip. Forward flip has positive Y.
    pub flip_rel_torque: Vec3A,
    /// Integer tick count that drives `jump_time`.
    pub jump_ticks: u32,
    /// Seconds since flip activity last changed. Advances on ground.
    pub flip_time: f32,
    /// True during a flip, not an auto-flip or after a flip.
    pub is_flipping: bool,
    /// True during a jump.
    pub is_jumping: bool,
    /// True if jump hold was released during sustain.
    pub jump_hold_broken: bool,
    /// Time in air after jump ends. Zero if the car did not jump.
    pub air_time_since_jump: f32,
    /// Boost amount, normally from 0 to 100.
    pub boost: f32,
    /// True while auto-flip recovery plays.
    pub is_auto_flipping: bool,
    /// Seconds left in auto-flip recovery.
    pub auto_flip_timer: f32,
    /// Roll direction sign for auto-flip angular velocity.
    pub auto_flip_torque_scale: f32,
    /// Seconds until this car can bump or demo again.
    pub bump_cooldown_timer: f32,
    /// Arena index of the last bumped or demoed car.
    pub bump_cooldown_victim: Option<usize>,
    /// Last arena tick when this car added a ball-hit impulse.
    pub last_extra_hit_tick: Option<u64>,
    /// True while demolished.
    pub is_demoed: bool,
    /// Seconds until respawn when demolished.
    pub demo_respawn_timer: f32,
}

impl CarCore {
    pub const DEFAULT: Self = Self {
        phys: PhysState {
            pos: Vec3A::new(0.0, 0.0, consts::car::spawn::REST_Z),
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
        },
        controls: CarControls::DEFAULT,
        has_jumped: false,
        has_double_jumped: false,
        has_flipped: false,
        flip_rel_torque: Vec3A::ZERO,
        jump_ticks: 0,
        flip_time: 0.0,
        is_flipping: false,
        is_jumping: false,
        jump_hold_broken: false,
        air_time_since_jump: 0.0,
        boost: consts::car::boost::SPAWN_AMOUNT,
        is_auto_flipping: false,
        auto_flip_timer: 0.0,
        auto_flip_torque_scale: 0.0,
        bump_cooldown_timer: 0.0,
        bump_cooldown_victim: None,
        last_extra_hit_tick: None,
        is_demoed: false,
        demo_respawn_timer: 0.0,
    };

    /// Clear jump and flip state for ground placement. Clamp boost to `0..100`.
    /// Calculate wheel contacts with the next tick or [`crate::Arena::refresh_car_contacts`].
    #[must_use]
    pub fn grounded(phys: PhysState, boost: f32) -> Self {
        Self {
            phys,
            boost: boost.clamp(0.0, consts::car::boost::MAX),
            has_jumped: false,
            has_double_jumped: false,
            has_flipped: false,
            is_jumping: false,
            is_flipping: false,
            ..Self::DEFAULT
        }
    }

    /// Create an airborne state with flip reset ready.
    #[must_use]
    pub fn aerial(phys: PhysState) -> Self {
        Self {
            phys,
            has_jumped: false,
            has_double_jumped: false,
            has_flipped: false,
            is_jumping: false,
            is_flipping: false,
            ..Self::DEFAULT
        }
    }

    /// Create an active jump state with the given elapsed tick count.
    #[must_use]
    pub fn jumping(phys: PhysState, ticks: u32) -> Self {
        Self {
            phys,
            has_jumped: true,
            is_jumping: true,
            jump_ticks: ticks,
            ..Self::DEFAULT
        }
    }

    /// Create an active flip state with the given relative torque.
    #[must_use]
    pub fn flipping(phys: PhysState, rel_torque: Vec3A) -> Self {
        Self {
            phys,
            has_jumped: true,
            has_flipped: true,
            is_flipping: true,
            flip_rel_torque: rel_torque,
            ..Self::DEFAULT
        }
    }

    #[must_use]
    pub fn with_controls(mut self, controls: CarControls) -> Self {
        self.controls = controls;
        self
    }

    #[must_use]
    pub fn with_boost(mut self, boost: f32) -> Self {
        self.boost = boost.clamp(0.0, consts::car::boost::MAX);
        self
    }

    /// Set linear velocity.
    #[must_use]
    pub fn with_vel(mut self, vel: Vec3A) -> Self {
        self.phys.vel = vel;
        self
    }

    /// Set position.
    #[must_use]
    pub fn with_pos(mut self, pos: Vec3A) -> Self {
        self.phys.pos = pos;
        self
    }

    /// Set rotation matrix.
    #[must_use]
    pub fn with_rot_mat(mut self, rot_mat: Mat3A) -> Self {
        self.phys.rot_mat = rot_mat;
        self
    }

    /// Set angular velocity.
    #[must_use]
    pub fn with_ang_vel(mut self, ang_vel: Vec3A) -> Self {
        self.phys.ang_vel = ang_vel;
        self
    }

    /// Check core invariants. This check is opt-in.
    pub fn debug_validate(&self) {
        debug_assert!(!self.is_jumping || self.has_jumped);
        debug_assert!(!self.is_flipping || self.has_flipped);
        debug_assert!((0.0..=consts::car::boost::MAX).contains(&self.boost));
    }
}

impl Default for CarCore {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Tuning values used to seed or replace a car snapshot.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CarTuning {
    /// Smoothed handbrake value from 0 to 1.
    pub handbrake_val: f32,
    /// True above supersonic speed, with a grace period.
    pub is_supersonic: bool,
    /// Seconds since speed fell below start speed while still supersonic.
    pub supersonic_grace_timer: f32,
    /// Seconds since boost was last held.
    pub time_since_boosted: f32,
    /// Seconds spent boosting continuously.
    pub boosting_time: f32,
    /// True if the car boosted this tick.
    pub is_boosting: bool,
    /// Total time spent in the air.
    pub air_time: f32,
}

impl CarTuning {
    pub const DEFAULT: Self = Self {
        handbrake_val: 0.0,
        is_supersonic: false,
        supersonic_grace_timer: 0.0,
        time_since_boosted: 0.0,
        boosting_time: 0.0,
        is_boosting: false,
        air_time: 0.0,
    };

    /// Derive supersonic state from velocity.
    #[must_use]
    pub fn from_speed(vel: Vec3A) -> Self {
        Self {
            is_supersonic: vel.length_squared()
                >= consts::car::supersonic::START_SPEED * consts::car::supersonic::START_SPEED,
            ..Self::DEFAULT
        }
    }

    /// Copy tuning values from an existing snapshot without changing their bits.
    #[must_use]
    pub const fn preserve(state: &CarState) -> Self {
        state.tuning()
    }
}

impl Default for CarTuning {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Collision and previous-control data. Read this data with the cache getters.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct CarCache {
    pub is_on_ground: bool,
    pub wheels_with_contact: [Option<RaycastHitInfo>; 4],
    pub world_contact_normal: Option<Vec3A>,
    pub prev_controls: CarControls,
}

impl Default for CarCache {
    fn default() -> Self {
        Self {
            is_on_ground: true,
            wheels_with_contact: [None; 4],
            world_contact_normal: None,
            prev_controls: CarControls::DEFAULT,
        }
    }
}

/// Full mutable car snapshot.
///
/// This type has flat public fields for inspection and replay.
/// Use [`CarCore`] with [`crate::Arena::set_car_state`] for normal changes.
/// Use [`CarTuning`] to preserve replay seeds with the full-state setter.
/// Times are seconds. Ticks use `TICK_TIME` (`1/120` s).
/// `DEFAULT` spawns at rest with spawn boost.
///
/// Read flat fields and build safe replacement values:
///
/// ```
/// use rocketsim::{CarCore, CarState, CarTuning};
///
/// let snapshot = CarState::DEFAULT;
/// let position = snapshot.pos;
/// let core = snapshot.core();
/// let tuning = snapshot.tuning();
/// let replacement = CarState::from(CarCore::grounded(core.phys, core.boost)).with_tuning(tuning);
/// assert_eq!(position, replacement.pos);
/// ```
///
/// Cache fields are private. Read them through getters:
///
/// ```compile_fail
/// use rocketsim::CarState;
/// let state = CarState::DEFAULT;
/// let _ground = state.cache.is_on_ground;
/// ```
///
/// Cache fields cannot be set through a struct literal:
///
/// ```compile_fail
/// use rocketsim::CarState;
/// let _state = CarState { cache: todo!(), ..CarState::DEFAULT };
/// ```
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CarState {
    pub phys: PhysState,
    /// Controls to simulate the car with.
    pub controls: CarControls,
    /// Whether the car jumped to get into the air.
    pub has_jumped: bool,
    /// Whether the car double jumped and is still in the air.
    pub has_double_jumped: bool,
    /// Whether the car has flipped in the air.
    pub has_flipped: bool,
    /// Relative torque direction of the flip.
    pub flip_rel_torque: Vec3A,
    /// Tick count for jump time.
    pub jump_ticks: u32,
    /// Seconds since flip activity changed. Advances on ground.
    pub flip_time: f32,
    /// True during a flip, not an auto-flip or after a flip.
    pub is_flipping: bool,
    /// True during a jump.
    pub is_jumping: bool,
    /// True if jump hold was released during sustain.
    pub jump_hold_broken: bool,
    /// Total time spent in the air.
    pub air_time: f32,
    /// Time in air after the jump ends. Zero if no jump occurred.
    pub air_time_since_jump: f32,
    /// Boost amount, normally from 0 to 100.
    pub boost: f32,
    /// Smoothed handbrake value from 0 to 1.
    pub handbrake_val: f32,
    /// True above supersonic speed, with a grace period.
    pub is_supersonic: bool,
    /// Time in the supersonic grace period.
    pub supersonic_grace_timer: f32,
    /// Seconds since boost was last held.
    pub time_since_boosted: f32,
    /// Seconds spent boosting continuously.
    pub boosting_time: f32,
    /// True if the car boosted this tick.
    pub is_boosting: bool,
    /// True while auto-flip recovery plays.
    pub is_auto_flipping: bool,
    /// Seconds left in auto-flip recovery.
    pub auto_flip_timer: f32,
    /// Roll direction sign for auto-flip angular velocity.
    pub auto_flip_torque_scale: f32,
    /// Seconds until this car can bump or demo again.
    pub bump_cooldown_timer: f32,
    /// Arena index of the last bumped or demoed car.
    pub bump_cooldown_victim: Option<usize>,
    /// Last arena tick when this car added a ball-hit impulse.
    pub last_extra_hit_tick: Option<u64>,
    /// True while demolished.
    pub is_demoed: bool,
    /// Seconds until respawn when demolished.
    pub demo_respawn_timer: f32,
    pub(crate) cache: CarCache,
}

impl Default for CarState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl CarState {
    pub const DEFAULT: Self = Self {
        phys: CarCore::DEFAULT.phys,
        controls: CarCore::DEFAULT.controls,
        has_jumped: CarCore::DEFAULT.has_jumped,
        has_double_jumped: CarCore::DEFAULT.has_double_jumped,
        has_flipped: CarCore::DEFAULT.has_flipped,
        flip_rel_torque: CarCore::DEFAULT.flip_rel_torque,
        jump_ticks: CarCore::DEFAULT.jump_ticks,
        flip_time: CarCore::DEFAULT.flip_time,
        is_flipping: CarCore::DEFAULT.is_flipping,
        is_jumping: CarCore::DEFAULT.is_jumping,
        jump_hold_broken: CarCore::DEFAULT.jump_hold_broken,
        air_time: CarTuning::DEFAULT.air_time,
        air_time_since_jump: CarCore::DEFAULT.air_time_since_jump,
        boost: CarCore::DEFAULT.boost,
        handbrake_val: CarTuning::DEFAULT.handbrake_val,
        is_supersonic: CarTuning::DEFAULT.is_supersonic,
        supersonic_grace_timer: CarTuning::DEFAULT.supersonic_grace_timer,
        time_since_boosted: CarTuning::DEFAULT.time_since_boosted,
        boosting_time: CarTuning::DEFAULT.boosting_time,
        is_boosting: CarTuning::DEFAULT.is_boosting,
        is_auto_flipping: CarCore::DEFAULT.is_auto_flipping,
        auto_flip_timer: CarCore::DEFAULT.auto_flip_timer,
        auto_flip_torque_scale: CarCore::DEFAULT.auto_flip_torque_scale,
        bump_cooldown_timer: CarCore::DEFAULT.bump_cooldown_timer,
        bump_cooldown_victim: CarCore::DEFAULT.bump_cooldown_victim,
        last_extra_hit_tick: CarCore::DEFAULT.last_extra_hit_tick,
        is_demoed: CarCore::DEFAULT.is_demoed,
        demo_respawn_timer: CarCore::DEFAULT.demo_respawn_timer,
        cache: CarCache {
            is_on_ground: true,
            wheels_with_contact: [None; 4],
            world_contact_normal: None,
            prev_controls: CarControls::DEFAULT,
        },
    };

    /// Project the core fields without normalization.
    #[must_use]
    pub const fn core(&self) -> CarCore {
        CarCore {
            phys: self.phys,
            controls: self.controls,
            has_jumped: self.has_jumped,
            has_double_jumped: self.has_double_jumped,
            has_flipped: self.has_flipped,
            flip_rel_torque: self.flip_rel_torque,
            jump_ticks: self.jump_ticks,
            flip_time: self.flip_time,
            is_flipping: self.is_flipping,
            is_jumping: self.is_jumping,
            jump_hold_broken: self.jump_hold_broken,
            air_time_since_jump: self.air_time_since_jump,
            boost: self.boost,
            is_auto_flipping: self.is_auto_flipping,
            auto_flip_timer: self.auto_flip_timer,
            auto_flip_torque_scale: self.auto_flip_torque_scale,
            bump_cooldown_timer: self.bump_cooldown_timer,
            bump_cooldown_victim: self.bump_cooldown_victim,
            last_extra_hit_tick: self.last_extra_hit_tick,
            is_demoed: self.is_demoed,
            demo_respawn_timer: self.demo_respawn_timer,
        }
    }

    /// Project tuning fields without normalization.
    #[must_use]
    pub const fn tuning(&self) -> CarTuning {
        CarTuning {
            handbrake_val: self.handbrake_val,
            is_supersonic: self.is_supersonic,
            supersonic_grace_timer: self.supersonic_grace_timer,
            time_since_boosted: self.time_since_boosted,
            boosting_time: self.boosting_time,
            is_boosting: self.is_boosting,
            air_time: self.air_time,
        }
    }

    /// Apply a replay tuning seed exactly. This does not normalize values.
    #[must_use]
    pub fn with_tuning(mut self, tuning: CarTuning) -> Self {
        self.handbrake_val = tuning.handbrake_val;
        self.is_supersonic = tuning.is_supersonic;
        self.supersonic_grace_timer = tuning.supersonic_grace_timer;
        self.time_since_boosted = tuning.time_since_boosted;
        self.boosting_time = tuning.boosting_time;
        self.is_boosting = tuning.is_boosting;
        self.air_time = tuning.air_time;
        self
    }

    /// True on ground or airborne with a flip or double-jump available next tick.
    #[must_use]
    pub const fn has_flip_or_jump(&self) -> bool {
        self.cache.is_on_ground
            || (!self.has_flipped
                && !self.has_double_jumped
                && self.air_time_since_jump + consts::TICK_TIME
                    < consts::car::jump::DOUBLEJUMP_MAX_DELAY)
    }

    /// True when a wheel contact gave a fresh flip reset.
    #[must_use]
    pub const fn has_flip_reset(&self) -> bool {
        !self.cache.is_on_ground && self.has_flip_or_jump() && !self.has_jumped
    }

    /// True when airborne without having jumped.
    #[must_use]
    pub const fn got_flip_reset(&self) -> bool {
        !self.cache.is_on_ground && !self.has_jumped
    }

    /// Return the number of wheels in contact.
    #[must_use]
    pub fn num_wheels_in_contact(&self) -> usize {
        self.cache
            .wheels_with_contact
            .iter()
            .filter(|w| w.is_some())
            .count()
    }

    /// Return elapsed jump-hold time in seconds.
    #[must_use]
    pub const fn jump_time(&self) -> f32 {
        self.jump_ticks as f32 * consts::TICK_TIME
    }

    /// Grounded flag from simulation, a contact query, or imported observations.
    /// Imported ground flags can disagree with the wheel count. New states need a tick or
    /// [`crate::Arena::refresh_car_contacts`] to query simulated geometry.
    #[must_use]
    pub const fn is_on_ground(&self) -> bool {
        self.cache.is_on_ground
    }

    /// Return contact information for each wheel. The first two are front wheels.
    /// Imported contact flags use placeholder hit fields, not measured raycast geometry.
    #[must_use]
    pub const fn wheels_with_contact(&self) -> &[Option<RaycastHitInfo>; 4] {
        &self.cache.wheels_with_contact
    }

    /// Return the contact normal for a static world body, if present.
    #[must_use]
    pub const fn world_contact_normal(&self) -> Option<Vec3A> {
        self.cache.world_contact_normal
    }

    /// Return controls from the last simulation step.
    #[must_use]
    pub const fn prev_controls(&self) -> &CarControls {
        &self.cache.prev_controls
    }

    /// Import coarse contact observations without querying the world.
    ///
    /// Each true wheel flag gets placeholder hit fields. These values are not measured
    /// raycast geometry. Ground state is set independently of the wheel flags.
    /// Keeps the world-contact normal, previous controls, and all other state.
    /// `Arena::set_car_state(core)` clears these observations. The full-state setter and
    /// full-state editor preserve them.
    pub fn set_contact_flags(&mut self, is_on_ground: bool, wheel_contacts: [bool; 4]) {
        self.cache.is_on_ground = is_on_ground;
        self.cache.wheels_with_contact = wheel_contacts.map(|has_contact| {
            has_contact.then_some(RaycastHitInfo {
                hit_point: Vec3A::ZERO,
                hit_normal: Vec3A::Z,
                hit_fraction: 0.0,
                user_info: crate::UserInfoType::None,
            })
        });
    }

    /// Restore input history for full-state replay.
    /// Normal `set_car_state(core)` sets previous controls to current controls.
    /// Use this inside an `edit_car_state_full` closure when replay needs prior inputs.
    pub const fn set_prev_controls(&mut self, controls: CarControls) {
        self.cache.prev_controls = controls;
    }

    /// Check invariants for a state seed. This check is opt-in.
    ///
    /// Do not call this after every simulation tick. Live state can contain
    /// phase transitions, and `flip_time` can advance while grounded.
    pub fn debug_validate(&self) {
        self.core().debug_validate();
        debug_assert!((0.0..=1.0).contains(&self.handbrake_val));
        debug_assert!(self.is_supersonic || self.supersonic_grace_timer == 0.0);
        if self.cache.is_on_ground {
            debug_assert!(!self.has_double_jumped && !self.has_flipped && !self.is_flipping);
            debug_assert_eq!(self.air_time, 0.0);
            debug_assert_eq!(self.flip_time, 0.0);
        }
    }
}

impl From<CarCore> for CarState {
    fn from(mut core: CarCore) -> Self {
        core.boost = core.boost.clamp(0.0, consts::car::boost::MAX);
        if core.is_jumping {
            core.has_jumped = true;
        }

        if core.is_flipping {
            core.has_flipped = true;
        }

        let tuning = CarTuning::from_speed(core.phys.vel);
        Self {
            phys: core.phys,
            controls: core.controls,
            has_jumped: core.has_jumped,
            has_double_jumped: core.has_double_jumped,
            has_flipped: core.has_flipped,
            flip_rel_torque: core.flip_rel_torque,
            jump_ticks: core.jump_ticks,
            flip_time: core.flip_time,
            is_flipping: core.is_flipping,
            is_jumping: core.is_jumping,
            jump_hold_broken: core.jump_hold_broken,
            air_time: tuning.air_time,
            air_time_since_jump: core.air_time_since_jump,
            boost: core.boost,
            handbrake_val: tuning.handbrake_val,
            is_supersonic: tuning.is_supersonic,
            supersonic_grace_timer: tuning.supersonic_grace_timer,
            time_since_boosted: tuning.time_since_boosted,
            boosting_time: tuning.boosting_time,
            is_boosting: tuning.is_boosting,
            is_auto_flipping: core.is_auto_flipping,
            auto_flip_timer: core.auto_flip_timer,
            auto_flip_torque_scale: core.auto_flip_torque_scale,
            bump_cooldown_timer: core.bump_cooldown_timer,
            bump_cooldown_victim: core.bump_cooldown_victim,
            last_extra_hit_tick: core.last_extra_hit_tick,
            is_demoed: core.is_demoed,
            demo_respawn_timer: core.demo_respawn_timer,
            cache: CarCache {
                is_on_ground: false,
                wheels_with_contact: [None; 4],
                world_contact_normal: None,
                prev_controls: core.controls,
            },
        }
    }
}

impl Deref for CarCore {
    type Target = PhysState;
    fn deref(&self) -> &Self::Target {
        &self.phys
    }
}
impl DerefMut for CarCore {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.phys
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
