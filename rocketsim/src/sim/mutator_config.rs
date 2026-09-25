use glam::Vec3A;

use crate::{GameMode, sim::consts};

/// How demos are awarded on car-car contact (see `Arena` bumper logic).
#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
pub enum DemoMode {
    /// Supersonic + in-cone bumps demolish (teammates spared by default).
    #[default]
    Normal,
    /// Any car-car contact demolishes.
    OnContact,
    /// Bumps only, never demos.
    Disabled,
}

/// Tunable game physics. Start from [`MutatorConfig::new`] for a game mode,
/// then override individual fields via [`crate::ArenaConfig::with_mutators`].
///
/// Units are Unreal units (uu) and seconds unless noted; masses are Bullet
/// units. `1.0` scales (`ball_hit_extra_force_scale`, `bump_force_scale`)
/// mean "default Rocket League behavior".
#[derive(Clone, Copy, Debug)]
pub struct MutatorConfig {
    /// Gravity vector in uu/s² (default `(0, 0, -650)`).
    pub gravity: Vec3A,
    /// Car mass in Bullet units (default 180).
    pub car_mass: f32,
    /// Ball mass in Bullet units (30, 50 for Snowday puck).
    pub ball_mass: f32,
    /// Ball speed clamp in uu/s (default 6000).
    pub ball_max_speed: f32,
    /// Ball linear damping (default 0.03).
    pub ball_drag: f32,
    /// Sustained jump acceleration in uu/s².
    pub jump_accel: f32,
    /// Instant jump velocity kick in uu/s.
    pub jump_immediate_force: f32,
    /// Boost acceleration on ground in uu/s².
    pub boost_accel_ground: f32,
    /// Boost acceleration in air in uu/s².
    pub boost_accel_air: f32,
    /// Boost drained per second (0 in Heatseeker = infinite boost).
    pub boost_used_per_second: f32,
    /// Seconds a demoed car waits before respawning.
    pub respawn_delay: f32,
    /// Seconds a bumper ignores further bumps after one connects.
    pub bump_cooldown_time: f32,
    /// Boost tank capacity (default 100).
    pub car_max_boost_amount: f32,
    /// Boost granted on spawn/kickoff (100 in Heatseeker/Dropshot).
    pub car_spawn_boost_amount: f32,
    /// Boost granted by a small pad (default 12).
    pub boost_pad_amount_small: f32,
    /// Boost granted by a big pad (default 100).
    pub boost_pad_amount_big: f32,
    /// Seconds until a big pad respawns (default 10).
    pub boost_pad_cooldown_big: f32,
    /// Seconds until a small pad respawns (default 4).
    pub boost_pad_cooldown_small: f32,

    /// Multiplier on the extra car->ball hit impulse (default 1).
    pub ball_hit_extra_force_scale: f32,
    /// Multiplier on car->car bump impulses (default 1).
    pub bump_force_scale: f32,
    /// Ball radius in uu (varies by mode, see `consts::ball::get_radius`).
    pub ball_radius: f32,
    /// Allow flipping without a flip reset.
    pub unlimited_flips: bool,
    /// Allow double-jumping without leaving the ground first.
    pub unlimited_double_jumps: bool,
    /// Passively recharge boost (default on in Dropshot).
    pub recharge_boost_enabled: bool,
    /// Boost recharged per second when enabled.
    pub recharge_boost_per_second: f32,
    /// Seconds after boosting before recharge starts.
    pub recharge_boost_delay: f32,
    /// Demo rule (default [`DemoMode::Normal`]).
    pub demo_mode: DemoMode,
    /// When `false` (default), teammates can't demo each other.
    pub enable_team_demos: bool,
    /// Only used if the game mode has soccar goals (i.e. soccar, heatseeker, snowday)
    pub goal_base_threshold_y: f32,
}

impl Default for MutatorConfig {
    fn default() -> Self {
        const { Self::new(GameMode::Soccar) }
    }
}

impl MutatorConfig {
    /// Defaults for a game mode (Snowday puck mass, Heatseeker infinite
    /// boost/spawn boost, Dropshot recharge + ball radius, ...).
    ///
    /// # Example
    ///
    /// ```no_run
    /// use rocketsim::{ArenaConfig, GameMode, MutatorConfig};
    /// let mutators = MutatorConfig::new(GameMode::Soccar);
    /// let config = ArenaConfig::new(GameMode::Soccar).with_mutators(mutators);
    /// ```
    #[must_use]
    pub const fn new(game_mode: GameMode) -> Self {
        Self {
            gravity: Vec3A::new(0., 0., consts::GRAVITY_Z),
            car_mass: consts::car::MASS_BT,
            ball_mass: if matches!(game_mode, GameMode::Snowday) {
                consts::snowday::PUCK_MASS_BT
            } else {
                consts::ball::MASS_BT
            },
            ball_max_speed: consts::ball::MAX_SPEED,
            ball_drag: consts::ball::DRAG,
            jump_accel: consts::car::jump::ACCEL,
            jump_immediate_force: consts::car::jump::IMMEDIATE_FORCE,
            boost_accel_ground: consts::car::boost::ACCEL_GROUND,
            boost_accel_air: consts::car::boost::ACCEL_AIR,
            boost_used_per_second: match game_mode {
                GameMode::Heatseeker => 0.0,
                _ => consts::car::boost::USED_PER_SECOND,
            },
            respawn_delay: consts::car::spawn::RESPAWN_TIME,
            bump_cooldown_time: consts::car::bump::COOLDOWN_TIME,
            car_max_boost_amount: consts::car::boost::MAX,
            car_spawn_boost_amount: match game_mode {
                GameMode::Heatseeker | GameMode::Dropshot => 100.,
                _ => consts::car::boost::SPAWN_AMOUNT,
            },
            boost_pad_amount_big: consts::boost_pads::BOOST_AMOUNT_BIG,
            boost_pad_amount_small: consts::boost_pads::BOOST_AMOUNT_SMALL,
            boost_pad_cooldown_big: consts::boost_pads::COOLDOWN_BIG,
            boost_pad_cooldown_small: consts::boost_pads::COOLDOWN_SMALL,
            ball_hit_extra_force_scale: 1.,
            bump_force_scale: 1.,
            ball_radius: consts::ball::get_radius(game_mode),
            unlimited_flips: false,
            unlimited_double_jumps: false,
            recharge_boost_enabled: matches!(game_mode, GameMode::Dropshot),
            recharge_boost_per_second: consts::car::boost::RECHARGE_PER_SECOND,
            recharge_boost_delay: consts::car::boost::RECHARGE_DELAY,
            demo_mode: DemoMode::Normal,
            enable_team_demos: false,
            goal_base_threshold_y: consts::goal::SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y,
        }
    }
}
