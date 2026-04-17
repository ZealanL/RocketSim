use glam::Vec3A;

use crate::{GameMode, sim::consts};

#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
pub enum DemoMode {
    #[default]
    Normal,
    OnContact,
    Disabled,
}

#[derive(Clone, Copy, Debug)]
pub struct MutatorConfig {
    pub gravity: Vec3A,
    pub car_mass: f32,
    pub ball_mass: f32,
    pub ball_max_speed: f32,
    pub ball_drag: f32,
    pub jump_accel: f32,
    pub jump_immediate_force: f32,
    pub boost_accel_ground: f32,
    pub boost_accel_air: f32,
    pub boost_used_per_second: f32,
    pub respawn_delay: f32,
    pub bump_cooldown_time: f32,
    pub car_max_boost_amount: f32,
    pub car_spawn_boost_amount: f32,
    pub boost_pad_amount_small: f32,
    pub boost_pad_amount_big: f32,
    pub boost_pad_cooldown_big: f32,
    pub boost_pad_cooldown_small: f32,

    pub ball_hit_extra_force_scale: f32,
    pub bump_force_scale: f32,
    pub bump_requires_front_hit: bool,
    pub ball_radius: f32,
    pub unlimited_flips: bool,
    pub unlimited_double_jumps: bool,
    pub recharge_boost_enabled: bool,
    pub recharge_boost_per_second: f32,
    pub recharge_boost_delay: f32,
    pub demo_mode: DemoMode,
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
            bump_requires_front_hit: false, // No longer required in newer Rocket League versions
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
