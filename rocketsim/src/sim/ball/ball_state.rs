use std::ops::{Deref, DerefMut};

use glam::{Mat3A, Vec3A};

use crate::{PhysState, consts, consts::heatseeker};

/// Heatseeker targeting state: which goal is targeted and how fast.
///
/// `y_target_dir`: `0` = no target yet, `1` = Orange goal (`+Y`),
/// `-1` = Blue goal (`-Y`). `cur_target_speed` (uu/s) grows per hit;
/// `time_since_hit` (s) gates the speedup.
#[derive(Clone, Copy, Debug)]
pub struct HeatseekerInfo {
    /// Which net the ball should seek towards;
    /// When 0, no net
    pub y_target_dir: i8,
    /// Current seek speed target in uu/s (starts at `INITIAL_TARGET_SPEED`).
    pub cur_target_speed: f32,
    /// Seconds since the last car touch (gates `MIN_SPEEDUP_INTERVAL`).
    pub time_since_hit: f32,
}

impl Default for HeatseekerInfo {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl HeatseekerInfo {
    pub const DEFAULT: Self = Self {
        y_target_dir: 0,
        cur_target_speed: heatseeker::INITIAL_TARGET_SPEED,
        time_since_hit: 0.,
    };
}

/// Dropshot charge/damage state.
///
/// The ball charges on hard hits (`accumulated_hit_force`), then damages
/// tiles on a fast downward impact on the target side (`y_target_dir`:
/// `0` = none, `-1` = Blue side, `1` = Orange side). Damage AoE grows with
/// `charge_level` (1/7/19 tiles). `last_damage_tick` rate-limits damage.
#[derive(Clone, Copy, Debug)]
pub struct DropshotInfo {
    /// Charge level number, which controls the radius of damage when hitting tiles
    /// 1 = damages r=1 -> 1 tile
    /// 2 = damages r=2 -> 7 tiles
    /// 3 = damages r=3 -> 19 tiles
    pub charge_level: u8,
    /// Resets when a tile is damaged
    pub accumulated_hit_force: f32,
    /// Which side of the field the ball can damage (0=none, -1=blue, 1=orange)
    pub y_target_dir: i8,
    /// Last arena tick that damaged tiles (rate-limit), if any.
    pub last_damage_tick: Option<u64>,
}

impl Default for DropshotInfo {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl DropshotInfo {
    pub const DEFAULT: Self = Self {
        charge_level: 1,
        accumulated_hit_force: 0.,
        y_target_dir: 0,
        last_damage_tick: None,
    };
}

/// Ball physics + mode state.
///
/// Derefs to [`crate::PhysState`] so `ball_state.pos` works directly.
/// `tick_count_since_kickoff` drives the Hoops/Dropshot launch delay.
/// `DEFAULT` spawns the ball at rest at center (`REST_Z` height).
#[derive(Clone, Copy, Debug)]
pub struct BallState {
    pub phys: PhysState,
    pub hs_info: HeatseekerInfo,
    pub ds_info: DropshotInfo,
    pub tick_count_since_kickoff: u64,
}

impl Default for BallState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl BallState {
    pub const DEFAULT: Self = Self {
        phys: PhysState {
            pos: Vec3A::new(0.0, 0.0, consts::ball::REST_Z),
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
        },
        hs_info: HeatseekerInfo::DEFAULT,
        ds_info: DropshotInfo::DEFAULT,
        tick_count_since_kickoff: 0,
    };
}

impl Deref for BallState {
    type Target = PhysState;
    fn deref(&self) -> &Self::Target {
        &self.phys
    }
}

impl DerefMut for BallState {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.phys
    }
}
