use std::ops::{Deref, DerefMut};

use glam::{Mat3A, Vec3A};

use crate::{PhysState, consts, consts::heatseeker};

#[derive(Clone, Copy, Debug)]
pub struct HeatseekerInfo {
    /// Which net the ball should seek towards;
    /// When 0, no net
    pub y_target_dir: i8,
    pub cur_target_speed: f32,
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

#[derive(Clone, Copy, Debug)]
pub struct DropshotInfo {
    /// Charge level number, which controls the radius of damage when hitting tiles
    /// 1 = damages r=1 -> 1 tile
    /// 2 = damages r=2 -> 7 tiles
    /// 3 = damages r=3 -> 19 tiles
    pub charge_level: i32,
    /// Resets when a tile is damaged
    pub accumulated_hit_force: f32,
    /// Which side of the field the ball can damage (0=none, -1=blue, 1=orange)
    pub y_target_dir: i8,
    pub has_damaged: bool,
    /// Only valid if `has_damaged`
    pub last_damage_tick: u64,
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
        has_damaged: false,
        last_damage_tick: 0,
    };
}

#[derive(Clone, Copy, Debug)]
pub struct BallState {
    pub phys: PhysState,
    pub hs_info: HeatseekerInfo,
    pub ds_info: DropshotInfo,

    /// Used for preventing repeated extra impulse updates
    pub last_extra_hit_tick: Option<u64>,
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
        last_extra_hit_tick: None,
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
