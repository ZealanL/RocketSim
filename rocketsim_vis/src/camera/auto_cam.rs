//! Automatic ball-cam: keeps ball-cam on unless a rule says otherwise
//! (stage 8a).
//!
//! Each rule is a Schmitt trigger (separate enter/exit thresholds plus a
//! minimum active time) evaluated on EMA-filtered signals (`tau ~= 0.15` s).
//! Any active rule votes ball-cam *off*; the global state adds its own
//! minimum on/off holds and a short blend. With no active rule — or when the
//! followed car is demoed — ball-cam defaults on.
//!
//! Drive with [`update`](Self::update) every frame (pure in `(state, dt)`,
//! `dt` clamped to `[0, 1/30]`, no RNG/wall-clock).

use std::fmt::Display;

use rocketsim::{ArenaState, consts::arena::get_aabb};

use crate::camera::clamp_dt;

/// Why ball-cam is currently off (for HUD display).
///
/// R1 dribble, R2 overhead, R3 close-touch, R4 retreat, R5 wall, R6 demo-run,
/// R8 post-touch latch. Priority is R1 > R2 > R3 = R8 > R6 > R5 > R4.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoCamRule {
    /// Ball pinned close on the ground (dribble carry).
    Dribble,
    /// Ball directly overhead.
    Overhead,
    /// About to touch a fast-closing ball.
    CloseTouch,
    /// Ball just touched and separating (short latch).
    PostTouch,
    /// Chasing an opponent for a demo while the ball is elsewhere.
    DemoRun,
    /// Up on (or under) the wall.
    Wall,
    /// Driving away from the ball at speed.
    Retreat,
}

impl AutoCamRule {
    /// Short HUD label.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Dribble => "dribble",
            Self::Overhead => "overhead",
            Self::CloseTouch => "close touch",
            Self::PostTouch => "post touch",
            Self::DemoRun => "demo run",
            Self::Wall => "wall",
            Self::Retreat => "retreat",
        }
    }
}

impl Display for AutoCamRule {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Per-rule Schmitt latch with a minimum active time.
#[derive(Debug, Clone, Copy)]
struct RuleHold {
    active: bool,
    time: f32,
}

impl RuleHold {
    const fn inactive() -> Self {
        Self {
            active: false,
            time: 0.0,
        }
    }

    fn step(&mut self, enter: bool, exit: bool, hold: f32, dt: f32) {
        if self.active {
            self.time += dt;
            if exit && self.time >= hold {
                self.active = false;
                self.time = 0.0;
            }
        } else if enter {
            self.active = true;
            self.time = 0.0;
        }
    }
}

/// Automatic ball-cam state for one followed car.
pub struct AutoCam {
    car_idx: usize,
    on: bool,
    blend: f32,
    time_on: f32,
    time_off: f32,
    init: bool,
    ema_d: f32,
    ema_dz: f32,
    ema_h: f32,
    ema_speed: f32,
    ema_dot: f32,
    ema_wall: f32,
    ema_closing: f32,
    prev_ema_d: f32,
    // Priority order: R1 > R2 > R3 = R8 > R6 > R5 > R4.
    rules: [AutoCamRuleSlot; 7],
}

/// Rule slot in priority order (first active wins [`AutoCam::off_reason`]).
#[derive(Debug, Clone, Copy)]
struct AutoCamRuleSlot {
    rule: AutoCamRule,
    hold: RuleHold,
}

impl AutoCam {
    /// Minimum ball-cam ON time before a rule may turn it off (s).
    pub const ON_MIN_HOLD: f32 = 0.30;
    /// Minimum ball-cam OFF time before it may turn back on (s).
    pub const OFF_MIN_HOLD: f32 = 0.45;
    /// Blend time between on/off (s).
    pub const BLEND_T: f32 = 0.30;
    /// EMA time constant for the rule signals (s).
    pub const EMA_TAU: f32 = 0.15;

    const COS_15_DEG: f32 = 0.965_925_8;
    const COS_20_DEG: f32 = 0.939_692_6;
    const COS_25_DEG: f32 = 0.906_307_8;
    const COS_30_DEG: f32 = 0.866_025_4;

    /// Watches `car_idx`; starts with ball-cam on.
    pub fn new(car_idx: usize) -> Self {
        const SLOT: AutoCamRuleSlot = AutoCamRuleSlot {
            rule: AutoCamRule::Dribble,
            hold: RuleHold::inactive(),
        };
        let mut rules = [SLOT; 7];
        rules[1].rule = AutoCamRule::Overhead;
        rules[2].rule = AutoCamRule::CloseTouch;
        rules[3].rule = AutoCamRule::PostTouch;
        rules[4].rule = AutoCamRule::DemoRun;
        rules[5].rule = AutoCamRule::Wall;
        rules[6].rule = AutoCamRule::Retreat;
        Self {
            car_idx,
            on: true,
            blend: 1.0,
            time_on: 0.0,
            time_off: 0.0,
            init: false,
            ema_d: 0.0,
            ema_dz: 0.0,
            ema_h: 0.0,
            ema_speed: 0.0,
            ema_dot: 0.0,
            ema_wall: 0.0,
            ema_closing: 0.0,
            prev_ema_d: 0.0,
            rules,
        }
    }

    /// Followed car index.
    pub fn car_idx(&self) -> usize {
        self.car_idx
    }

    /// True while ball-cam is on.
    pub fn ball_cam(&self) -> bool {
        self.on
    }

    /// Blend factor (`1` = fully ball-cam), ramped over [`BLEND_T`](Self::BLEND_T).
    pub fn blend(&self) -> f32 {
        self.blend
    }

    /// Highest-priority active rule, or `None` while ball-cam is wanted.
    pub fn off_reason(&self) -> Option<AutoCamRule> {
        self.rules
            .iter()
            .find(|slot| slot.hold.active)
            .map(|slot| slot.rule)
    }

    fn deactivate_all(&mut self) {
        for slot in &mut self.rules {
            slot.hold.active = false;
            slot.hold.time = 0.0;
        }
    }

    /// Advances the state by `dt` seconds; returns whether ball-cam is on.
    pub fn update(&mut self, arena_state: &ArenaState, dt: f32) -> bool {
        let dt = clamp_dt(dt);
        let Some((info, car)) = arena_state.cars.get(self.car_idx) else {
            self.deactivate_all();
            self.on = true;
            return true;
        };

        // Demoed cars can't be usefully framed: default ball-cam on.
        if car.is_demoed {
            self.deactivate_all();
            self.on = true;
            return true;
        }

        if self.on {
            self.time_on += dt;
        } else {
            self.time_off += dt;
        }

        let to_ball = arena_state.ball.pos - car.pos;
        let d_raw = to_ball.length();
        let dz_raw = arena_state.ball.pos.z - car.pos.z;
        let h_raw = to_ball.truncate().length();
        let speed_raw = car.vel.length();
        let aabb = get_aabb(arena_state.game_mode());
        let wall_raw = (aabb.max.x - car.pos.x.abs())
            .min(aabb.max.y - car.pos.y.abs())
            .max(0.0);

        let fwd_raw = car.get_forward_dir().truncate();
        let to_ball_2d = to_ball.truncate();
        let dot_raw = if fwd_raw.length_squared() > 1e-8 && to_ball_2d.length_squared() > 1e-8 {
            fwd_raw
                .normalize()
                .dot(to_ball_2d.normalize())
                .clamp(-1.0, 1.0)
        } else {
            1.0
        };

        let alpha = 1.0 - (-dt / Self::EMA_TAU).exp();
        let mut post_touch_trigger = false;
        if !self.init {
            self.ema_d = d_raw;
            self.ema_dz = dz_raw;
            self.ema_h = h_raw;
            self.ema_speed = speed_raw;
            self.ema_dot = dot_raw;
            self.ema_wall = wall_raw;
            self.ema_closing = 0.0;
            self.prev_ema_d = d_raw;
            self.init = true;
        } else {
            self.ema_d += (d_raw - self.ema_d) * alpha;
            self.ema_dz += (dz_raw - self.ema_dz) * alpha;
            self.ema_h += (h_raw - self.ema_h) * alpha;
            self.ema_speed += (speed_raw - self.ema_speed) * alpha;
            self.ema_dot += (dot_raw - self.ema_dot) * alpha;
            self.ema_wall += (wall_raw - self.ema_wall) * alpha;
            let closing_raw = if dt > 1e-6 {
                (self.prev_ema_d - self.ema_d) / dt
            } else {
                0.0
            };
            self.ema_closing += (closing_raw - self.ema_closing) * alpha;
            // R8: ball was close and is now separating fast.
            post_touch_trigger = self.prev_ema_d < 600.0 && self.ema_closing < -800.0;
            self.prev_ema_d = self.ema_d;
        }

        let d = self.ema_d;
        let dz = self.ema_dz;
        let h = self.ema_h;
        let speed = self.ema_speed;
        let dot = self.ema_dot;
        let wall = self.ema_wall;
        let closing = self.ema_closing;

        // R1 dribble: ball pinned close and low while grounded.
        self.rules[0].hold.step(
            d < 270.0 && dz < 150.0 && car.is_on_ground,
            d > 380.0,
            0.8,
            dt,
        );
        // R2 overhead: ball right above the car.
        self.rules[1].hold.step(
            h < 250.0 && dz > 300.0,
            h > 400.0 || dz < 200.0,
            Self::OFF_MIN_HOLD,
            dt,
        );
        // R3 close touch: ball close and closing fast.
        self.rules[2].hold.step(
            d < 450.0 && closing > 1200.0,
            d > 650.0,
            Self::OFF_MIN_HOLD,
            dt,
        );
        // R8 post touch: rising distance latch.
        self.rules[3].hold.step(post_touch_trigger, true, 0.6, dt);

        // R6 demo run: aimed at a nearby opponent while the ball is elsewhere.
        let mut opp_dist = f32::INFINITY;
        let mut opp_dot = -1.0;
        let mut opp_valid = false;
        if fwd_raw.length_squared() > 1e-8 && speed > 1500.0 {
            let fwd = fwd_raw.normalize();
            for (j, (other_info, other)) in arena_state.cars.iter().enumerate() {
                if j == self.car_idx || other_info.team == info.team || other.is_demoed {
                    continue;
                }
                let to_opp = other.pos - car.pos;
                let dist = to_opp.length();
                if dist < opp_dist {
                    opp_dist = dist;
                    let to_opp_2d = to_opp.truncate();
                    opp_dot = if to_opp_2d.length_squared() > 1e-8 {
                        fwd.dot(to_opp_2d.normalize()).clamp(-1.0, 1.0)
                    } else {
                        -1.0
                    };
                    opp_valid = true;
                }
            }
        }
        let r6_enter = opp_valid
            && opp_dist < 2500.0
            && opp_dot > Self::COS_15_DEG
            && (dot < Self::COS_30_DEG || d > 2000.0)
            && speed > 1500.0;
        let r6_exit = !opp_valid
            || opp_dist > 2800.0
            || opp_dot < Self::COS_20_DEG
            || speed < 1200.0
            || (dot > Self::COS_25_DEG && d < 1800.0);
        self.rules[4].hold.step(r6_enter, r6_exit, 0.8, dt);
        if self.rules[4].hold.active && self.rules[4].hold.time > 1.5 {
            self.rules[4].hold.active = false;
            self.rules[4].hold.time = 0.0;
        }

        // R5 wall: near the side wall and off the ground line.
        self.rules[5].hold.step(
            wall < 350.0 && car.pos.z > 150.0,
            wall > 550.0,
            Self::OFF_MIN_HOLD,
            dt,
        );
        // R4 retreat: driving away from a distant ball at speed.
        self.rules[6].hold.step(
            dot < -0.2 && d > 1500.0 && speed > 500.0,
            dot > 0.0 || d < 1200.0,
            Self::OFF_MIN_HOLD,
            dt,
        );

        let want_off = self.rules.iter().any(|slot| slot.hold.active);
        if want_off {
            if self.on && self.time_on >= Self::ON_MIN_HOLD {
                self.on = false;
                self.time_off = 0.0;
            }
        } else if !self.on && self.time_off >= Self::OFF_MIN_HOLD {
            self.on = true;
            self.time_on = 0.0;
        }

        let blend_target = if self.on { 1.0 } else { 0.0 };
        let step = if dt > 0.0 { dt / Self::BLEND_T } else { 0.0 };
        if self.blend < blend_target {
            self.blend = (self.blend + step).min(blend_target);
        } else {
            self.blend = (self.blend - step).max(blend_target);
        }

        self.on
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec3A;
    use rocketsim::{ArenaState, CarBodyConfig, CarInfo, CarState, GameMode, Team};

    use super::*;

    fn arena_with(ball_pos: Vec3A) -> ArenaState {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        let mut car = CarState::DEFAULT;
        car.pos = Vec3A::new(0.0, 0.0, 17.0);
        arena.cars.push((
            CarInfo {
                idx: 0,
                team: Team::Blue,
                config: CarBodyConfig::OCTANE,
            },
            car,
        ));
        arena.ball.pos = ball_pos;
        arena.ball.vel = Vec3A::ZERO;
        arena
    }

    #[test]
    fn defaults_to_ball_cam() {
        let arena = arena_with(Vec3A::new(0.0, 2000.0, 93.0));
        let mut auto = AutoCam::new(0);
        assert!(auto.update(&arena, 1.0 / 30.0));
        assert_eq!(auto.off_reason(), None);
        assert_eq!(auto.blend(), 1.0);
    }

    #[test]
    fn dribble_turns_off_and_back_on() {
        // Ball pinned in front on the ground: R1 enter.
        let close = arena_with(Vec3A::new(200.0, 0.0, 93.0));
        let mut auto = AutoCam::new(0);
        for _ in 0..15 {
            auto.update(&close, 1.0 / 30.0);
        }
        assert!(!auto.ball_cam());
        assert_eq!(auto.off_reason(), Some(AutoCamRule::Dribble));

        // Ball launched away: R1 exit, then the OFF hold releases.
        let far = arena_with(Vec3A::new(0.0, 5000.0, 93.0));
        for _ in 0..20 {
            auto.update(&far, 1.0 / 30.0);
        }
        assert!(auto.ball_cam());
        assert_eq!(auto.off_reason(), None);
    }

    #[test]
    fn retreat_fires_driving_away() {
        let mut arena = arena_with(Vec3A::new(-3000.0, 0.0, 93.0));
        // Car facing +X at speed, ball far behind.
        arena.cars[0].1.vel = Vec3A::new(1000.0, 0.0, 0.0);
        let mut auto = AutoCam::new(0);
        for _ in 0..30 {
            auto.update(&arena, 1.0 / 30.0);
        }
        assert!(!auto.ball_cam());
        assert_eq!(auto.off_reason(), Some(AutoCamRule::Retreat));
    }

    #[test]
    fn demoed_defaults_on() {
        let mut arena = arena_with(Vec3A::new(200.0, 0.0, 93.0));
        arena.cars[0].1.is_demoed = true;
        let mut auto = AutoCam::new(0);
        assert!(auto.update(&arena, 1.0 / 30.0));
        assert_eq!(auto.off_reason(), None);
    }
}
