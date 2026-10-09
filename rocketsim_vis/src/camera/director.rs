//! Broadcast director: auto-picks overview or one car (stage 8a).
//!
//! Each valid candidate (overview plus every non-demoed car) is scored every
//! frame; the director sticks with its pick subject to hold times and only
//! switches for a clearly better shot — except hard goal threats, which cut
//! in (rate-limited). All timing runs on caller `dt` (clamped); no RNG or
//! wall-clock.
//!
//! Per-car score: `S = 1.0·P + 2.0·T + 0.5·E + 0.5·A` (proximity, shot
//! threat, ball energy, activity). Overview: `S = 0.55 + 0.3·C − 0.3·E`
//! (closest-car spread minus energy).

use glam::Vec3A;
use rocketsim::{ArenaState, consts::arena::get_aabb};

use crate::camera::{clamp_dt, smootherstep01};

/// Auto-spectator target: fixed overview or following one car (ball-cam).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DirectorTarget {
    /// Fixed spectator view of the whole field.
    Overview,
    /// Follow `arena.cars[i]` with forced ball-cam.
    Car(usize),
}

/// Broadcast director: picks [`DirectorTarget`] from the arena state.
pub struct DirectorState {
    current: DirectorTarget,
    time_on_target: f32,
    time_since_force: f32,
    last_switch_forced: bool,
    blend_age: f32,
}

impl DirectorState {
    /// Minimum time on a target before a normal switch (s).
    pub const MIN_HOLD: f32 = 3.0;
    /// Minimum time on a forced shot before any switch (s).
    pub const MIN_SHOT: f32 = 1.0;
    /// Score margin the best candidate needs to take over.
    pub const MARGIN: f32 = 0.20;
    /// Shot threat that forces an immediate cut.
    pub const FORCE_T: f32 = 0.80;
    /// Cooldown between forced cuts (s).
    pub const FORCE_CD: f32 = 2.0;
    /// Time on a target after which the switch margin is dropped (s).
    pub const MAX_HOLD: f32 = 10.0;
    /// Cut blend duration (s), applied as a smoothstep.
    pub const BLEND_T: f32 = 0.7;

    /// Ball speed above which a shot can threaten (uu/s).
    pub const SHOT_MIN_SPEED: f32 = 800.0;
    /// Ball-velocity alignment with the goal direction for a threat.
    pub const SHOT_MIN_ALIGN: f32 = 0.70;
    /// Time for the ball to reach the goal mouth for a threat (s).
    pub const SHOT_MAX_T_GOAL: f32 = 3.0;
    /// Ball speed below which (near center) the state counts as kickoff.
    pub const KICKOFF_MAX_SPEED: f32 = 400.0;
    /// Ball distance from center below which (when slow) counts as kickoff.
    pub const KICKOFF_MAX_DIST: f32 = 1500.0;

    /// Starts on [`DirectorTarget::Overview`].
    pub fn new() -> Self {
        Self {
            current: DirectorTarget::Overview,
            time_on_target: 0.0,
            time_since_force: Self::FORCE_CD,
            last_switch_forced: false,
            blend_age: Self::BLEND_T,
        }
    }

    /// Currently selected target.
    pub fn target(&self) -> DirectorTarget {
        self.current
    }

    /// Cut blend factor (`0` = just cut, `1` = settled), over [`BLEND_T`](Self::BLEND_T).
    pub fn blend(&self) -> f32 {
        smootherstep01(self.blend_age / Self::BLEND_T)
    }

    /// Time spent on the current target (s).
    pub fn time_on_target(&self) -> f32 {
        self.time_on_target
    }

    /// Goal-mouth point for the `side` end of the field (`+1` = +Y).
    fn goal_center(arena: &ArenaState, side: f32) -> Vec3A {
        let aabb = get_aabb(arena.game_mode());
        Vec3A::new(0.0, side * aabb.max.y, aabb.max.z * 0.15)
    }

    /// Graded shot threat (`0`, or alignment `0.70..1.0`) toward `goal`.
    fn shot_threat(ball_pos: Vec3A, ball_vel: Vec3A, goal: Vec3A) -> f32 {
        let speed = ball_vel.length();
        if speed <= Self::SHOT_MIN_SPEED {
            return 0.0;
        }
        let to_goal = goal - ball_pos;
        let dist = to_goal.length();
        if dist < 1.0 {
            return 0.0;
        }
        let align = ball_vel.dot(to_goal) / (speed * dist);
        if align > Self::SHOT_MIN_ALIGN && dist / speed < Self::SHOT_MAX_T_GOAL {
            align.clamp(0.0, 1.0)
        } else {
            0.0
        }
    }

    /// True while the ball is dead at center (kickoff); threats are muted.
    fn is_kickoff(arena: &ArenaState) -> bool {
        arena.ball.vel.length() < Self::KICKOFF_MAX_SPEED
            && arena.ball.pos.truncate().length() < Self::KICKOFF_MAX_DIST
    }

    /// Activity bonus: boosting, supersonic, or lining up a demo.
    fn activity(arena: &ArenaState, idx: usize) -> f32 {
        let (info, car) = &arena.cars[idx];
        if car.is_boosting || car.is_supersonic {
            return 1.0;
        }
        // Demo edge: fast, aimed at a nearby opponent from behind-ish.
        let fwd = car.get_forward_dir().truncate();
        if fwd.length_squared() < 1e-8 || car.vel.length() < 1500.0 {
            return 0.0;
        }
        let fwd = fwd.normalize();
        let aimed = arena
            .cars
            .iter()
            .enumerate()
            .any(|(j, (other_info, other))| {
                j != idx
                    && other_info.team != info.team
                    && !other.is_demoed
                    && (other.pos - car.pos).length() < 700.0
                    && {
                        let to_opp = (other.pos - car.pos).truncate();
                        to_opp.length_squared() > 1e-8 && fwd.dot(to_opp.normalize()) > 0.8660
                    }
            });
        if aimed { 1.0 } else { 0.0 }
    }

    fn switch_to(&mut self, target: DirectorTarget, forced: bool) {
        self.current = target;
        self.time_on_target = 0.0;
        self.blend_age = 0.0;
        self.last_switch_forced = forced;
    }

    /// Advances the director by `dt` seconds and returns the target.
    pub fn update(&mut self, arena_state: &ArenaState, dt: f32) -> DirectorTarget {
        let dt = clamp_dt(dt);
        self.time_on_target += dt;
        self.time_since_force += dt;
        self.blend_age += dt;

        let ball_speed = arena_state.ball.vel.length();
        let energy = (ball_speed / 3000.0).clamp(0.0, 1.0);
        let kickoff = Self::is_kickoff(arena_state);

        // Per-car scores (`None` = demoed, not a valid candidate).
        let mut scores: Vec<Option<(f32, f32)>> = Vec::with_capacity(arena_state.num_cars());
        let mut min_dist = f32::INFINITY;
        for (i, (info, car)) in arena_state.cars.iter().enumerate() {
            if car.is_demoed {
                scores.push(None);
                continue;
            }
            let d = (arena_state.ball.pos - car.pos).length();
            min_dist = min_dist.min(d);
            let proximity = 1.0 / (1.0 + d / 1500.0);
            let threat = if kickoff {
                0.0
            } else {
                Self::shot_threat(
                    arena_state.ball.pos,
                    arena_state.ball.vel,
                    Self::goal_center(arena_state, -info.team.get_y_dir()),
                )
            };
            let score = 1.0 * proximity
                + 2.0 * threat
                + 0.5 * energy
                + 0.5 * Self::activity(arena_state, i);
            scores.push(Some((score, threat)));
        }

        let spread = if min_dist.is_finite() {
            (min_dist / 3000.0).clamp(0.0, 1.0)
        } else {
            1.0
        };
        let overview_score = 0.55 + 0.3 * spread - 0.3 * energy;

        let mut best = DirectorTarget::Overview;
        let mut best_score = overview_score;
        let mut best_threat = 0.0;
        for (i, score) in scores.iter().enumerate() {
            if let Some((score, threat)) = score
                && *score > best_score
            {
                best = DirectorTarget::Car(i);
                best_score = *score;
                best_threat = *threat;
            }
        }

        let (current_valid, current_score) = match self.current {
            DirectorTarget::Overview => (true, overview_score),
            DirectorTarget::Car(i) => match scores.get(i).copied().flatten() {
                Some((score, _)) => (true, score),
                None => (false, f32::NEG_INFINITY),
            },
        };

        if !current_valid {
            self.switch_to(best, false);
        } else if best != self.current {
            let force = best_threat >= Self::FORCE_T
                && self.time_since_force >= Self::FORCE_CD
                && self.time_on_target >= Self::MIN_SHOT;
            if force {
                self.time_since_force = 0.0;
                self.switch_to(best, true);
            } else {
                let hold = if self.last_switch_forced {
                    Self::MIN_SHOT
                } else {
                    Self::MIN_HOLD
                };
                let margin = if self.time_on_target >= Self::MAX_HOLD {
                    0.0
                } else {
                    Self::MARGIN
                };
                if self.time_on_target >= hold && best_score > current_score + margin {
                    self.switch_to(best, false);
                }
            }
        }

        self.current
    }
}

impl Default for DirectorState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use rocketsim::{ArenaState, CarBodyConfig, CarInfo, CarState, GameMode, Team};

    use super::*;

    fn car_at(idx: usize, team: Team, x: f32, y: f32) -> (CarInfo, CarState) {
        let mut car = CarState::DEFAULT;
        car.pos = Vec3A::new(x, y, 17.0);
        (
            CarInfo {
                idx,
                team,
                config: CarBodyConfig::OCTANE,
            },
            car,
        )
    }

    #[test]
    fn kickoff_holds_overview() {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.cars.push(car_at(0, Team::Blue, -2048.0, -2560.0));
        arena.cars.push(car_at(1, Team::Orange, 2048.0, 2560.0));
        // Ball dead at center: kickoff mutes all threats.
        arena.ball.pos = Vec3A::new(0.0, 0.0, 93.0);
        arena.ball.vel = Vec3A::ZERO;

        let mut director = DirectorState::new();
        let mut target = DirectorTarget::Overview;
        for _ in 0..150 {
            target = director.update(&arena, 1.0 / 30.0);
        }
        assert_eq!(target, DirectorTarget::Overview);
    }

    #[test]
    fn shot_threat_takes_over() {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.cars.push(car_at(0, Team::Blue, 0.0, -1000.0));
        // Ball roaring at the Orange (+Y) goal.
        arena.ball.pos = Vec3A::new(0.0, 0.0, 93.0);
        arena.ball.vel = Vec3A::new(0.0, 3000.0, 0.0);

        let mut director = DirectorState::new();
        let mut target = DirectorTarget::Overview;
        for _ in 0..130 {
            target = director.update(&arena, 1.0 / 30.0);
        }
        assert_eq!(target, DirectorTarget::Car(0));
    }

    #[test]
    fn demoed_car_is_dropped() {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.cars.push(car_at(0, Team::Blue, 0.0, -1000.0));
        arena.ball.pos = Vec3A::new(0.0, 0.0, 93.0);
        arena.ball.vel = Vec3A::new(0.0, 3000.0, 0.0);

        let mut director = DirectorState::new();
        for _ in 0..130 {
            director.update(&arena, 1.0 / 30.0);
        }
        assert_eq!(director.target(), DirectorTarget::Car(0));

        arena.cars[0].1.is_demoed = true;
        assert_eq!(
            director.update(&arena, 1.0 / 30.0),
            DirectorTarget::Overview
        );
    }
}
