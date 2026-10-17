//! Broadcast director: picks overview or one car each frame.
//!
//! Candidates score every frame; the pick sticks through hold times and
//! switches only for a clearly better shot — except hard goal threats,
//! which cut in (rate-limited). All timing runs on caller `dt`; no RNG or
//! wall-clock.
//!
//! Per-car scores combine ball proximity, energy, activity, aerial focus,
//! likely-shot attacker priority, and projected-save opportunity. Scores are
//! heuristics. Scoring projections use simple motion and do not model arena
//! collision meshes.

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
    current_car_id: Option<usize>,
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

    /// Ball speed above which a projected scoring event is considered (uu/s).
    pub const SHOT_MIN_SPEED: f32 = 800.0;
    /// Minimum useful alignment with a Soccar goal direction.
    pub const SHOT_MIN_ALIGN: f32 = 0.70;
    /// Maximum time horizon for a projected scoring event (s).
    pub const SHOT_MAX_T_GOAL: f32 = 3.0;
    /// Ball speed below which the centered spawn ball can count as kickoff.
    pub const KICKOFF_MAX_SPEED: f32 = 200.0;
    /// Aerial-play score bonus (mirrors RL's "focus on aerials" autocam:
    /// an airborne car under a high ball is where the play is).
    pub const AERIAL_BONUS: f32 = 0.25;
    /// Ball height above which a car counts as contesting an aerial (uu).
    pub const AERIAL_BALL_Z: f32 = 600.0;
    /// Car-ball distance under which the aerial bonus applies (uu).
    pub const AERIAL_MAX_DIST: f32 = 1500.0;
    /// Ball horizontal distance from center for a kickoff spawn (uu).
    pub const KICKOFF_MAX_DIST: f32 = 250.0;
    const BALL_GRAVITY: f32 = 650.0;
    /// Approximate ball radius for floor and Soccar-mouth clearance. Snowday
    /// uses puck geometry, which this lightweight projection does not model.
    const BALL_RADIUS: f32 = 92.75;
    const FLOOR_RESTITUTION: f32 = 0.60;
    const PROJECTION_STEP: f32 = 1.0 / 120.0;
    const REST_BOUNCE_SPEED: f32 = 40.0;

    /// Starts on [`DirectorTarget::Overview`].
    pub fn new() -> Self {
        Self {
            current: DirectorTarget::Overview,
            current_car_id: None,
            time_on_target: 0.0,
            time_since_force: Self::FORCE_CD,
            last_switch_forced: false,
            blend_age: Self::BLEND_T,
        }
    }

    /// Reset target and timing state to a clean overview.
    pub fn reset(&mut self) {
        self.current = DirectorTarget::Overview;
        self.current_car_id = None;
        self.time_on_target = 0.0;
        self.time_since_force = Self::FORCE_CD;
        self.last_switch_forced = false;
        self.blend_age = Self::BLEND_T;
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

    /// Project a scoring event with constant horizontal velocity, gravity,
    /// and a simple damped floor bounce. This is a camera heuristic, not
    /// RocketSim collision physics.
    /// Unsupported scoring geometry returns no event rather than using a
    /// Soccar goal shape for another mode.
    fn scoring_event(arena: &ArenaState) -> Option<(f32, Vec3A, f32)> {
        use rocketsim::GameMode;

        let mode = arena.game_mode();
        if !matches!(
            mode,
            GameMode::Soccar | GameMode::Heatseeker | GameMode::Snowday | GameMode::Hoops
        ) {
            return None;
        }
        let p = arena.ball.pos;
        let v = arena.ball.vel;
        let side = v.y.signum();
        if mode == GameMode::Hoops {
            // Hoops is a horizontal ring. Project to the descending crossing
            // of its score height, then test the ring radius in the floor plane.
            let ring_z = rocketsim::consts::goal::HOOPS_GOAL_SCORE_THRESHOLD_Z;
            let height = p.z - ring_z;
            let discriminant = v.z * v.z + 2.0 * Self::BALL_GRAVITY * height;
            if discriminant < 0.0 || (v.length_squared() < 1.0 && height < 350.0) {
                return None;
            }
            // The larger root is the descending crossing. It also handles a
            // ball rising from below the ring before falling through it.
            let t = (v.z + discriminant.sqrt()) / Self::BALL_GRAVITY;
            if !(0.0..=Self::SHOT_MAX_T_GOAL).contains(&t) {
                return None;
            }
            let predicted = Vec3A::new(p.x + v.x * t, p.y + v.y * t, ring_z);
            let side = predicted.y.signum();
            if side == 0.0 {
                return None;
            }
            let dy = predicted.y.abs() * 0.9 - 2770.0;
            let ring_dist = (predicted.x * predicted.x + dy * dy).sqrt();
            let threat = if ring_dist < 716.0 {
                (1.0 - ring_dist / 716.0).clamp(0.25, 1.0)
            } else {
                0.0
            };
            return (threat > 0.0).then_some((threat, predicted, t));
        }

        if v.length() < Self::SHOT_MIN_SPEED || v.y.abs() < 1.0 {
            return None;
        }
        let bounds = get_aabb(mode);
        let plane_y = side * bounds.max.y;
        let mut pos = p;
        let mut vel = v;
        let mut elapsed = 0.0;
        while elapsed < Self::SHOT_MAX_T_GOAL {
            let dt = Self::PROJECTION_STEP.min(Self::SHOT_MAX_T_GOAL - elapsed);
            vel.z -= Self::BALL_GRAVITY * dt;
            let next = pos + vel * dt;
            let next_time = elapsed + dt;
            if next.z < Self::BALL_RADIUS {
                pos.z = Self::BALL_RADIUS;
                if vel.z.abs() < Self::REST_BOUNCE_SPEED {
                    vel.z = 0.0;
                } else {
                    vel.z = vel.z.abs() * Self::FLOOR_RESTITUTION;
                }
            } else {
                pos.z = next.z;
            }
            pos.x = next.x;
            pos.y = next.y;
            if (pos.y - plane_y) * side >= 0.0 {
                let span = pos.y - (pos.y - vel.y * dt);
                let fraction = if span.abs() > 1e-4 {
                    ((plane_y - (pos.y - vel.y * dt)) / span).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let predicted = Vec3A::new(pos.x - vel.x * dt * (1.0 - fraction), plane_y, pos.z);
                let align = (v.y.abs() / v.length()).clamp(0.0, 1.0);
                let threat = if align >= Self::SHOT_MIN_ALIGN
                    && predicted.x.abs() < 893.0 - Self::BALL_RADIUS
                    && (Self::BALL_RADIUS..642.0).contains(&predicted.z)
                {
                    align
                } else {
                    0.0
                };
                return (threat > 0.0).then_some((threat, predicted, elapsed + dt * fraction));
            }
            elapsed = next_time;
        }
        None
    }

    /// Aerial bonus for `activity`-style scoring: an airborne car within
    /// range of a high ball. Returns [`AERIAL_BONUS`](Self::AERIAL_BONUS)
    /// or `0.0`.
    fn aerial_bonus(arena_state: &ArenaState, idx: usize, dist: f32) -> f32 {
        let car = &arena_state.cars[idx].1;
        if !car.is_on_ground()
            && arena_state.ball.pos.z > Self::AERIAL_BALL_Z
            && dist < Self::AERIAL_MAX_DIST
        {
            Self::AERIAL_BONUS
        } else {
            0.0
        }
    }

    /// True while the ball is dead at center (kickoff); threats are muted.
    fn is_kickoff(arena: &ArenaState) -> bool {
        arena.ball.vel.length() < Self::KICKOFF_MAX_SPEED
            && arena.ball.pos.truncate().length() < Self::KICKOFF_MAX_DIST
            && (arena.ball.pos.z - rocketsim::consts::ball::REST_Z).abs() < 60.0
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

        // `DirectorTarget::Car` is a vector index for the caller, but the
        // selected player is tracked by RocketSim's stable car id.
        let selected_car_missing = self
            .current_car_id
            .is_some_and(|id| !arena_state.cars.iter().any(|(info, _)| info.idx == id));
        if let Some(id) = self.current_car_id {
            self.current = arena_state
                .cars
                .iter()
                .position(|(info, _)| info.idx == id)
                .map(DirectorTarget::Car)
                .unwrap_or(DirectorTarget::Overview);
        }

        let ball_speed = arena_state.ball.vel.length();
        let energy = (ball_speed / 3000.0).clamp(0.0, 1.0);
        let kickoff = Self::is_kickoff(arena_state);

        // Score each player independently. A scoring event boosts only the
        // most plausible attacker and defenders close to its projected path.
        let event = if kickoff {
            None
        } else {
            Self::scoring_event(arena_state)
        };
        let mut attacker = None;
        let mut attacker_value = 0.0_f32;
        let mut defender_best: Option<(usize, f32)> = None;
        if let Some((_, event_pos, event_time)) = event {
            let side = event_pos.y.signum();
            for (i, (info, car)) in arena_state.cars.iter().enumerate() {
                if car.is_demoed {
                    continue;
                }
                let attacked_goal_side = -info.team.get_y_dir();
                let predicted_car = car.pos + car.vel * event_time.min(1.5);
                if attacked_goal_side == side {
                    let delta = arena_state.ball.pos - car.pos;
                    let closing = if delta.length_squared() > 1.0 {
                        car.vel.dot(delta.normalize()).clamp(-1000.0, 2000.0) / 2000.0
                    } else {
                        0.0
                    };
                    let reach = (delta.length() / (car.vel.length().max(900.0))).clamp(0.0, 3.0);
                    let value = 1.0 / (1.0 + reach) + 0.25 * closing.max(0.0);
                    if value > attacker_value {
                        attacker = Some(i);
                        attacker_value = value;
                    }
                } else {
                    let dist = (predicted_car - event_pos).length();
                    if dist < 2200.0 {
                        let value = 1.0 / (1.0 + dist / 900.0);
                        if defender_best.is_none_or(|(_, best)| value > best) {
                            defender_best = Some((i, value));
                        }
                    }
                }
            }
        }

        let mut scores: Vec<Option<(f32, f32)>> = Vec::with_capacity(arena_state.num_cars());
        let mut min_dist = f32::INFINITY;
        for (i, (_, car)) in arena_state.cars.iter().enumerate() {
            if car.is_demoed {
                scores.push(None);
                continue;
            }
            let d = (arena_state.ball.pos - car.pos).length();
            min_dist = min_dist.min(d);
            let proximity = 1.0 / (1.0 + d / 1500.0);
            let is_attacker = attacker == Some(i);
            let save_weight = defender_best
                .filter(|(candidate, _)| *candidate == i)
                .map_or(0.0, |(_, weight)| weight);
            let threat = event.map_or(0.0, |(threat, _, _)| {
                threat * if is_attacker { 1.0 } else { save_weight * 0.9 }
            });
            let attack_bonus = if is_attacker {
                0.75 * attacker_value
            } else {
                0.0
            };
            let save_bonus = 1.1 * save_weight * event.map_or(0.0, |(threat, _, _)| threat);
            let score = proximity
                + 0.5 * energy
                + 0.5 * Self::activity(arena_state, i)
                + Self::aerial_bonus(arena_state, i, d)
                + attack_bonus
                + save_bonus;
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

        let (current_valid, current_score) = if selected_car_missing {
            (false, f32::NEG_INFINITY)
        } else {
            match self.current {
                DirectorTarget::Overview => (true, overview_score),
                DirectorTarget::Car(i) => match scores.get(i).copied().flatten() {
                    Some((score, _)) => (true, score),
                    None => (false, f32::NEG_INFINITY),
                },
            }
        };

        if !current_valid {
            self.switch_to(best, false);
            self.current_car_id = match best {
                DirectorTarget::Car(i) => Some(arena_state.cars[i].0.idx),
                DirectorTarget::Overview => None,
            };
        } else if best != self.current {
            let force = best_threat >= Self::FORCE_T
                && self.time_since_force >= Self::FORCE_CD
                && self.time_on_target >= Self::MIN_SHOT;
            if force {
                self.time_since_force = 0.0;
                self.switch_to(best, true);
                self.current_car_id = match best {
                    DirectorTarget::Car(i) => Some(arena_state.cars[i].0.idx),
                    DirectorTarget::Overview => None,
                };
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
                    self.current_car_id = match best {
                        DirectorTarget::Car(i) => Some(arena_state.cars[i].0.idx),
                        DirectorTarget::Overview => None,
                    };
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
    fn kickoff_requires_a_low_slow_ball_at_spawn_center() {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.ball.vel = Vec3A::ZERO;
        arena.ball.pos = Vec3A::new(0.0, 0.0, rocketsim::consts::ball::REST_Z);
        assert!(DirectorState::is_kickoff(&arena));
        arena.ball.pos.x = DirectorState::KICKOFF_MAX_DIST + 1.0;
        assert!(!DirectorState::is_kickoff(&arena));
        arena.ball.pos = Vec3A::new(0.0, 0.0, rocketsim::consts::ball::REST_Z + 100.0);
        assert!(!DirectorState::is_kickoff(&arena));
    }

    #[test]
    fn hoops_accepts_vertical_dunks_and_rising_balls_below_rim() {
        let mut arena = ArenaState::new_empty(GameMode::Hoops);
        // The projected location, not Y velocity, identifies the hoop side.
        arena.ball.pos = Vec3A::new(0.0, 3077.0, 1000.0);
        arena.ball.vel = Vec3A::ZERO;
        assert!(DirectorState::scoring_event(&arena).is_some());

        // A rising ball below ring height can still cross on its downward arc.
        arena.ball.pos.z = 100.0;
        arena.ball.vel.z = 1000.0;
        assert!(DirectorState::scoring_event(&arena).is_some());

        arena.ball.pos.x = 1500.0;
        assert!(DirectorState::scoring_event(&arena).is_none());
    }

    #[test]
    fn floor_rolling_and_bounced_soccar_shots_are_projected() {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.ball.pos = Vec3A::new(0.0, 3000.0, DirectorState::BALL_RADIUS);
        arena.ball.vel = Vec3A::new(0.0, 3000.0, 0.0);
        let rolling = DirectorState::scoring_event(&arena).unwrap();
        assert!(rolling.2 > 0.6 && rolling.2 < 0.8);

        // The descending ball bounces before reaching the goal plane.
        arena.ball.pos = Vec3A::new(0.0, 3000.0, 300.0);
        arena.ball.vel = Vec3A::new(0.0, 3000.0, -1000.0);
        let bounced = DirectorState::scoring_event(&arena).unwrap();
        assert!(bounced.2 > 0.6 && bounced.2 < 0.8);
        assert!(bounced.1.z > DirectorState::BALL_RADIUS);

        arena.ball.pos.x = 1200.0;
        assert!(DirectorState::scoring_event(&arena).is_none());
    }

    #[test]
    fn wide_and_high_projections_are_not_scoring_events() {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.ball.pos = Vec3A::new(1500.0, 3000.0, 500.0);
        arena.ball.vel = Vec3A::new(0.0, 3000.0, 0.0);
        assert!(DirectorState::scoring_event(&arena).is_none());
        arena.ball.pos = Vec3A::new(0.0, 3000.0, 1100.0);
        assert!(DirectorState::scoring_event(&arena).is_none());
    }

    #[test]
    fn shot_threat_takes_over() {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.cars.push(car_at(0, Team::Blue, 0.0, 2500.0));
        // Ball projected toward the Orange (+Y) goal.
        arena.ball.pos = Vec3A::new(0.0, 3000.0, 500.0);
        arena.ball.vel = Vec3A::new(0.0, 3000.0, 0.0);

        let mut director = DirectorState::new();
        let mut target = DirectorTarget::Overview;
        for _ in 0..130 {
            target = director.update(&arena, 1.0 / 30.0);
        }
        assert_eq!(target, DirectorTarget::Car(0));
    }

    #[test]
    fn shot_start_selects_relevant_player_and_stays_stable_after_event_ends() {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.cars.push(car_at(3, Team::Blue, 0.0, 2600.0));
        arena.ball.pos = Vec3A::new(0.0, 3000.0, 500.0);
        arena.ball.vel = Vec3A::new(0.0, 3000.0, 0.0);
        let mut director = DirectorState::new();
        for _ in 0..100 {
            director.update(&arena, 1.0 / 30.0);
        }
        assert_eq!(director.target(), DirectorTarget::Car(0));
        arena.ball.pos = Vec3A::new(2500.0, 3000.0, 500.0);
        assert!(DirectorState::scoring_event(&arena).is_none());
        let retained = director.update(&arena, 1.0 / 30.0);
        assert_eq!(retained, DirectorTarget::Car(0));
    }

    #[test]
    fn scoring_event_favors_relevant_attacker_over_distant_booster() {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        let mut shooter = car_at(3, Team::Blue, 0.0, 2600.0);
        shooter.1.vel = Vec3A::new(0.0, 1000.0, 0.0);
        let mut booster = car_at(8, Team::Blue, 3000.0, -3500.0);
        booster.1 = booster.1.with_tuning(rocketsim::CarTuning {
            is_boosting: true,
            ..booster.1.tuning()
        });
        arena.cars.extend([shooter, booster]);
        arena.ball.pos = Vec3A::new(0.0, 3000.0, 500.0);
        arena.ball.vel = Vec3A::new(0.0, 3000.0, 0.0);
        let mut director = DirectorState::new();
        for _ in 0..100 {
            director.update(&arena, 1.0 / 30.0);
        }
        assert_eq!(director.target(), DirectorTarget::Car(0));
    }

    #[test]
    fn plausible_defender_gets_save_view() {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.cars.push(car_at(0, Team::Blue, 0.0, 2500.0));
        arena.cars.push(car_at(1, Team::Orange, 0.0, 4800.0));
        arena.ball.pos = Vec3A::new(0.0, 3000.0, 500.0);
        arena.ball.vel = Vec3A::new(0.0, 3000.0, 0.0);
        let mut director = DirectorState::new();
        for _ in 0..100 {
            director.update(&arena, 1.0 / 30.0);
        }
        assert_eq!(director.target(), DirectorTarget::Car(1));
    }

    #[test]
    fn selected_player_identity_survives_reorder_and_removal() {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.cars.push(car_at(3, Team::Blue, 0.0, 2600.0));
        arena.cars.push(car_at(8, Team::Blue, 3000.0, -3500.0));
        arena.ball.pos = Vec3A::new(0.0, 3000.0, 500.0);
        arena.ball.vel = Vec3A::new(0.0, 3000.0, 0.0);
        let mut director = DirectorState::new();
        for _ in 0..100 {
            director.update(&arena, 1.0 / 30.0);
        }
        assert_eq!(director.target(), DirectorTarget::Car(0));
        arena.cars.swap(0, 1);
        assert_eq!(director.update(&arena, 1.0 / 30.0), DirectorTarget::Car(1));
        arena.cars.remove(1);
        assert_eq!(director.update(&arena, 1.0 / 30.0), DirectorTarget::Car(0));
    }

    #[test]
    fn unsupported_mode_has_no_soccar_shot_event() {
        let mut arena = ArenaState::new_empty(GameMode::Dropshot);
        arena.ball.pos = Vec3A::new(0.0, 3000.0, 500.0);
        arena.ball.vel = Vec3A::new(0.0, 3000.0, 0.0);
        assert!(DirectorState::scoring_event(&arena).is_none());
    }

    #[test]
    fn demoed_car_is_dropped() {
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.cars.push(car_at(0, Team::Blue, 0.0, 2500.0));
        arena.ball.pos = Vec3A::new(0.0, 3000.0, 500.0);
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

    #[test]
    fn aerial_play_wins_the_cut() {
        // High ball, one grounded car at 1211uu and one AIRBORNE car at
        // 583uu: proximity alone (0.72 vs overview+margin 0.808) would hold
        // Overview, but the aerial bonus (+0.25) cuts to the airborne car.
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.cars.push(car_at(0, Team::Blue, 0.0, -1000.0));
        arena.cars.push(car_at(1, Team::Orange, 0.0, 300.0));
        arena.cars[1].1.pos.z = 200.0;
        arena.cars[1].1.set_contact_flags(false, [false; 4]);
        arena.ball.pos = Vec3A::new(0.0, 0.0, 700.0);
        arena.ball.vel = Vec3A::ZERO;

        let mut director = DirectorState::new();
        let mut target = DirectorTarget::Overview;
        for _ in 0..150 {
            target = director.update(&arena, 1.0 / 30.0);
        }
        assert_eq!(target, DirectorTarget::Car(1));

        // Same geometry grounded: no bonus, Overview holds.
        let mut arena = ArenaState::new_empty(GameMode::Soccar);
        arena.cars.push(car_at(0, Team::Blue, 0.0, -1000.0));
        arena.cars.push(car_at(1, Team::Orange, 0.0, 300.0));
        arena.cars[1].1.pos.z = 200.0;
        arena.ball.pos = Vec3A::new(0.0, 0.0, 700.0);
        arena.ball.vel = Vec3A::ZERO;

        let mut director = DirectorState::new();
        let mut target = DirectorTarget::Overview;
        for _ in 0..150 {
            target = director.update(&arena, 1.0 / 30.0);
        }
        assert_eq!(target, DirectorTarget::Overview);
    }
}
