//! Cheap RL-style car effects: flip-reset glow, jump dust, dodge
//! shockwaves.
//!
//! Pure CPU state machine over [`ArenaState`] snapshots: edge-detects
//! jump/dodge/reset events per car and emits fading [`Segment`]s
//! (particles + expanding rings) for the line pass.
//!
//! Trigger contract (pinned by tests):
//! * reset flash + wheel glow: 0-2 → 3+ wheels on ball/car surfaces with
//!   ZERO wheels on the world. The v1 code additionally required airborne,
//!   but that clause is UNSATISFIABLE in real physics — 3+ contacts of any
//!   kind grounds the car (`is_on_ground`), so it could only ever fire in
//!   synthetic states (v1's glow was dead in real gameplay). v2 fires on
//!   acquisition instead; sustained dribbling stays quiet anyway (edge needs
//!   prev<3, and world contact unlatches), and a 1s per-car cooldown stops
//!   contact flicker from strobing.
//! * wheel glow latches until a wheel finds the WORLD (or a flip starts).
//!   Ball/car touchdowns keep the glow: the reset was just earned, and the
//!   sim really does report `is_on_ground` while parked on the ball.
//! * jump dust: 4 wheels × 6 streaks on the jump edge.
//! * dodge and double-jump share one orange shockwave ring.
//! * world touchdowns emit nothing: RL lands with dust puffs, not rings,
//!   and a floor ring next to the reset flash read as a reset (removed).
//!
//! Match to the real game (researched Oct 2026): since RL v2.66 (Season 22)
//! the official flip-reset indicator is a quick white holographic circle
//! burst under the car + audio tick when 3+ wheels touch the ball, shown
//! only as a momentary burst with no held glow. Our flash copies that look
//! (warm-white tight burst under the car). The latched wheel glow is a
//! deliberate deviation — it lets spectators see a banked reset — and it
//! can never appear on world contact (world touch unlatches it the same
//! tick).

use std::collections::VecDeque;

use glam::{Vec2, Vec3A};
use rocketsim::{ArenaState, CarBodyConfig, RaycastHitInfo, UserInfoType};

use crate::lines::{Color, Segment};

/// Particle cap (oldest evicted past it).
const MAX_PARTICLES: usize = 512;
/// Points per ring circle (segments = `RING_SEGS`).
const RING_SEGS: usize = 24;

/// Reset flash/glow: warm white toward RL's white holographic reset
/// burst (v2.66). Kept slightly warm so it never reads as the
/// neutral-gray landing ring in this renderer.
const RESET_FLASH: Color = Color::new(1.0, 0.96, 0.86, 1.0);
/// Dodge/double-jump shockwave orange.
const DODGE_ORANGE: Color = Color::new(1.0, 0.6, 0.2, 1.0);
/// Jump dust / landing ring gray.
const DUST_GRAY: Color = Color::new(0.75, 0.75, 0.75, 1.0);

/// Seconds between reset flashes per car (stops contact flicker strobing).
const FLASH_COOLDOWN: f32 = 1.0;

#[derive(Debug, Clone, Copy)]
struct CarBits {
    seen: bool,
    jumping: bool,
    flipping: bool,
    jump_ticks: u32,
    double_jumped: bool,
    /// Wheels currently touching the ball or another car.
    touch_count: u8,
    /// Reset glow latched: set on the touch edge, cleared on landing/flip.
    reset_latched: bool,
    /// Time until another reset flash may fire.
    flash_cd: f32,
}

impl Default for CarBits {
    fn default() -> Self {
        Self {
            seen: false,
            jumping: false,
            flipping: false,
            jump_ticks: 0,
            double_jumped: false,
            touch_count: 0,
            reset_latched: false,
            flash_cd: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Particle {
    pos: Vec3A,
    vel: Vec3A,
    age: f32,
    life: f32,
    color: Color,
    width: f32,
}

#[derive(Debug, Clone, Copy)]
struct Ring {
    center: Vec3A,
    u: Vec3A,
    v: Vec3A,
    r0: f32,
    r1: f32,
    age: f32,
    life: f32,
    color: Color,
    width: f32,
}

/// Per-car jump/dodge/reset effects. Feed every tick; emits segments.
#[derive(Debug, Default)]
pub struct CarFx {
    prev: Vec<CarBits>,
    parts: VecDeque<Particle>,
    rings: Vec<Ring>,
}

/// Wheels touching the ball or another car (reset-granting contacts).
fn reset_touch_count(contacts: [Option<RaycastHitInfo>; 4]) -> u8 {
    let mut n = 0;
    for hit in contacts.into_iter().flatten() {
        if matches!(hit.user_info, UserInfoType::Ball | UserInfoType::Car) {
            n += 1;
        }
    }
    n
}

/// Four wheel world positions: raycast hit when in contact, else estimated
/// from the suspension connection points.
fn wheel_world(
    car_pos: Vec3A,
    rot: glam::Mat3A,
    config: CarBodyConfig,
    contacts: [Option<RaycastHitInfo>; 4],
) -> [Vec3A; 4] {
    let pairs = [config.front_wheels, config.back_wheels];
    let mut out = [Vec3A::ZERO; 4];
    for (pair_idx, pair) in pairs.iter().enumerate() {
        for side in 0..2 {
            let i = pair_idx * 2 + side;
            if let Some(hit) = contacts[i] {
                out[i] = hit.hit_point;
                continue;
            }
            // Suspension hangs down (-Z); wheel center sits below the mount.
            let y_sign = if side == 0 { 1.0 } else { -1.0 };
            let local = Vec3A::new(
                pair.connection_point_offset.x,
                pair.connection_point_offset.y * y_sign,
                pair.connection_point_offset.z - pair.suspension_rest_length * 0.6,
            );
            out[i] = car_pos + rot.mul_vec3a(local);
        }
    }
    out
}

/// Expands a ring into [`RING_SEGS`] segments (consecutive circle points).
fn ring_segments(ring: &Ring) -> Vec<Segment> {
    let t = ring.age / ring.life;
    let radius = ring.r0 + (ring.r1 - ring.r0) * t;
    let fade = 1.0 - t;
    let color = ring.color.with_alpha(ring.color.a * fade);
    let width = ring.width * fade;
    let mut pts = Vec::with_capacity(RING_SEGS + 1);
    for k in 0..=RING_SEGS {
        let a = (k as f32 / RING_SEGS as f32) * std::f32::consts::TAU;
        pts.push(ring.center + ring.u * a.cos() * radius + ring.v * a.sin() * radius);
    }
    pts.windows(2)
        .map(|w| Segment {
            a: w[0],
            b: w[1],
            color_a: color,
            color_b: color,
            width_a: width,
            width_b: width,
        })
        .collect()
}

impl CarFx {
    /// Advances effects by `dt` (clamped to `[0, 1/30]`) and returns fresh
    /// line segments.
    pub fn update(&mut self, state: &ArenaState, dt: f32) -> Vec<Segment> {
        let dt = dt.clamp(0.0, 1.0 / 30.0);
        let mut segs = Vec::new();

        for (idx, (info, car)) in state.cars.iter().enumerate() {
            if self.prev.len() <= idx {
                self.prev.resize(idx + 1, CarBits::default());
            }
            let prev = self.prev[idx];
            let first_tick = !prev.seen;
            let touch = reset_touch_count(car.wheels_with_contact);
            // Resets are airborne events on ball/car surfaces: any wheel on
            // the world (static geometry, tiles) vetoes the trigger, so
            // grounded dribbling and world landings stay quiet.
            let world_touch = car
                .wheels_with_contact
                .into_iter()
                .flatten()
                .any(|hit| !matches!(hit.user_info, UserInfoType::Ball | UserInfoType::Car));
            let mut latched = prev.reset_latched;
            let mut flash_cd = (prev.flash_cd - dt).max(0.0);
            // Acquisition edge: 0-2 ball/car wheels → 3+. No airborne
            // clause (see module docs: it made the trigger dead). The
            // world veto + latch rules keep grounded play quiet.
            let touch_edge = !first_tick
                && !world_touch
                && prev.touch_count < 3
                && touch >= 3
                && flash_cd <= 0.0;
            if touch_edge {
                latched = true;
                flash_cd = FLASH_COOLDOWN;
            }
            let flipped = !first_tick && car.is_flipping && !prev.flipping;
            let double_edge = !first_tick && car.has_double_jumped && !prev.double_jumped;
            // Touching the world ends the show (pinches, scrapes,
            // touchdowns). Ball/car touchdowns do NOT unlatch: the sim sets
            // `is_on_ground` while parked on the ball, and the reset was
            // just earned, so the glow rides along until real ground/flip.
            if flipped || world_touch {
                latched = false;
            }
            let cur = CarBits {
                seen: true,
                jumping: car.is_jumping,
                flipping: car.is_flipping,
                jump_ticks: car.jump_ticks,
                double_jumped: car.has_double_jumped,
                touch_count: touch,
                reset_latched: latched,
                flash_cd,
            };

            if !car.is_demoed {
                let wheels =
                    wheel_world(car.pos, car.rot_mat, info.config, car.wheels_with_contact);

                // Flip-reset pickup flash at the car's underside, oriented to
                // the car (not the world). A quick burst like RL's own
                // indicator: tight, short-lived, warm white.
                if touch_edge {
                    let fwd = car
                        .get_forward_dir()
                        .truncate()
                        .try_normalize()
                        .unwrap_or(Vec2::X)
                        .extend(0.0)
                        .to_vec3a();
                    let side = fwd.cross(Vec3A::Z).normalize_or_zero();
                    let bottom = car.pos + info.config.hitbox_pos_offset
                        - Vec3A::new(0.0, 0.0, info.config.hitbox_size.z * 0.5);
                    self.rings.push(Ring {
                        center: bottom,
                        u: fwd,
                        v: if side.length_squared() > 1e-6 {
                            side
                        } else {
                            Vec3A::Y
                        },
                        r0: 50.0,
                        r1: 150.0,
                        age: 0.0,
                        life: 0.35,
                        color: RESET_FLASH,
                        width: 5.0,
                    });
                }

                // Jump dust at the wheels: 4 × 6 streaks.
                let jumped = !first_tick
                    && ((cur.jump_ticks == 0 && prev.jump_ticks != 0)
                        || (cur.jumping && !prev.jumping));
                if jumped {
                    for w in wheels {
                        for k in 0..6 {
                            let a = (k as f32 / 6.0) * std::f32::consts::TAU;
                            let vel = Vec3A::new(a.cos() * 160.0, a.sin() * 160.0, 120.0);
                            self.push_part(w, vel, 0.35, DUST_GRAY.with_alpha(0.8), 3.0);
                        }
                    }
                }

                // Double jump and dodge share one effect: the dodge-style
                // orange shockwave. Dodges fan it around the flip-torque
                // axis; double jumps (no torque) use the car's horizontal
                // plane instead.
                if double_edge || flipped {
                    let (u, v) = if flipped {
                        let axis = if car.flip_rel_torque.length_squared() > 1e-6 {
                            car.flip_rel_torque.normalize()
                        } else {
                            Vec3A::Z
                        };
                        let helper = if axis.z.abs() < 0.9 {
                            Vec3A::Z
                        } else {
                            Vec3A::X
                        };
                        let u = axis.cross(helper).normalize_or_zero();
                        (u, axis.cross(u).normalize_or_zero())
                    } else {
                        let fwd = car
                            .get_forward_dir()
                            .truncate()
                            .try_normalize()
                            .unwrap_or(Vec2::X)
                            .extend(0.0)
                            .to_vec3a();
                        (fwd, fwd.cross(Vec3A::Z).normalize_or_zero())
                    };
                    self.rings.push(Ring {
                        center: car.pos,
                        u,
                        v,
                        r0: 50.0,
                        r1: 220.0,
                        age: 0.0,
                        life: 0.45,
                        color: DODGE_ORANGE,
                        width: 5.0,
                    });
                }

                // Latched reset: dim glowing ring around each wheel (axle =
                // car Y). Quiet on purpose — RL shows only the momentary
                // burst, and this held indicator must never outshine it.
                if latched {
                    let right = car.rot_mat.y_axis.try_normalize().unwrap_or(Vec3A::X);
                    let up = Vec3A::Z;
                    for w in wheels {
                        let mut pts = Vec::with_capacity(RING_SEGS + 1);
                        for k in 0..=RING_SEGS {
                            let a = (k as f32 / RING_SEGS as f32) * std::f32::consts::TAU;
                            pts.push(w + right * a.cos() * 24.0 + up * a.sin() * 24.0);
                        }
                        let color = RESET_FLASH.with_alpha(0.7);
                        segs.extend(pts.windows(2).map(|pair| Segment {
                            a: pair[0],
                            b: pair[1],
                            color_a: color,
                            color_b: color,
                            width_a: 2.5,
                            width_b: 2.5,
                        }));
                    }
                }
            }

            self.prev[idx] = cur;
        }

        // Integrate particles -> short motion streaks.
        for p in self.parts.iter_mut() {
            p.age += dt;
            p.pos += p.vel * dt;
            p.vel *= 1.0 - 2.0 * dt;
            p.vel.z -= 300.0 * dt;
        }
        self.parts.retain(|p| p.age < p.life);
        for p in &self.parts {
            let t = 1.0 - p.age / p.life;
            segs.push(Segment {
                a: p.pos,
                b: p.pos - p.vel * 0.03,
                color_a: p.color.with_alpha(p.color.a * t),
                color_b: p.color.with_alpha(0.0),
                width_a: p.width * t,
                width_b: 0.0,
            });
        }

        // Expand rings -> circle segments.
        for r in self.rings.iter_mut() {
            r.age += dt;
        }
        self.rings.retain(|r| r.age < r.life);
        for r in &self.rings {
            segs.extend(ring_segments(r));
        }

        segs
    }

    fn push_part(&mut self, pos: Vec3A, vel: Vec3A, life: f32, color: Color, width: f32) {
        if self.parts.len() >= MAX_PARTICLES {
            self.parts.pop_front();
        }
        self.parts.push_back(Particle {
            pos,
            vel,
            age: 0.0,
            life,
            color,
            width,
        });
    }
}

#[cfg(test)]
mod tests {
    use rocketsim::{CarBodyConfig, CarInfo, CarState, GameMode, Team};

    use super::*;

    fn state_with_car() -> ArenaState {
        let mut s = ArenaState::new_empty(GameMode::Soccar);
        let mut car = CarState::DEFAULT;
        car.pos = Vec3A::new(0.0, 0.0, 100.0);
        s.cars.push((
            CarInfo {
                idx: 0,
                team: Team::Blue,
                config: CarBodyConfig::OCTANE,
            },
            car,
        ));
        s
    }

    /// Flash ring (24 segs) + 4 wheel rings (4×24): the latched look.
    const LATCHED_SEGS: usize = 24 + 4 * 24;

    #[test]
    fn quiet_when_parked() {
        let s = state_with_car();
        let mut fx = CarFx::default();
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());
    }

    #[test]
    fn jump_edge_emits_dust() {
        let mut s = state_with_car();
        let mut fx = CarFx::default();
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());

        s.cars[0].1.is_jumping = true;
        s.cars[0].1.is_on_ground = false;
        let segs = fx.update(&s, 1.0 / 120.0);
        assert!(!segs.is_empty(), "jump edge must emit dust");
        // 4 wheels x 6 particles, each a 2-point streak.
        assert_eq!(segs.len(), 24);
    }

    #[test]
    fn reset_fires_on_three_wheel_ball_touch() {
        let mut s = state_with_car();
        s.cars[0].1.is_on_ground = false;
        let mut fx = CarFx::default();
        // First sight: no edges even with contacts present.
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());

        // Two wheels on the ball: below threshold, still quiet.
        let ball_hit = RaycastHitInfo {
            user_info: UserInfoType::Ball,
            ..Default::default()
        };
        s.cars[0].1.wheels_with_contact[0] = Some(ball_hit);
        s.cars[0].1.wheels_with_contact[1] = Some(ball_hit);
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());

        // Third wheel (on a car — also counts): flash + 4 wheel rings.
        let car_hit = RaycastHitInfo {
            user_info: UserInfoType::Car,
            ..Default::default()
        };
        s.cars[0].1.wheels_with_contact[2] = Some(car_hit);
        assert_eq!(fx.update(&s, 1.0 / 120.0).len(), LATCHED_SEGS);

        // Wheels leave: latched glow persists without new edges.
        s.cars[0].1.wheels_with_contact = [None; 4];
        assert_eq!(fx.update(&s, 1.0 / 120.0).len(), LATCHED_SEGS);

        // Landing spends the reset: flash aged out, glow gone.
        // Real field contacts report `UserInfoType::None` (ground probe),
        // so the touchdown carries world wheels — that world touch is what
        // unlatches the glow. Touchdowns emit no ring of their own.
        let world_hit = RaycastHitInfo {
            user_info: UserInfoType::None,
            ..Default::default()
        };
        s.cars[0].1.wheels_with_contact = [Some(world_hit); 4];
        s.cars[0].1.is_on_ground = true;
        // The touchdown tick itself: glow unlatched same-tick, so only the
        // aging pickup flash (24) remains — no new ring of any kind.
        assert_eq!(fx.update(&s, 1.0 / 120.0).len(), RING_SEGS);
        for _ in 0..120 {
            fx.update(&s, 1.0 / 120.0);
        }
        // Flash lifetime (0.35s) expired over the full second. Glow
        // unlatched on touchdown.
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());
    }

    #[test]
    fn world_touch_vetoes_reset() {
        // Airborne, 3 wheels on the ball, but the 4th scrapes the world:
        // no reset flash, no glow.
        let mut s = state_with_car();
        s.cars[0].1.is_on_ground = false;
        let ball_hit = RaycastHitInfo {
            user_info: UserInfoType::Ball,
            ..Default::default()
        };
        let world_hit = RaycastHitInfo {
            user_info: UserInfoType::None,
            ..Default::default()
        };
        s.cars[0].1.wheels_with_contact = [
            Some(ball_hit),
            Some(ball_hit),
            Some(ball_hit),
            Some(world_hit),
        ];
        let mut fx = CarFx::default();
        for _ in 0..5 {
            assert!(fx.update(&s, 1.0 / 120.0).is_empty());
        }
    }

    #[test]
    fn grounded_pickup_flashes_once_then_quiet() {
        // Grounded ball acquisition still flashes (v1's airborne clause made
        // this dead in real physics); sustained contact stays quiet; flicker
        // inside the cooldown adds no second ring (would read 144).
        let mut s = state_with_car();
        let ball_hit = RaycastHitInfo {
            user_info: UserInfoType::Ball,
            ..Default::default()
        };
        let mut fx = CarFx::default();
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());

        // 0 -> 4 grounded: flash (24) + glow (96).
        s.cars[0].1.wheels_with_contact = [Some(ball_hit); 4];
        assert_eq!(fx.update(&s, 1.0 / 120.0).len(), LATCHED_SEGS);

        // Flicker 4 -> 2 -> 4 inside the cooldown: still exactly one ring.
        s.cars[0].1.wheels_with_contact = [Some(ball_hit), Some(ball_hit), None, None];
        assert_eq!(fx.update(&s, 1.0 / 120.0).len(), LATCHED_SEGS);
        s.cars[0].1.wheels_with_contact = [Some(ball_hit); 4];
        assert_eq!(fx.update(&s, 1.0 / 120.0).len(), LATCHED_SEGS);

        // Sustain past flash expiry (0.35s): glow alone remains.
        for _ in 0..150 {
            fx.update(&s, 1.0 / 120.0);
        }
        assert_eq!(fx.update(&s, 1.0 / 120.0).len(), LATCHED_SEGS - RING_SEGS);
    }

    #[test]
    fn double_jump_emits_ring() {
        let mut s = state_with_car();
        s.cars[0].1.is_on_ground = false;
        let mut fx = CarFx::default();
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());

        s.cars[0].1.has_double_jumped = true;
        // One shockwave ring = 24 segments.
        assert_eq!(fx.update(&s, 1.0 / 120.0).len(), RING_SEGS);
    }

    #[test]
    fn dodge_emits_ring() {
        let mut s = state_with_car();
        s.cars[0].1.is_on_ground = false;
        let mut fx = CarFx::default();
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());

        s.cars[0].1.is_flipping = true;
        assert_eq!(fx.update(&s, 1.0 / 120.0).len(), RING_SEGS);
    }

    #[test]
    fn world_touchdown_is_quiet() {
        // Touchdown on the world (field contacts report
        // `UserInfoType::None` in the real sim) emits nothing: no floor
        // ring, and nothing reset-colored.
        let mut s = state_with_car();
        s.cars[0].1.is_on_ground = false;
        let mut fx = CarFx::default();
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());

        let world_hit = RaycastHitInfo {
            user_info: UserInfoType::None,
            ..Default::default()
        };
        s.cars[0].1.wheels_with_contact = [Some(world_hit); 4];
        s.cars[0].1.is_on_ground = true;
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());
        // Settled: still nothing.
        for _ in 0..60 {
            assert!(fx.update(&s, 1.0 / 120.0).is_empty());
        }
    }

    /// Warm-white reset channels; discriminates the reset flash/glow from
    /// jump dust gray (0.75) and the orange dodge ring (g = 0.6).
    fn is_reset_color(c: Color) -> bool {
        c.r > 0.95 && c.g > 0.9
    }

    #[test]
    fn ground_touchdown_clears_latched_glow_same_tick() {
        // Regression: the reset FX must never play on a ground landing.
        // Latch the glow via a ball touch, let the pickup flash expire
        // mid-air, then touch down on the world: that tick emits nothing
        // and the glow clears, and no reset color ever returns.
        let mut s = state_with_car();
        s.cars[0].1.is_on_ground = false;
        let mut fx = CarFx::default();
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());

        let ball_hit = RaycastHitInfo {
            user_info: UserInfoType::Ball,
            ..Default::default()
        };
        s.cars[0].1.wheels_with_contact = [Some(ball_hit); 4];
        assert_eq!(fx.update(&s, 1.0 / 120.0).len(), LATCHED_SEGS);

        // Wheels leave; glow rides along while the flash (0.35s) dies.
        s.cars[0].1.wheels_with_contact = [None; 4];
        for _ in 0..60 {
            fx.update(&s, 1.0 / 120.0);
        }
        assert_eq!(
            fx.update(&s, 1.0 / 120.0).len(),
            LATCHED_SEGS - RING_SEGS,
            "latched glow outlives the flash mid-air"
        );

        // World touchdown: nothing emitted, glow cleared this same tick.
        let world_hit = RaycastHitInfo {
            user_info: UserInfoType::None,
            ..Default::default()
        };
        s.cars[0].1.wheels_with_contact = [Some(world_hit); 4];
        s.cars[0].1.is_on_ground = true;
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());

        // Settled: still nothing, reset colors never come back.
        for _ in 0..60 {
            let segs = fx.update(&s, 1.0 / 120.0);
            assert!(segs.iter().all(|sg| !is_reset_color(sg.color_a)));
        }
    }

    #[test]
    fn ball_touchdown_emits_flash_and_glow_only() {
        // Landing on the BALL also sets `is_on_ground` (3+ contacts of any
        // kind ground the car — real-sim probe); acquisition emits flash +
        // glow only, nothing else.
        let mut s = state_with_car();
        s.cars[0].1.is_on_ground = false;
        let mut fx = CarFx::default();
        assert!(fx.update(&s, 1.0 / 120.0).is_empty());

        let ball_hit = RaycastHitInfo {
            user_info: UserInfoType::Ball,
            ..Default::default()
        };
        s.cars[0].1.wheels_with_contact = [Some(ball_hit); 4];
        s.cars[0].1.is_on_ground = true;
        let segs = fx.update(&s, 1.0 / 120.0);
        // Flash ring (24) + 4 wheel-glow rings (96).
        assert_eq!(segs.len(), LATCHED_SEGS);
        assert!(
            segs.iter().all(|sg| is_reset_color(sg.color_a)),
            "ball touchdown must be reset colors only"
        );
    }
}
