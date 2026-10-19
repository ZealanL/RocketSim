//! Cheap RL-style car effects: flip-reset glow, jump dust, dodge rings.
//!
//! Pure CPU state machine over snapshots: edge-detects jump/dodge/reset
//! per car and emits fading [`Segment`]s (particles + rings).
//!
//! Trigger rules (pinned by tests):
//! * reset flash + wheel glow: 0-2 -> 3+ wheels on ball/car surfaces, no
//!   wheel on the world. No airborne clause: 3+ contacts of any kind
//!   ground the car, so that clause made v1's glow dead in real play.
//! * glow latches until a wheel finds the WORLD (ball/car touchdowns keep
//!   it: the sim reports `is_on_ground` while parked on the ball).
//! * jump dust: 4 wheels x 6 streaks on the jump edge.
//! * dodge and double-jump share one orange ring; world touchdowns are quiet.
//!
//! RL match (researched Oct 2026): since v2.66 the official reset cue is a
//! quick white burst under the car with no held glow. The flash copies that
//! look; the latched wheel glow is a deliberate spectator deviation (never
//! on world contact: world touch unlatches the same tick).

use std::collections::VecDeque;

use glam::{Vec2, Vec3A};
use rocketsim::{ArenaState, CarBodyConfig, RaycastHitInfo, UserInfoType};

use crate::geometry::lines::{Color, Segment};

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

/// Minimum ball/car wheel contacts that grant the reset cue.
const RESET_WHEEL_THRESHOLD: u8 = 3;
/// Seconds between reset flashes per car (stops contact flicker strobing).
const FLASH_COOLDOWN: f32 = 1.0;

#[derive(Debug, Clone, Copy)]
struct CarBits {
    seen: bool,
    jumping: bool,
    flipping: bool,
    jump_time: f32,
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
            jump_time: 0.0,
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
        self.update_with_contacts(state, dt, |_, car| *car.wheels_with_contact())
    }

    fn update_with_contacts(
        &mut self,
        state: &ArenaState,
        dt: f32,
        mut contacts_for: impl FnMut(usize, &rocketsim::CarState) -> [Option<RaycastHitInfo>; 4],
    ) -> Vec<Segment> {
        let dt = dt.clamp(0.0, 1.0 / 30.0);
        // One segment per live particle plus a full ring per live ring;
        // latched wheel-glows grow past this (capacity only, never output).
        let mut segs = Vec::with_capacity(self.parts.len() + self.rings.len() * RING_SEGS);

        for (idx, (info, car)) in state.cars.iter().enumerate() {
            if self.prev.len() <= idx {
                self.prev.resize(idx + 1, CarBits::default());
            }
            let prev = self.prev[idx];
            let first_tick = !prev.seen;
            let contacts = contacts_for(idx, car);
            let touch = reset_touch_count(contacts);
            // Resets need ball/car surfaces only: any world wheel vetoes,
            // so grounded dribbling and landings stay quiet.
            let world_touch = contacts
                .into_iter()
                .flatten()
                .any(|hit| !matches!(hit.user_info, UserInfoType::Ball | UserInfoType::Car));
            let mut latched = prev.reset_latched;
            let mut flash_cd = (prev.flash_cd - dt).max(0.0);
            // Edge: 0-2 ball/car wheels -> 3+ (no airborne clause: it made
            // the trigger dead; the world veto + latch rules keep play quiet).
            let touch_edge = !first_tick
                && !world_touch
                && prev.touch_count < RESET_WHEEL_THRESHOLD
                && touch >= RESET_WHEEL_THRESHOLD
                && flash_cd <= 0.0;
            if touch_edge {
                latched = true;
                flash_cd = FLASH_COOLDOWN;
            }
            let flipped = !first_tick && car.is_flipping && !prev.flipping;
            let double_edge = !first_tick && car.has_double_jumped && !prev.double_jumped;
            // World touch ends the show. Ball/car touchdowns do NOT unlatch
            // (the sim grounds cars parked on the ball).
            if flipped || world_touch {
                latched = false;
            }
            let cur = CarBits {
                seen: true,
                jumping: car.is_jumping,
                flipping: car.is_flipping,
                jump_time: car.jump_time(),
                double_jumped: car.has_double_jumped,
                touch_count: touch,
                reset_latched: latched,
                flash_cd,
            };

            if !car.is_demoed {
                let wheels = wheel_world(car.pos, car.rot_mat, info.config, contacts);

                // Reset flash at the car's underside, car-oriented. Quick
                // burst like RL's own cue: tight, short, warm white.
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
                    && ((cur.jump_time == 0.0 && prev.jump_time != 0.0)
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

                // Double jump and dodge share the orange ring: around the
                // flip-torque axis for dodges, the horizontal plane else.
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

                // Latched reset: dim ring per wheel. Quiet on purpose: RL
                // shows only the burst, so the held cue stays dimmer.
                // Segments emit directly (no per-wheel point Vec): the
                // endpoints match the `windows(2)` expansion exactly.
                if latched {
                    let right = car.rot_mat.y_axis.try_normalize().unwrap_or(Vec3A::X);
                    let up = Vec3A::Z;
                    let color = RESET_FLASH.with_alpha(0.7);
                    for w in wheels {
                        let mut prev = w + right * 24.0;
                        for k in 1..=RING_SEGS {
                            let a = (k as f32 / RING_SEGS as f32) * std::f32::consts::TAU;
                            let cur = w + right * a.cos() * 24.0 + up * a.sin() * 24.0;
                            segs.push(Segment {
                                a: prev,
                                b: cur,
                                color_a: color,
                                color_b: color,
                                width_a: 2.5,
                                width_b: 2.5,
                            });
                            prev = cur;
                        }
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
    use rocketsim::{CarBodyConfig, CarCore, CarInfo, CarState, GameMode, Team};

    use super::*;

    fn state_with_car() -> ArenaState {
        let mut state = ArenaState::new_empty(GameMode::Soccar);
        let car = CarState::from(CarCore::DEFAULT.with_pos(Vec3A::new(0.0, 0.0, 100.0)));
        state.cars.push((
            CarInfo {
                idx: 0,
                team: Team::Blue,
                config: CarBodyConfig::OCTANE,
            },
            car,
        ));
        state
    }

    fn set_core(state: &mut ArenaState, core: CarCore) {
        let car = &mut state.cars[0].1;
        *car = CarState::from(core).with_tuning(car.tuning());
    }

    fn update_with_hits(
        fx: &mut CarFx,
        state: &ArenaState,
        hits: [Option<RaycastHitInfo>; 4],
    ) -> Vec<Segment> {
        fx.update_with_contacts(state, 1.0 / 120.0, |_, _| hits)
    }

    fn ball_hit() -> Option<RaycastHitInfo> {
        Some(RaycastHitInfo {
            user_info: UserInfoType::Ball,
            ..Default::default()
        })
    }

    fn world_hit() -> Option<RaycastHitInfo> {
        Some(RaycastHitInfo {
            user_info: UserInfoType::None,
            ..Default::default()
        })
    }

    fn is_reset_color(color: Color) -> bool {
        color.r > 0.95 && color.g > 0.9
    }

    const LATCHED_SEGS: usize = RING_SEGS * 5;

    #[test]
    fn quiet_when_parked() {
        let state = state_with_car();
        let mut fx = CarFx::default();
        assert!(fx.update(&state, 1.0 / 120.0).is_empty());
    }

    #[test]
    fn jump_edge_emits_dust() {
        let mut state = state_with_car();
        let mut fx = CarFx::default();
        assert!(fx.update(&state, 1.0 / 120.0).is_empty());

        let phys = state.cars[0].1.phys;
        set_core(&mut state, CarCore::jumping(phys, 1));
        state.cars[0].1.set_contact_flags(false, [false; 4]);
        assert_eq!(fx.update(&state, 1.0 / 120.0).len(), 24);
    }

    #[test]
    fn reset_fires_at_three_ball_or_car_wheels_and_latches() {
        let state = state_with_car();
        let mut fx = CarFx::default();
        let none = [None; 4];
        assert!(update_with_hits(&mut fx, &state, none).is_empty());

        let two = [ball_hit(), ball_hit(), None, None];
        assert!(update_with_hits(&mut fx, &state, two).is_empty());

        let ball_and_car = [
            ball_hit(),
            ball_hit(),
            Some(RaycastHitInfo {
                user_info: UserInfoType::Car,
                ..Default::default()
            }),
            None,
        ];
        assert_eq!(
            update_with_hits(&mut fx, &state, ball_and_car).len(),
            LATCHED_SEGS
        );
        assert_eq!(update_with_hits(&mut fx, &state, none).len(), LATCHED_SEGS);
    }

    #[test]
    fn world_touch_vetoes_reset_acquisition() {
        let state = state_with_car();
        let mut fx = CarFx::default();
        let hits = [ball_hit(), ball_hit(), ball_hit(), world_hit()];
        assert!(update_with_hits(&mut fx, &state, [None; 4]).is_empty());
        for _ in 0..5 {
            assert!(update_with_hits(&mut fx, &state, hits).is_empty());
        }
    }

    #[test]
    fn grounded_pickup_flashes_once_and_glow_survives_flash_expiry() {
        let state = state_with_car();
        let mut fx = CarFx::default();
        assert!(update_with_hits(&mut fx, &state, [None; 4]).is_empty());
        assert_eq!(
            update_with_hits(&mut fx, &state, [ball_hit(); 4]).len(),
            LATCHED_SEGS
        );

        let two = [ball_hit(), ball_hit(), None, None];
        assert_eq!(update_with_hits(&mut fx, &state, two).len(), LATCHED_SEGS);
        assert_eq!(
            update_with_hits(&mut fx, &state, [ball_hit(); 4]).len(),
            LATCHED_SEGS
        );
        for _ in 0..150 {
            update_with_hits(&mut fx, &state, [ball_hit(); 4]);
        }
        assert_eq!(
            update_with_hits(&mut fx, &state, [ball_hit(); 4]).len(),
            4 * RING_SEGS
        );
    }

    #[test]
    fn reset_rearms_after_cooldown_and_a_new_threshold_crossing() {
        let state = state_with_car();
        let mut fx = CarFx::default();
        update_with_hits(&mut fx, &state, [None; 4]);
        assert_eq!(
            update_with_hits(&mut fx, &state, [ball_hit(); 4]).len(),
            LATCHED_SEGS
        );
        update_with_hits(&mut fx, &state, [None; 4]);
        for _ in 0..125 {
            update_with_hits(&mut fx, &state, [None; 4]);
        }
        assert_eq!(
            update_with_hits(&mut fx, &state, [ball_hit(); 4]).len(),
            LATCHED_SEGS
        );
    }

    #[test]
    fn demo_suppresses_reset_effects() {
        let mut state = state_with_car();
        let core = state.cars[0].1.core();
        let mut demoed = core;
        demoed.is_demoed = true;
        set_core(&mut state, demoed);
        let mut fx = CarFx::default();
        assert!(update_with_hits(&mut fx, &state, [None; 4]).is_empty());
        assert!(update_with_hits(&mut fx, &state, [ball_hit(); 4]).is_empty());

        let mut active = state.cars[0].1.core();
        active.is_demoed = false;
        set_core(&mut state, active);
        assert_eq!(
            update_with_hits(&mut fx, &state, [ball_hit(); 4]).len(),
            4 * RING_SEGS,
            "the demoed acquisition latches glow but does not add a flash"
        );
    }

    #[test]
    fn air_tricks_share_one_ring() {
        for flip in [false, true] {
            let mut state = state_with_car();
            let phys = state.cars[0].1.phys;
            set_core(&mut state, CarCore::aerial(phys));
            state.cars[0].1.set_contact_flags(false, [false; 4]);
            let mut fx = CarFx::default();
            assert!(update_with_hits(&mut fx, &state, [None; 4]).is_empty());

            let core = if flip {
                CarCore::flipping(phys, Vec3A::Y)
            } else {
                let mut core = state.cars[0].1.core();
                core.has_double_jumped = true;
                core
            };
            set_core(&mut state, core);
            state.cars[0].1.set_contact_flags(false, [false; 4]);
            assert_eq!(
                update_with_hits(&mut fx, &state, [None; 4]).len(),
                RING_SEGS
            );
        }
    }

    #[test]
    fn world_landing_clears_latch_and_emits_no_reset_color() {
        let mut state = state_with_car();
        let mut fx = CarFx::default();
        update_with_hits(&mut fx, &state, [None; 4]);
        assert_eq!(
            update_with_hits(&mut fx, &state, [ball_hit(); 4]).len(),
            LATCHED_SEGS
        );
        for _ in 0..60 {
            update_with_hits(&mut fx, &state, [None; 4]);
        }
        assert_eq!(
            update_with_hits(&mut fx, &state, [None; 4]).len(),
            4 * RING_SEGS
        );

        state.cars[0].1.set_contact_flags(true, [true; 4]);
        let segments = update_with_hits(&mut fx, &state, [world_hit(); 4]);
        assert!(
            segments
                .iter()
                .all(|segment| !is_reset_color(segment.color_a))
        );
        for _ in 0..60 {
            let segments = update_with_hits(&mut fx, &state, [world_hit(); 4]);
            assert!(
                segments
                    .iter()
                    .all(|segment| !is_reset_color(segment.color_a))
            );
        }
    }

    #[test]
    fn ball_landing_emits_flash_and_glow_only() {
        let mut state = state_with_car();
        let mut fx = CarFx::default();
        update_with_hits(&mut fx, &state, [None; 4]);
        state.cars[0].1.set_contact_flags(true, [true; 4]);
        let segments = update_with_hits(&mut fx, &state, [ball_hit(); 4]);
        assert_eq!(segments.len(), LATCHED_SEGS);
        assert!(
            segments
                .iter()
                .all(|segment| is_reset_color(segment.color_a))
        );
    }

    #[test]
    fn flip_or_jump_availability_uses_next_tick_boundary() {
        let mut core = CarCore::aerial(CarCore::DEFAULT.phys);
        let mut state = CarState::from(core);
        state.set_contact_flags(false, [false; 4]);
        assert!(state.has_flip_or_jump());
        assert!(state.has_flip_reset());
        assert!(state.got_flip_reset());

        core.air_time_since_jump =
            rocketsim::consts::car::jump::DOUBLEJUMP_MAX_DELAY - rocketsim::consts::TICK_TIME * 1.5;
        let mut state = CarState::from(core);
        state.set_contact_flags(false, [false; 4]);
        assert!(state.has_flip_or_jump());
        core.air_time_since_jump =
            rocketsim::consts::car::jump::DOUBLEJUMP_MAX_DELAY - rocketsim::consts::TICK_TIME;
        let mut state = CarState::from(core);
        state.set_contact_flags(false, [false; 4]);
        assert!(!state.has_flip_or_jump());
    }

    #[test]
    fn jump_availability_ends_after_jump() {
        let mut core = CarCore::jumping(CarCore::DEFAULT.phys, 1);
        core.air_time_since_jump = rocketsim::consts::car::jump::DOUBLEJUMP_MAX_DELAY;
        let mut state = CarState::from(core);
        state.set_contact_flags(false, [false; 4]);
        assert!(!state.has_flip_or_jump());
        assert!(!state.has_flip_reset());
        assert!(!state.got_flip_reset());
    }
}
