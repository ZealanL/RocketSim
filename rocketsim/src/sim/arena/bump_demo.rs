use std::mem;

use glam::{Mat3A, Vec3A};

use super::Arena;
use crate::{
    CarBodyConfig, CarState, DemoMode,
    bullet::collision::narrowphase::manifold_point::ManifoldPoint,
    consts::{self, BT_TO_UU, TICK_TIME, UU_TO_BT},
    sim::{ArenaEvent, CarHitCarEvent},
};

/// Keep a vector whose squared length is already 1 bit-exact,
/// and use a zero vector below the small guard.
fn normalize_unless_one(v: Vec3A) -> Vec3A {
    let len_sq = v.length_squared();
    if len_sq != 1.0 {
        if len_sq < 1e-8 {
            Vec3A::ZERO
        } else {
            v * (1.0 / len_sq.sqrt())
        }
    } else {
        v
    }
}

fn hitbox_center(state: &CarState, config: &CarBodyConfig) -> Vec3A {
    let rot_mat = state.phys.rot_mat;
    let offset = config.hitbox_pos_offset;
    state.phys.pos + rot_mat.transpose_mul_vec3a(offset)
}

/// The victim's hitbox center direction is measured in the attacker's frame.
/// Each angle removes the other axis component scaled by 1/1.01 (1.01 when negative).
///
/// Pitch is the angle off the forward/up plane;
/// yaw is the angle off the forward/right plane.
/// The caller supplies both limits so the bump and demo gates cannot drift apart.
fn is_within_forward_cone(
    attacker: &CarState,
    attacker_config: &CarBodyConfig,
    victim: &CarState,
    victim_config: &CarBodyConfig,
    reverse: bool,
    max_yaw_deg: f32,
    max_pitch_deg: f32,
) -> bool {
    let fwd = attacker.phys.get_forward_dir();
    let delta = normalize_unless_one(
        hitbox_center(victim, victim_config) - hitbox_center(attacker, attacker_config),
    );
    let fwd = if reverse { -fwd } else { fwd };

    let angle_deg = |axis: Vec3A| {
        let mut k = delta.dot(axis);
        k = if k >= 0.0 { k / 1.01 } else { k * 1.01 };
        let projected = normalize_unless_one(delta - axis * k);
        fwd.dot(projected).clamp(-1.0, 1.0).acos() * 57.29578
    };

    // Pitch first: the PR helper rejects on pitch before it reads yaw.
    if angle_deg(attacker.phys.get_right_dir()) > max_pitch_deg {
        return false;
    }
    angle_deg(attacker.phys.get_up_dir()) <= max_yaw_deg
}

/// Result of the same-tick demolition gate for one contact direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DemoGate {
    /// Neither car is demolished. The contact may demolish or bump.
    Normal,
    /// The contact must be ignored.
    Skip,
    /// The attacker was demolished earlier in this tick by this victim. It may
    /// still demolish the victim, but it must not bump it.
    DemoOnly,
}

impl DemoGate {
    /// Only an undemolished pair may bump.
    const fn may_bump(self) -> bool {
        matches!(self, Self::Normal)
    }
}

/// The same-tick demolition gate.
fn demo_gate(
    attacker_demoed: bool,
    victim_demoed: bool,
    attacker_demoed_by_victim_this_tick: bool,
) -> DemoGate {
    if !(attacker_demoed || victim_demoed) {
        DemoGate::Normal
    } else if victim_demoed || !attacker_demoed_by_victim_this_tick {
        DemoGate::Skip
    } else {
        DemoGate::DemoOnly
    }
}

/// Integrate a car pose over `t` seconds with a constant velocity and
/// angular velocity.
fn integrate_pose(state: &mut CarState, vel: Vec3A, t: f32) {
    state.phys.pos += vel * t;

    let ang_vel = state.phys.ang_vel;
    let ang_speed = ang_vel.length();
    let theta = ang_speed * t;
    if ang_speed > 0.0 && theta != 0.0 {
        let axis = ang_vel / ang_speed;
        let c = theta.cos();
        let s = theta.sin();
        let rot = |x: Vec3A| x * c + axis.cross(x) * s + axis * (axis.dot(x) * (1.0 - c));
        let rot_mat = state.phys.rot_mat;
        state.phys.rot_mat = Mat3A::from_cols(
            rot(rot_mat.x_axis),
            rot(rot_mat.y_axis),
            rot(rot_mat.z_axis),
        );
    }
}

/// Separating-axis depth of two oriented hitboxes, in uu.
/// Positive values are separated;
/// the depth is the largest gap over the 15 candidate axes.
fn hitbox_separation(
    a: &CarState,
    a_config: &CarBodyConfig,
    b: &CarState,
    b_config: &CarBodyConfig,
) -> f32 {
    let a_rot = a.phys.rot_mat;
    let b_rot = b.phys.rot_mat;
    let a_half = a_config.hitbox_size * 0.5;
    let b_half = b_config.hitbox_size * 0.5;

    let a_center = a.phys.pos + a_rot * a_config.hitbox_pos_offset;
    let b_center = b.phys.pos + b_rot * b_config.hitbox_pos_offset;
    let delta = b_center - a_center;

    let mut best = -1e30f32;
    let mut test = |axis: Vec3A| {
        // Reach of each box along `axis`:
        // the row-dot projections onto its own axes,
        // scaled by the half extents.
        let a_reach = a_rot.transpose_mul_vec3a(axis).abs();
        let b_reach = b_rot.transpose_mul_vec3a(axis).abs();

        let mut depth = delta.dot(axis).abs();
        for i in 0..3 {
            depth -= a_reach[i] * a_half[i] + b_reach[i] * b_half[i];
        }

        best = best.max(depth);
    };

    let a_cols = [a_rot.x_axis, a_rot.y_axis, a_rot.z_axis];
    let b_cols = [b_rot.x_axis, b_rot.y_axis, b_rot.z_axis];
    for col in a_cols.into_iter().chain(b_cols) {
        test(col);
    }

    for a_col in a_cols {
        for b_col in b_cols {
            let axis = a_col.cross(b_col);
            let len = axis.length();
            if len > 1e-6 {
                test(axis / len);
            }
        }
    }

    best
}

/// Replace both poses with the time-of-impact poses that the game uses for
/// the demolition cone.
///
/// Both hitboxes are swept from one tick before to one tick after their
/// start-of-tick poses. The hit is where they overlap by the allowed
/// penetration. With no hit, the start-of-tick poses stay.
fn get_time_of_impact_poses(
    a: &mut CarState,
    a_config: &CarBodyConfig,
    a_vel: Vec3A,
    b: &mut CarState,
    b_config: &CarBodyConfig,
    b_vel: Vec3A,
    tick_time: f32,
) {
    /// Allowed penetration 0.04 BT minus btContinuousConvexCollision's 0.001 BT.
    const PENETRATION: f32 = 0.039 * 50.0;

    let separation = |lambda: f32| {
        let t = (2.0 * lambda - 1.0) * tick_time;
        let mut ai = *a;
        let mut bi = *b;
        integrate_pose(&mut ai, a_vel, t);
        integrate_pose(&mut bi, b_vel, t);
        hitbox_separation(&ai, a_config, &bi, b_config) + PENETRATION
    };

    let lambda = if separation(0.0) <= 0.0 {
        0.0
    } else if separation(1.0) > 0.0 {
        return;
    } else {
        let mut lo = 0.0f32;
        let mut hi = 1.0f32;
        for _ in 0..24 {
            let mid = (lo + hi) * 0.5;
            if separation(mid) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        hi
    };

    let t = (2.0 * lambda - 1.0) * tick_time;
    integrate_pose(a, a_vel, t);
    integrate_pose(b, b_vel, t);
}

fn should_demolish(
    attacker: &CarState,
    attacker_config: &CarBodyConfig,
    victim: &CarState,
    victim_config: &CarBodyConfig,
    tick_time: f32,
) -> bool {
    use consts::car::{demo, supersonic};

    if !attacker.is_supersonic {
        return false;
    }

    let start_vel = attacker.phys.vel;
    let forward_speed = start_vel.dot(attacker.phys.get_forward_dir());
    if forward_speed.abs() < supersonic::MAINTAIN_MIN_SPEED {
        return false;
    }

    let mut attacker_pose = *attacker;
    let mut victim_pose = *victim;
    get_time_of_impact_poses(
        &mut attacker_pose,
        attacker_config,
        start_vel,
        &mut victim_pose,
        victim_config,
        victim.phys.vel,
        tick_time,
    );

    is_within_forward_cone(
        &attacker_pose,
        attacker_config,
        &victim_pose,
        victim_config,
        forward_speed < 0.0,
        demo::YAW_LIMIT_DEG,
        demo::PITCH_LIMIT_DEG,
    )
}

impl Arena {
    pub(super) fn on_car_car_collision(
        &mut self,
        car_1_idx: usize,
        car_2_idx: usize,
        manifold_point: &ManifoldPoint,
    ) {
        let Ok(both_cars) = self.cars.get_disjoint_mut([car_1_idx, car_2_idx]) else {
            panic!(
                "on_car_car_collision() called with invalid or duplicate car indices: {car_1_idx}, {car_2_idx}"
            );
        };

        let (mut attacker, mut victim) = both_cars.into();

        // Test collision both ways
        for is_swapped in [false, true] {
            let mut attacker_idx = car_1_idx;
            let mut victim_idx = car_2_idx;
            if is_swapped {
                mem::swap(&mut attacker, &mut victim);
                mem::swap(&mut attacker_idx, &mut victim_idx);
            }

            let attacker_state = &attacker.state;
            let victim_state = &victim.state;

            // A car demolished by the other car earlier in this tick can still demolish it,
            // but not bump it.
            let gate = demo_gate(
                attacker_state.is_demoed,
                victim_state.is_demoed,
                attacker.state.cache.demo_tick == self.tick_count
                    && attacker.state.cache.demoer_idx == Some(victim_idx),
            );
            if gate == DemoGate::Skip {
                continue;
            }

            let delta_pos = victim_state.phys.pos - attacker_state.phys.pos;
            if attacker_state.phys.vel.dot(delta_pos) < 0.0 {
                // Moving away from the other car
                continue;
            }

            let vel_dir = attacker_state.phys.vel.normalize_or_zero();
            let dir_to_victim = delta_pos.normalize_or_zero();

            let speed_towards_other_car = attacker_state.phys.vel.dot(dir_to_victim);
            let other_car_away_speed = victim_state.phys.vel.dot(vel_dir);
            if speed_towards_other_car <= other_car_away_speed {
                // Going towards other car slower than they're going away
                continue;
            }

            // The game bumps only when the other car's hitbox center is in the
            // bumper's 70 deg forward cone.
            let bumper = is_within_forward_cone(
                attacker_state,
                &attacker.info.config,
                victim_state,
                &victim.info.config,
                false,
                consts::car::bump::YAW_LIMIT_DEG,
                consts::car::bump::PITCH_LIMIT_DEG,
            );

            // Demolitions are checked before the bump cooldown.
            let mut is_demo = match self.config.mutators.demo_mode {
                DemoMode::OnContact => true,
                DemoMode::Disabled => false,
                DemoMode::Normal => should_demolish(
                    attacker_state,
                    &attacker.info.config,
                    victim_state,
                    &victim.info.config,
                    TICK_TIME,
                ),
            };
            if is_demo && !self.config.mutators.enable_team_demos {
                is_demo = attacker.team != victim.team;
            }

            let contact_point = if is_swapped {
                manifold_point.pos_world_on_b
            } else {
                manifold_point.pos_world_on_a
            } * BT_TO_UU;

            if is_demo {
                victim.demolish(self.config.mutators.respawn_delay);
                victim.state.cache.demo_tick = self.tick_count;
                victim.state.cache.demoer_idx = Some(attacker_idx);
                self.events.push(ArenaEvent::CarHitCar(CarHitCarEvent {
                    bumper_car_idx: attacker_idx,
                    victim_car_idx: victim_idx,
                    contact_point,
                    is_demo: true,
                }));
                continue;
            }
            if !gate.may_bump() {
                continue;
            }

            if attacker_state.bump_cooldown_timer > 0.0
                && attacker_state
                    .bump_cooldown_victim
                    .is_none_or(|v| v == victim_idx)
            {
                // In cooldown for this victim pair (None = all victims)
                continue;
            }

            if bumper {
                use consts::curves;

                let ground_hit = victim_state.cache.is_on_ground;
                let attacker_speed = attacker_state.phys.vel.length().min(2200.0);
                let bump_impulse = if ground_hit {
                    // Grounded victim: Ground and Z curves on total attacker speed.
                    let base_scale = curves::BUMP_VEL_AMOUNT_GROUND.get_output(attacker_speed);
                    let upward_force = curves::BUMP_UPWARD_VEL_AMOUNT.get_output(attacker_speed);
                    let hit_up_dir = victim_state.phys.rot_mat.z_axis;

                    (vel_dir * base_scale) + (hit_up_dir * upward_force)
                } else {
                    // Airborne victim: Air curve on total attacker speed; no up term.
                    vel_dir * curves::BUMP_VEL_AMOUNT_AIR.get_output(attacker_speed)
                };

                victim.vel_impulse_cache +=
                    bump_impulse * self.config.mutators.bump_force_scale * UU_TO_BT;
            }

            // The game's next bump needs the physics time strictly past the cooldown: half a tick longer.
            // It also starts without a bump.
            attacker.state.bump_cooldown_timer =
                self.config.mutators.bump_cooldown_time + TICK_TIME * 0.5;
            attacker.state.bump_cooldown_victim = Some(victim_idx);

            if bumper {
                self.events.push(ArenaEvent::CarHitCar(CarHitCarEvent {
                    bumper_car_idx: attacker_idx,
                    victim_car_idx: victim_idx,
                    contact_point,
                    is_demo: false,
                }));
            }
        }
    }
}
