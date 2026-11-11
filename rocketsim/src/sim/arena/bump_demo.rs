mod cone;
mod sweep;

use std::mem;

use cone::is_within_forward_cone;
use sweep::get_time_of_impact_poses;

use super::{Arena, ContactGeometry};
use crate::{
    CarBodyConfig, CarState, DemoMode,
    consts::{self, BT_TO_UU, TICK_TIME, UU_TO_BT},
    sim::{ArenaEvent, CarHitCarEvent},
};

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
    let forward_speed = start_vel.dot(cone::forward_axis(attacker));
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
        geometry: &ContactGeometry,
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
                geometry.pos_world_on_b
            } else {
                geometry.pos_world_on_a
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
