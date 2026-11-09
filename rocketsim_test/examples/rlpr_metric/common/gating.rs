use glam::Vec3A;
use rocketsim_test::rlpr::{TOUCH_FRAME_UNKNOWN, tick_record::TickRecord};

use super::{
    backend::ReplayBackend,
    rows::{any_teleport, frame_is_contiguous, tick_is_frozen, tick_is_kickoff_stasis},
};

/// Ball-kick magnitude that counts as an extra-hit firing, in uu/s.
///
/// A Bullet step integrates position from the velocity that exists at the end
/// of the step, so an untouched recorded tick satisfies
/// `(p[t]-p[t-1])/dt == v[t]`. The residual between the two is the velocity
/// change that never moved the position — the kick the touch produced.
/// [`compute_extra_firings`] uses this to tell a real kick from noise.
pub const BALL_KICK_TOL_UU_S: f32 = 1.0;

/// Velocity change the recorded ball state applied without moving its
/// position, in uu/s: `(p[t]-p[t-1])/dt - v[t]`.
fn kick_residual(prev_pos: Vec3A, cur_pos: Vec3A, cur_vel: Vec3A) -> Vec3A {
    (cur_pos - prev_pos) / rocketsim::consts::TICK_TIME - cur_vel
}
/// Extra-hit firing state for one car on one tick: did RL apply the ball
/// extra impulse during the step into this tick?
///
/// Inferred in the metric from raw touch frames plus ball kick evidence —
/// never in the capture plugin, which records engine values verbatim.
/// See [`compute_extra_firings`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExtraFiring {
    /// RL fired the extra impulse for this car on this tick: a recent touch
    /// plus a ball kick coincide with unambiguous attribution.
    Fire,
    /// No extra impulse on this tick for this car.
    NoFire,
    /// Cannot decide: no touch data (pre-v9), ambiguous attribution, or a
    /// kick with no recent touch. Restore leaves the sim alone there.
    Unknown,
}

/// True when a raw touch frame means "touched within the last step".
///
/// Accepts both the plugin's documented timing and same-frame touches, so
/// a sampling-order change on either side cannot silently shift every touch
/// out of the window. Zero and [`TOUCH_FRAME_UNKNOWN`] never count: no real
/// touch happens at kickoff frames 0-1 (cars start too far from the ball),
/// so 0 can only be the engine's never-touched encoding here. Confirmed on
/// the v9 re-recordings: `is_touching_ball` matches touch frame == physics
/// frame - 1 with zero mismatches, and same-frame touches occur as well.
fn touch_recent(last_touch_frame: u32, phys_frame: u32) -> bool {
    last_touch_frame != 0
        && last_touch_frame != TOUCH_FRAME_UNKNOWN
        && last_touch_frame <= phys_frame
        && phys_frame - last_touch_frame <= 1
}

/// Detect RL extra-hit firings, one entry per tick per car.
///
/// A firing needs all three at once: a recent touch for exactly one car,
/// ball kick above [`BALL_KICK_TOL_UU_S`] on the same tick (the kick the
/// touch produced), and unambiguous attribution. Anything else is
/// [`ExtraFiring::NoFire`] (clean ball) or [`ExtraFiring::Unknown`]
/// (ambiguous: kick with zero or several recent touches, or no touch data
/// at all when `has_touch_state` is false).
///
/// Unknown is fail-safe by construction: [`restore_extra_cooldown`] leaves
/// the sim alone there, so unknown ticks replay with live gating evolution.
pub fn compute_extra_firings(ticks: &[TickRecord], has_touch_state: bool) -> Vec<Vec<ExtraFiring>> {
    let mut out = Vec::with_capacity(ticks.len());
    for (index, tick) in ticks.iter().enumerate() {
        let count = tick.car_records.len();
        if index == 0 || !has_touch_state {
            out.push(vec![ExtraFiring::Unknown; count]);
            continue;
        }
        let prev = &ticks[index - 1];
        // Run breaks carry no physics: a goal reset looks like a giant kick
        // next to genuinely recent touches. Only continuous steps can fire.
        // (Kickoff stasis still computes normally below: its ball is clean,
        // so it resolves to NoFire, which correctly clears stale gating.)
        if prev.car_records.len() != count
            || !frame_is_contiguous(prev, tick)
            || tick_is_frozen(prev, tick)
            || any_teleport(prev, tick)
        {
            out.push(vec![ExtraFiring::Unknown; count]);
            continue;
        }
        let ball_drift = kick_residual(
            prev.ball_record.pos.into(),
            tick.ball_record.pos.into(),
            tick.ball_record.lin_vel.into(),
        )
        .length();
        if ball_drift <= BALL_KICK_TOL_UU_S {
            out.push(vec![ExtraFiring::NoFire; count]);
            continue;
        }
        let mut recent = Vec::new();
        for (slot, car) in tick.car_records.iter().enumerate() {
            if !car.is_available() {
                continue;
            }
            if touch_recent(car.last_ball_touch_frame, car.phys.physics_frame) {
                recent.push(slot);
            }
        }
        // Exactly one recent touch attributes the kick; zero or several
        // means the mechanism is unclear, so stay Unknown (restore leaves
        // the sim alone there, corrupting nothing).
        //
        // World contact shares the tick: the kick may be contact
        // friction/separation rather than the extra impulse (e.g. a
        // friction-cone-consistent kick while ceiling-grazing), and a
        // validated Fire here would suppress the sim on the next reset even
        // when RL refires there. Downgrade to Unknown; the sim replays with
        // live gating evolution.
        if recent.len() == 1 && !tick.ball_record.has_world_contact {
            let mut row = vec![ExtraFiring::NoFire; count];
            row[recent[0]] = ExtraFiring::Fire;
            out.push(row);
        } else {
            out.push(vec![ExtraFiring::Unknown; count]);
        }
    }
    out
}

/// Restore extra-hit gating from RL truth after a reset.
///
/// For each car: a validated firing at the reset state spends the impulse so
/// the replay does not apply it a second time; a validated non-firing clears
/// stale sim gating so a real RL kick is not wrongly suppressed; unknown
/// leaves the sim's live gate evolution alone, which tracks RL better than
/// any blind default when fed truth poses each reset. Backends without
/// cooldown support (see [`ReplayBackend::supports_cooldown_restore`])
/// ignore this entirely.
pub fn restore_extra_cooldown<B: ReplayBackend>(
    backend: &mut B,
    firings: &[Vec<ExtraFiring>],
    state_index: usize,
) {
    if !backend.supports_cooldown_restore() {
        return;
    }
    let Some(cars) = firings.get(state_index) else {
        return;
    };
    for (car_idx, firing) in cars.iter().enumerate() {
        match firing {
            ExtraFiring::Fire => backend.suppress_next_extra_hit(car_idx, true),
            ExtraFiring::NoFire => backend.suppress_next_extra_hit(car_idx, false),
            ExtraFiring::Unknown => {}
        }
    }
}
/// RL car-bump event: the attacker bumped the victim on the step into `tick`.
///
/// Inferred in the metric from a victim velocity kick plus car contact —
/// never in the capture plugin, which records engine values verbatim.
/// See [`compute_bump_events`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BumpEvent {
    /// Index of the tick the bump landed on (kick visible here).
    pub tick: usize,
    /// Slot of the bumped car.
    pub victim: usize,
    /// Slot of the bumping car (nearest touching car to the victim).
    pub attacker: usize,
}

/// Minimum car-velocity kick that counts as a bump, in uu/s.
///
/// Above first/double-jump self-kicks (292 uu/s). Dodge self-kicks are
/// bigger (flip impulse scale 500) but always coincide with the victim's
/// flip start, so the flip-start guard excludes them instead of the
/// threshold. Smaller bumps (slow attacker) fall below this and replay
/// with live cooldown evolution; their errors are small.
pub const BUMP_KICK_TOL_UU_S: f32 = 400.0;

/// Reconstructed bump-cooldown window in ticks.
///
/// This is the simulator's 0.25 s window at 120 Hz, not a proven native clock.
/// The event detector can miss or misattribute bumps.
pub const BUMP_COOLDOWN_TICKS: usize = 30;

/// Detect RL bump events, in tick order.
///
/// A bump needs all three at once on the same tick: a victim velocity kick
/// above [`BUMP_KICK_TOL_UU_S`], car contact on the victim, and a touching
/// partner to attribute it to (the nearest one). Kicks that coincide with
/// the victim's own flip start are dodge self-kicks, not bumps. Anything
/// else replays with live cooldown evolution. Run breaks never carry bumps
/// (same continuity guards as [`compute_extra_firings`]).
///
/// One event per unordered pair per tick: when both partners kick (mutual
/// contact), only the larger kick is the bump victim. Emitting both
/// directions would suppress both attackers' cooldowns and mask real sim
/// bumps.
pub fn compute_bump_events(ticks: &[TickRecord]) -> Vec<BumpEvent> {
    let mut out = Vec::new();
    for (index, tick) in ticks.iter().enumerate() {
        let count = tick.car_records.len();
        if index == 0 || count == 0 {
            continue;
        }
        let prev = &ticks[index - 1];
        if prev.car_records.len() != count
            || !frame_is_contiguous(prev, tick)
            || tick_is_frozen(prev, tick)
            || any_teleport(prev, tick)
            || tick_is_kickoff_stasis(prev, tick)
        {
            continue;
        }
        // (victim, attacker, kick) candidates this tick.
        let mut cands: Vec<(usize, usize, f32)> = Vec::new();
        for (slot, car) in tick.car_records.iter().enumerate() {
            if !car.is_available() || !car.is_touching_car {
                continue;
            }
            let kick = kick_residual(
                prev.car_records[slot].phys.pos.into(),
                car.phys.pos.into(),
                car.phys.lin_vel.into(),
            )
            .length();
            if kick <= BUMP_KICK_TOL_UU_S {
                continue;
            }
            // Own dodge impulse, not a bump: the flip starts on this tick.
            if car.is_flipping && !prev.car_records[slot].is_flipping {
                continue;
            }
            let victim_pos: Vec3A = car.phys.pos.into();
            let attacker = (0..count)
                .filter(|&other| {
                    other != slot
                        && tick.car_records[other].is_available()
                        && tick.car_records[other].is_touching_car
                })
                .min_by(|&a, &b| {
                    let da: Vec3A = tick.car_records[a].phys.pos.into();
                    let db: Vec3A = tick.car_records[b].phys.pos.into();
                    (da - victim_pos)
                        .length_squared()
                        .partial_cmp(&(db - victim_pos).length_squared())
                        .unwrap_or(std::cmp::Ordering::Equal)
                });
            if let Some(attacker) = attacker {
                cands.push((slot, attacker, kick));
            }
        }
        // Deduplicate mirrored pairs: keep the larger kick as the victim.
        cands.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));
        let mut used = vec![false; count];
        for (victim, attacker, _) in cands {
            let (lo, hi) = if victim < attacker {
                (victim, attacker)
            } else {
                (attacker, victim)
            };
            // Mark both slots used: a car joins at most one bump per tick.
            if used[lo] || used[hi] {
                continue;
            }
            used[lo] = true;
            used[hi] = true;
            out.push(BumpEvent {
                tick: index,
                victim,
                attacker,
            });
        }
    }
    out
}

/// Restore bump-cooldown gating from RL truth after a reset.
///
/// Use only events at or before the reset state in the current continuous
/// run. A future bump must not clear the gate before its outcome is predicted.
/// Apply past events in order, so the latest event for each attacker selects
/// its victim and remaining timer. This is inferred history, not a captured
/// native timer. Outside the window, keep the simulator's live gate.
/// Backends without [`ReplayBackend::supports_bump_restore`] ignore it.
pub fn restore_bump_cooldown<B: ReplayBackend>(
    backend: &mut B,
    events: &[BumpEvent],
    run_start_index: usize,
    state_index: usize,
) {
    if !backend.supports_bump_restore() {
        return;
    }
    for event in events {
        if event.tick >= run_start_index
            && state_index >= event.tick
            && state_index - event.tick < BUMP_COOLDOWN_TICKS
        {
            let elapsed = (state_index - event.tick) as f32 * rocketsim::consts::TICK_TIME;
            let remaining = rocketsim::consts::car::bump::COOLDOWN_TIME - elapsed;
            if remaining > 0.0 {
                backend.set_bump_cooldown(event.attacker, event.victim, remaining);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::test_support::*;

    #[test]
    fn extra_firing_needs_touch_kick_and_attribution() {
        // No touch data at all: everything Unknown, even with a kick.
        let ticks = vec![
            steady_tick(0, 0.0, 1200.0),
            kick_tick(1, 10.0, (2000.0, 0.0, 0.0), 1),
        ];
        let firings = compute_extra_firings(&ticks, false);
        assert_eq!(firings[1], vec![ExtraFiring::Unknown]);

        // Clean ball with a recent touch: light touch, no extra.
        let ticks = vec![
            steady_tick(0, 0.0, 1200.0),
            kick_tick(1, 10.0, (0.0, 0.0, 0.0), 1),
        ];
        // Ball never moved: position still (0,0,500), velocity zeroed here
        // keeps the pair drift-clean.
        let firings = compute_extra_firings(&ticks, true);
        assert_eq!(firings[1], vec![ExtraFiring::NoFire]);

        // Kick plus exactly one recent touch: validated firing.
        let ticks = vec![
            steady_tick(0, 0.0, 1200.0),
            kick_tick(1, 10.0, (2000.0, 0.0, 0.0), 1),
        ];
        let firings = compute_extra_firings(&ticks, true);
        assert_eq!(firings[1], vec![ExtraFiring::Fire]);

        // Same-frame touch (zero frames back) also counts.
        let ticks = vec![
            steady_tick(5, 40.0, 1200.0),
            kick_tick(6, 50.0, (2000.0, 0.0, 0.0), 6),
        ];
        assert_eq!(
            compute_extra_firings(&ticks, true)[1],
            vec![ExtraFiring::Fire]
        );

        // Stale touch (two frames back) with a fresh kick: unattributable.
        let ticks = vec![
            steady_tick(5, 0.0, 1200.0),
            kick_tick(6, 10.0, (2000.0, 0.0, 0.0), 4),
        ];
        assert_eq!(
            compute_extra_firings(&ticks, true)[1],
            vec![ExtraFiring::Unknown]
        );

        // Kick across a teleport: run break, not a firing, even with a
        // recent touch stamped on the arrival.
        let mut arrival = kick_tick(1, 5000.0, (2000.0, 0.0, 0.0), 1);
        arrival.ball_record.physics_frame = 1;
        let ticks = vec![steady_tick(0, 0.0, 1200.0), arrival];
        assert!(any_teleport(&ticks[0], &ticks[1]));
        assert_eq!(
            compute_extra_firings(&ticks, true)[1],
            vec![ExtraFiring::Unknown]
        );
    }
    #[test]
    fn extra_firing_multi_touch_is_ambiguous() {
        // Two cars, both recently touched, one ball kick: either could have
        // hit it, so neither is validated.
        let mut t0 = steady_tick(0, 0.0, 1200.0);
        t0.car_records.push(t0.car_records[0].clone());
        let mut t1 = kick_tick(1, 10.0, (2000.0, 0.0, 0.0), 1);
        t1.car_records.push(t1.car_records[0].clone());
        t1.car_records[1].last_ball_touch_frame = 1;
        let ticks = vec![t0, t1];
        assert_eq!(
            compute_extra_firings(&ticks, true)[1],
            vec![ExtraFiring::Unknown, ExtraFiring::Unknown]
        );
        // Only car 1 recent: attributed to car 1.
        let mut t1 = kick_tick(1, 10.0, (2000.0, 0.0, 0.0), 0);
        t1.car_records.push(t1.car_records[0].clone());
        t1.car_records[0].last_ball_touch_frame = 0;
        t1.car_records[1].last_ball_touch_frame = 1;
        let mut t0 = steady_tick(0, 0.0, 1200.0);
        t0.car_records.push(t0.car_records[0].clone());
        let ticks = vec![t0, t1];
        assert_eq!(
            compute_extra_firings(&ticks, true)[1],
            vec![ExtraFiring::NoFire, ExtraFiring::Fire]
        );
    }
    /// Two cars driving steadily; the second tick kicks one car's velocity
    /// (position keeps advancing 10 uu/tick) with car contact on both.
    /// Residual for a kicked car is |1200 - vx| uu/s.
    fn bump_ticks(kick_slot: usize, kick_vx: f32) -> Vec<TickRecord> {
        let mut t0 = steady_tick(0, 0.0, 1200.0);
        t0.car_records.push(t0.car_records[0].clone());
        t0.car_records[1].phys.pos = vec(90.0, 0.0, 100.0);
        let mut t1 = steady_tick(1, 10.0, 1200.0);
        t1.car_records.push(t1.car_records[0].clone());
        t1.car_records[1].phys.pos = vec(100.0, 0.0, 100.0);
        t1.car_records[kick_slot].phys.lin_vel = vec(kick_vx, 0.0, 0.0);
        t1.car_records[0].is_touching_car = true;
        t1.car_records[1].is_touching_car = true;
        vec![t0, t1]
    }
    #[test]
    fn bump_event_needs_kick_contact_and_attribution() {
        // Victim kick 800 uu/s with contact: one event, nearest partner.
        let ticks = bump_ticks(0, 2000.0);
        assert_eq!(
            compute_bump_events(&ticks),
            vec![BumpEvent {
                tick: 1,
                victim: 0,
                attacker: 1
            }]
        );
        // Sub-threshold kick (200 uu/s): no event.
        let ticks = bump_ticks(0, 1000.0);
        assert!(compute_bump_events(&ticks).is_empty());
        // Kick without contact: no event.
        let mut ticks = bump_ticks(0, 2000.0);
        ticks[1].car_records[0].is_touching_car = false;
        ticks[1].car_records[1].is_touching_car = false;
        assert!(compute_bump_events(&ticks).is_empty());
    }

    #[test]
    fn bump_mirrored_pair_keeps_larger_kick() {
        // Both partners kick: only the larger kick is the victim, so the
        // restore suppresses one attacker, not both.
        let mut ticks = bump_ticks(0, 2000.0);
        ticks[1].car_records[1].phys.lin_vel = vec(0.0, 0.0, 0.0);
        assert_eq!(
            compute_bump_events(&ticks),
            vec![BumpEvent {
                tick: 1,
                victim: 1,
                attacker: 0
            }]
        );
    }

    #[test]
    fn bump_flip_start_is_dodge_not_bump() {
        // Victim's flip starts on the kick tick: own dodge impulse.
        let mut ticks = bump_ticks(0, 2000.0);
        ticks[1].car_records[0].is_flipping = true;
        assert!(compute_bump_events(&ticks).is_empty());
        // Flip ongoing since the previous tick: not a flip start, still a bump.
        ticks[0].car_records[0].is_flipping = true;
        assert_eq!(compute_bump_events(&ticks).len(), 1);
    }
    #[test]
    fn restore_bump_cooldown_uses_only_past_events_in_this_run() {
        let events = vec![BumpEvent {
            tick: 5,
            victim: 1,
            attacker: 0,
        }];
        let mut backend = MirrorBackend::new(&[steady_tick(0, 0.0, 1200.0)]);
        backend.supports_bump = true;
        restore_bump_cooldown(&mut backend, &events, 0, 4);
        assert!(
            backend.bump_calls.is_empty(),
            "future outcome must not seed the gate"
        );
        restore_bump_cooldown(&mut backend, &events, 0, 5);
        restore_bump_cooldown(&mut backend, &events, 0, 34);
        restore_bump_cooldown(&mut backend, &events, 0, 35);
        restore_bump_cooldown(&mut backend, &events, 0, 99);
        restore_bump_cooldown(&mut backend, &events, 6, 7);
        assert_eq!(backend.bump_calls.len(), 2);
        assert_eq!(backend.bump_calls[0], (0, 1, 0.25));
        assert!(backend.bump_calls[1].2 > 0.0 && backend.bump_calls[1].2 < 0.25);

        let mut unsupported = MirrorBackend::new(&[steady_tick(0, 0.0, 1200.0)]);
        restore_bump_cooldown(&mut unsupported, &events, 0, 5);
        assert!(unsupported.bump_calls.is_empty());
    }
    #[test]
    fn bump_restore_does_not_depend_on_future_outcomes() {
        let past = BumpEvent {
            tick: 5,
            victim: 1,
            attacker: 0,
        };
        let future = BumpEvent {
            tick: 11,
            victim: 2,
            attacker: 0,
        };
        let mut expected = Vec::new();
        for events in [vec![past], vec![past, future]] {
            let mut backend = MirrorBackend::new(&[steady_tick(0, 0.0, 1200.0)]);
            backend.supports_bump = true;
            restore_bump_cooldown(&mut backend, &events, 0, 10);
            if expected.is_empty() {
                expected = backend.bump_calls;
            } else {
                assert_eq!(backend.bump_calls, expected);
            }
        }
        assert_eq!(expected.len(), 1);
        assert_eq!((expected[0].0, expected[0].1), (0, 1));
    }
    #[test]
    fn restore_extra_cooldown_blocks_clears_and_leaves() {
        let mut backend = MirrorBackend::new(&[steady_tick(0, 0.0, 1200.0)]);
        backend.supports_cooldown = true;
        let firings = vec![
            vec![ExtraFiring::Unknown],
            vec![ExtraFiring::Fire],
            vec![ExtraFiring::NoFire],
        ];
        restore_extra_cooldown(&mut backend, &firings, 1);
        restore_extra_cooldown(&mut backend, &firings, 2);
        restore_extra_cooldown(&mut backend, &firings, 0);
        restore_extra_cooldown(&mut backend, &firings, 99);
        assert_eq!(backend.suppress_calls, vec![(0, true), (0, false)]);

        // Backends without support stay untouched (v2 / legacy path).
        let mut backend = MirrorBackend::new(&[steady_tick(0, 0.0, 1200.0)]);
        restore_extra_cooldown(&mut backend, &firings, 1);
        assert!(backend.suppress_calls.is_empty());
    }
}
