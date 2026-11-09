use rocketsim_test::rlpr::tick_record::TickRecord;

use super::{
    backend::ReplayBackend,
    gating::{BumpEvent, ExtraFiring, restore_bump_cooldown, restore_extra_cooldown},
    rows::reconstruct_handbrake,
};

/// Refresh hidden wheel state after a reset without advancing dynamics.
///
/// Settles the sticky-wheel gate with wheel raycasts at the recorded pose,
/// replacing the old multi-tick warmup: scored bodies stay exactly at the
/// recorded state, so scoring can start on the next tick.
pub fn settle_reset_state<B: ReplayBackend>(backend: &mut B, state: &TickRecord) {
    backend.set_state(state);
    backend.refresh_sticky_gates();
}

/// Restore reconstructed handbrake state at a segment state.
///
/// Use this fallback only for versions v2-v7. Version v8 records the value.
/// Harness only, not physics.
pub fn restore_handbrake_seed<B: ReplayBackend>(
    backend: &mut B,
    ticks: &[TickRecord],
    run_start: usize,
    state_index: usize,
) {
    let Some(state) = ticks.get(state_index) else {
        return;
    };
    for car_idx in 0..state.car_records.len() {
        if let Some(value) = reconstruct_handbrake(ticks, run_start, state_index, car_idx) {
            backend.set_handbrake_value(car_idx, value);
        }
    }
}

/// Whether the jump hold was broken before one recorded state.
///
/// The new jump-sustain latch needs continuous hold since jump start: a
/// release ends the sustain after the minimum ticks and re-pressing never
/// resumes it. RLPR records no release latch, so scan back through the run
/// to the jump start and report whether the button stayed held on every
/// jumping step. At a run boundary, assume unbroken (same approximation as
/// the handbrake seed). Harness only, not physics.
pub fn jump_hold_broken_at(
    ticks: &[TickRecord],
    run_start: usize,
    state_index: usize,
    car_idx: usize,
) -> bool {
    let Some(state) = ticks.get(state_index) else {
        return false;
    };
    let Some(car) = state.car_records.get(car_idx) else {
        return false;
    };
    if !car.is_jumping {
        return false;
    }
    // Walk back over contiguous jumping ticks to the jump start.
    let mut start = state_index;
    while start > run_start
        && ticks[start - 1]
            .car_records
            .get(car_idx)
            .is_some_and(|prev| prev.is_jumping)
    {
        start -= 1;
    }
    // `prev_controls[u]` drove step u-1 -> u: every step since the start
    // step must be held for the hold to be continuous.
    for tick in ticks.iter().take(state_index + 1).skip(start + 1) {
        let held = tick
            .car_records
            .get(car_idx)
            .is_some_and(|tick_car| tick_car.prev_controls.jump);
        if !held {
            return true;
        }
    }
    false
}

/// Restore reconstructed jump-hold continuity after a reset.
///
/// No RLPR version records the release latch. Reconstruct it from control
/// history for restored mid-jump states, and clear it explicitly for
/// restored non-jumping states so stale latch values cannot leak across
/// segment restores. Harness only, not physics.
pub fn restore_jump_hold_continuity<B: ReplayBackend>(
    backend: &mut B,
    ticks: &[TickRecord],
    run_start: usize,
    state_index: usize,
) {
    let Some(state) = ticks.get(state_index) else {
        return;
    };
    for car_idx in 0..state.car_records.len() {
        backend.set_jump_hold_broken(
            car_idx,
            jump_hold_broken_at(ticks, run_start, state_index, car_idx),
        );
    }
}

/// Restore recorded boost latch state after a reset.
///
/// Apply direct recorded state only when the format carries it.
/// Write raw time only when armed. Write zero when disarmed.
/// A stale disarmed time would inflate the next arm and expire the latch early.
/// Older versions keep live-latch evolution: this is a no-op for them.
/// The parser keeps the raw value. This guard is adaptation, not physics.
/// Harness only, not physics.
pub fn restore_recorded_boost_state<B: ReplayBackend>(
    backend: &mut B,
    ticks: &[TickRecord],
    state_index: usize,
    has_boost_state: bool,
) {
    if !has_boost_state {
        return;
    }
    let Some(state) = ticks.get(state_index) else {
        return;
    };
    for (car_idx, car) in state.car_records.iter().enumerate() {
        let time = if car.is_boosting {
            car.boosting_time
        } else {
            0.0
        };
        backend.set_boost_state(car_idx, car.is_boosting, time);
    }
}

/// Restore recorded handbrake integrator state after a reset.
///
/// Apply direct recorded state only when the format carries it.
/// Older versions keep the reconstruction fallback: this is a no-op for them.
/// The backend setter clamps the value. Harness only, not physics.
pub fn restore_recorded_handbrake<B: ReplayBackend>(
    backend: &mut B,
    ticks: &[TickRecord],
    state_index: usize,
    has_handbrake_state: bool,
) {
    if !has_handbrake_state {
        return;
    }
    let Some(state) = ticks.get(state_index) else {
        return;
    };
    for (car_idx, car) in state.car_records.iter().enumerate() {
        backend.set_handbrake_value(car_idx, car.handbrake_val);
    }
}

/// Full seed at one recorded state. Shared by segment starts and the
/// kickoff-stasis release.
///
/// Order matches the existing segment seed: recorded handbrake, optional
/// handbrake reconstruction, recorded boost, extra and bump gating, jump-hold
/// continuity, native body state, auxiliary state, then wheel gates.
#[allow(clippy::too_many_arguments)]
pub(super) fn seed_recorded_state<B: ReplayBackend>(
    backend: &mut B,
    ticks: &[TickRecord],
    state_index: usize,
    run_start_index: usize,
    firings: &[Vec<ExtraFiring>],
    bumps: &[BumpEvent],
    has_boost_state: bool,
    has_handbrake_state: bool,
) {
    backend.reset(&ticks[state_index]);
    restore_recorded_handbrake(backend, ticks, state_index, has_handbrake_state);
    if !has_handbrake_state {
        restore_handbrake_seed(backend, ticks, run_start_index, state_index);
    }
    restore_recorded_boost_state(backend, ticks, state_index, has_boost_state);
    restore_extra_cooldown(backend, firings, state_index);
    restore_bump_cooldown(backend, bumps, run_start_index, state_index);
    restore_jump_hold_continuity(backend, ticks, run_start_index, state_index);
    backend.restore_body_state(&ticks[state_index]);
    backend.restore_auxiliary_state(ticks, state_index);
    backend.refresh_sticky_gates();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::test_support::*;

    #[test]
    fn recorded_boost_state_restores_exact_values() {
        let mut tick = quiet_tick(1, 10.0);
        tick.car_records[0].is_boosting = true;
        tick.car_records[0].boosting_time = 0.05;
        let ticks = vec![quiet_tick(0, 0.0), tick];
        let mut probe = BoostProbe {
            boost_calls: vec![],
        };
        restore_recorded_boost_state(&mut probe, &ticks, 1, true);
        assert_eq!(probe.boost_calls, vec![(0, true, 0.05)]);
    }

    #[test]
    fn disarmed_large_time_restores_as_zero() {
        let mut tick = quiet_tick(1, 10.0);
        tick.car_records[0].is_boosting = false;
        tick.car_records[0].boosting_time = 5.0;
        let ticks = vec![quiet_tick(0, 0.0), tick];
        let mut probe = BoostProbe {
            boost_calls: vec![],
        };
        restore_recorded_boost_state(&mut probe, &ticks, 1, true);
        assert_eq!(probe.boost_calls, vec![(0, false, 0.0)]);
    }

    #[test]
    fn legacy_recordings_do_not_overwrite_the_latch() {
        let mut tick = quiet_tick(1, 10.0);
        tick.car_records[0].is_boosting = true;
        tick.car_records[0].boosting_time = 0.05;
        let ticks = vec![quiet_tick(0, 0.0), tick];
        let mut probe = BoostProbe {
            boost_calls: vec![],
        };
        restore_recorded_boost_state(&mut probe, &ticks, 1, false);
        assert!(probe.boost_calls.is_empty());
        restore_recorded_boost_state(&mut probe, &ticks, 99, true);
        assert!(probe.boost_calls.is_empty());
    }
    #[test]
    fn recorded_handbrake_restores_exact_value() {
        let mut tick = quiet_tick(1, 10.0);
        tick.car_records[0].handbrake_val = 0.875;
        let ticks = vec![quiet_tick(0, 0.0), tick];
        let mut probe = HandbrakeProbe {
            brake_calls: vec![],
        };
        restore_recorded_handbrake(&mut probe, &ticks, 1, true);
        assert_eq!(probe.brake_calls, vec![(0, 0.875)]);
    }

    #[test]
    fn legacy_handbrake_fallback_stays_live() {
        let mut tick = quiet_tick(1, 10.0);
        tick.car_records[0].handbrake_val = 0.875;
        let ticks = vec![quiet_tick(0, 0.0), tick];
        let mut probe = HandbrakeProbe {
            brake_calls: vec![],
        };
        restore_recorded_handbrake(&mut probe, &ticks, 1, false);
        assert!(probe.brake_calls.is_empty());
        restore_recorded_handbrake(&mut probe, &ticks, 99, true);
        assert!(probe.brake_calls.is_empty());
    }
    #[test]
    fn jump_hold_activation_tick_reports_unbroken() {
        // The jump start tick itself has no prior step to release on.
        let ticks = jump_history(&[(true, true)]);
        assert!(!jump_hold_broken_at(&ticks, 0, 0, 0));
    }

    #[test]
    fn jump_hold_release_repress_reports_broken() {
        // Press, release on the step into tick 1, re-press.
        let ticks = jump_history(&[(true, true), (true, false), (true, true), (true, true)]);
        assert!(jump_hold_broken_at(&ticks, 0, 3, 0));
    }

    #[test]
    fn jump_hold_continuous_reports_unbroken() {
        let ticks = jump_history(&[(true, true), (true, true), (true, true), (true, true)]);
        assert!(!jump_hold_broken_at(&ticks, 0, 3, 0));
    }

    #[test]
    fn jump_hold_inactive_reports_unbroken() {
        let ticks = jump_history(&[(true, true), (false, false)]);
        assert!(!jump_hold_broken_at(&ticks, 0, 1, 0));
    }

    #[test]
    fn jump_hold_run_boundary_truncation_assumes_unbroken() {
        // Release on the step into tick 1 sits before the run start: the
        // scan cannot see it and assumes unbroken (documented limitation).
        let ticks = jump_history(&[
            (true, true),
            (true, false),
            (true, true),
            (true, true),
            (true, true),
            (true, true),
        ]);
        assert!(jump_hold_broken_at(&ticks, 0, 5, 0));
        assert!(!jump_hold_broken_at(&ticks, 3, 5, 0));
    }

    #[test]
    fn jump_hold_no_cross_car_bleed() {
        let mut ticks = jump_history(&[(true, true), (true, false), (true, true)]);
        for tick in ticks.iter_mut() {
            let mut peer = tick.car_records[0];
            peer.prev_controls.jump = true;
            tick.car_records.push(peer);
        }
        assert!(jump_hold_broken_at(&ticks, 0, 2, 0));
        assert!(!jump_hold_broken_at(&ticks, 0, 2, 1));
    }
    #[test]
    fn restore_jump_hold_writes_every_car_explicitly() {
        let mut ticks = jump_history(&[(true, true), (true, false), (true, true)]);
        // Second car never jumps: restore must clear it explicitly so stale
        // latch values cannot leak across segment restores.
        for tick in ticks.iter_mut() {
            tick.car_records.push(tick.car_records[0]);
            tick.car_records[1].is_jumping = false;
        }
        let mut backend = LatchProbe { calls: Vec::new() };
        restore_jump_hold_continuity(&mut backend, &ticks, 0, 2);
        assert_eq!(backend.calls, vec![(0, true), (1, false)]);
    }
}
