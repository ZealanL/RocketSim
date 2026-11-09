use rocketsim_test::rlpr::{cpp_records::ControlsRecord, tick_record::TickRecord};

use super::{
    backend::{ReplayBackend, ResetMode, SimContactEvents, snapshot_from_tick},
    coverage::{AvailabilityCounts, availability_counts},
    gating::{
        compute_bump_events, compute_extra_firings, restore_bump_cooldown, restore_extra_cooldown,
    },
    restore::{
        restore_handbrake_seed, restore_jump_hold_continuity, restore_recorded_boost_state,
        restore_recorded_handbrake, seed_recorded_state, settle_reset_state,
    },
    rows::{Segment, run_start, tick_is_kickoff_stasis},
    scoring::{EvalReport, Tolerances, classify_tick, normalized_error},
};

/// Report plus skip counts from one [`evaluate`] run.
/// Skipped counts hold transitions removed as kickoff stasis. The sim still
/// steps through them.
#[derive(Clone, Debug, Default)]
pub struct EvalOutcome {
    pub report: EvalReport,
    pub skipped_transitions: usize,
    pub skipped_car_ticks: usize,
    /// Row availability coverage. Reporting only; never changes scoring.
    pub availability: AvailabilityCounts,
}

/// Run each segment open-loop and aggregate every car-tick into one report.
/// Every arena car steps with its own recorded controls, so car-car
/// contacts are real sim observations. Resets at each segment start and at
/// a kickoff-stasis release, steps with target `prev_controls`; every tick
/// after the reset is scored.
/// The reset restores recorded state and reconstructed history only. The
/// v12 restore report lists fields that stay outside replay truth.
/// Support counts car-ticks: each scored tick contributes one sample per car.
/// With `use_sim_events`, sim-observed contacts also label ticks;
/// otherwise labels come from RL flags alone.
/// Kickoff-stasis transitions still step the sim but add no support, pass, or
/// error. When a stasis span is followed immediately by live physics, the
/// backend is full-seeded at the last stasis row before that step. The live
/// tick still scores. A frozen, teleport, or mask boundary makes the first
/// live row a new segment instead, so the recorded corpus path is unchanged.
/// Every other tick scores, including contact ticks: raw touch frames
/// identify RL's extra-hit firings and gating is restored from RL truth, so
/// the sim reproduces the tick instead of double-applying it. Runs, chunks,
/// and seeds are unchanged. Counts land in [`EvalOutcome`].
/// `has_boost_state` must be true only when the recording version carries
/// recorded boost latch state; older versions keep live-latch evolution.
/// `has_handbrake_state` must be true only when the recording version
/// carries recorded handbrake state; older versions keep reconstruction.
/// `has_touch_state` must be true only when the recording version carries
/// raw last ball touch frames; older versions infer nothing (every firing
/// reads Unknown) and replay with live gating evolution.
#[allow(clippy::too_many_arguments)]
pub fn evaluate<B: ReplayBackend>(
    backend: &mut B,
    ticks: &[TickRecord],
    segments: &[Segment],
    reset_each_tick: bool,
    use_sim_events: bool,
    has_boost_state: bool,
    has_handbrake_state: bool,
    has_touch_state: bool,
) -> EvalOutcome {
    let mut outcome = EvalOutcome {
        availability: availability_counts(ticks),
        ..EvalOutcome::default()
    };
    let report = &mut outcome.report;
    let tol = Tolerances::for_mode(reset_each_tick);
    let firings = compute_extra_firings(ticks, has_touch_state);
    let bumps = compute_bump_events(ticks);
    backend.prepare_replay(ticks);
    for segment in segments {
        if segment.end() > ticks.len() {
            continue;
        }
        let segment_run_start = run_start(ticks, segment.start);
        if !reset_each_tick {
            // Seed the prior-tick wheel gate at the recorded pose.
            // Raycast only. Scored bodies stay at the recorded state.
            seed_recorded_state(
                backend,
                ticks,
                segment.start,
                segment_run_start,
                &firings,
                &bumps,
                has_boost_state,
                has_handbrake_state,
            );
        }
        for offset in 1..segment.len {
            let target_index = segment.start + offset;
            let target = &ticks[target_index];
            if reset_each_tick {
                let state_index = target_index - 1;
                // Settle the sticky-wheel gate at each segment start with
                // wheel raycasts at the reset pose (no dynamics advance), so
                // post-teleport transitions start from valid hidden state.
                // Mid-run resets keep the warmed suspension caches: they
                // track the foreign trajectory, which sits one step error
                // away from truth, while a fresh raycast invents a static
                // pose with zero relative velocity.
                if offset == 1 {
                    settle_reset_state(backend, &ticks[state_index]);
                } else {
                    backend.set_state(&ticks[state_index]);
                }
                restore_recorded_boost_state(backend, ticks, state_index, has_boost_state);
                restore_recorded_handbrake(backend, ticks, state_index, has_handbrake_state);
                restore_extra_cooldown(backend, &firings, state_index);
                restore_bump_cooldown(backend, &bumps, segment_run_start, state_index);
                restore_jump_hold_continuity(backend, ticks, segment_run_start, state_index);
                if !has_handbrake_state && offset == 1 {
                    restore_handbrake_seed(backend, ticks, segment_run_start, state_index);
                }
                backend.restore_body_state(&ticks[state_index]);
                backend.restore_auxiliary_state(ticks, state_index);
            }
            let prev_is_stasis = tick_is_kickoff_stasis(&ticks[target_index - 1], target);
            // A frozen, teleport, or mask-change successor starts a new
            // segment, so the first live transition here is contiguous. If
            // the previous transition was stasis, full-seed at the last
            // countdown row before stepping. The live tick still scores.
            // Reset-each-tick already restored every field for this state.
            let release_reseed = !reset_each_tick
                && offset > 1
                && !prev_is_stasis
                && tick_is_kickoff_stasis(&ticks[target_index - 2], &ticks[target_index - 1]);
            if release_reseed {
                let state_index = target_index - 1;
                seed_recorded_state(
                    backend,
                    ticks,
                    state_index,
                    run_start(ticks, state_index),
                    &firings,
                    &bumps,
                    has_boost_state,
                    has_handbrake_state,
                );
            }
            let controls: Vec<ControlsRecord> = target
                .car_records
                .iter()
                .map(|car| car.prev_controls)
                .collect();
            if controls.is_empty() {
                continue;
            }
            let sim_events = backend.step(&controls);
            if prev_is_stasis {
                outcome.skipped_transitions += 1;
                outcome.skipped_car_ticks += target.availability_mask().count_ones() as usize;
                continue;
            }
            for car_idx in 0..controls.len() {
                let Some(truth) = snapshot_from_tick(target, car_idx) else {
                    continue;
                };
                let sim = if use_sim_events {
                    sim_events.get(car_idx).copied().unwrap_or_default()
                } else {
                    SimContactEvents::default()
                };
                let norm = normalized_error(&backend.snapshot(car_idx), &truth, &tol);
                report.add(classify_tick(target, car_idx, sim), target_index, norm);
            }
        }
    }
    outcome
}

/// Motion-only diagnostic between full reset and segmented.
///
/// Keep [`split_segments`] boundaries. Seed fully at each segment start,
/// same order as [`evaluate`]: `reset`, recorded handbrake plus fallback
/// seed, recorded boost, extra/bump cooldowns, jump-hold continuity, native
/// body state, `refresh_sticky_gates`.
/// Then call [`ReplayBackend::reset_motion`] each transition: only
/// car/ball `pos`/`vel`/`ang_vel`, orientation and logical history preserved.
/// No per-step restores or sticky refresh. Native v12 body state is applied
/// only at the full segment seed. One-tick budgets unchanged.
/// Stasis still steps unscored. Motion at `offset == 1` repeats seed values.
/// When a stasis span is followed immediately by live physics, that one
/// release transition uses the full seed at the last stasis row, then returns
/// to motion-only resets. This prevents the release from carrying unknown
/// countdown evolution.
#[allow(clippy::too_many_arguments)]
pub fn evaluate_motion_each_tick<B: ReplayBackend>(
    backend: &mut B,
    ticks: &[TickRecord],
    segments: &[Segment],
    use_sim_events: bool,
    has_boost_state: bool,
    has_handbrake_state: bool,
    has_touch_state: bool,
) -> EvalOutcome {
    let mut outcome = EvalOutcome {
        availability: availability_counts(ticks),
        ..EvalOutcome::default()
    };
    let report = &mut outcome.report;
    let tol = Tolerances::for_reset_mode(ResetMode::ResetMotionEachTick);
    let firings = compute_extra_firings(ticks, has_touch_state);
    let bumps = compute_bump_events(ticks);
    backend.prepare_replay(ticks);
    for segment in segments {
        if segment.end() > ticks.len() {
            continue;
        }
        let segment_run_start = run_start(ticks, segment.start);
        // Full seed once per segment, same order as `evaluate`.
        seed_recorded_state(
            backend,
            ticks,
            segment.start,
            segment_run_start,
            &firings,
            &bumps,
            has_boost_state,
            has_handbrake_state,
        );
        for offset in 1..segment.len {
            let target_index = segment.start + offset;
            let target = &ticks[target_index];
            let prev_is_stasis = tick_is_kickoff_stasis(&ticks[target_index - 1], target);
            let release_reseed = offset > 1
                && !prev_is_stasis
                && tick_is_kickoff_stasis(&ticks[target_index - 2], &ticks[target_index - 1]);
            if release_reseed {
                // The countdown evolution is not restored by motion-only
                // resets. Full-seed at the last countdown row for this one
                // release step. Ordinary motion-only resets resume after it.
                let state_index = target_index - 1;
                seed_recorded_state(
                    backend,
                    ticks,
                    state_index,
                    run_start(ticks, state_index),
                    &firings,
                    &bumps,
                    has_boost_state,
                    has_handbrake_state,
                );
            } else {
                // Motion-only: no per-step restores or sticky refresh.
                backend.reset_motion(&ticks[target_index - 1]);
            }
            let controls: Vec<ControlsRecord> = target
                .car_records
                .iter()
                .map(|car| car.prev_controls)
                .collect();
            if controls.is_empty() {
                continue;
            }
            let sim_events = backend.step(&controls);
            if prev_is_stasis {
                outcome.skipped_transitions += 1;
                outcome.skipped_car_ticks += target.availability_mask().count_ones() as usize;
                continue;
            }
            for car_idx in 0..controls.len() {
                let Some(truth) = snapshot_from_tick(target, car_idx) else {
                    continue;
                };
                let sim = if use_sim_events {
                    sim_events.get(car_idx).copied().unwrap_or_default()
                } else {
                    SimContactEvents::default()
                };
                let norm = normalized_error(&backend.snapshot(car_idx), &truth, &tol);
                report.add(classify_tick(target, car_idx, sim), target_index, norm);
            }
        }
    }
    outcome
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::{
        collapse_duplicate_ticks, kickoff_stasis_targets, split_segments, test_support::*,
        tick_car_count, tick_is_frozen,
    };

    #[test]
    fn evaluate_scores_only_available_car_ticks() {
        let ticks: Vec<_> = (0..5)
            .map(|frame| multi_tick(frame, frame as f32 * 10.0, 6, 0b11_0111))
            .collect();
        let segments = split_segments(&ticks, config(100));
        assert_eq!(segments, vec![Segment { start: 0, len: 5 }]);
        let mut backend = MirrorBackend::new(&ticks);
        let outcome = evaluate(
            &mut backend,
            &ticks,
            &segments,
            false,
            false,
            false,
            false,
            false,
        );
        // Four scored ticks, five available cars each. The unavailable slot
        // is never support and never counted as a skip.
        assert_eq!(outcome.report.total.support, 20);
        assert_eq!(outcome.report.total.passed, 20);
        assert_eq!(outcome.availability.available_car_ticks, 25);
        assert_eq!(outcome.availability.unavailable_car_ticks, 5);
        assert_eq!(outcome.availability.no_physics_rows, 0);
        assert_eq!(outcome.skipped_car_ticks, 0);
    }
    #[test]
    fn long_no_physics_run_keeps_raw_counts_but_scores_available_only() {
        let mut ticks = Vec::new();
        for level in 0..50u64 {
            ticks.push(guard_tick(level, 6));
        }
        for frame in 0..5u32 {
            ticks.push(multi_tick(frame, frame as f32 * 10.0, 6, 0b11_1111));
        }

        // Raw counts describe the file. The long guard run stays in them.
        let raw = availability_counts(&ticks);
        assert_eq!(raw.rows, 55);
        assert_eq!(raw.no_physics_rows, 50);
        assert_eq!(raw.available_car_ticks, 30);
        assert_eq!(raw.unavailable_car_ticks, 300);

        // The guard run collapses to one row. Evaluated counts differ.
        let collapsed = collapse_duplicate_ticks(&ticks);
        assert_eq!(collapsed.len(), 6, "50 guard rows collapse to one");
        let evaluated = availability_counts(&collapsed);
        assert_eq!(evaluated.rows, 6);
        assert_eq!(evaluated.no_physics_rows, 1);
        assert_eq!(evaluated.available_car_ticks, 30);
        assert_eq!(evaluated.unavailable_car_ticks, 6);

        // Only the five physics rows form a segment. The guard row flushes
        // the run and is never scored.
        let segments = split_segments(&collapsed, config(100));
        assert_eq!(segments, vec![Segment { start: 1, len: 5 }]);
        let mut backend = ConstantBackend;
        let outcome = evaluate(
            &mut backend,
            &collapsed,
            &segments,
            false,
            false,
            false,
            false,
            false,
        );
        assert_eq!(outcome.report.total.support, 4 * 6);
        assert_eq!(outcome.availability, evaluated);
    }
    #[test]
    fn splits_on_car_count_change_and_labels_per_car() {
        let mut ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        // Tick 2 gains a second car that touches the ball. Tick 3 keeps both
        // cars so the two-car run is long enough to chunk (single-tick runs
        // hold no scored tick and are dropped).
        let mut second = ticks[2].car_records[0].clone();
        second.is_touching_ball = true;
        ticks[2].car_records.push(second);
        let third_second = ticks[3].car_records[0].clone();
        ticks[3].car_records.push(third_second);
        let segments = split_segments(&ticks, config(4));
        // No segment spans the count change at index 2: runs break there,
        // so every segment lies on one side and the change tick heads
        // its own run.
        for s in &segments {
            let counts: Vec<usize> = (s.start..s.end())
                .map(|i| tick_car_count(&ticks[i]))
                .collect();
            assert!(
                counts.windows(2).all(|w| w[0] == w[1]),
                "segment {s:?} spans count change"
            );
        }
        assert!(segments.iter().any(|s| s.start == 2));
        // Labels are per car.
        let tick2 = &ticks[2];
        assert!(!classify_tick(tick2, 0, SimContactEvents::default()).car_ball);
        assert!(classify_tick(tick2, 1, SimContactEvents::default()).car_ball);
        assert!(snapshot_from_tick(&ticks[2], 1).is_some());
        assert!(snapshot_from_tick(&ticks[2], 2).is_none());
        // Car 0 still evaluates over the clean run.
        let mut backend = MirrorBackend::new(&ticks);
        let report = evaluate(
            &mut backend,
            &ticks,
            &segments,
            false,
            true,
            false,
            false,
            false,
        )
        .report;
        assert!(report.total.support > 0);
    }

    #[test]
    fn evaluate_combines_car_ticks() {
        // Two-car run: every scored tick counts once per car.
        let mut ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        for tick in &mut ticks {
            tick.car_records.push(tick.car_records[0].clone());
        }
        let mut backend = MirrorBackend::new(&ticks);
        let segments = vec![Segment { start: 0, len: 4 }];
        let report = evaluate(
            &mut backend,
            &ticks,
            &segments,
            false,
            true,
            false,
            false,
            false,
        )
        .report;
        assert_eq!(report.total.support, 6);
        assert_eq!(report.total.passed, 6);
        assert_eq!(report.no_contact.support, 6);
    }

    #[test]
    fn evaluate_counts_overlapping_support() {
        let mut ticks: Vec<_> = (0..6).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        ticks[4].car_records[0].is_touching_ball = true;
        ticks[4].car_records[0].wheels[0].has_contact = true;
        let mut backend = MirrorBackend::new(&ticks);
        let segments = vec![Segment { start: 0, len: 6 }];
        let report = evaluate(
            &mut backend,
            &ticks,
            &segments,
            false,
            true,
            false,
            false,
            false,
        )
        .report;
        assert_eq!(report.total.support, 5);
        assert_eq!(report.total.passed, 5);
        assert_eq!(report.car_ball.support, 1);
        assert_eq!(report.wheel_world.support, 1);
        assert_eq!(report.no_contact.support, 4);

        let mut backend = MirrorBackend::new(&ticks);
        let report = evaluate(
            &mut backend,
            &ticks,
            &segments,
            true,
            true,
            false,
            false,
            false,
        )
        .report;
        assert_eq!(report.total.support, 5);
        assert_eq!(report.total.passed, 5);
    }
    #[test]
    fn validated_firing_target_scores_and_counts() {
        // MirrorBackend replays truth exactly (norm 0): the firing tick
        // scores and passes.
        let ticks = vec![
            steady_tick(0, 0.0, 1200.0),
            kick_tick(1, 10.0, (2000.0, 0.0, 0.0), 1),
        ];
        let segments = vec![Segment { start: 0, len: 2 }];
        let mut backend = MirrorBackend::new(&ticks);
        backend.supports_cooldown = true;
        let outcome = evaluate(
            &mut backend,
            &ticks,
            &segments,
            true,
            true,
            false,
            false,
            true,
        );
        assert_eq!(outcome.report.total.support, 1);
        assert_eq!(outcome.report.total.passed, 1);
        // Reset state tick 0 predates any step into the file: Unknown, so
        // the restore correctly leaves the fresh backend alone.
        assert!(backend.suppress_calls.is_empty());
    }

    #[test]
    fn unvalidated_drift_scores_with_restore_on() {
        // Kick with a stale touch: ambiguous, but v9 restore removes the
        // exclusion entirely — the tick is scored, not skipped. MirrorBackend
        // replays truth exactly, so it passes.
        let ticks = vec![
            steady_tick(5, 0.0, 1200.0),
            kick_tick(6, 10.0, (2000.0, 0.0, 0.0), 4),
        ];
        let segments = vec![Segment { start: 0, len: 2 }];
        let mut backend = MirrorBackend::new(&ticks);
        backend.supports_cooldown = true;
        let outcome = evaluate(
            &mut backend,
            &ticks,
            &segments,
            true,
            true,
            false,
            false,
            true,
        );
        assert_eq!(outcome.report.total.support, 1);
        assert_eq!(outcome.report.total.passed, 1);
    }

    #[test]
    fn upcoming_validated_firing_scores_both() {
        // Firing lands one tick after the target. No exclusion anywhere:
        // both the clean target and the firing tick score.
        let ticks = vec![
            steady_tick(0, 0.0, 1200.0),
            steady_tick(1, 10.0, 1200.0),
            kick_tick(2, 20.0, (2000.0, 0.0, 0.0), 2),
        ];
        let segments = vec![Segment { start: 0, len: 3 }];
        let mut backend = MirrorBackend::new(&ticks);
        backend.supports_cooldown = true;
        let outcome = evaluate(
            &mut backend,
            &ticks,
            &segments,
            true,
            true,
            false,
            false,
            true,
        );
        assert_eq!(outcome.report.total.support, 2);
        assert_eq!(outcome.report.total.passed, 2);
    }

    #[test]
    fn restore_inactive_still_scores() {
        // Same validated firing, incapable backend (v2 path): no restore,
        // but no exclusion either — the tick scores with live gating.
        let ticks = vec![
            steady_tick(0, 0.0, 1200.0),
            kick_tick(1, 10.0, (2000.0, 0.0, 0.0), 1),
        ];
        let segments = vec![Segment { start: 0, len: 2 }];
        let mut backend = MirrorBackend::new(&ticks);
        let outcome = evaluate(
            &mut backend,
            &ticks,
            &segments,
            true,
            true,
            false,
            false,
            true,
        );
        assert_eq!(outcome.report.total.support, 1);
        assert_eq!(outcome.report.total.passed, 1);
        // Detection still runs (diagnostic) even though this backend
        // cannot restore gating.
        assert!(backend.suppress_calls.is_empty());
    }
    #[test]
    fn evaluate_skips_stasis_but_steps() {
        let ticks: Vec<TickRecord> = [99u32, 100, 101, 102]
            .iter()
            .map(|&f| {
                let x = match f {
                    99 => 0.0,
                    100 => 10.0,
                    101 => 10.5,
                    _ => 20.0,
                };
                let (ball_world, hxy) = if f == 100 || f == 101 {
                    (true, 0.0)
                } else {
                    (false, 200.0)
                };
                let mut t = make_tick(
                    f,
                    (x, 0.0, 17.0),
                    (0.0, 0.0, 92.75),
                    (hxy, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                    false,
                    ball_world,
                    true,
                    true,
                );
                // Keep the non-stasis transitions physically self-consistent:
                // velocity must match the position delta (10 uu and 9.5 uu
                // per tick). The stasis target keeps near-zero velocity by
                // construction; that pair is skipped as stasis first.
                let vel = match f {
                    100 => 1200.0,
                    102 => 1140.0,
                    _ => hxy,
                };
                t.car_records[0].phys.lin_vel = vec(vel, 0.0, 0.0);
                t.car_records.push(t.car_records[0].clone());
                t
            })
            .collect();
        assert!(tick_is_kickoff_stasis(&ticks[1], &ticks[2]));
        assert!(!tick_is_kickoff_stasis(&ticks[2], &ticks[3]));
        let segments = vec![Segment { start: 0, len: 4 }];
        let mut backend = MirrorBackend::new(&ticks);
        let outcome = evaluate(
            &mut backend,
            &ticks,
            &segments,
            true,
            true,
            false,
            false,
            false,
        );
        assert_eq!(outcome.skipped_transitions, 1);
        assert_eq!(outcome.skipped_car_ticks, 2);
        assert_eq!(outcome.report.total.support, 4);
        assert_eq!(outcome.report.total.passed, 4);
        assert_eq!(kickoff_stasis_targets(&ticks), vec![2]);
    }
    #[test]
    fn segmented_refresh_runs_once_per_segment() {
        let ticks: Vec<_> = (0..6).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        let segments = vec![Segment { start: 0, len: 3 }, Segment { start: 3, len: 3 }];
        let mut backend = StickyGateProbe::new(&ticks);
        evaluate(
            &mut backend,
            &ticks,
            &segments,
            false,
            true,
            true,
            true,
            false,
        );
        assert_eq!(backend.refreshes, segments.len());
        let reset_pos = backend.events.iter().position(|e| e == "reset").unwrap();
        let refresh_pos = backend.events.iter().position(|e| e == "refresh").unwrap();
        let step_pos = backend.events.iter().position(|e| e == "step").unwrap();
        let handbrake_pos = backend
            .events
            .iter()
            .position(|e| e == "handbrake")
            .unwrap();
        let boost_pos = backend.events.iter().position(|e| e == "boost").unwrap();
        assert!(reset_pos < handbrake_pos);
        assert!(handbrake_pos < boost_pos);
        assert!(boost_pos < refresh_pos);
        assert!(refresh_pos < step_pos);
        let jump_pos = backend.events.iter().position(|e| e == "jump").unwrap();
        assert!(boost_pos < jump_pos);
        assert!(jump_pos < refresh_pos);
        let body_pos = backend.events.iter().position(|e| e == "body").unwrap();
        assert!(jump_pos < body_pos);
        let auxiliary_pos = backend
            .events
            .iter()
            .position(|e| e == "auxiliary")
            .unwrap();
        assert!(body_pos < auxiliary_pos);
        assert!(auxiliary_pos < refresh_pos);
        let mut backend = StickyGateProbe::new(&ticks);
        evaluate(
            &mut backend,
            &ticks,
            &segments,
            false,
            true,
            true,
            false,
            false,
        );
        assert_eq!(backend.refreshes, segments.len());
        // Reset-each-tick mode settles the gate at each segment start too.
        let mut backend = StickyGateProbe::new(&ticks);
        evaluate(
            &mut backend,
            &ticks,
            &segments,
            true,
            true,
            true,
            true,
            false,
        );
        assert_eq!(backend.refreshes, segments.len());
        let position = |name: &str| {
            backend
                .events
                .iter()
                .position(|event| event == name)
                .unwrap()
        };
        // `settle_reset_state` runs `set_state` and the sticky refresh first.
        assert!(position("set_state") < position("refresh"));
        assert!(position("refresh") < position("boost"));
        assert!(position("boost") < position("handbrake"));
        assert!(position("handbrake") < position("jump"));
        assert!(position("jump") < position("body"));
        assert!(position("body") < position("step"));
    }
    #[test]
    fn motion_seed_once_per_segment_no_optional_per_step_restores() {
        // Two overlapping segments over six quiet ticks: targets 1,2,4,5.
        let ticks: Vec<_> = (0..6).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        let segments = vec![Segment { start: 0, len: 3 }, Segment { start: 3, len: 3 }];
        let mut backend = MotionScheduleProbe::new(&ticks);
        let outcome =
            evaluate_motion_each_tick(&mut backend, &ticks, &segments, true, true, true, false);
        // Full seed once per segment.
        assert_eq!(backend.resets, 2);
        assert_eq!(backend.refreshes, 2);
        // One boost/handbrake/jump write per car per segment seed.
        assert_eq!(backend.boost_calls.len(), 2);
        assert_eq!(backend.brake_calls.len(), 2);
        assert_eq!(backend.jump_calls.len(), 2);
        // Motion reset each transition (four targets), never full set_state.
        assert_eq!(backend.motions, 4);
        // Body state follows the two segment seeds only.
        assert_eq!(backend.body_restores, 2);
        assert_eq!(backend.set_states, 0);
        assert_eq!(backend.steps, 4);
        // Mirror replays truth, so every car-tick passes.
        assert_eq!(outcome.report.total.support, 4);
        assert_eq!(outcome.report.total.passed, 4);
        assert_eq!(outcome.skipped_transitions, 0);
    }

    #[test]
    fn legacy_evaluate_never_calls_reset_motion() {
        let ticks: Vec<_> = (0..6).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        let segments = vec![Segment { start: 0, len: 3 }, Segment { start: 3, len: 3 }];
        // Segmented path: resets per segment, no motion, no full per-tick sets.
        let mut backend = MotionScheduleProbe::new(&ticks);
        evaluate(
            &mut backend,
            &ticks,
            &segments,
            false,
            true,
            true,
            true,
            false,
        );
        assert_eq!(backend.resets, 2);
        assert_eq!(backend.motions, 0);
        assert_eq!(backend.set_states, 0);
        assert_eq!(backend.refreshes, 2);
        assert_eq!(backend.body_restores, 2);
        // Full-reset path: per-tick set_state, segment-start refreshes only.
        let mut backend = MotionScheduleProbe::new(&ticks);
        evaluate(
            &mut backend,
            &ticks,
            &segments,
            true,
            true,
            true,
            true,
            false,
        );
        assert_eq!(backend.resets, 0);
        assert_eq!(backend.motions, 0);
        assert_eq!(backend.set_states, 4);
        assert_eq!(backend.refreshes, 2);
        assert_eq!(backend.body_restores, 4);
    }

    #[test]
    fn motion_skips_stasis_but_steps() {
        // Same stasis shape as `evaluate_skips_stasis_but_steps`: 1->2 is
        // kickoff stasis (stepped, unscored), 0->1 and 2->3 score.
        let ticks: Vec<TickRecord> = [99u32, 100, 101, 102]
            .iter()
            .map(|&f| {
                let x = match f {
                    99 => 0.0,
                    100 => 10.0,
                    101 => 10.5,
                    _ => 20.0,
                };
                let (ball_world, hxy) = if f == 100 || f == 101 {
                    (true, 0.0)
                } else {
                    (false, 200.0)
                };
                let mut t = make_tick(
                    f,
                    (x, 0.0, 17.0),
                    (0.0, 0.0, 92.75),
                    (hxy, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                    false,
                    ball_world,
                    true,
                    true,
                );
                let vel = match f {
                    100 => 1200.0,
                    102 => 1140.0,
                    _ => hxy,
                };
                t.car_records[0].phys.lin_vel = vec(vel, 0.0, 0.0);
                t.car_records.push(t.car_records[0].clone());
                t
            })
            .collect();
        assert!(tick_is_kickoff_stasis(&ticks[1], &ticks[2]));
        let segments = vec![Segment { start: 0, len: 4 }];
        let mut backend = MotionScheduleProbe::new(&ticks);
        let outcome =
            evaluate_motion_each_tick(&mut backend, &ticks, &segments, true, false, false, false);
        // Motion resets run for the skipped transition. The first live
        // transition after stasis uses one full seed at the last stasis row.
        assert_eq!(backend.resets, 2);
        assert_eq!(backend.motions, 2);
        assert_eq!(backend.steps, 3);
        assert_eq!(backend.set_states, 0);
        assert_eq!(backend.reset_rows, vec![0, 2]);
        assert_eq!(outcome.skipped_transitions, 1);
        assert_eq!(outcome.skipped_car_ticks, 2);
        assert_eq!(outcome.report.total.support, 4);
        assert_eq!(outcome.report.total.passed, 4);
    }
    #[test]
    fn evaluate_reseeds_after_stasis_before_live_release() {
        let ticks = stasis_release_ticks();
        assert!(tick_is_kickoff_stasis(&ticks[1], &ticks[2]));
        assert!(!tick_is_kickoff_stasis(&ticks[2], &ticks[3]));
        let segments = vec![Segment { start: 0, len: 5 }];

        let mut backend = MotionScheduleProbe::new(&ticks);
        let outcome = evaluate(
            &mut backend,
            &ticks,
            &segments,
            false,
            true,
            false,
            false,
            false,
        );
        // Full seed at the segment start and at the last countdown row.
        assert_eq!(backend.reset_rows, vec![0, 2]);
        assert_eq!(backend.resets, 2);
        assert_eq!(backend.motions, 0);
        assert_eq!(backend.set_states, 0);
        assert_eq!(backend.refreshes, 2);
        assert_eq!(backend.steps, 4);
        // The release and continuation apply their own target controls.
        assert_eq!(backend.step_controls[2], target_controls(&ticks[3]));
        assert_eq!(backend.step_controls[3], target_controls(&ticks[4]));
        // The live release and continuation score. One stasis pair is skipped.
        assert_eq!(outcome.report.total.support, 6);
        assert_eq!(outcome.report.total.passed, 6);
        assert_eq!(outcome.skipped_transitions, 1);
        assert_eq!(outcome.skipped_car_ticks, 2);

        // Reset-each-tick already restored every field. The guard must not
        // add a duplicate seed, and support must stay the same.
        let mut backend = MotionScheduleProbe::new(&ticks);
        let outcome = evaluate(
            &mut backend,
            &ticks,
            &segments,
            true,
            true,
            false,
            false,
            false,
        );
        assert_eq!(backend.resets, 0);
        assert_eq!(backend.set_states, 4);
        assert_eq!(backend.refreshes, 1);
        assert_eq!(outcome.report.total.support, 6);
        assert_eq!(outcome.report.total.passed, 6);
        assert_eq!(outcome.skipped_transitions, 1);
        assert_eq!(outcome.skipped_car_ticks, 2);
    }

    #[test]
    fn evaluate_motion_reseeds_after_stasis_before_live_release() {
        let ticks = stasis_release_ticks();
        let segments = vec![Segment { start: 0, len: 5 }];
        let mut backend = MotionScheduleProbe::new(&ticks);
        let outcome =
            evaluate_motion_each_tick(&mut backend, &ticks, &segments, true, false, false, false);
        // Segment seed at row 0 and one full release seed at row 2.
        assert_eq!(backend.reset_rows, vec![0, 2]);
        assert_eq!(backend.resets, 2);
        // Ordinary motion-only resets cover targets 1, 2, and 4. The release
        // at target 3 uses the full seed.
        assert_eq!(backend.motions, 3);
        assert_eq!(backend.steps, 4);
        assert_eq!(backend.set_states, 0);
        assert_eq!(backend.refreshes, 2);
        assert_eq!(backend.step_controls[2], target_controls(&ticks[3]));
        assert_eq!(backend.step_controls[3], target_controls(&ticks[4]));
        assert_eq!(outcome.report.total.support, 6);
        assert_eq!(outcome.report.total.passed, 6);
        assert_eq!(outcome.skipped_transitions, 1);
        assert_eq!(outcome.skipped_car_ticks, 2);
    }

    #[test]
    fn evaluate_does_not_reseed_after_frozen_separated_stasis() {
        let mut ticks = stasis_release_ticks();
        // Recorded corpus pattern: an exact-frozen row separates the stasis
        // span from the live release. A mask or teleport boundary has the
        // same effect because it starts a new segment.
        let mut frozen = ticks[2].clone();
        for car in &mut frozen.car_records {
            car.phys.physics_frame += 1;
        }
        frozen.ball_record.physics_frame += 1;
        frozen.level_tick = ticks[2].level_tick + 1;
        ticks[3] = frozen;
        let next_frame = ticks[3].car_records[0].phys.physics_frame + 1;
        for car in &mut ticks[4].car_records {
            car.phys.physics_frame = next_frame;
        }
        ticks[4].ball_record.physics_frame = next_frame;
        ticks[4].level_tick = ticks[3].level_tick + 1;

        assert!(tick_is_frozen(&ticks[2], &ticks[3]));
        let segments = split_segments(&ticks, config(8));
        assert_eq!(
            segments,
            vec![Segment { start: 0, len: 3 }, Segment { start: 3, len: 2 }]
        );

        let mut backend = MotionScheduleProbe::new(&ticks);
        let outcome = evaluate(
            &mut backend,
            &ticks,
            &segments,
            false,
            true,
            false,
            false,
            false,
        );
        // Segment seeds at rows 0 and 3. No extra seed at row 2.
        assert_eq!(backend.reset_rows, vec![0, 3]);
        assert_eq!(backend.resets, 2);
        assert_eq!(backend.steps, 3);
        assert_eq!(outcome.report.total.support, 4);
        assert_eq!(outcome.report.total.passed, 4);
        assert_eq!(outcome.skipped_transitions, 1);
        assert_eq!(outcome.skipped_car_ticks, 2);
    }
    #[test]
    fn motion_multi_car_support_and_boundaries() {
        // Two-car run over two segments: every target scores once per car.
        let mut ticks: Vec<_> = (0..6).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        for tick in &mut ticks {
            tick.car_records.push(tick.car_records[0].clone());
        }
        let segments = vec![Segment { start: 0, len: 3 }, Segment { start: 3, len: 3 }];
        let mut backend = MotionScheduleProbe::new(&ticks);
        let outcome =
            evaluate_motion_each_tick(&mut backend, &ticks, &segments, true, true, true, false);
        assert_eq!(backend.resets, 2);
        assert_eq!(backend.motions, 4);
        assert_eq!(backend.set_states, 0);
        // Four targets times two cars.
        assert_eq!(outcome.report.total.support, 8);
        assert_eq!(outcome.report.total.passed, 8);
        // Teleport boundary: tick 2 jumps away, so no segment spans 1->2.
        let mut split: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        split[2].car_records[0].phys.pos = vec(5000.0, 0.0, 100.0);
        split[3].car_records[0].phys.pos = vec(5010.0, 0.0, 100.0);
        let segments = split_segments(&split, config(8));
        for segment in &segments {
            let range = segment.start..segment.end();
            assert!(!(range.contains(&1) && range.contains(&2)));
        }
        // Motion mode keeps the same boundaries: one full seed per segment.
        let mut backend = MotionScheduleProbe::new(&split);
        let outcome =
            evaluate_motion_each_tick(&mut backend, &split, &segments, true, false, false, false);
        assert_eq!(backend.resets, segments.len());
        assert_eq!(backend.set_states, 0);
        assert_eq!(backend.motions, backend.steps);
        assert_eq!(
            outcome.report.total.support + outcome.skipped_transitions,
            backend.steps
        );
    }

    #[test]
    fn steps_use_target_prev_controls() {
        let mut ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        for (index, tick) in ticks.iter_mut().enumerate() {
            let controls = &mut tick.car_records[0].prev_controls;
            controls.throttle = index as f32 / 10.0;
            controls.steer = -(index as f32) / 10.0;
            controls.jump = index % 2 == 1;
        }
        let segments = vec![Segment { start: 0, len: 4 }];
        let expected: Vec<Vec<ControlsRecord>> = ticks[1..]
            .iter()
            .map(|tick| vec![tick.car_records[0].prev_controls])
            .collect();

        for reset_each_tick in [false, true] {
            let mut backend = MirrorBackend::new(&ticks);
            evaluate(
                &mut backend,
                &ticks,
                &segments,
                reset_each_tick,
                true,
                false,
                false,
                false,
            );
            assert_eq!(
                backend.controls, expected,
                "reset_each_tick={reset_each_tick}"
            );
        }
    }
}
