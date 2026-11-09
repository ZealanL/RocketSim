use rocketsim_test::rlpr::tick_record::TickRecord;

use super::rows::{
    MAX_SCORED_CARS, Segment, any_teleport, frame_is_contiguous, tick_car_count, tick_is_frozen,
    tick_is_kickoff_stasis,
};

/// Mechanical cause for one incoming transition that is not scored.
///
/// The order matches `split_segments`: the first true condition wins. This is
/// a reporting classification only and never changes scoring. A pair is not
/// called a demo from an unavailable endpoint; the endpoint cause is reported
/// as `unavailable_physics_ball`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TransitionCause {
    Unavailable,
    InvalidRoster,
    ChangedRoster,
    ChangedMask,
    FrameGap,
    Frozen,
    Teleport,
}

/// Per-recording transition counts. Counts are transitions, not car-ticks.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct TransitionCounts {
    total: usize,
    scored: usize,
    stepped_stasis: usize,
    unavailable_physics_ball: usize,
    invalid_roster: usize,
    changed_roster: usize,
    changed_availability_mask: usize,
    native_frame_gap: usize,
    frozen: usize,
    teleport: usize,
    unclassified: usize,
    contiguous_eligible: usize,
    boundary_seeds: usize,
}

impl TransitionCounts {
    fn excluded(&self) -> usize {
        self.unavailable_physics_ball
            + self.invalid_roster
            + self.changed_roster
            + self.changed_availability_mask
            + self.native_frame_gap
            + self.frozen
            + self.teleport
            + self.unclassified
    }

    fn reconciles(&self) -> bool {
        self.scored + self.stepped_stasis + self.excluded() == self.total
            && self.scored + self.stepped_stasis == self.contiguous_eligible
            && self.unclassified == 0
    }

    fn report_line(&self) -> String {
        format!(
            "transition report: transitions {} scored {} stepped_stasis {} excluded {} boundary_seeds {}; breaks unavailable_physics_ball {} invalid_roster {} changed_roster {} changed_availability_mask {} native_frame_gap {} frozen {} teleport {} unclassified {}; eligible {} covered {} reconciles {}",
            self.total,
            self.scored,
            self.stepped_stasis,
            self.excluded(),
            self.boundary_seeds,
            self.unavailable_physics_ball,
            self.invalid_roster,
            self.changed_roster,
            self.changed_availability_mask,
            self.native_frame_gap,
            self.frozen,
            self.teleport,
            self.unclassified,
            self.contiguous_eligible,
            self.scored + self.stepped_stasis,
            self.reconciles(),
        )
    }
}

fn transition_cause(prev: &TickRecord, tick: &TickRecord) -> Option<TransitionCause> {
    // Primary order matches `split_segments`.
    if !tick.has_physics() || !tick.ball_available() {
        return Some(TransitionCause::Unavailable);
    }
    let count = tick_car_count(tick);
    if count == 0 || count > MAX_SCORED_CARS {
        return Some(TransitionCause::InvalidRoster);
    }
    if !prev.has_physics() || !prev.ball_available() {
        return Some(TransitionCause::Unavailable);
    }
    let prev_count = tick_car_count(prev);
    if prev_count == 0 || prev_count > MAX_SCORED_CARS {
        return Some(TransitionCause::InvalidRoster);
    }
    if prev_count != count {
        return Some(TransitionCause::ChangedRoster);
    }
    if prev.availability_mask() != tick.availability_mask() {
        return Some(TransitionCause::ChangedMask);
    }
    if !frame_is_contiguous(prev, tick) {
        return Some(TransitionCause::FrameGap);
    }
    if tick_is_frozen(prev, tick) {
        return Some(TransitionCause::Frozen);
    }
    if any_teleport(prev, tick) {
        return Some(TransitionCause::Teleport);
    }
    None
}

fn flush_counted_run(end: usize, start: &mut Option<usize>, seeds: &mut usize) {
    if let Some(begin) = start.take()
        && end - begin >= 2
    {
        *seeds += 1;
    }
}

fn transition_counts(ticks: &[TickRecord], segments: &[Segment]) -> TransitionCounts {
    let mut counts = TransitionCounts {
        total: ticks.len().saturating_sub(1),
        ..TransitionCounts::default()
    };
    if ticks.len() < 2 {
        return counts;
    }

    // A transition is covered when it is an offset >= 1 in any actual
    // segment. Overlapping chunk seeds keep every eligible pair covered once.
    let mut target = vec![false; ticks.len()];
    for segment in segments {
        if segment.end() > ticks.len() {
            continue;
        }
        for offset in 1..segment.len {
            target[segment.start + offset] = true;
        }
    }

    // Boundary seeds: runs with at least one scored target. This mirrors
    // `push_chunks`, which keeps a run only when it has two or more rows.
    let mut run_start: Option<usize> = None;
    for (index, tick) in ticks.iter().enumerate() {
        if !tick.has_physics() || !tick.ball_available() {
            flush_counted_run(index, &mut run_start, &mut counts.boundary_seeds);
            continue;
        }
        let count = tick_car_count(tick);
        if count == 0 || count > MAX_SCORED_CARS {
            flush_counted_run(index, &mut run_start, &mut counts.boundary_seeds);
            continue;
        }
        match run_start {
            None => run_start = Some(index),
            Some(_) => {
                let prev = &ticks[index - 1];
                if tick_car_count(prev) != count
                    || prev.availability_mask() != tick.availability_mask()
                    || !frame_is_contiguous(prev, tick)
                    || tick_is_frozen(prev, tick)
                    || any_teleport(prev, tick)
                {
                    flush_counted_run(index, &mut run_start, &mut counts.boundary_seeds);
                    run_start = Some(index);
                }
            }
        }
    }
    flush_counted_run(ticks.len(), &mut run_start, &mut counts.boundary_seeds);

    for index in 1..ticks.len() {
        let prev = &ticks[index - 1];
        let tick = &ticks[index];
        let cause = transition_cause(prev, tick);
        if cause.is_none() {
            counts.contiguous_eligible += 1;
        }
        if target[index] {
            if cause.is_some() {
                counts.unclassified += 1;
            } else if tick_is_kickoff_stasis(prev, tick) {
                counts.stepped_stasis += 1;
            } else {
                counts.scored += 1;
            }
            continue;
        }
        match cause {
            Some(TransitionCause::Unavailable) => counts.unavailable_physics_ball += 1,
            Some(TransitionCause::InvalidRoster) => counts.invalid_roster += 1,
            Some(TransitionCause::ChangedRoster) => counts.changed_roster += 1,
            Some(TransitionCause::ChangedMask) => counts.changed_availability_mask += 1,
            Some(TransitionCause::FrameGap) => counts.native_frame_gap += 1,
            Some(TransitionCause::Frozen) => counts.frozen += 1,
            Some(TransitionCause::Teleport) => counts.teleport += 1,
            None => counts.unclassified += 1,
        }
    }
    counts
}

/// One mechanical transition summary for the CLI. Reporting only.
pub fn transition_report(ticks: &[TickRecord], segments: &[Segment]) -> String {
    transition_counts(ticks, segments).report_line()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::{evaluate, split_segments, test_support::*};

    /// One table row for the mechanical transition report.
    #[derive(Default)]
    struct TransitionCase {
        name: &'static str,
        ticks: Vec<TickRecord>,
        scored: usize,
        scored_car_ticks: usize,
        stasis: usize,
        stasis_car_ticks: usize,
        unavailable: usize,
        invalid_roster: usize,
        changed_roster: usize,
        changed_mask: usize,
        frame_gap: usize,
        frozen: usize,
        teleport: usize,
        boundary_seeds: usize,
    }

    fn teleport_ticks() -> Vec<TickRecord> {
        let mut ticks: Vec<_> = (0..4)
            .map(|i| multi_tick(i, i as f32 * 10.0, 2, 0b11))
            .collect();
        ticks[2].car_records[0].phys.pos = vec(1000.0, 0.0, 100.0);
        ticks[3].car_records[0].phys.pos = vec(1010.0, 0.0, 100.0);
        ticks
    }

    fn respawn_ticks() -> Vec<TickRecord> {
        [0b11, 0b11, 0b01, 0b11, 0b11]
            .iter()
            .enumerate()
            .map(|(i, &mask)| multi_tick(i as u32, i as f32 * 10.0, 2, mask))
            .collect()
    }

    fn frozen_ticks() -> Vec<TickRecord> {
        let mut ticks: Vec<_> = (0..2)
            .map(|i| multi_tick(i, i as f32 * 10.0, 2, 0b11))
            .collect();
        let mut frozen = ticks[1].clone();
        for car in &mut frozen.car_records {
            car.phys.physics_frame += 1;
        }
        frozen.ball_record.physics_frame += 1;
        frozen.level_tick += 1;
        ticks.push(frozen);
        ticks.push(multi_tick(3, 30.0, 2, 0b11));
        ticks
    }

    fn frame_gap_ticks() -> Vec<TickRecord> {
        vec![
            multi_tick(0, 0.0, 2, 0b11),
            multi_tick(1, 10.0, 2, 0b11),
            multi_tick(3, 20.0, 2, 0b11),
            multi_tick(4, 30.0, 2, 0b11),
        ]
    }

    fn changed_roster_ticks() -> Vec<TickRecord> {
        vec![
            multi_tick(0, 0.0, 2, 0b11),
            multi_tick(1, 10.0, 2, 0b11),
            multi_tick(2, 20.0, 1, 0b01),
            multi_tick(3, 30.0, 1, 0b01),
        ]
    }

    fn invalid_roster_ticks() -> Vec<TickRecord> {
        vec![
            multi_tick(0, 0.0, 9, 0x1FF),
            multi_tick(1, 10.0, 2, 0b11),
            multi_tick(2, 20.0, 2, 0b11),
        ]
    }

    fn unavailable_ticks() -> Vec<TickRecord> {
        vec![
            multi_tick(0, 0.0, 2, 0b11),
            guard_tick(1, 2),
            multi_tick(2, 20.0, 2, 0b11),
            multi_tick(3, 30.0, 2, 0b11),
        ]
    }

    #[test]
    fn transition_report_matches_segments_and_scoring() {
        let cases = vec![
            TransitionCase {
                name: "teleport",
                ticks: teleport_ticks(),
                scored: 2,
                scored_car_ticks: 4,
                teleport: 1,
                boundary_seeds: 2,
                ..TransitionCase::default()
            },
            TransitionCase {
                name: "respawn_availability",
                ticks: respawn_ticks(),
                scored: 2,
                scored_car_ticks: 4,
                changed_mask: 2,
                boundary_seeds: 2,
                ..TransitionCase::default()
            },
            TransitionCase {
                name: "frozen",
                ticks: frozen_ticks(),
                scored: 2,
                scored_car_ticks: 4,
                frozen: 1,
                boundary_seeds: 2,
                ..TransitionCase::default()
            },
            TransitionCase {
                name: "native_frame_gap",
                ticks: frame_gap_ticks(),
                scored: 2,
                scored_car_ticks: 4,
                frame_gap: 1,
                boundary_seeds: 2,
                ..TransitionCase::default()
            },
            TransitionCase {
                name: "countdown_direct_release",
                ticks: stasis_release_ticks(),
                scored: 3,
                scored_car_ticks: 6,
                stasis: 1,
                stasis_car_ticks: 2,
                boundary_seeds: 1,
                ..TransitionCase::default()
            },
            TransitionCase {
                name: "changed_roster",
                ticks: changed_roster_ticks(),
                scored: 2,
                scored_car_ticks: 3,
                changed_roster: 1,
                boundary_seeds: 2,
                ..TransitionCase::default()
            },
            TransitionCase {
                name: "invalid_roster",
                ticks: invalid_roster_ticks(),
                scored: 1,
                scored_car_ticks: 2,
                invalid_roster: 1,
                boundary_seeds: 1,
                ..TransitionCase::default()
            },
            TransitionCase {
                name: "unavailable_physics_ball",
                ticks: unavailable_ticks(),
                scored: 1,
                scored_car_ticks: 2,
                unavailable: 2,
                boundary_seeds: 1,
                ..TransitionCase::default()
            },
        ];

        for case in cases {
            let segments = split_segments(&case.ticks, config(120));
            let report = transition_counts(&case.ticks, &segments);
            assert_eq!(report.total, case.ticks.len() - 1, "{} total", case.name);
            assert_eq!(report.scored, case.scored, "{} scored", case.name);
            assert_eq!(
                report.stepped_stasis, case.stasis,
                "{} stepped stasis",
                case.name
            );
            assert_eq!(
                report.unavailable_physics_ball, case.unavailable,
                "{} unavailable",
                case.name
            );
            assert_eq!(
                report.invalid_roster, case.invalid_roster,
                "{} invalid roster",
                case.name
            );
            assert_eq!(
                report.changed_roster, case.changed_roster,
                "{} changed roster",
                case.name
            );
            assert_eq!(
                report.changed_availability_mask, case.changed_mask,
                "{} changed mask",
                case.name
            );
            assert_eq!(
                report.native_frame_gap, case.frame_gap,
                "{} frame gap",
                case.name
            );
            assert_eq!(report.frozen, case.frozen, "{} frozen", case.name);
            assert_eq!(report.teleport, case.teleport, "{} teleport", case.name);
            assert_eq!(
                report.excluded(),
                case.unavailable
                    + case.invalid_roster
                    + case.changed_roster
                    + case.changed_mask
                    + case.frame_gap
                    + case.frozen
                    + case.teleport,
                "{} excluded",
                case.name
            );
            assert_eq!(
                report.boundary_seeds, case.boundary_seeds,
                "{} boundary seeds",
                case.name
            );
            assert!(
                report.reconciles(),
                "{} reconciliation: {}",
                case.name,
                report.report_line()
            );

            let mut backend = MotionScheduleProbe::new(&case.ticks);
            let outcome = evaluate(
                &mut backend,
                &case.ticks,
                &segments,
                false,
                true,
                false,
                false,
                false,
            );
            assert_eq!(
                outcome.report.total.support, case.scored_car_ticks,
                "{} scored car-ticks",
                case.name
            );
            assert_eq!(
                outcome.report.total.passed, case.scored_car_ticks,
                "{} passed car-ticks",
                case.name
            );
            assert_eq!(
                outcome.skipped_transitions, case.stasis,
                "{} skipped transitions",
                case.name
            );
            assert_eq!(
                outcome.skipped_car_ticks, case.stasis_car_ticks,
                "{} skipped car-ticks",
                case.name
            );
        }
    }
}
