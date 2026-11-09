use glam::Vec3A;
use rocketsim_test::rlpr::tick_record::TickRecord;

// Max plausible per-tick travel in Unreal units. Fastest ball (~6000 UU/s)
// covers ~50 UU per 120 Hz tick; supersonic cars ~20 UU. Anything beyond
// this is a teleport: kickoff/goal reset, demo respawn, or respawn snap.
// Teleports always break runs (see split_segments) so no scored tick ever
// spans one, and every post-teleport run starts with a fresh state-set.
pub const CAR_TELEPORT_DIST: f32 = 500.0;
pub const BALL_TELEPORT_DIST: f32 = 500.0;
/// Segment length config.
#[derive(Clone, Copy, Debug)]
pub struct SegmentConfig {
    pub segment_ticks: usize,
}

impl SegmentConfig {
    /// Segment length must be non-zero (zero would never advance chunking).
    pub fn is_valid(&self) -> bool {
        self.segment_ticks > 0
    }
}

/// Non-overlapping run of recording ticks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Segment {
    pub start: usize,
    pub len: usize,
}

impl Segment {
    /// End index (exclusive).
    pub fn end(&self) -> usize {
        self.start + self.len
    }
}

/// Number of cars in the tick.
pub fn tick_car_count(tick: &TickRecord) -> usize {
    tick.car_records.len()
}
/// Max cars scored per tick.
pub const MAX_SCORED_CARS: usize = 8;
/// Physics frames advance by one for every available car and the ball.
///
/// A mask change, a no-physics row, or an unavailable ball breaks the run.
/// For v2-v10 every car and the ball are available, so the old behavior is
/// unchanged.
pub fn frame_is_contiguous(from: &TickRecord, to: &TickRecord) -> bool {
    if !from.has_physics() || !to.has_physics() {
        return false;
    }
    if !from.ball_available() || !to.ball_available() {
        return false;
    }
    if from.availability_mask() != to.availability_mask() {
        return false;
    }
    let n = from.car_records.len();
    if n == 0 || n > MAX_SCORED_CARS || to.car_records.len() != n {
        return false;
    }
    from.car_records
        .iter()
        .zip(to.car_records.iter())
        .all(|(a, b)| !a.is_available() || b.phys.physics_frame == a.phys.physics_frame + 1)
        && to.ball_record.physics_frame == from.ball_record.physics_frame + 1
}

/// No body moved between ticks (pause or replay stall).
pub fn tick_is_frozen(from: &TickRecord, to: &TickRecord) -> bool {
    let n = from.car_records.len();
    if n == 0 || to.car_records.len() != n {
        return false;
    }
    let cars_static = from
        .car_records
        .iter()
        .zip(to.car_records.iter())
        .filter(|(a, _)| a.is_available())
        .all(|(a, b)| {
            a.phys.pos == b.phys.pos
                && a.phys.lin_vel == b.phys.lin_vel
                && a.phys.ang_vel == b.phys.ang_vel
        });
    cars_static
        && from.ball_record.pos == to.ball_record.pos
        && from.ball_record.lin_vel == to.ball_record.lin_vel
        && from.ball_record.ang_vel == to.ball_record.ang_vel
}

/// Kickoff countdown stasis, rule `kickoff-stasis-v1`. Metric only.
///
/// True when the game step is paused at kickoff while the recorder still
/// emits live controls and wheel contact flags. The ball sits unchanged
/// at the kickoff spot with a world-contact flag. Every car creeps with
/// epsilon motion and near-zero horizontal velocity. Exact `tick_is_frozen`
/// misses these rows because cars drift up to ~0.67 UU per tick. The
/// horizontal-velocity and ball-contact gates separate countdown spans
/// from genuine drive-off (ball contact false, horizontal speed 20+ UU/s)
/// and from slow play elsewhere (ball off center or cars driving).
/// Epsilon-only: exact-frozen pairs return false and stay on that path.
pub const KICKOFF_STASIS_RULE: &str = "kickoff-stasis-v1";
pub const KICKOFF_STASIS_MAX_CAR_MOVE: f32 = 1.0;
pub const KICKOFF_STASIS_MAX_CAR_SPEED: f32 = 100.0;
pub const KICKOFF_STASIS_MAX_CAR_HXY: f32 = 5.0;
pub const KICKOFF_STASIS_BALL_CENTER_TOL: f32 = 1.0;

/// True when one transition is kickoff countdown stasis. See above.
pub fn tick_is_kickoff_stasis(from: &TickRecord, to: &TickRecord) -> bool {
    if tick_is_frozen(from, to) {
        return false;
    }
    let n = from.car_records.len();
    if n == 0 || to.car_records.len() != n {
        return false;
    }
    if !frame_is_contiguous(from, to) || any_teleport(from, to) {
        return false;
    }
    let from_ball: Vec3A = from.ball_record.pos.into();
    let to_ball: Vec3A = to.ball_record.pos.into();
    if (to_ball - from_ball).length() != 0.0 {
        return false;
    }
    if to_ball.x.abs() > KICKOFF_STASIS_BALL_CENTER_TOL
        || to_ball.y.abs() > KICKOFF_STASIS_BALL_CENTER_TOL
    {
        return false;
    }
    if !to.ball_record.has_world_contact {
        return false;
    }
    from.car_records
        .iter()
        .zip(to.car_records.iter())
        .filter(|(a, _)| a.is_available())
        .all(|(a, b)| {
            let pa: Vec3A = a.phys.pos.into();
            let pb: Vec3A = b.phys.pos.into();
            if (pb - pa).length() >= KICKOFF_STASIS_MAX_CAR_MOVE {
                return false;
            }
            let v: Vec3A = b.phys.lin_vel.into();
            if v.length() >= KICKOFF_STASIS_MAX_CAR_SPEED {
                return false;
            }
            if glam::Vec2::new(v.x, v.y).length() >= KICKOFF_STASIS_MAX_CAR_HXY {
                return false;
            }
            b.wheels.iter().any(|wheel| wheel.has_contact)
        })
}

/// Target tick indices with a kickoff-stasis incoming transition.
/// One entry per detected transition.
/// The aggregate-only CLI does not report per-file spans; unit tests cover this.
#[allow(dead_code)]
pub fn kickoff_stasis_targets(ticks: &[TickRecord]) -> Vec<usize> {
    let mut out = Vec::new();
    for i in 1..ticks.len() {
        if tick_is_kickoff_stasis(&ticks[i - 1], &ticks[i]) {
            out.push(i);
        }
    }
    out
}

/// Collapse verbatim duplicate rows.
///
/// Drops a tick only when it is frozen relative to the previous kept row
/// ([`tick_is_frozen`]) and every car and ball physics frame is equal
/// ([`tick_frames_equal`]). A frame-advancing frozen row is a recorded step
/// and stays. `split_segments` breaks runs at frozen transitions, so a kept
/// freeze row cannot score a frozen step. Same-frame duplicate runs keep
/// their first row, as before.
///
/// This is metric preprocessing, not parsing: [`Recording::from_file`] keeps
/// the file faithful. Some recorders emit thousands of repeated rows while
/// others never do, so the metric normalizes before scoring.
pub fn collapse_duplicate_ticks(ticks: &[TickRecord]) -> Vec<TickRecord> {
    let mut out = Vec::with_capacity(ticks.len());
    for tick in ticks {
        let duplicate = out
            .last()
            .is_some_and(|prev| tick_is_frozen(prev, tick) && tick_frames_equal(prev, tick));
        if !duplicate {
            out.push(tick.clone());
        }
    }
    out
}

/// True when all car and ball physics frames match.
fn tick_frames_equal(a: &TickRecord, b: &TickRecord) -> bool {
    a.car_records.len() == b.car_records.len()
        && a.car_records
            .iter()
            .zip(&b.car_records)
            .all(|(x, y)| x.phys.physics_frame == y.phys.physics_frame)
        && a.ball_record.physics_frame == b.ball_record.physics_frame
}
/// Split ticks into non-overlapping segments.
///
/// Break runs at frame gaps, frozen transitions, ticks without cars,
/// ticks with too many cars, car-count changes, and teleports (any car
/// or the ball jumping further than one tick of travel allows: kickoff
/// and goal resets, demo respawns). A teleport arrival always starts a
/// new run, so the backend state-sets it before scoring resumes and no
/// scored tick ever spans the discontinuity.
/// Chunk each run into groups of `segment_ticks`.
/// Drop groups with no scored ticks. Reset only at segment starts.
pub fn split_segments(ticks: &[TickRecord], config: SegmentConfig) -> Vec<Segment> {
    if !config.is_valid() {
        return Vec::new();
    }
    let mut segments = Vec::new();
    let mut run_start: Option<usize> = None;

    let mut flush_run = |end: usize, run_start: &mut Option<usize>| {
        if let Some(start) = run_start.take() {
            push_chunks(&mut segments, start, end, config);
        }
    };

    for (index, tick) in ticks.iter().enumerate() {
        if !tick.has_physics() || !tick.ball_available() {
            flush_run(index, &mut run_start);
            continue;
        }
        let count = tick_car_count(tick);
        if count == 0 || count > MAX_SCORED_CARS {
            flush_run(index, &mut run_start);
            continue;
        }
        match run_start {
            None => {
                run_start = Some(index);
            }
            Some(_) => {
                let prev = &ticks[index - 1];
                if tick_car_count(prev) != count
                    || prev.availability_mask() != tick.availability_mask()
                    || !frame_is_contiguous(prev, tick)
                    || tick_is_frozen(prev, tick)
                    || any_teleport(prev, tick)
                {
                    flush_run(index, &mut run_start);
                    run_start = Some(index);
                }
            }
        }
    }
    flush_run(ticks.len(), &mut run_start);
    segments
}

/// Any car or the ball teleported between ticks (reset/respawn snap).
/// Counts are equal here; split_segments guarantees it before calling.
pub fn any_teleport(from: &TickRecord, to: &TickRecord) -> bool {
    if ball_teleported(from, to) {
        return true;
    }
    from.car_records
        .iter()
        .zip(to.car_records.iter())
        .filter(|(a, _)| a.is_available())
        .any(|(a, b)| {
            let pa: Vec3A = a.phys.pos.into();
            let pb: Vec3A = b.phys.pos.into();
            (pa - pb).length() >= CAR_TELEPORT_DIST
        })
}

/// The ball jumped further than one tick of travel allows.
fn ball_teleported(from: &TickRecord, to: &TickRecord) -> bool {
    let pa: Vec3A = from.ball_record.pos.into();
    let pb: Vec3A = to.ball_record.pos.into();
    (pa - pb).length() >= BALL_TELEPORT_DIST
}

/// Find the start of the clean run that contains `segment_start`.
///
/// A segment can start in the middle of a run. Do not use the segment start
/// as the handbrake history start in that case. Stop at the same boundaries
/// used by [`split_segments`].
pub fn run_start(ticks: &[TickRecord], segment_start: usize) -> usize {
    let mut start = segment_start.min(ticks.len());
    if start == ticks.len() {
        return start;
    }
    while start > 0 {
        let from = &ticks[start - 1];
        let to = &ticks[start];
        if tick_car_count(from) != tick_car_count(to)
            || !to.has_physics()
            || !to.ball_available()
            || from.availability_mask() != to.availability_mask()
            || !frame_is_contiguous(from, to)
            || tick_is_frozen(from, to)
            || any_teleport(from, to)
        {
            break;
        }
        start -= 1;
    }
    start
}

/// Reconstruct the handbrake integrator at one recorded state.
///
/// Versions v2-v7 do not store `handbrake_val`. Version v8 records it.
/// Use this fallback only when the format lacks direct state.
/// `prev_controls` at tick `i` is the control used to produce tick `i`,
/// so include tick `state_index`.
/// Integrate from the true contiguous `run_start`. Alternating histories
/// can retain state older than 60 ticks, so do not truncate the window.
/// At a run boundary, assume the value before the run was zero.
pub fn reconstruct_handbrake(
    ticks: &[TickRecord],
    run_start: usize,
    state_index: usize,
    car_idx: usize,
) -> Option<f32> {
    if state_index >= ticks.len() || run_start > state_index {
        return None;
    }

    let first = run_start;
    let mut value = 0.0;
    for tick in &ticks[first..=state_index] {
        let car = tick.car_records.get(car_idx)?;
        let delta = if car.prev_controls.handbrake {
            rocketsim::consts::car::drive::POWERSLIDE_RISE_RATE
        } else {
            -rocketsim::consts::car::drive::POWERSLIDE_FALL_RATE
        } * rocketsim::consts::TICK_TIME;
        value = (value + delta).clamp(0.0, 1.0);
    }
    Some(value)
}

/// Chunk one clean run. Every tick after the reset is scored, so every
/// non-empty chunk is kept.
fn push_chunks(segments: &mut Vec<Segment>, start: usize, end: usize, config: SegmentConfig) {
    let mut offset = start;
    while end - offset >= 2 {
        // A chunk needs a reset tick plus at least one scored tick, so
        // clamp the length to >= 2. This also guarantees `offset` advances
        // every iteration: with segment_ticks = 1, `len` was 1 and
        // `offset += len - 1` never moved (infinite loop pushing identical
        // chunks). Lengths >= 2 behave exactly as before.
        let len = config.segment_ticks.max(2).min(end - offset);
        segments.push(Segment { start: offset, len });
        offset += len - 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::test_support::*;

    #[test]
    fn split_segments_breaks_at_guard_and_mask_change() {
        let ticks = vec![
            multi_tick(0, 0.0, 6, 0b11_1111),
            multi_tick(1, 10.0, 6, 0b11_1111),
            multi_tick(2, 20.0, 6, 0b11_1111),
            guard_tick(3, 6),
            multi_tick(5, 50.0, 6, 0b11_0111),
            multi_tick(6, 60.0, 6, 0b11_0111),
            multi_tick(7, 70.0, 6, 0b11_1111),
            multi_tick(8, 80.0, 6, 0b11_1111),
        ];
        let segments = split_segments(&ticks, config(100));
        assert_eq!(
            segments,
            vec![
                Segment { start: 0, len: 3 },
                Segment { start: 4, len: 2 },
                Segment { start: 6, len: 2 },
            ]
        );
        // No segment spans a guard row or an availability change.
        for segment in &segments {
            let mask = ticks[segment.start].availability_mask();
            for i in segment.start..segment.end() {
                assert_eq!(ticks[i].availability_mask(), mask);
                assert!(ticks[i].has_physics());
                assert!(ticks[i].ball_available());
                if i > segment.start {
                    assert!(frame_is_contiguous(&ticks[i - 1], &ticks[i]));
                }
            }
        }
    }
    #[test]
    fn splits_clean_run_into_overlapping_chunks() {
        let ticks: Vec<_> = (0..10).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        let segments = split_segments(&ticks, config(4));
        assert_eq!(
            segments,
            vec![
                Segment { start: 0, len: 4 },
                Segment { start: 3, len: 4 },
                Segment { start: 6, len: 4 },
            ]
        );
    }

    #[test]
    fn keeps_short_tail_chunks() {
        // Every tick after the reset is scored, so even a short tail chunk
        // is kept (warmup used to drop chunks with no scored ticks). The
        // tail overlaps by one tick, so tick 4 is still scored.
        let ticks: Vec<_> = (0..5).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        let segments = split_segments(&ticks, config(4));
        assert_eq!(
            segments,
            vec![Segment { start: 0, len: 4 }, Segment { start: 3, len: 2 }]
        );
    }

    #[test]
    fn rejects_invalid_config() {
        let ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32)).collect();
        assert!(split_segments(&ticks, config(0)).is_empty());
        assert!(!split_segments(&ticks, config(4)).is_empty());
        assert!(!split_segments(&ticks, config(1)).is_empty());
    }

    #[test]
    fn breaks_at_frame_gap_and_missing_car() {
        let mut ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        ticks.push(quiet_tick(10, 40.0));
        ticks.push(quiet_tick(11, 50.0));
        ticks.push(TickRecord {
            level_tick: 0,
            row_flags: rocketsim_test::rlpr::ROW_FLAGS_BALL_AVAILABLE,
            reserved0: 0,
            car_records: vec![],
            ball_record: ticks.last().unwrap().ball_record,
            replay_state: None,
        });
        ticks.push(quiet_tick(12, 60.0));
        ticks.push(quiet_tick(13, 70.0));
        let segments = split_segments(&ticks, config(4));
        for window in segments.windows(2) {
            assert!(window[0].end() <= window[1].start);
        }
        for segment in &segments {
            for i in segment.start..segment.end() {
                assert_eq!(ticks[i].car_records.len(), 1);
                if i > segment.start {
                    assert!(frame_is_contiguous(&ticks[i - 1], &ticks[i]));
                }
            }
        }
        // Missing-car tick is excluded; segments stay contiguous.
        assert!(segments.iter().all(|s| s.start != 6 || s.len == 1));
        assert!(!segments.iter().any(|s| (s.start..s.end()).contains(&6)));
    }
    #[test]
    fn breaks_at_frozen_transition() {
        let mut ticks: Vec<_> = (0..3).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        let mut frozen = ticks[2].clone();
        frozen.car_records[0].phys.physics_frame = 3;
        frozen.ball_record.physics_frame = 3;
        ticks.push(frozen);
        ticks.push(quiet_tick(4, 40.0));
        let segments = split_segments(&ticks, config(8));
        for segment in &segments {
            let range = segment.start..segment.end();
            assert!(!(range.contains(&2) && range.contains(&3)));
        }
    }
    #[test]
    fn splits_on_car_teleport() {
        let mut ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        // Tick 2 snaps 5000 UU away with contiguous frames: a reset, not play.
        ticks[2].car_records[0].phys.pos = vec(5000.0, 0.0, 100.0);
        ticks[3].car_records[0].phys.pos = vec(5010.0, 0.0, 100.0);
        let segments = split_segments(&ticks, config(8));
        for segment in &segments {
            let range = segment.start..segment.end();
            assert!(!(range.contains(&1) && range.contains(&2)));
        }
    }

    #[test]
    fn splits_on_ball_teleport() {
        let mut ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 10.0)).collect();
        ticks[2].ball_record.pos = vec(0.0, 5000.0, 100.0);
        ticks[3].ball_record.pos = vec(0.0, 5010.0, 100.0);
        let segments = split_segments(&ticks, config(8));
        for segment in &segments {
            let range = segment.start..segment.end();
            assert!(!(range.contains(&1) && range.contains(&2)));
        }
    }

    #[test]
    fn fast_legal_motion_keeps_run() {
        // 400 UU per tick is fast but legal: no teleport split.
        let ticks: Vec<_> = (0..4).map(|i| quiet_tick(i, i as f32 * 400.0)).collect();
        let segments = split_segments(&ticks, config(8));
        assert_eq!(segments, vec![Segment { start: 0, len: 4 }]);
    }
    #[test]
    fn reconstructs_handbrake_with_sixty_tick_fall_and_run_boundary() {
        let mut ticks: Vec<_> = (0..84).map(|i| quiet_tick(i, i as f32)).collect();
        for tick in ticks.iter_mut().take(24) {
            tick.car_records[0].prev_controls.handbrake = true;
        }
        assert_eq!(reconstruct_handbrake(&ticks, 0, 23, 0), Some(1.0));
        let washed = reconstruct_handbrake(&ticks, 0, 83, 0).unwrap();
        assert!(washed.abs() < 1e-6, "washout {washed}");

        ticks[0].car_records[0].prev_controls.handbrake = true;
        ticks[1].car_records[0].prev_controls.handbrake = false;
        let bounded = reconstruct_handbrake(&ticks, 1, 1, 0).unwrap();
        assert_eq!(bounded, 0.0);
    }

    #[test]
    fn reconstructs_handbrake_from_history_older_than_sixty_ticks() {
        // 70 alternating ticks stay below saturation: full run keeps the
        // first 5 press cycles that a 60-tick window drops.
        let mut ticks: Vec<_> = (0..70).map(|i| quiet_tick(i, i as f32)).collect();
        for (i, tick) in ticks.iter_mut().enumerate() {
            tick.car_records[0].prev_controls.handbrake = i % 2 == 0;
        }
        let full = reconstruct_handbrake(&ticks, 0, 69, 0).unwrap();
        let truncated = reconstruct_handbrake(&ticks, 10, 69, 0).unwrap();
        assert!((full - 0.875).abs() < 1e-5, "full run value {full}");
        assert!(
            (truncated - 0.75).abs() < 1e-5,
            "truncated value {truncated}"
        );
        assert!((full - truncated - 0.125).abs() < 1e-5);
    }
    #[test]
    fn collapse_duplicate_ticks_drops_only_zero_information_rows() {
        let mut dup = steady_tick(1, 10.0, 1200.0);
        let ticks = vec![
            steady_tick(0, 0.0, 1200.0),
            steady_tick(1, 10.0, 1200.0),
            dup.clone(),
            steady_tick(2, 20.0, 1200.0),
        ];
        let collapsed = collapse_duplicate_ticks(&ticks);
        assert_eq!(collapsed.len(), 3);
        assert_eq!(collapsed[1].car_records[0].phys.pos.x, 10.0);
        // Same frame but different state is real data, not a duplicate:
        // collapse keys on state equality, never on frames.
        dup.ball_record.physics_frame = 0;
        dup.car_records[0].phys.physics_frame = 0;
        let ticks = vec![steady_tick(0, 0.0, 1200.0), dup];
        assert_eq!(collapse_duplicate_ticks(&ticks).len(), 2);
        let _ = dup;
    }
    fn advancing_freeze_ticks() -> Vec<TickRecord> {
        let ball = (300.0, 0.0, 500.0);
        vec![
            make_tick(
                0,
                (0.0, 0.0, 100.0),
                ball,
                (100.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                false,
                false,
                false,
                false,
            ),
            make_tick(
                1,
                (20.0, 0.0, 100.0),
                ball,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                false,
                false,
                false,
                false,
            ),
            make_tick(
                2,
                (20.0, 0.0, 100.0),
                ball,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                false,
                false,
                false,
                false,
            ),
            make_tick(
                3,
                (20.0, 40.0, 100.0),
                ball,
                (0.0, 200.0, 0.0),
                (0.0, 0.0, 0.0),
                false,
                false,
                false,
                false,
            ),
        ]
    }

    #[test]
    fn collapse_preserves_advancing_frames_and_scores_both_steps() {
        // P -> A -> B -> S, with A and B frozen but on advancing frames.
        // Both the entering step P -> A and the resumed step B -> S score.
        let ticks = advancing_freeze_ticks();
        let collapsed = collapse_duplicate_ticks(&ticks);

        assert_eq!(collapsed.len(), ticks.len());
        let frames: Vec<u32> = collapsed
            .iter()
            .map(|tick| tick.ball_record.physics_frame)
            .collect();
        assert_eq!(frames, vec![0, 1, 2, 3]);

        let segments = split_segments(&collapsed, config(8));
        assert_eq!(
            segments,
            vec![Segment { start: 0, len: 2 }, Segment { start: 2, len: 2 }]
        );
        let scored_frames: Vec<u32> = segments
            .iter()
            .map(|segment| collapsed[segment.start + 1].ball_record.physics_frame)
            .collect();
        assert_eq!(scored_frames, vec![1, 3]);
    }

    #[test]
    fn collapse_mixed_same_frame_then_advancing_keeps_both_steps() {
        // P -> A(f1) -> B(f1) -> C(f2) -> S(f3). B is a verbatim duplicate.
        // A and C are frame-advancing rows and must stay.
        let ball = (300.0, 0.0, 500.0);
        let p = make_tick(
            0,
            (0.0, 0.0, 100.0),
            ball,
            (100.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            false,
            false,
            false,
        );
        let a = make_tick(
            1,
            (20.0, 0.0, 100.0),
            ball,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            false,
            false,
            false,
        );
        let b = a.clone();
        let c = make_tick(
            2,
            (20.0, 0.0, 100.0),
            ball,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            false,
            false,
            false,
        );
        let s = make_tick(
            3,
            (20.0, 40.0, 100.0),
            ball,
            (0.0, 200.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            false,
            false,
            false,
        );
        let collapsed = collapse_duplicate_ticks(&[p, a, b, c, s]);

        assert_eq!(collapsed.len(), 4);
        let frames: Vec<u32> = collapsed
            .iter()
            .map(|tick| tick.ball_record.physics_frame)
            .collect();
        assert_eq!(frames, vec![0, 1, 2, 3]);

        let segments = split_segments(&collapsed, config(8));
        assert_eq!(
            segments,
            vec![Segment { start: 0, len: 2 }, Segment { start: 2, len: 2 }]
        );
        let scored_frames: Vec<u32> = segments
            .iter()
            .map(|segment| collapsed[segment.start + 1].ball_record.physics_frame)
            .collect();
        assert_eq!(scored_frames, vec![1, 3]);
    }

    #[test]
    fn collapse_keeps_first_same_frame_duplicate_for_control_variants() {
        // Same-frame rows carry no physics step. The current first-row
        // behavior stays, even when the duplicate carries new controls.
        for throttle in [0.0, 1.0] {
            let a = quiet_tick(0, 0.0);
            let mut b = quiet_tick(0, 0.0);
            b.car_records[0].prev_controls.throttle = throttle;
            let c = quiet_tick(1, 10.0);
            let collapsed = collapse_duplicate_ticks(&[a, b, c]);

            assert_eq!(collapsed.len(), 2);
            assert_eq!(collapsed[0].ball_record.physics_frame, 0);
            assert_eq!(collapsed[0].car_records[0].prev_controls.throttle, 0.0);
        }
    }

    #[test]
    fn collapse_does_not_merge_same_frame_state_change() {
        // Different state is real data even when the frame is unchanged.
        let a = quiet_tick(0, 0.0);
        let b = quiet_tick(0, 10.0);
        let c = quiet_tick(1, 20.0);
        let collapsed = collapse_duplicate_ticks(&[a, b, c]);

        assert_eq!(collapsed.len(), 3);
    }
    fn kickoff_pair(
        car_move: f32,
        car_hxy: f32,
        car_vz: f32,
        ball_pos: (f32, f32, f32),
        ball_world: bool,
        wheels: bool,
    ) -> (TickRecord, TickRecord) {
        let mut from = make_tick(
            100,
            (2048.0, -2560.0, 17.0),
            ball_pos,
            (car_hxy, 0.0, car_vz),
            (0.0, 0.0, 0.0),
            false,
            ball_world,
            true,
            wheels,
        );
        from.car_records[0].phys.lin_vel = vec(car_hxy, 0.0, car_vz);
        let mut to = make_tick(
            101,
            (2048.0 + car_move, -2560.0, 17.0),
            ball_pos,
            (car_hxy, 0.0, car_vz),
            (0.0, 0.0, 0.0),
            false,
            ball_world,
            true,
            wheels,
        );
        to.car_records[0].phys.lin_vel = vec(car_hxy, 0.0, car_vz);
        (from, to)
    }

    #[test]
    fn stasis_flags_observed_spawn_variants() {
        let spawns = [
            (2048.0, -2560.0),
            (-256.0, -3840.0),
            (256.0, -3840.0),
            (-2048.0, -2560.0),
            (0.0, -4608.0),
        ];
        for (x, y) in spawns {
            let from = make_tick(
                100,
                (x, y, 17.0),
                (0.0, 0.0, 92.75),
                (0.0, 0.0, -79.88),
                (0.0, 0.0, 0.0),
                false,
                true,
                true,
                true,
            );
            let mut to = from.clone();
            to.car_records[0].phys.pos.x += 0.5;
            to.car_records[0].phys.physics_frame = 101;
            to.ball_record.physics_frame = 101;
            assert!(!tick_is_frozen(&from, &to));
            assert!(tick_is_kickoff_stasis(&from, &to));
        }
    }

    #[test]
    fn stasis_rejects_driveoff_idle_and_edges() {
        let (from, to) = kickoff_pair(0.3, 30.0, 0.0, (0.0, 0.0, 92.75), false, true);
        assert!(!tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(0.3, 30.0, 0.0, (0.0, 0.0, 92.75), true, true);
        assert!(!tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(0.3, 0.0, 0.0, (1000.0, 500.0, 92.75), true, true);
        assert!(!tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(5.0, 200.0, 0.0, (0.0, 0.0, 92.75), true, true);
        assert!(!tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(0.99, 0.0, 0.0, (0.0, 0.0, 92.75), true, true);
        assert!(tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(1.01, 0.0, 0.0, (0.0, 0.0, 92.75), true, true);
        assert!(!tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(0.3, 4.9, 0.0, (0.0, 0.0, 92.75), true, true);
        assert!(tick_is_kickoff_stasis(&from, &to));
        let (from, to) = kickoff_pair(0.3, 5.1, 0.0, (0.0, 0.0, 92.75), true, true);
        assert!(!tick_is_kickoff_stasis(&from, &to));
        let from = make_tick(
            100,
            (2048.0, -2560.0, 17.0),
            (0.0, 0.0, 92.75),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            true,
            true,
            true,
        );
        let mut to = from.clone();
        to.car_records[0].phys.physics_frame = 101;
        to.ball_record.physics_frame = 101;
        assert!(tick_is_frozen(&from, &to));
        assert!(!tick_is_kickoff_stasis(&from, &to));
    }

    #[test]
    fn stasis_keeps_segmentation_and_seeds() {
        let lead = make_tick(
            99,
            (0.0, 0.0, 17.0),
            (0.0, 0.0, 92.75),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            true,
            true,
            true,
        );
        let from = make_tick(
            100,
            (10.0, 0.0, 17.0),
            (0.0, 0.0, 92.75),
            (0.0, 0.0, -79.88),
            (0.0, 0.0, 0.0),
            false,
            true,
            true,
            true,
        );
        let to = make_tick(
            101,
            (10.5, 0.0, 17.0),
            (0.0, 0.0, 92.75),
            (0.0, 0.0, -79.88),
            (0.0, 0.0, 0.0),
            false,
            true,
            true,
            true,
        );
        let after = make_tick(
            102,
            (20.0, 0.0, 17.0),
            (0.0, 0.0, 92.75),
            (200.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            false,
            true,
            true,
        );
        assert!(tick_is_kickoff_stasis(&from, &to));
        let ticks = vec![lead, from, to, after];
        assert_eq!(
            split_segments(&ticks, config(8)),
            vec![Segment { start: 0, len: 4 }]
        );
        assert_eq!(run_start(&ticks, 2), 0);
    }
}
