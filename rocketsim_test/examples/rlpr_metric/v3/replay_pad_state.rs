//! Derived boost-pad history from RLPR v12 pad rows.
//!
//! RLPR v12 does not record a pad cooldown or a pad last-pickup tick. It
//! records these raw pad fields:
//!
//! - `PickupData2.PickedUp` as a byte counter (`pickup_counter`);
//! - `RespawnDelay` as the pad configuration in seconds;
//! - `bNoPickup` as a boolean;
//! - `IsTouchingAVehicle()` as a boolean;
//! - `PickupData2.Instigator` as a recorded car slot, or `-1`.
//!
//! This module derives a cooldown from consecutive rows. A derived value is
//! not a native value. Do not report it as a native cooldown.
//!
//! # Counter model
//!
//! The counter increments by one on each pad event. These checks support the
//! model on build 56b4:
//!
//! - A controlled small pad shows `0 -> 1 -> 3 -> 5` while a car stays on the
//!   pad. Each odd value agrees with one real boost grant.
//! - A controlled small pad and a controlled big pad show `0 -> 1 -> 2`. Only
//!   the first step grants boost. The second step is the respawn.
//! - The field note `rlpr-v12-controlled-pad-counter-2026-11-03.md` accepts
//!   odd values as pickups and even values as respawns. It keeps the raw value
//!   when the row does not prove an event.
//! - The 2026-11-03 corpus agrees. In the `nexto` capture, 112 even-to-odd
//!   changes have a nearby car boost grant on 111 rows. In the `wisp`
//!   capture, 360 even-to-odd changes have a nearby boost grant on 359 rows.
//!   Every odd-to-even change has no grant.
//!
//! Therefore a pad is in cooldown only while its counter is odd. An even
//! counter means that the last event was a respawn or that no event has
//! occurred. Both states are active.
//!
//! This model is scoped to RLPR v12 on build 56b4. It is not a native field
//! proof. If a counter jumps by more than two or wraps, this module starts a
//! new baseline. A capture frame gap does not erase an observed pickup, because
//! the pad clock continues in real time. A pickup before the pad history start
//! is `Unknown`.
//!
//! # Timing
//!
//! A pickup is visible on the first row with the new odd counter. The pickup
//! occurred after the previous observation and at or before that row. A row
//! gap increases the age bound but never removes it. The respawn is visible
//! after the cooldown:
//!
//! - small pad, 4 s: 478 to 481 ticks in the corpus;
//! - big pad, 10 s: 1197 to 1201 ticks in the corpus.
//!
//! [`PAD_BOUNDARY_GUARD_TICKS`] keeps a margin on both sides of the cooldown
//! boundary. A pad near the boundary is `Unknown`. It is not reported as
//! active.
//!
//! # Pad history start
//!
//! The test helper `pad_history_start` finds the first row whose pad clock is
//! still usable at the query row. The build precomputes it with a forward
//! scan. The scan starts a new epoch only at a proven pad reset or a proven
//! discontinuity:
//!
//! - a non-increasing ball physics frame;
//! - a ball teleport at or above [`PAD_HISTORY_BALL_TELEPORT_UU`];
//! - a change in `round_active`;
//! - a `ball_has_been_hit` true-to-false change, which marks a kickoff reset.
//!
//! A `ball_has_been_hit` false-to-true change is the first touch. It is not a
//! pad reset and does not break the pad clock.
//!
//! It crosses these events without a break:
//!
//! - a car availability or car-count change;
//! - a car teleport;
//! - a capture frame gap with an increasing ball frame;
//! - a row without a ball, because a missing ball alone is not a pad reset.
//!
//! A round transition is conservative. The module cannot prove that the pad
//! clock survives a goal or kickoff reset. A pickup before the history start is
//! `Unknown`.
//!
//! Do not use `common::run_start` as the pad history start. That helper also
//! breaks on car-only events, and the harness rebuilds the Arena there. A new
//! Arena has active pads, so the derived cooldown must restore across that
//! boundary.
//!
//! # Mapping
//!
//! The recorder sorts pad rows by native position: X, then Y, then Z. The
//! RocketSim pad grid sorts by Y, then X. Match pads by position, never by
//! array index.
//!
//! The native small pads at `|y| = 2302` map to the RocketSim layout at
//! `|y| = 2300`. The native pad Z value differs from the RocketSim value. The
//! matcher uses X and Y only. The smallest X and Y separation between two
//! Soccar pads is 1061 uu.
//!
//! The expected row count is the Arena pad count. A row with a different count
//! is `RowShapeMismatch`. A short mapping row cannot lower the expected count.
//!
//! # Limits
//!
//! - `is_touching_vehicle` has no car identity. The native ordered `Touching`
//!   list is not captured. This module does not restore membership.
//! - `instigator_slot` is not a RocketSim pad field. This module does not
//!   restore it.
//! - `respawn_delay_s` is configuration. This module uses it only after it
//!   agrees with the RocketSim cooldown for the pad class.
//! - The module assumes the default RocketSim Soccar mutators. It cannot read
//!   the Arena `max_cooldown`. Do not use it with a custom
//!   `boost_pad_cooldown_*` value. The matcher also refuses a row when the
//!   recorded delay differs from the default for the pad class.
//! - A row without pad positions trusts the contract pad order. A row with a
//!   different position skips that pad row.
//! - An `Unknown` pad does not write the Arena. It keeps the current Arena
//!   value. At a proven round reset, the caller may reset the Arena pads to
//!   active before the restore. Do not reset pads at a car-only boundary.
//! - The event timeline is built in one forward pass. Classification uses
//!   only events at or before the query row. The only forward lookup is the
//!   immutable pad position map.

use std::fmt::{self, Display, Formatter};

use rocketsim::{
    Arena, BoostPadConfig,
    consts::{
        TICK_RATE, TICK_TIME,
        boost_pads::{COOLDOWN_BIG, COOLDOWN_SMALL},
    },
};
use rocketsim_test::rlpr::{
    row_ext::{GROUP_PADS, PadExt},
    tick_record::TickRecord,
};

/// Maximum X/Y distance between a recorded pad and a RocketSim pad, in uu.
///
/// The four diagonal small pads have a 2 uu Y difference. The closest two
/// Soccar pads are 1061 uu apart in X and Y, so this tolerance cannot select
/// the wrong pad.
pub const PAD_POSITION_XY_TOL_UU: f32 = 4.0;

/// Maximum difference between the recorded respawn delay and the RocketSim
/// pad-class cooldown, in seconds.
pub const PAD_DELAY_TOL_S: f32 = 1.0e-3;

/// Guard on each side of a cooldown boundary, in ticks.
///
/// The corpus shows a 4 s pad respawn at 478 to 481 ticks and a 10 s pad
/// respawn at 1197 to 1201 ticks. Four ticks cover both spreads.
pub const PAD_BOUNDARY_GUARD_TICKS: u32 = 4;

/// Ball teleport distance for the pad history start, in uu.
///
/// The value matches the metric ball teleport limit. A ball jump at or above
/// this distance is a reset or kickoff candidate. The pad history must not
/// cross it.
pub const PAD_HISTORY_BALL_TELEPORT_UU: f32 = 500.0;

/// Native small-pad `BoostType` value.
const BOOST_TYPE_SMALL: u8 = 2;
/// Native big-pad `BoostType` value.
const BOOST_TYPE_BIG: u8 = 1;

/// Result for one RocketSim pad at one row.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PadAvailability {
    /// The pad can be picked up. The counter is even.
    Active,
    /// The pad is in a derived cooldown.
    Cooldown {
        /// Derived cooldown that remains at the queried row, in seconds.
        remaining_s: f32,
    },
    /// The row does not prove the pad state. Do not change the Arena.
    Unknown(PadUnknown),
}

/// Reason that a pad state is not known.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PadUnknown {
    /// The query row is outside the recording.
    OutOfRange,
    /// The row has no pad group, or the group has no pads.
    NoPadGroup,
    /// The pad recorded index is not in the row.
    PadMissingFromRow,
    /// The pad has no `pickup_counter` field.
    MissingCounter,
    /// The native `bNoPickup` flag is set. The counter does not define the state.
    NoPickup,
    /// The pad has no `respawn_delay_s` field.
    MissingDelay,
    /// The row has no usable native physics frame.
    MissingFrame,
    /// The row has no recorded position for the pad.
    MissingPosition,
    /// The recorded position does not match the mapped RocketSim pad.
    PositionMismatch,
    /// The recorded row count or order does not agree with the mapping.
    RowShapeMismatch,
    /// The pad state before this row is not usable.
    NoBaseline,
    /// The last event is a pickup before the pad history start.
    PickupBeforeHistory,
    /// The last event is a respawn but the counter is odd.
    CounterConflict,
    /// The recorded delay is not the RocketSim delay for this pad class.
    DelayMismatch,
    /// The pad is too close to the cooldown boundary.
    Boundary,
    /// The counter is odd long after the configured cooldown.
    StalePickup,
    /// The Arena has no pad at the mapped index.
    ArenaMissingPad,
}

impl Display for PadUnknown {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::OutOfRange => "out of range",
            Self::NoPadGroup => "no pad group",
            Self::PadMissingFromRow => "pad missing from row",
            Self::MissingCounter => "missing pickup counter",
            Self::NoPickup => "no-pickup flag set",
            Self::MissingDelay => "missing respawn delay",
            Self::MissingFrame => "missing physics frame",
            Self::MissingPosition => "missing pad position",
            Self::PositionMismatch => "pad position mismatch",
            Self::RowShapeMismatch => "pad row shape mismatch",
            Self::NoBaseline => "no usable pad baseline",
            Self::PickupBeforeHistory => "pickup before pad history start",
            Self::CounterConflict => "counter conflict",
            Self::DelayMismatch => "respawn delay mismatch",
            Self::Boundary => "cooldown boundary",
            Self::StalePickup => "stale pickup counter",
            Self::ArenaMissingPad => "arena pad missing",
        })
    }
}

/// Build-time coverage of one recording.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PadReplayCoverage {
    /// RocketSim pads in the arena layout.
    pub sim_pads: usize,
    /// Recorded pads that map to a RocketSim pad.
    pub mapped_pads: usize,
    /// Rows with a pad group.
    pub rows_with_pad_group: usize,
    /// Rows without a pad group.
    pub rows_without_pad_group: usize,
    /// Counter changes that the model accepted.
    pub events: usize,
    /// Tracking resets from a first row or an unusable counter jump.
    pub resets: usize,
}

impl Display for PadReplayCoverage {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "pads mapped {}/{}, rows with pads {}, without {}, events {}, resets {}",
            self.mapped_pads,
            self.sim_pads,
            self.rows_with_pad_group,
            self.rows_without_pad_group,
            self.events,
            self.resets
        )
    }
}

/// Result of one restore call.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PadRestoreReport {
    /// RocketSim pads in the arena layout.
    pub sim_pads: usize,
    /// Pads that the recording maps.
    pub mapped_pads: usize,
    /// Pads set to active.
    pub active: usize,
    /// Pads set to a derived cooldown.
    pub cooldown: usize,
    /// Pads left unchanged.
    pub unknown: usize,
    /// Unknown counts by reason, in first-seen order.
    pub unknown_reasons: Vec<(PadUnknown, usize)>,
}

impl PadRestoreReport {
    /// Add one unknown reason.
    fn add_unknown(&mut self, reason: PadUnknown) {
        self.unknown += 1;
        if let Some((_, count)) = self
            .unknown_reasons
            .iter_mut()
            .find(|(known, _)| *known == reason)
        {
            *count += 1;
        } else {
            self.unknown_reasons.push((reason, 1));
        }
    }

    /// Add the counts of another restore call.
    ///
    /// The pad counts keep the largest value. Use the result for aggregate
    /// output only. Do not use it as evidence for one reset.
    pub fn merge(&mut self, other: &Self) {
        self.sim_pads = self.sim_pads.max(other.sim_pads);
        self.mapped_pads = self.mapped_pads.max(other.mapped_pads);
        self.active += other.active;
        self.cooldown += other.cooldown;
        self.unknown += other.unknown;
        for &(reason, count) in &other.unknown_reasons {
            if let Some((_, total)) = self
                .unknown_reasons
                .iter_mut()
                .find(|(known, _)| *known == reason)
            {
                *total += count;
            } else {
                self.unknown_reasons.push((reason, count));
            }
        }
    }

    /// One-line summary for the metric report.
    #[must_use]
    pub fn summary_line(&self) -> String {
        let reasons = if self.unknown_reasons.is_empty() {
            String::new()
        } else {
            let mut text = String::from(" (");
            for (index, (reason, count)) in self.unknown_reasons.iter().enumerate() {
                if index != 0 {
                    text.push_str(", ");
                }
                text.push_str(&format!("{reason}: {count}"));
            }
            text.push(')');
            text
        };
        format!(
            "pads: active {}, cooldown {}, unchanged {}/{}{}",
            self.active, self.cooldown, self.unknown, self.sim_pads, reasons
        )
    }
}

impl Display for PadRestoreReport {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.summary_line())
    }
}

/// Event kinds that the counter model accepts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PadEventKind {
    /// Tracking restarted. No history is valid before this row.
    Reset,
    /// The counter changed to an odd value.
    Pickup,
    /// The counter changed to an even value.
    Respawn,
}

/// One accepted pad event.
#[derive(Clone, Copy, Debug)]
struct PadEvent {
    /// Row where the new counter value first appears.
    row: usize,
    /// Physics frame of `row`.
    frame: Option<u64>,
    /// Earliest frame where the event can have occurred.
    ///
    /// The value is the previous observed frame plus one. Without a previous
    /// observation it is `frame`.
    earliest_frame: Option<u64>,
    counter: Option<u8>,
    delay_s: f32,
    kind: PadEventKind,
}

/// One recorded pad that mapped to a RocketSim pad.
#[derive(Clone, Debug)]
struct PadTrack {
    /// Index in the recorded pad list.
    recorded_idx: usize,
    /// Index in the RocketSim pad layout.
    sim_idx: usize,
    /// RocketSim cooldown for the pad class, in seconds.
    expected_delay_s: f32,
    previous_frame: Option<u64>,
    previous_counter: Option<u8>,
    events: Vec<PadEvent>,
}

impl PadTrack {
    fn new(recorded_idx: usize, sim_idx: usize, is_big: bool) -> Self {
        Self {
            recorded_idx,
            sim_idx,
            expected_delay_s: if is_big { COOLDOWN_BIG } else { COOLDOWN_SMALL },
            previous_frame: None,
            previous_counter: None,
            events: Vec::new(),
        }
    }

    /// Process one recorded pad row.
    ///
    /// A missing row, a missing field, or a frame gap does not erase the last
    /// event. The next usable observation compares its counter with the last
    /// observed counter. The previous frame gives the earliest possible event
    /// time for that comparison.
    fn observe(&mut self, row: usize, frame: Option<u64>, pad: &PadExt, expected_pos: glam::Vec3A) {
        let (Some(frame), Some(counter), Some(delay_s)) =
            (frame, pad.pickup_counter, pad.respawn_delay_s)
        else {
            return;
        };
        if pad
            .pos
            .is_some_and(|pos| !position_matches(pos, expected_pos))
        {
            return;
        }
        // A non-increasing frame means the recording order changed. Start a
        // new baseline instead of subtracting frames.
        if self
            .previous_frame
            .is_some_and(|previous| frame <= previous)
        {
            self.previous_frame = None;
            self.previous_counter = None;
        }

        let earliest_frame = self.previous_frame.map_or(frame, |previous| previous + 1);
        let Some(previous_counter) = self.previous_counter else {
            self.events.push(PadEvent {
                row,
                frame: Some(frame),
                earliest_frame: Some(frame),
                counter: Some(counter),
                delay_s,
                kind: PadEventKind::Reset,
            });
            self.previous_frame = Some(frame);
            self.previous_counter = Some(counter);
            return;
        };

        if counter != previous_counter {
            let delta = counter.wrapping_sub(previous_counter);
            // Each event adds one. A delta of two can still identify the last
            // event by parity. A larger delta is not proven.
            let kind = match delta {
                1 | 2 => {
                    if counter % 2 == 1 {
                        PadEventKind::Pickup
                    } else {
                        PadEventKind::Respawn
                    }
                }
                _ => PadEventKind::Reset,
            };
            self.events.push(PadEvent {
                row,
                frame: Some(frame),
                earliest_frame: Some(earliest_frame),
                counter: Some(counter),
                delay_s,
                kind,
            });
        }

        self.previous_frame = Some(frame);
        self.previous_counter = Some(counter);
    }

    /// Last event at or before `row`.
    fn event_at(&self, row: usize) -> Option<&PadEvent> {
        let count = self.events.partition_point(|event| event.row <= row);
        count
            .checked_sub(1)
            .and_then(|index| self.events.get(index))
    }
}

/// Mapping from recorded pad entries to RocketSim pad indices.
#[derive(Clone, Debug, Default)]
pub struct PadReplay {
    configs: Vec<BoostPadConfig>,
    tracks: Vec<PadTrack>,
    sim_to_track: Vec<Option<usize>>,
    recorded_pad_count: usize,
    /// Pad history start for every recording row.
    history_starts: Vec<usize>,
    coverage: PadReplayCoverage,
}

impl PadReplay {
    /// Build the pad history for one recording.
    ///
    /// `configs` is [`Arena::get_all_boost_pad_configs`] in RocketSim order.
    /// The function scans the recording once. Memory holds the pad events and
    /// one history-start index per row.
    #[must_use]
    pub fn build(ticks: &[TickRecord], configs: &[BoostPadConfig]) -> Self {
        let mut replay = Self {
            configs: configs.to_vec(),
            tracks: Vec::new(),
            sim_to_track: vec![None; configs.len()],
            recorded_pad_count: 0,
            history_starts: build_history_starts(ticks),
            coverage: PadReplayCoverage {
                sim_pads: configs.len(),
                ..PadReplayCoverage::default()
            },
        };
        if configs.is_empty() {
            return replay;
        }

        let Some(mapping_row) = best_mapping_row(ticks, configs.len()) else {
            return replay;
        };
        // The expected count is the Arena pad count. A short first or sparse
        // row must not reject later full rows.
        replay.recorded_pad_count = configs.len();
        let map = map_pads(mapping_row, configs);
        for (recorded_idx, sim_idx) in map.into_iter().enumerate() {
            if let Some(sim_idx) = sim_idx {
                let track = PadTrack::new(recorded_idx, sim_idx, configs[sim_idx].is_big);
                replay.sim_to_track[sim_idx] = Some(replay.tracks.len());
                replay.tracks.push(track);
            }
        }
        replay.coverage.mapped_pads = replay.tracks.len();

        let sim_positions: Vec<glam::Vec3A> = configs.iter().map(|config| config.pos).collect();
        for (row, tick) in ticks.iter().enumerate() {
            let frame = row_frame(tick);
            let Some(pads) = pad_group(tick) else {
                replay.coverage.rows_without_pad_group += 1;
                continue;
            };
            replay.coverage.rows_with_pad_group += 1;
            if pads.len() != replay.recorded_pad_count {
                continue;
            }
            for track in &mut replay.tracks {
                let Some(pad) = pads.get(track.recorded_idx) else {
                    continue;
                };
                let before = track.events.len();
                track.observe(row, frame, pad, sim_positions[track.sim_idx]);
                if track.events.len() > before {
                    match track.events.last().map(|event| event.kind) {
                        Some(PadEventKind::Reset) => replay.coverage.resets += 1,
                        Some(PadEventKind::Pickup | PadEventKind::Respawn) => {
                            replay.coverage.events += 1;
                        }
                        None => {}
                    }
                }
            }
        }

        replay
    }

    /// Coverage of the build pass.
    #[must_use]
    pub fn coverage(&self) -> PadReplayCoverage {
        self.coverage
    }

    /// True when at least one recorded pad mapped.
    #[must_use]
    pub fn has_mapping(&self) -> bool {
        !self.tracks.is_empty()
    }

    /// State for every RocketSim pad at one recorded row.
    ///
    /// The function derives the pad history start from the recording. Do not
    /// pass the metric run start. The history crosses car-only boundaries and
    /// capture frame gaps, and stops at a proven pad reset.
    #[must_use]
    pub fn state_at(&self, ticks: &[TickRecord], row: usize) -> Vec<PadAvailability> {
        let mut states = vec![PadAvailability::Unknown(PadUnknown::NoPadGroup); self.configs.len()];
        let Some(tick) = ticks.get(row) else {
            states.fill(PadAvailability::Unknown(PadUnknown::OutOfRange));
            return states;
        };
        let Some(pads) = pad_group(tick) else {
            return states;
        };
        if pads.len() != self.recorded_pad_count {
            states.fill(PadAvailability::Unknown(PadUnknown::RowShapeMismatch));
            return states;
        }
        let frame = row_frame(tick);
        let history_start = self.history_starts.get(row).copied().unwrap_or(0);
        for (sim_idx, state) in states.iter_mut().enumerate() {
            *state = match self.sim_to_track[sim_idx] {
                Some(track_idx) => {
                    self.classify(&self.tracks[track_idx], pads, frame, row, history_start)
                }
                None => PadAvailability::Unknown(PadUnknown::PadMissingFromRow),
            };
        }
        states
    }

    /// Classify one mapped pad from one row.
    ///
    /// Validate the pad delay and the native frame before the parity result.
    /// An even counter is active only when both values are valid.
    fn classify(
        &self,
        track: &PadTrack,
        pads: &[PadExt],
        frame: Option<u64>,
        row: usize,
        history_start: usize,
    ) -> PadAvailability {
        let Some(pad) = pads.get(track.recorded_idx) else {
            return PadAvailability::Unknown(PadUnknown::PadMissingFromRow);
        };
        let Some(counter) = pad.pickup_counter else {
            return PadAvailability::Unknown(PadUnknown::MissingCounter);
        };
        if pad.no_pickup == Some(true) {
            // The native no-pickup gate can keep the pad unavailable without a
            // counter change. RocketSim has no per-pad equivalent.
            return PadAvailability::Unknown(PadUnknown::NoPickup);
        }
        let Some(pos) = pad.pos else {
            return PadAvailability::Unknown(PadUnknown::MissingPosition);
        };
        if !position_matches(pos, self.configs[track.sim_idx].pos) {
            return PadAvailability::Unknown(PadUnknown::PositionMismatch);
        }
        // Validate the delay and the native frame before the parity result. An
        // even counter is not proof when the row cannot place the state.
        let Some(delay_s) = pad.respawn_delay_s else {
            return PadAvailability::Unknown(PadUnknown::MissingDelay);
        };
        if !delays_agree(delay_s, track.expected_delay_s) {
            return PadAvailability::Unknown(PadUnknown::DelayMismatch);
        }
        let Some(frame) = frame else {
            return PadAvailability::Unknown(PadUnknown::MissingFrame);
        };

        // An even counter means that the last event was a respawn, or that no
        // pickup has occurred. Both are active.
        if counter % 2 == 0 {
            return PadAvailability::Active;
        }

        let Some(event) = track.event_at(row) else {
            return PadAvailability::Unknown(PadUnknown::NoBaseline);
        };
        if event.row < history_start {
            return PadAvailability::Unknown(match event.kind {
                PadEventKind::Pickup => PadUnknown::PickupBeforeHistory,
                _ => PadUnknown::NoBaseline,
            });
        }
        match event.kind {
            PadEventKind::Pickup => {}
            PadEventKind::Respawn => {
                return PadAvailability::Unknown(PadUnknown::CounterConflict);
            }
            PadEventKind::Reset => {
                return PadAvailability::Unknown(PadUnknown::NoBaseline);
            }
        }
        if event.counter != Some(counter) {
            return PadAvailability::Unknown(PadUnknown::CounterConflict);
        }
        // A configuration change after the pickup makes the start delay
        // ambiguous.
        if !delays_agree(event.delay_s, delay_s) {
            return PadAvailability::Unknown(PadUnknown::DelayMismatch);
        }

        let (Some(event_frame), Some(earliest_frame)) = (event.frame, event.earliest_frame) else {
            return PadAvailability::Unknown(PadUnknown::MissingFrame);
        };
        if frame < event_frame {
            return PadAvailability::Unknown(PadUnknown::CounterConflict);
        }

        let delay_ticks = (track.expected_delay_s * TICK_RATE).round() as u64;
        // The value uses the visible change row. The safety predicate uses the
        // earliest possible event frame, which covers a lost row at the event.
        let elapsed_ticks = frame - event_frame;
        let max_age_ticks = frame - earliest_frame;
        let guard = u64::from(PAD_BOUNDARY_GUARD_TICKS);
        if max_age_ticks + guard >= delay_ticks {
            return PadAvailability::Unknown(if elapsed_ticks > delay_ticks + guard {
                PadUnknown::StalePickup
            } else {
                PadUnknown::Boundary
            });
        }

        // The counter is odd and well inside the cooldown. Report the larger
        // remaining value. It never claims availability too early.
        let remaining_ticks = delay_ticks - elapsed_ticks;
        PadAvailability::Cooldown {
            remaining_s: remaining_ticks as f32 * TICK_TIME,
        }
    }

    /// Apply the derived pad state to an Arena.
    ///
    /// Call this after all state setters of one reset, including a roster
    /// rebuild. Only `Active` and `Cooldown` pads are written. An `Unknown` pad
    /// keeps its current Arena value. At a proven round reset, the caller may
    /// reset the Arena pads to active before this call.
    #[must_use]
    pub fn restore(&self, arena: &mut Arena, ticks: &[TickRecord], row: usize) -> PadRestoreReport {
        let states = self.state_at(ticks, row);
        let mut report = PadRestoreReport {
            sim_pads: arena.num_boost_pads(),
            mapped_pads: self.tracks.len(),
            ..PadRestoreReport::default()
        };
        for (sim_idx, state) in states.into_iter().enumerate() {
            if sim_idx >= report.sim_pads {
                report.add_unknown(PadUnknown::ArenaMissingPad);
                continue;
            }
            match state {
                PadAvailability::Active => {
                    arena.edit_boost_pad_state(sim_idx, |pad| pad.cooldown = 0.0);
                    report.active += 1;
                }
                PadAvailability::Cooldown { remaining_s } => {
                    arena.edit_boost_pad_state(sim_idx, |pad| pad.cooldown = remaining_s);
                    report.cooldown += 1;
                }
                PadAvailability::Unknown(reason) => report.add_unknown(reason),
            }
        }
        report
    }
}

/// First row whose pad clock is usable at `row`.
///
/// This test-only helper recomputes the history starts from the recording. The
/// [`PadReplay`] hot path uses the precomputed vector from the build pass. Walk
/// forward and start a new epoch only at a proven reset or discontinuity. A car
/// availability change, a car teleport, a frame gap, or a missing ball is not
/// a pad reset. A ball teleport, a non-increasing ball frame, or a round flag
/// change is a pad reset candidate.
#[cfg(test)]
#[must_use]
pub fn pad_history_start(ticks: &[TickRecord], row: usize) -> usize {
    let starts = build_history_starts(ticks);
    starts
        .get(row)
        .copied()
        .unwrap_or_else(|| starts.last().copied().unwrap_or(0))
}

/// Precompute the pad history start for every row.
///
/// The scan carries the last observed ball and round flags across rows that
/// lack them. A missing ball alone does not break the history. A `round_active`
/// change or a `ball_has_been_hit` true-to-false reset starts a new epoch.
fn build_history_starts(ticks: &[TickRecord]) -> Vec<usize> {
    let mut starts = Vec::with_capacity(ticks.len());
    let mut start = 0;
    let mut last_ball: Option<BallSnapshot> = None;
    let mut last_round: Option<RoundSnapshot> = None;
    for (row, tick) in ticks.iter().enumerate() {
        let mut reset = false;
        if let Some(ball) = ball_snapshot(tick) {
            if let Some(previous) = last_ball
                && (ball.frame <= previous.frame
                    || ball.pos.distance_squared(previous.pos)
                        >= PAD_HISTORY_BALL_TELEPORT_UU * PAD_HISTORY_BALL_TELEPORT_UU)
            {
                reset = true;
            }
            last_ball = Some(ball);
        }
        if let Some(round) = round_snapshot(tick) {
            if let Some(previous) = last_round
                && (previous.round_active != round.round_active
                    || ball_hit_reset(previous.ball_has_been_hit, round.ball_has_been_hit))
            {
                reset = true;
            }
            last_round = Some(round);
        }
        if reset {
            start = row;
        }
        starts.push(start);
    }
    starts
}

#[derive(Clone, Copy, Debug)]
struct BallSnapshot {
    frame: u64,
    pos: glam::Vec3A,
}

fn ball_snapshot(tick: &TickRecord) -> Option<BallSnapshot> {
    let frame = row_frame(tick)?;
    Some(BallSnapshot {
        frame,
        pos: glam::Vec3A::from(tick.ball_record.pos),
    })
}

#[derive(Clone, Copy, Debug)]
struct RoundSnapshot {
    ball_has_been_hit: Option<bool>,
    round_active: Option<bool>,
}

/// True when `ball_has_been_hit` marks a kickoff reset.
///
/// The native flag is true during play. It becomes false at a kickoff reset.
/// It becomes true again at the first touch. Only the true-to-false change can
/// reset pads.
fn ball_hit_reset(previous: Option<bool>, current: Option<bool>) -> bool {
    matches!((previous, current), (Some(true), Some(false)))
}

fn round_snapshot(tick: &TickRecord) -> Option<RoundSnapshot> {
    let global = tick.replay_state.as_ref()?.global.as_ref()?;
    Some(RoundSnapshot {
        ball_has_been_hit: global.ball_has_been_hit,
        round_active: global.round_active,
    })
}

/// Physics frame of one row, when the ball row is usable.
fn row_frame(tick: &TickRecord) -> Option<u64> {
    if !tick.ball_available() {
        return None;
    }
    let frame = u64::from(tick.ball_record.physics_frame);
    (frame != u64::from(rocketsim_test::rlpr::NATIVE_FRAME_UNKNOWN)).then_some(frame)
}

/// Pad records of one row, when the group is present.
fn pad_group(tick: &TickRecord) -> Option<&[PadExt]> {
    let ext = tick.replay_state.as_ref()?;
    if ext.group_mask & (1 << GROUP_PADS) == 0 || ext.pads.is_empty() {
        return None;
    }
    Some(&ext.pads)
}

/// Find the row that gives the best pad position map.
///
/// Prefer a row with the same count as the RocketSim layout and a position on
/// every pad. Fall back to the row with the most positions. The build always
/// uses the RocketSim pad count as the expected row count, so a sparse mapping
/// row cannot lower the count for later rows.
fn best_mapping_row(ticks: &[TickRecord], sim_pads: usize) -> Option<&[PadExt]> {
    let mut best: Option<(usize, &[PadExt])> = None;
    for tick in ticks {
        let Some(pads) = pad_group(tick) else {
            continue;
        };
        let positions = pads.iter().filter(|pad| pad.pos.is_some()).count();
        let complete = pads.len() == sim_pads && positions == pads.len();
        if complete {
            return Some(pads);
        }
        if best.is_none_or(|(best_positions, _)| positions > best_positions) {
            best = Some((positions, pads));
        }
    }
    best.map(|(_, pads)| pads)
}

/// Map recorded pads to RocketSim pad indices.
///
/// Return `None` for an unmatched or ambiguous pad. Match X and Y only. Use
/// the native `BoostType` as a size-class filter when it is present.
fn map_pads(pads: &[PadExt], configs: &[BoostPadConfig]) -> Vec<Option<usize>> {
    let mut mapping = vec![None; pads.len()];
    let mut taken = vec![false; configs.len()];
    for (recorded_idx, pad) in pads.iter().enumerate() {
        let Some(pos) = pad.pos else {
            continue;
        };
        let is_big = match pad.boost_type {
            Some(BOOST_TYPE_BIG) => Some(true),
            Some(BOOST_TYPE_SMALL) => Some(false),
            _ => None,
        };
        let mut best: Option<(f32, usize)> = None;
        let mut ambiguous = false;
        for (sim_idx, config) in configs.iter().enumerate() {
            if taken[sim_idx] || is_big.is_some_and(|is_big| is_big != config.is_big) {
                continue;
            }
            let dx = pos[0] - config.pos.x;
            let dy = pos[1] - config.pos.y;
            let distance = (dx * dx + dy * dy).sqrt();
            if distance > PAD_POSITION_XY_TOL_UU {
                continue;
            }
            if best.is_some() {
                ambiguous = true;
            } else {
                best = Some((distance, sim_idx));
            }
        }
        if let Some((_, sim_idx)) = best {
            if ambiguous {
                continue;
            }
            mapping[recorded_idx] = Some(sim_idx);
            taken[sim_idx] = true;
        }
    }
    mapping
}

/// True when a recorded position matches a RocketSim pad in X and Y.
fn position_matches(pos: [f32; 3], expected: glam::Vec3A) -> bool {
    let dx = pos[0] - expected.x;
    let dy = pos[1] - expected.y;
    (dx * dx + dy * dy).sqrt() <= PAD_POSITION_XY_TOL_UU
}

/// True when a recorded respawn delay matches the RocketSim class cooldown.
fn delays_agree(recorded_s: f32, expected_s: f32) -> bool {
    recorded_s.is_finite() && (recorded_s - expected_s).abs() <= PAD_DELAY_TOL_S
}

#[cfg(test)]
mod tests {
    use glam::Vec3A;
    use rocketsim_test::rlpr::{
        NATIVE_FRAME_UNKNOWN, ROW_FLAGS_BALL_AVAILABLE, RowExt,
        cpp_records::{CarRecord, PhysRecord, VecRecord},
        row_ext::{
            GLOBAL_KICKOFF, GROUP_GLOBAL, GlobalExt, PAD_INSTIGATOR, PAD_PICKUP_HISTORY,
            PAD_POSE_BOOST, PAD_TOUCH,
        },
    };

    use super::*;

    const SMALL_POS: [f32; 3] = [-2048.0, 1036.0, 64.082];
    const BIG_POS: [f32; 3] = [-3584.0, 0.0, 72.0];

    fn configs() -> Vec<BoostPadConfig> {
        vec![
            BoostPadConfig {
                pos: Vec3A::new(-2048.0, 1036.0, 70.0),
                is_big: false,
            },
            BoostPadConfig {
                pos: Vec3A::new(-3584.0, 0.0, 73.0),
                is_big: true,
            },
        ]
    }

    fn pad(pos: [f32; 3], counter: u8, delay_s: f32) -> PadExt {
        let is_big = delay_s > 5.0;
        PadExt {
            field_mask: PAD_POSE_BOOST | PAD_PICKUP_HISTORY | PAD_TOUCH | PAD_INSTIGATOR,
            pos: Some(pos),
            boost_amount: Some(if is_big { 9999.0 } else { 0.12 }),
            boost_type: Some(if is_big {
                BOOST_TYPE_BIG
            } else {
                BOOST_TYPE_SMALL
            }),
            pickup_counter: Some(counter),
            no_pickup: Some(false),
            respawn_delay_s: Some(delay_s),
            instigator_slot: Some(-1),
            is_touching_vehicle: Some(false),
        }
    }

    fn tick(frame: u32, pads: Vec<PadExt>) -> TickRecord {
        // `PhysRecord` has private fields, so a test must zero it.
        let mut ball_record: PhysRecord = unsafe { std::mem::zeroed() };
        ball_record.physics_frame = frame;
        TickRecord {
            level_tick: u64::from(frame),
            row_flags: ROW_FLAGS_BALL_AVAILABLE,
            reserved0: 0,
            car_records: Vec::new(),
            ball_record,
            replay_state: Some(RowExt {
                version: 1,
                group_mask: 1 << GROUP_PADS,
                pads,
                ..RowExt::default()
            }),
        }
    }

    fn empty_tick(frame: u32) -> TickRecord {
        TickRecord {
            replay_state: Some(RowExt::default()),
            ..tick(frame, Vec::new())
        }
    }

    fn car(available: bool) -> CarRecord {
        let mut record: CarRecord = unsafe { std::mem::zeroed() };
        record.physics_available = u8::from(available);
        record.phys.physics_frame = if available { 0 } else { NATIVE_FRAME_UNKNOWN };
        record
    }

    fn tick_with_ball(frame: u32, pads: Vec<PadExt>, pos: [f32; 3]) -> TickRecord {
        let mut tick = tick(frame, pads);
        tick.ball_record.pos = VecRecord::new(pos[0], pos[1], pos[2]);
        tick
    }

    fn tick_with_round(
        frame: u32,
        pads: Vec<PadExt>,
        round_active: bool,
        ball_has_been_hit: bool,
    ) -> TickRecord {
        let mut tick = tick(frame, pads);
        if let Some(ext) = tick.replay_state.as_mut() {
            ext.group_mask |= 1 << GROUP_GLOBAL;
            ext.global = Some(GlobalExt {
                field_mask: GLOBAL_KICKOFF,
                ball_has_been_hit: Some(ball_has_been_hit),
                round_active: Some(round_active),
                seconds_remaining_countdown: Some(10),
                countdown_time: Some(3),
            });
        }
        tick
    }

    fn tick_without_ball(frame: u32, pads: Vec<PadExt>) -> TickRecord {
        let mut tick = tick(frame, pads);
        tick.row_flags = 0;
        tick.ball_record.physics_frame = NATIVE_FRAME_UNKNOWN;
        tick
    }

    #[test]
    fn maps_native_positions_to_rocketsim_order() {
        let recorded = vec![
            pad([-1788.0, -2302.0, 64.082], 0, 4.0),
            pad([-1788.0, 2302.0, 64.082], 0, 4.0),
            pad(BIG_POS, 0, 10.0),
        ];
        let sim = vec![
            BoostPadConfig {
                pos: Vec3A::new(-1788.0, 2300.0, 70.0),
                is_big: false,
            },
            BoostPadConfig {
                pos: Vec3A::new(-1788.0, -2300.0, 70.0),
                is_big: false,
            },
            BoostPadConfig {
                pos: Vec3A::new(-3584.0, 0.0, 73.0),
                is_big: true,
            },
        ];

        assert_eq!(map_pads(&recorded, &sim), vec![Some(1), Some(0), Some(2)]);
    }

    #[test]
    fn mapping_rejects_far_and_wrong_class_pads() {
        let recorded = vec![
            pad([0.0, 0.0, 64.0], 0, 4.0),
            // A small pad 500 uu from the big pad must not take the big slot.
            pad([-3584.0, 500.0, 64.0], 0, 4.0),
        ];
        let sim = vec![BoostPadConfig {
            pos: Vec3A::new(-3584.0, 0.0, 73.0),
            is_big: true,
        }];

        assert_eq!(map_pads(&recorded, &sim), vec![None, None]);
    }

    fn timeline(counters: &[(u32, u8, u8)]) -> Vec<TickRecord> {
        counters
            .iter()
            .map(|&(frame, small, big)| {
                tick(
                    frame,
                    vec![pad(SMALL_POS, small, 4.0), pad(BIG_POS, big, 10.0)],
                )
            })
            .collect()
    }

    /// Build one row per frame, so the pad clock stays contiguous.
    fn contiguous(start: u32, end: u32, changes: &[(u32, u8, u8)]) -> Vec<TickRecord> {
        let mut ticks = Vec::with_capacity((end - start + 1) as usize);
        let (mut small, mut big) = (0, 0);
        for frame in start..=end {
            for &(change_frame, new_small, new_big) in changes {
                if change_frame == frame {
                    small = new_small;
                    big = new_big;
                }
            }
            ticks.push(tick(
                frame,
                vec![pad(SMALL_POS, small, 4.0), pad(BIG_POS, big, 10.0)],
            ));
        }
        ticks
    }

    #[test]
    fn even_counter_is_active_and_odd_counter_is_cooldown() {
        let ticks = contiguous(100, 581, &[(101, 1, 1), (581, 2, 2)]);
        let replay = PadReplay::build(&ticks, &configs());
        assert_eq!(replay.coverage().mapped_pads, 2);

        let row0 = replay.state_at(&ticks, 0);
        assert_eq!(row0[0], PadAvailability::Active);
        assert_eq!(row0[1], PadAvailability::Active);

        let row1 = replay.state_at(&ticks, 1);
        assert_eq!(
            row1[0],
            PadAvailability::Cooldown {
                remaining_s: 480.0 * TICK_TIME
            }
        );
        assert_eq!(
            row1[1],
            PadAvailability::Cooldown {
                remaining_s: 1200.0 * TICK_TIME
            }
        );

        // The pickup row is at most one tick before the event.
        let row2 = replay.state_at(&ticks, 400);
        assert_eq!(
            row2[0],
            PadAvailability::Cooldown {
                remaining_s: 81.0 * TICK_TIME
            }
        );
        assert_eq!(
            row2[1],
            PadAvailability::Cooldown {
                remaining_s: 801.0 * TICK_TIME
            }
        );

        assert_eq!(replay.state_at(&ticks, 481)[0], PadAvailability::Active);
    }

    #[test]
    fn cooldown_boundary_is_unknown_and_not_active() {
        let ticks = contiguous(100, 581, &[(101, 1, 1), (581, 2, 2)]);
        let replay = PadReplay::build(&ticks, &configs());

        // 477 to 479 elapsed ticks leave 3 ticks or less. The guard refuses
        // these rows.
        for row in [478, 479, 480] {
            assert_eq!(
                replay.state_at(&ticks, row)[0],
                PadAvailability::Unknown(PadUnknown::Boundary)
            );
        }
        assert_eq!(replay.state_at(&ticks, 481)[0], PadAvailability::Active);
    }

    #[test]
    fn pickup_before_pad_history_is_unknown() {
        let mut ticks = contiguous(100, 200, &[(101, 1, 1)]);
        for tick in ticks.iter_mut() {
            if tick.ball_record.physics_frame >= 150 {
                tick.ball_record.pos = VecRecord::new(1000.0, 0.0, 0.0);
            }
        }
        let replay = PadReplay::build(&ticks, &configs());

        // Before the teleport the pickup clock is usable.
        assert!(matches!(
            replay.state_at(&ticks, 49)[0],
            PadAvailability::Cooldown { .. }
        ));
        // At the teleport the history starts. The pickup is before it.
        assert_eq!(
            replay.state_at(&ticks, 50)[0],
            PadAvailability::Unknown(PadUnknown::PickupBeforeHistory)
        );
        assert_eq!(
            replay.state_at(&ticks, 100)[0],
            PadAvailability::Unknown(PadUnknown::PickupBeforeHistory)
        );
    }

    #[test]
    fn frame_gap_keeps_the_observed_pickup_clock() {
        // The pickup is observed at frame 101. The next row is at frame 300.
        // Time advanced, so the pad is still cooling.
        let ticks = timeline(&[(100, 0, 0), (101, 1, 1), (300, 1, 1)]);
        let replay = PadReplay::build(&ticks, &configs());

        // An increasing ball frame is a capture gap, not a pad reset.
        assert_eq!(pad_history_start(&ticks, 2), 0);
        assert_eq!(
            replay.state_at(&ticks, 2)[0],
            PadAvailability::Cooldown {
                remaining_s: 281.0 * TICK_TIME
            }
        );
    }

    #[test]
    fn missing_pad_group_keeps_the_observed_pickup_clock() {
        let mut ticks = timeline(&[(100, 0, 0), (101, 1, 1)]);
        ticks.push(empty_tick(102));
        ticks.push(tick(
            103,
            vec![pad(SMALL_POS, 1, 4.0), pad(BIG_POS, 1, 10.0)],
        ));
        let replay = PadReplay::build(&ticks, &configs());

        assert_eq!(
            replay.state_at(&ticks, 2)[0],
            PadAvailability::Unknown(PadUnknown::NoPadGroup)
        );
        assert_eq!(
            replay.state_at(&ticks, 3)[0],
            PadAvailability::Cooldown {
                remaining_s: 478.0 * TICK_TIME
            }
        );
    }

    #[test]
    fn large_counter_jump_is_unknown() {
        let ticks = timeline(&[(100, 0, 0), (101, 1, 1), (102, 5, 2)]);
        let replay = PadReplay::build(&ticks, &configs());

        assert_eq!(
            replay.state_at(&ticks, 2)[0],
            PadAvailability::Unknown(PadUnknown::NoBaseline)
        );
    }

    #[test]
    fn recorded_delay_must_match_the_pad_class() {
        let ticks: Vec<TickRecord> = (100..=200u32)
            .map(|frame| {
                let small = if frame == 100 { 0 } else { 1 };
                let small_delay = if frame == 100 { 4.0 } else { 3.0 };
                tick(
                    frame,
                    vec![
                        pad(SMALL_POS, small, small_delay),
                        pad(BIG_POS, small, 10.0),
                    ],
                )
            })
            .collect();
        let replay = PadReplay::build(&ticks, &configs());
        let row = 100;

        assert_eq!(
            replay.state_at(&ticks, row)[0],
            PadAvailability::Unknown(PadUnknown::DelayMismatch)
        );
        assert!(matches!(
            replay.state_at(&ticks, row)[1],
            PadAvailability::Cooldown { .. }
        ));
    }

    #[test]
    fn even_counter_with_wrong_delay_is_unknown() {
        let ticks = vec![tick(
            100,
            vec![pad(SMALL_POS, 2, 3.0), pad(BIG_POS, 0, 10.0)],
        )];
        let replay = PadReplay::build(&ticks, &configs());

        assert_eq!(
            replay.state_at(&ticks, 0)[0],
            PadAvailability::Unknown(PadUnknown::DelayMismatch)
        );
        assert_eq!(replay.state_at(&ticks, 0)[1], PadAvailability::Active);
    }

    #[test]
    fn even_counter_without_frame_is_unknown() {
        let pads = vec![pad(SMALL_POS, 2, 4.0), pad(BIG_POS, 0, 10.0)];
        let ticks = vec![tick(100, pads.clone()), tick_without_ball(101, pads)];
        let replay = PadReplay::build(&ticks, &configs());

        assert_eq!(
            replay.state_at(&ticks, 1)[0],
            PadAvailability::Unknown(PadUnknown::MissingFrame)
        );
        assert_eq!(
            replay.state_at(&ticks, 1)[1],
            PadAvailability::Unknown(PadUnknown::MissingFrame)
        );
    }

    #[test]
    fn stale_odd_counter_is_unknown() {
        let ticks = contiguous(100, 700, &[(101, 1, 1)]);
        let replay = PadReplay::build(&ticks, &configs());

        // 599 elapsed ticks are past the 4 s cooldown and the guard.
        assert_eq!(
            replay.state_at(&ticks, 600)[0],
            PadAvailability::Unknown(PadUnknown::StalePickup)
        );
        // The big pad is still inside its 10 s cooldown.
        assert!(matches!(
            replay.state_at(&ticks, 600)[1],
            PadAvailability::Cooldown { .. }
        ));
    }

    #[test]
    fn wrap_counter_classifies_last_event_parity() {
        let ticks = timeline(&[(100, 254, 0), (101, 255, 0), (102, 0, 0), (103, 1, 0)]);
        let replay = PadReplay::build(&ticks, &configs());

        assert!(matches!(
            replay.state_at(&ticks, 1)[0],
            PadAvailability::Cooldown { .. }
        ));
        assert_eq!(replay.state_at(&ticks, 2)[0], PadAvailability::Active);
        assert!(matches!(
            replay.state_at(&ticks, 3)[0],
            PadAvailability::Cooldown { .. }
        ));
    }

    #[test]
    fn mask_change_midpoint_cooldown_survives() {
        let mut ticks = contiguous(100, 300, &[(101, 1, 1)]);
        for tick in ticks.iter_mut() {
            tick.car_records = if tick.ball_record.physics_frame < 150 {
                vec![car(true), car(true)]
            } else {
                // A mask change rebuilds the Arena with active pads.
                vec![car(true), car(false)]
            };
        }
        let replay = PadReplay::build(&ticks, &configs());

        assert_eq!(pad_history_start(&ticks, 100), 0);
        assert_eq!(
            replay.state_at(&ticks, 100)[0],
            PadAvailability::Cooldown {
                remaining_s: 381.0 * TICK_TIME
            }
        );
    }

    #[test]
    fn pad_history_start_stops_at_ball_teleport() {
        let pads = vec![pad(SMALL_POS, 1, 4.0), pad(BIG_POS, 1, 10.0)];
        let ticks = vec![
            tick_with_ball(100, pads.clone(), [0.0, 0.0, 0.0]),
            tick_with_ball(101, pads.clone(), [0.0, 0.0, 0.0]),
            tick_with_ball(102, pads.clone(), [0.0, 0.0, 0.0]),
            tick_with_ball(103, pads.clone(), [1000.0, 0.0, 0.0]),
        ];

        assert_eq!(pad_history_start(&ticks, 2), 0);
        assert_eq!(pad_history_start(&ticks, 3), 3);

        // The missing ball row does not hide a proven teleport.
        let across_missing = vec![
            tick_with_ball(100, pads.clone(), [0.0, 0.0, 0.0]),
            tick_without_ball(101, pads.clone()),
            tick_with_ball(102, pads, [1000.0, 0.0, 0.0]),
        ];
        assert_eq!(pad_history_start(&across_missing, 2), 2);
    }

    #[test]
    fn pad_history_start_stops_at_round_event() {
        let pads = vec![pad(SMALL_POS, 1, 4.0), pad(BIG_POS, 1, 10.0)];
        let round_change = vec![
            tick_with_round(100, pads.clone(), true, true),
            tick_with_round(101, pads.clone(), true, true),
            tick_with_round(102, pads.clone(), false, true),
            tick_with_round(103, pads.clone(), false, true),
        ];
        assert_eq!(pad_history_start(&round_change, 1), 0);
        assert_eq!(pad_history_start(&round_change, 3), 2);

        let hit_change = vec![
            tick_with_round(100, pads.clone(), true, true),
            tick_with_round(101, pads.clone(), true, false),
            tick_with_round(102, pads.clone(), true, false),
        ];
        assert_eq!(pad_history_start(&hit_change, 2), 1);

        // A first touch (false to true) is not a pad reset.
        let first_touch = vec![
            tick_with_round(100, pads.clone(), true, false),
            tick_with_round(101, pads.clone(), true, true),
            tick_with_round(102, pads.clone(), true, true),
        ];
        assert_eq!(pad_history_start(&first_touch, 2), 0);

        // The missing ball row does not hide a proven round change.
        let across_missing = vec![
            tick_with_round(100, pads.clone(), true, true),
            tick_without_ball(101, pads.clone()),
            tick_with_round(102, pads, false, true),
        ];
        assert_eq!(pad_history_start(&across_missing, 2), 2);
    }

    #[test]
    fn first_touch_midpoint_cooldown_survives_mask_change() {
        let mut ticks = contiguous(100, 200, &[(101, 1, 1)]);
        for tick in ticks.iter_mut() {
            let frame = tick.ball_record.physics_frame;
            let hit = frame >= 150;
            let ext = tick.replay_state.as_mut().expect("row extension");
            ext.group_mask |= 1 << GROUP_GLOBAL;
            ext.global = Some(GlobalExt {
                field_mask: GLOBAL_KICKOFF,
                ball_has_been_hit: Some(hit),
                round_active: Some(true),
                seconds_remaining_countdown: Some(10),
                countdown_time: Some(3),
            });
            tick.car_records = if hit {
                // A mask change rebuilds the Arena with active pads.
                vec![car(true), car(false)]
            } else {
                vec![car(true), car(true)]
            };
        }
        let replay = PadReplay::build(&ticks, &configs());

        // The first touch is at frame 150. It is not a pad reset, so the
        // pickup clock crosses it and the mask change.
        assert_eq!(pad_history_start(&ticks, 99), 0);
        assert_eq!(pad_history_start(&ticks, 100), 0);
        assert_eq!(
            replay.state_at(&ticks, 100)[0],
            PadAvailability::Cooldown {
                remaining_s: 381.0 * TICK_TIME
            }
        );
    }

    #[test]
    fn missing_ball_is_not_a_history_break() {
        let pads = vec![pad(SMALL_POS, 1, 4.0), pad(BIG_POS, 1, 10.0)];
        let ticks = vec![
            tick_with_ball(100, pads.clone(), [0.0, 0.0, 0.0]),
            tick_with_ball(101, pads.clone(), [0.0, 0.0, 0.0]),
            tick_without_ball(102, pads.clone()),
            tick_with_ball(103, pads, [0.0, 0.0, 0.0]),
        ];
        let replay = PadReplay::build(&ticks, &configs());

        assert_eq!(pad_history_start(&ticks, 3), 0);
        assert_eq!(
            replay.state_at(&ticks, 2),
            vec![
                PadAvailability::Unknown(PadUnknown::MissingFrame),
                PadAvailability::Unknown(PadUnknown::MissingFrame),
            ]
        );
    }

    #[test]
    fn short_mapping_row_does_not_lower_expected_pad_count() {
        let mut no_pos = pad(BIG_POS, 0, 10.0);
        no_pos.pos = None;
        no_pos.field_mask = PAD_PICKUP_HISTORY;
        let ticks = vec![
            tick(100, vec![pad(SMALL_POS, 0, 4.0)]),
            tick(101, vec![pad(SMALL_POS, 0, 4.0), no_pos.clone()]),
            tick(102, vec![pad(SMALL_POS, 1, 4.0), no_pos]),
        ];
        let replay = PadReplay::build(&ticks, &configs());

        assert!(matches!(
            replay.state_at(&ticks, 2)[0],
            PadAvailability::Cooldown { .. }
        ));
        assert_eq!(
            replay.state_at(&ticks, 2)[1],
            PadAvailability::Unknown(PadUnknown::PadMissingFromRow)
        );
    }

    #[test]
    fn even_counter_after_a_double_step_is_active() {
        // Two events between rows: respawn then pickup, or pickup then
        // respawn. The new parity names the last event.
        let ticks = timeline(&[(100, 0, 0), (101, 2, 2)]);
        let replay = PadReplay::build(&ticks, &configs());

        assert_eq!(replay.state_at(&ticks, 1)[0], PadAvailability::Active);
        assert_eq!(replay.state_at(&ticks, 1)[1], PadAvailability::Active);
    }

    #[test]
    fn no_pickup_flag_is_unknown() {
        let mut blocked = pad(SMALL_POS, 1, 4.0);
        blocked.no_pickup = Some(true);
        let ticks = vec![
            tick(100, vec![pad(SMALL_POS, 0, 4.0), pad(BIG_POS, 0, 10.0)]),
            tick(101, vec![pad(SMALL_POS, 1, 4.0), pad(BIG_POS, 1, 10.0)]),
            tick(102, vec![blocked, pad(BIG_POS, 1, 10.0)]),
        ];
        let replay = PadReplay::build(&ticks, &configs());

        assert_eq!(
            replay.state_at(&ticks, 2)[0],
            PadAvailability::Unknown(PadUnknown::NoPickup)
        );
    }

    #[test]
    fn missing_field_mask_counts_as_unknown() {
        let mut sparse = pad(SMALL_POS, 1, 4.0);
        sparse.pickup_counter = None;
        sparse.field_mask = PAD_POSE_BOOST;
        let ticks = vec![
            tick(100, vec![pad(SMALL_POS, 0, 4.0), pad(BIG_POS, 0, 10.0)]),
            tick(101, vec![pad(SMALL_POS, 1, 4.0), pad(BIG_POS, 1, 10.0)]),
            tick(200, vec![sparse, pad(BIG_POS, 1, 10.0)]),
        ];
        let replay = PadReplay::build(&ticks, &configs());

        assert_eq!(
            replay.state_at(&ticks, 2)[0],
            PadAvailability::Unknown(PadUnknown::MissingCounter)
        );
    }
}
