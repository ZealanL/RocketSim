//! Prefix-derived RLPR v12 car clocks.
//!
//! The recording gives the native autoflip and jump activity clocks. It does
//! not give the remaining autoflip timer or the core airborne clocks. This
//! module scans one contiguous run of recording rows and rebuilds those
//! values with the same operations that the core uses.
//!
//! ## Autoflip timer
//!
//! The native `FlipCar` component has an elapsed `ActivityTime` clock. The
//! core `auto_flip_timer` is a remaining time. The core sets it at activation
//! from the pre-tick orientation and decrements it once per tick.
//!
//! The reconstuction uses four facts:
//!
//! - The first active row is the activation tick. The row before it holds the
//!   pre-tick orientation.
//! - The native `ActivityTime` reads one tick on the activation row.
//! - The native clock and the core timer both advance by one tick per
//!   simulated row inside one contiguous run.
//! - The core duration is `TIME * |roll| / PI` with the roll from the
//!   pre-activation orientation.
//!
//! The remaining timer is `duration - activity_time`. The core stores a small
//! negative value on the last active tick. Keep the raw value. The next tick
//! clears the flag.
//!
//! ## Airborne clocks
//!
//! [`CarState::air_time`] and [`CarState::air_time_since_jump`] follow the
//! core reset order in `update_double_jump_or_flip`:
//!
//! - The core reads the contact cache at the start of the tick. That cache
//!   is the previous recorded row's ground state.
//! - A grounded previous row resets both clocks to zero.
//! - An airborne previous row adds one tick to `air_time`. It adds one tick
//!   to `air_time_since_jump` only when the current row has `has_jumped`, is
//!   not jumping, and the previous row was not jumping. Otherwise it resets
//!   `air_time_since_jump` to zero.
//! - A landing row keeps the airborne time. The next row resets it.
//!
//! A run start has no previous row. Then the previous ground is unknown, so
//! both clocks are unknown. The jump condition alone can still prove
//! `air_time_since_jump == 0` when the current row is jumping or has not
//! jumped.
//!
//! The rebuild uses the recorded ground and jump flags. The native
//! `jump_inactive_time` clock is a cross-check only. It does not reset on
//! landing and can diverge when `has_jumped` changes, so it is not the
//! primary source.
//!
//! ## Ground source
//!
//! A row's ground state is the native base `IsOnGround()` value. The v12
//! wheel world-contact group is distinct unsupported data and is not used.
//! The core wheel `has_contact` and `is_on_ground` stay authoritative.
//!
//! ## Runs and gaps
//!
//! A run uses the same row checks as [`super::common::run_start`], but only
//! for this car slot: constant car count, physics and ball data, consecutive
//! slot and ball frames, no frozen row, and no teleport. A change in another
//! car's availability does not start a new run for this slot. A frame gap
//! starts a new run. A value that needs a row before the run is unknown.
//!
//! Build the table from the deduplicated ticks. Usable frames must increase
//! strictly.
//!
//! ## Table validity
//!
//! A table is valid only when usable frames increase strictly. A duplicate
//! or decreasing frame clears the index at runtime and stores
//! [`CarHistoryInvalid`]. The apply then returns partial values. The caller can
//! count and warn with [`CarHistoryTable::invalid_reason`].
//!
//! ## Partial values
//!
//! An unknown autoflip timer clears the flag and the timer together. It does
//! not leave an active flag with a stale timer. An unknown airborne clock is
//! zero and the report marks it partial. The caller can count partial rows.
//! An absent autoflip field leaves the live simulator value in place.

use std::f32::consts::PI;

use glam::Mat3A;
use rocketsim::{
    CarState,
    consts::{TICK_TIME, car::autoflip::TIME as AUTOFLIP_TIME},
};
use rocketsim_test::rlpr::{row_ext::CarExt, tick_record::TickRecord};

use super::{
    common::{any_teleport, tick_car_count, tick_is_frozen},
    replay_car_state::{
        AutoFlipApply, CarRestoreReport, ClockApply, apply_jump_reset_zeroes, auto_flip_scale,
    },
};

/// Tolerance for one native clock step.
///
/// The captured clock delta is exact to one f32 ULP. This tolerance accepts
/// that rounding and rejects a skipped or doubled step.
const PHASE_TOL: f32 = 0.25 * TICK_TIME;

/// One recording row for one car slot.
///
/// The builder also detects a physics-frame gap. `new_run` marks a boundary
/// that the frame check cannot see, such as a teleport or a frozen row.
#[derive(Clone, Copy, Debug)]
pub(super) struct CarHistoryRow {
    /// Native car physics frame.
    pub physics_frame: u32,
    /// True when the row has physics, a ball, and this car slot.
    pub usable: bool,
    /// Native `IsOnGround()` flag. This is the authoritative ground state.
    pub native_on_ground: bool,
    /// Native jump component flag.
    pub is_jumping: bool,
    /// Native `GetbJumped()` flag.
    pub has_jumped: bool,
    /// Body orientation. Column 0 is forward, 1 is right, 2 is up.
    pub rot_mat: Mat3A,
    /// Native `FlipCar` active flag, when present.
    pub autoflip_active: Option<bool>,
    /// Native `FlipCar` activity clock, when present.
    pub autoflip_activity_time_s: Option<f32>,
    /// True when this row starts a new contiguous run.
    pub new_run: bool,
}

impl CarHistoryRow {
    /// Ground state used by the core contact cache.
    ///
    /// The native base `IsOnGround()` value is authoritative. The v12 wheel
    /// world-contact group is not a substitute.
    fn is_grounded(&self) -> bool {
        self.native_on_ground
    }
}

/// One precomputed history entry.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct CarHistoryEntry {
    usable: bool,
    autoflip: AutoFlipState,
    air_time: Option<f32>,
    air_time_since_jump: Option<f32>,
}

/// Reconstructed autoflip state for one row.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
enum AutoFlipState {
    #[default]
    Absent,
    Inactive,
    Timed {
        duration: f32,
        elapsed: f32,
    },
    Unknown,
}

/// Autoflip window under construction.
#[derive(Clone, Copy, Debug)]
struct Window {
    duration: f32,
    prev_activity: f32,
}

/// Per-row prefix state for one car slot.
///
/// Build one table per recording and car slot. Look up each row by its
/// physics frame. An invalid table has no entries.
#[derive(Clone, Debug, Default)]
pub(super) struct CarHistoryTable {
    frames: Vec<u32>,
    entries: Vec<CarHistoryEntry>,
    invalid: Option<CarHistoryInvalid>,
}

/// Reason the history frame index is not usable.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum CarHistoryInvalid {
    /// Two usable rows share a physics frame.
    DuplicateFrame(u32),
    /// A usable frame is lower than the previous usable frame.
    DecreasingFrame {
        /// Previous usable frame.
        previous: u32,
        /// Current usable frame.
        current: u32,
    },
}

impl std::fmt::Display for CarHistoryInvalid {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DuplicateFrame(frame) => write!(f, "duplicate usable frame {frame}"),
            Self::DecreasingFrame { previous, current } => {
                write!(f, "usable frame decreased from {previous} to {current}")
            }
        }
    }
}

impl CarHistoryTable {
    /// Build the table from the full recording.
    pub(super) fn from_ticks(ticks: &[TickRecord], slot: usize) -> Self {
        Self::build(&rows_from_ticks(ticks, slot))
    }

    /// Build the table from an explicit row list.
    pub(super) fn build(rows: &[CarHistoryRow]) -> Self {
        let mut frames = Vec::with_capacity(rows.len());
        let mut entries = Vec::with_capacity(rows.len());
        let mut air_value: Option<f32> = None;
        let mut jump_value: Option<f32> = None;
        let mut prev_ground: Option<bool> = None;
        let mut prev_jumping: Option<bool> = None;
        let mut prev_frame: Option<u32> = None;
        let mut prev_usable = false;
        let mut window: Option<Window> = None;

        for (index, row) in rows.iter().enumerate() {
            let frame_gap =
                prev_frame.is_some_and(|frame| row.physics_frame != frame.wrapping_add(1));
            let new_run = row.new_run || index == 0 || frame_gap || !row.usable || !prev_usable;

            if new_run {
                air_value = None;
                jump_value = None;
                prev_ground = None;
                prev_jumping = None;
                window = None;
            }

            if !row.usable {
                // Do not index unusable rows. A placeholder frame is
                // u32::MAX and would break the sorted frame lookup.
                prev_frame = Some(row.physics_frame);
                prev_usable = false;
                continue;
            }

            let ground = row.is_grounded();
            match prev_ground {
                Some(true) => {
                    // The core reads the previous row's ground cache.
                    air_value = Some(0.0);
                    jump_value = Some(0.0);
                }
                Some(false) => {
                    air_value = air_value.map(|value| value + TICK_TIME);
                    let jump_increases =
                        row.has_jumped && prev_jumping == Some(false) && !row.is_jumping;
                    jump_value = if jump_increases {
                        jump_value.map(|value| value + TICK_TIME)
                    } else {
                        Some(0.0)
                    };
                }
                None => {
                    // A run start has no previous ground row. Only a jump
                    // reset can prove zero.
                    air_value = None;
                    jump_value = if row.is_jumping || !row.has_jumped {
                        Some(0.0)
                    } else {
                        None
                    };
                }
            }

            let autoflip = autoflip_state(rows, index, new_run, &mut window);
            frames.push(row.physics_frame);
            entries.push(CarHistoryEntry {
                usable: true,
                autoflip,
                air_time: air_value,
                air_time_since_jump: jump_value,
            });
            prev_ground = Some(ground);
            prev_jumping = Some(row.is_jumping);
            prev_frame = Some(row.physics_frame);
            prev_usable = true;
        }

        let invalid = frames.windows(2).find_map(|pair| {
            if pair[0] == pair[1] {
                Some(CarHistoryInvalid::DuplicateFrame(pair[0]))
            } else if pair[0] > pair[1] {
                Some(CarHistoryInvalid::DecreasingFrame {
                    previous: pair[0],
                    current: pair[1],
                })
            } else {
                None
            }
        });
        if invalid.is_some() {
            frames.clear();
            entries.clear();
        }
        Self {
            frames,
            entries,
            invalid,
        }
    }

    /// Entry for one physics frame, or `None` when the frame is absent or
    /// the table is invalid.
    fn entry_for(&self, physics_frame: u32) -> Option<CarHistoryEntry> {
        let index = self.frames.partition_point(|&frame| frame <= physics_frame);
        let index = index.checked_sub(1)?;
        (self.frames[index] == physics_frame).then(|| self.entries[index])
    }

    /// True when the frame index is usable.
    pub(super) fn is_valid(&self) -> bool {
        self.invalid.is_none()
    }

    /// Reason the frame index is unusable, if any.
    ///
    /// The coordinator should count and warn for this reason. The apply falls
    /// back to partial values for an invalid table.
    pub(super) fn invalid_reason(&self) -> Option<CarHistoryInvalid> {
        self.invalid
    }
}

/// Apply the prefix-derived car clocks.
///
/// The caller must set the v11 row fields first. This function overrides the
/// no-history autoflip and airborne clocks with the reconstructed values.
/// `physics_frame` must come from the same recording as the table. An invalid
/// table or a missing frame returns partial airborne clocks.
pub(super) fn apply_car_replay_history(
    state: &mut CarState,
    ext: &CarExt,
    history: &CarHistoryTable,
    physics_frame: u32,
) -> CarRestoreReport {
    let mut report = CarRestoreReport::default();

    if let Some(flip_right) = ext.autoflip_right {
        state.auto_flip_torque_scale = auto_flip_scale(flip_right);
    }

    let entry = history.entry_for(physics_frame);
    match entry.map(|entry| entry.autoflip) {
        Some(AutoFlipState::Inactive) => {
            state.is_auto_flipping = false;
            state.auto_flip_timer = 0.0;
            report.autoflip = AutoFlipApply::Inactive;
        }
        Some(AutoFlipState::Timed { duration, elapsed }) => {
            let remaining = duration - elapsed;
            state.is_auto_flipping = true;
            state.auto_flip_timer = remaining;
            report.autoflip = AutoFlipApply::Timed {
                duration,
                remaining,
            };
        }
        Some(AutoFlipState::Unknown) => {
            state.is_auto_flipping = false;
            state.auto_flip_timer = 0.0;
            report.autoflip = AutoFlipApply::Unknown;
        }
        Some(AutoFlipState::Absent) => {
            if ext.autoflip_active.is_some() {
                // The table and the current row disagree. Clear both.
                state.is_auto_flipping = false;
                state.auto_flip_timer = 0.0;
                report.autoflip = AutoFlipApply::Unknown;
            }
        }
        None => match ext.autoflip_active {
            Some(false) => {
                state.is_auto_flipping = false;
                state.auto_flip_timer = 0.0;
                report.autoflip = AutoFlipApply::Inactive;
            }
            Some(true) => {
                state.is_auto_flipping = false;
                state.auto_flip_timer = 0.0;
                report.autoflip = AutoFlipApply::Unknown;
            }
            None => {}
        },
    }

    let (mut air_time, mut air_time_since_jump) = apply_jump_reset_zeroes(state);
    let usable_entry = entry.filter(|entry| entry.usable);
    if !history.is_valid() || usable_entry.is_none() {
        // An invalid or missing frame index cannot prove a row clock. Use
        // the deterministic partial zero, even when the current row is
        // grounded: a grounded row can be a landing tick.
        state.air_time = 0.0;
        state.air_time_since_jump = 0.0;
        air_time = ClockApply::PartialZero;
        air_time_since_jump = ClockApply::PartialZero;
    } else if let Some(entry) = usable_entry {
        air_time = match entry.air_time {
            Some(value) => {
                state.air_time = value;
                ClockApply::Exact(value)
            }
            None => {
                state.air_time = 0.0;
                ClockApply::PartialZero
            }
        };
        air_time_since_jump = match entry.air_time_since_jump {
            Some(value) => {
                state.air_time_since_jump = value;
                ClockApply::Exact(value)
            }
            None => {
                state.air_time_since_jump = 0.0;
                ClockApply::PartialZero
            }
        };
    }
    report.air_time = air_time;
    report.air_time_since_jump = air_time_since_jump;
    report
}

/// Build the per-slot rows from recording ticks.
fn rows_from_ticks(ticks: &[TickRecord], slot: usize) -> Vec<CarHistoryRow> {
    let mut rows = Vec::with_capacity(ticks.len());
    for (index, tick) in ticks.iter().enumerate() {
        let car = tick.car_records.get(slot);
        let usable = tick.car_available(slot) && tick.has_physics() && tick.ball_available();
        let new_run = index == 0 || !same_run(&ticks[index - 1], tick, slot);
        let ext = tick
            .replay_state
            .as_ref()
            .and_then(|state| state.cars.get(slot));
        rows.push(CarHistoryRow {
            physics_frame: car.map_or(0, |car| car.phys.physics_frame),
            usable,
            native_on_ground: car.is_some_and(|car| car.is_on_ground),
            is_jumping: car.is_some_and(|car| car.is_jumping),
            has_jumped: car.is_some_and(|car| car.has_jumped),
            rot_mat: car.map_or(Mat3A::IDENTITY, |car| {
                Mat3A::from_cols(
                    car.phys.rot.column(0).into(),
                    car.phys.rot.column(1).into(),
                    car.phys.rot.column(2).into(),
                )
            }),
            autoflip_active: ext.and_then(|ext| ext.autoflip_active),
            autoflip_activity_time_s: ext.and_then(|ext| ext.autoflip_activity_time_s),
            new_run,
        });
    }
    rows
}

/// Same run test for one car slot.
///
/// This keeps the row checks that matter for this slot. A change in another
/// car's availability does not break the slot history.
fn same_run(prev: &TickRecord, cur: &TickRecord, slot: usize) -> bool {
    tick_car_count(prev) == tick_car_count(cur)
        && prev.has_physics()
        && cur.has_physics()
        && prev.ball_available()
        && cur.ball_available()
        && prev.car_available(slot)
        && cur.car_available(slot)
        && prev.car_records[slot].phys.physics_frame.wrapping_add(1)
            == cur.car_records[slot].phys.physics_frame
        && prev.ball_record.physics_frame.wrapping_add(1) == cur.ball_record.physics_frame
        && !tick_is_frozen(prev, cur)
        && !any_teleport(prev, cur)
}

/// Reconstruct the autoflip state for one row.
fn autoflip_state(
    rows: &[CarHistoryRow],
    index: usize,
    new_run: bool,
    window: &mut Option<Window>,
) -> AutoFlipState {
    let row = &rows[index];
    let Some(active) = row.autoflip_active else {
        *window = None;
        return AutoFlipState::Absent;
    };
    if !active {
        *window = None;
        return AutoFlipState::Inactive;
    }
    if new_run {
        *window = None;
        return AutoFlipState::Unknown;
    }

    match rows[index - 1].autoflip_active {
        Some(true) => {
            let Some(current) = *window else {
                return AutoFlipState::Unknown;
            };
            let Some(activity) = row.autoflip_activity_time_s else {
                *window = None;
                return AutoFlipState::Unknown;
            };
            if phase_close(activity, current.prev_activity + TICK_TIME)
                && activity <= current.duration + TICK_TIME + PHASE_TOL
            {
                *window = Some(Window {
                    duration: current.duration,
                    prev_activity: activity,
                });
                AutoFlipState::Timed {
                    duration: current.duration,
                    elapsed: activity,
                }
            } else {
                *window = None;
                AutoFlipState::Unknown
            }
        }
        Some(false) => {
            let Some(activity) = row.autoflip_activity_time_s else {
                *window = None;
                return AutoFlipState::Unknown;
            };
            if !phase_close(activity, TICK_TIME) {
                *window = None;
                return AutoFlipState::Unknown;
            }
            let Some(duration) = core_autoflip_duration(rows[index - 1].rot_mat) else {
                *window = None;
                return AutoFlipState::Unknown;
            };
            *window = Some(Window {
                duration,
                prev_activity: activity,
            });
            AutoFlipState::Timed {
                duration,
                elapsed: activity,
            }
        }
        None => {
            *window = None;
            AutoFlipState::Unknown
        }
    }
}

/// Core autoflip activation duration.
///
/// This mirrors `rocketsim/src/sim/car/auto_flip.rs`. Keep it in sync with
/// the core. The roll comes from the orientation at the start of the
/// activation tick. Do not use a later rotated pose.
fn core_autoflip_duration(rot_mat: Mat3A) -> Option<f32> {
    let y_z = rot_mat.y_axis.z;
    let z_z = rot_mat.z_axis.z;
    let cy = (z_z * z_z + y_z * y_z).sqrt();
    let roll = if cy > 16.0 * f32::EPSILON {
        y_z.atan2(z_z)
    } else {
        0.0
    };
    let duration = AUTOFLIP_TIME * (roll.abs() / PI);
    duration.is_finite().then_some(duration)
}

/// True when a native clock step matches one tick.
fn phase_close(value: f32, expected: f32) -> bool {
    (value - expected).abs() <= PHASE_TOL
}

#[cfg(test)]
mod tests {
    use std::{path::PathBuf, sync::OnceLock};

    use glam::Vec3A;
    use rocketsim::CarState;
    use rocketsim_test::rlpr::Recording;

    use super::*;

    /// Roof-rest orientation. The core roll is exactly PI.
    fn roof_rot() -> Mat3A {
        Mat3A::from_cols(Vec3A::X, -Vec3A::Y, -Vec3A::Z)
    }

    fn row(
        physics_frame: u32,
        autoflip_active: Option<bool>,
        autoflip_activity_time_s: Option<f32>,
        rot_mat: Mat3A,
    ) -> CarHistoryRow {
        CarHistoryRow {
            physics_frame,
            usable: true,
            native_on_ground: false,
            is_jumping: false,
            has_jumped: false,
            rot_mat,
            autoflip_active,
            autoflip_activity_time_s,
            new_run: false,
        }
    }

    fn autoflip_ext(active: bool, activity: f32, right: bool) -> CarExt {
        CarExt {
            autoflip_active: Some(active),
            autoflip_right: Some(right),
            autoflip_activity_time_s: Some(activity),
            ..CarExt::default()
        }
    }

    fn apply(rows: &[CarHistoryRow], frame: u32, ext: &CarExt) -> (CarState, CarRestoreReport) {
        let table = CarHistoryTable::build(rows);
        let mut state = CarState::DEFAULT;
        let report = apply_car_replay_history(&mut state, ext, &table, frame);
        (state, report)
    }

    #[test]
    fn autoflip_uses_pre_activation_pose() {
        let t = TICK_TIME;
        let rows = vec![
            row(10, Some(false), Some(5.0), Mat3A::IDENTITY),
            row(11, Some(false), Some(5.0 + t), roof_rot()),
            row(12, Some(true), Some(t), Mat3A::IDENTITY),
            row(13, Some(true), Some(2.0 * t), Mat3A::IDENTITY),
            row(14, Some(true), Some(3.0 * t), Mat3A::IDENTITY),
        ];
        let ext = autoflip_ext(true, 3.0 * t, true);

        let (state, report) = apply(&rows, 14, &ext);

        let remaining = 0.4f32 - 3.0 * t;
        assert!(state.is_auto_flipping);
        assert_eq!(state.auto_flip_timer.to_bits(), remaining.to_bits());
        assert_eq!(state.auto_flip_torque_scale, 1.0);
        assert_eq!(
            report.autoflip,
            AutoFlipApply::Timed {
                duration: 0.4,
                remaining,
            }
        );
    }

    #[test]
    fn autoflip_duration_follows_pre_activation_roll() {
        let t = TICK_TIME;
        // Quarter roll at activation. The current rows are upright, so a
        // current-pose duration would return 0.0 instead of about 0.2.
        let pre = Mat3A::from_rotation_x(-std::f32::consts::FRAC_PI_2);
        let expected_duration = core_autoflip_duration(pre).unwrap();
        assert!(expected_duration < 0.21);
        let rows = vec![
            row(10, Some(false), Some(5.0), Mat3A::IDENTITY),
            row(11, Some(false), Some(5.0 + t), pre),
            row(12, Some(true), Some(t), Mat3A::IDENTITY),
            row(13, Some(true), Some(2.0 * t), Mat3A::IDENTITY),
        ];
        let ext = autoflip_ext(true, 2.0 * t, false);

        let (state, report) = apply(&rows, 13, &ext);

        assert!(state.is_auto_flipping);
        assert_eq!(state.auto_flip_torque_scale, -1.0);
        let remaining = expected_duration - 2.0 * t;
        assert_eq!(state.auto_flip_timer.to_bits(), remaining.to_bits());
        assert_eq!(
            report.autoflip,
            AutoFlipApply::Timed {
                duration: expected_duration,
                remaining,
            }
        );
    }

    #[test]
    fn autoflip_run_start_is_unknown() {
        let t = TICK_TIME;
        let rows = vec![
            row(10, Some(true), Some(t), roof_rot()),
            row(11, Some(true), Some(2.0 * t), roof_rot()),
        ];
        let ext = autoflip_ext(true, 2.0 * t, true);

        let (state, report) = apply(&rows, 11, &ext);

        assert!(!state.is_auto_flipping);
        assert_eq!(state.auto_flip_timer, 0.0);
        assert_eq!(report.autoflip, AutoFlipApply::Unknown);
    }

    #[test]
    fn autoflip_frame_gap_is_unknown() {
        let t = TICK_TIME;
        let rows = vec![
            row(10, Some(false), Some(5.0), Mat3A::IDENTITY),
            row(11, Some(false), Some(5.0 + t), roof_rot()),
            row(12, Some(true), Some(t), Mat3A::IDENTITY),
            // Frame 14 is not 13. The run starts again here.
            row(14, Some(true), Some(2.0 * t), Mat3A::IDENTITY),
        ];
        let ext = autoflip_ext(true, 2.0 * t, true);

        let (state, report) = apply(&rows, 14, &ext);

        assert!(!state.is_auto_flipping);
        assert_eq!(state.auto_flip_timer, 0.0);
        assert_eq!(report.autoflip, AutoFlipApply::Unknown);
    }

    #[test]
    fn autoflip_edge_phase_mismatch_is_unknown() {
        let t = TICK_TIME;
        let rows = vec![
            row(10, Some(false), Some(5.0), Mat3A::IDENTITY),
            row(11, Some(false), Some(5.0 + t), roof_rot()),
            // The activation row must read one tick.
            row(12, Some(true), Some(2.0 * t), Mat3A::IDENTITY),
        ];
        let ext = autoflip_ext(true, 2.0 * t, true);

        let (state, report) = apply(&rows, 12, &ext);

        assert!(!state.is_auto_flipping);
        assert_eq!(state.auto_flip_timer, 0.0);
        assert_eq!(report.autoflip, AutoFlipApply::Unknown);
    }

    #[test]
    fn inactive_row_clears_stale_timer() {
        let t = TICK_TIME;
        let rows = vec![
            row(10, Some(false), Some(5.0), Mat3A::IDENTITY),
            row(11, Some(false), Some(5.0 + t), roof_rot()),
            row(12, Some(true), Some(t), Mat3A::IDENTITY),
            row(13, Some(false), Some(t), Mat3A::IDENTITY),
        ];
        let ext = autoflip_ext(false, t, true);
        let table = CarHistoryTable::build(&rows);
        let mut state = CarState::DEFAULT;
        state.is_auto_flipping = true;
        state.auto_flip_timer = 0.25;

        let report = apply_car_replay_history(&mut state, &ext, &table, 13);

        assert!(!state.is_auto_flipping);
        assert_eq!(state.auto_flip_timer, 0.0);
        assert_eq!(report.autoflip, AutoFlipApply::Inactive);
    }

    #[test]
    fn air_clocks_use_previous_row_ground() {
        let t = TICK_TIME;
        let mut rows = Vec::new();
        let mut arms = |frame, ground, jumping, jumped| {
            let mut row = row(frame, None, None, Mat3A::IDENTITY);
            row.native_on_ground = ground;
            row.is_jumping = jumping;
            row.has_jumped = jumped;
            rows.push(row);
        };
        arms(10, true, false, false);
        arms(11, false, true, true);
        arms(12, false, false, true);
        arms(13, false, false, true);
        arms(14, false, false, true);
        let ext = CarExt::default();

        let (state, report) = apply(&rows, 14, &ext);

        // The core reads the previous row's ground. Row 11 is the first
        // airborne tick, so it keeps air_time 0. Rows 12..14 add one tick
        // each.
        let mut expected_air = 0.0f32;
        for _ in 0..3 {
            expected_air += t;
        }
        let mut expected_jump = 0.0f32;
        for _ in 0..2 {
            expected_jump += t;
        }
        assert_eq!(state.air_time.to_bits(), expected_air.to_bits());
        assert_eq!(state.air_time_since_jump.to_bits(), expected_jump.to_bits());
        assert_eq!(report.air_time, ClockApply::Exact(expected_air));
        assert_eq!(report.air_time_since_jump, ClockApply::Exact(expected_jump));
    }

    #[test]
    fn jump_end_row_resets_air_time_since_jump() {
        let t = TICK_TIME;
        let mut rows = Vec::new();
        let mut arms = |frame, ground, jumping, jumped| {
            let mut row = row(frame, None, None, Mat3A::IDENTITY);
            row.native_on_ground = ground;
            row.is_jumping = jumping;
            row.has_jumped = jumped;
            rows.push(row);
        };
        arms(10, true, false, false);
        arms(11, false, true, true);
        arms(12, false, false, true);
        let ext = CarExt::default();

        let (state, report) = apply(&rows, 12, &ext);

        // Row 11 is the first airborne tick after a grounded row, so it
        // keeps air_time 0. Row 12 adds one tick. The jump ends on row 12,
        // so air_time_since_jump resets to 0.
        assert_eq!(state.air_time.to_bits(), t.to_bits());
        assert_eq!(state.air_time_since_jump, 0.0);
        assert_eq!(report.air_time_since_jump, ClockApply::Exact(0.0));
    }

    #[test]
    fn landing_row_keeps_air_time() {
        let t = TICK_TIME;
        let mut rows = Vec::new();
        let mut arms = |frame, ground| {
            let mut row = row(frame, None, None, Mat3A::IDENTITY);
            row.native_on_ground = ground;
            rows.push(row);
        };
        arms(10, true);
        arms(11, false);
        arms(12, false);
        arms(13, true);
        arms(14, true);
        let ext = CarExt::default();

        let (landing, landing_report) = apply(&rows, 13, &ext);
        let (next, _) = apply(&rows, 14, &ext);

        let mut expected_landing = 0.0f32;
        for _ in 0..2 {
            expected_landing += t;
        }
        assert_eq!(landing.air_time.to_bits(), expected_landing.to_bits());
        assert!(landing.air_time > 0.0);
        assert_eq!(next.air_time, 0.0);
        assert_eq!(landing_report.air_time, ClockApply::Exact(expected_landing));
    }

    #[test]
    fn grounded_run_start_is_unknown() {
        let rows = vec![row(10, None, None, Mat3A::IDENTITY)];
        let mut with_ground = rows;
        with_ground[0].native_on_ground = true;
        let table = CarHistoryTable::build(&with_ground);
        let mut state = CarState::DEFAULT;
        state.set_contact_flags(true, [true; 4]);

        let report = apply_car_replay_history(&mut state, &CarExt::default(), &table, 10);

        assert_eq!(state.air_time, 0.0);
        assert_eq!(report.air_time, ClockApply::PartialZero);
    }

    #[test]
    fn run_start_airborne_clocks_are_partial_zero() {
        let mut rows = vec![
            row(10, None, None, Mat3A::IDENTITY),
            row(11, None, None, Mat3A::IDENTITY),
        ];
        for row in &mut rows {
            row.has_jumped = true;
        }
        let ext = CarExt::default();
        let table = CarHistoryTable::build(&rows);
        let mut state = CarState::DEFAULT;
        state.set_contact_flags(false, [false; 4]);
        state.has_jumped = true;

        let report = apply_car_replay_history(&mut state, &ext, &table, 11);

        assert_eq!(state.air_time, 0.0);
        assert_eq!(state.air_time_since_jump, 0.0);
        assert_eq!(report.air_time, ClockApply::PartialZero);
        assert_eq!(report.air_time_since_jump, ClockApply::PartialZero);
    }

    #[test]
    fn full_turtle_window_reconstructs_each_active_row() {
        let t = TICK_TIME;
        let mut rows = vec![
            row(0, Some(false), Some(10.0), Mat3A::IDENTITY),
            row(1, Some(false), Some(10.0 + t), roof_rot()),
        ];
        for step in 1..=48u32 {
            rows.push(row(
                1 + step,
                Some(true),
                Some(step as f32 * t),
                Mat3A::IDENTITY,
            ));
        }
        rows.push(row(50, Some(false), Some(t), Mat3A::IDENTITY));
        let table = CarHistoryTable::build(&rows);
        let mut state = CarState::DEFAULT;
        let frame = 25;
        let step = frame - 1;
        let ext = autoflip_ext(true, step as f32 * t, true);

        let report = apply_car_replay_history(&mut state, &ext, &table, frame);

        let remaining = 0.4f32 - step as f32 * t;
        assert!(state.is_auto_flipping);
        assert_eq!(state.auto_flip_timer.to_bits(), remaining.to_bits());
        assert_eq!(
            report.autoflip,
            AutoFlipApply::Timed {
                duration: 0.4,
                remaining,
            }
        );

        let mut last = CarState::DEFAULT;
        let last_report =
            apply_car_replay_history(&mut last, &autoflip_ext(true, 48.0 * t, true), &table, 49);
        assert!(last.is_auto_flipping);
        assert!(last.auto_flip_timer <= 0.0);
        assert!(matches!(last_report.autoflip, AutoFlipApply::Timed { .. }));

        let mut after = CarState::DEFAULT;
        after.is_auto_flipping = true;
        after.auto_flip_timer = 0.1;
        let after_report =
            apply_car_replay_history(&mut after, &autoflip_ext(false, t, true), &table, 50);
        assert!(!after.is_auto_flipping);
        assert_eq!(after.auto_flip_timer, 0.0);
        assert_eq!(after_report.autoflip, AutoFlipApply::Inactive);
    }

    #[test]
    fn adapter_ground_uses_native_base_flag() {
        let ticks = recording_ticks();
        let slot = 0;
        let start = usable_window(ticks, slot, 1);
        let mut row = ticks[start].clone();
        assert!(
            row.replay_state
                .as_ref()
                .is_some_and(|state| state.cars[slot].wheel_contacts.is_some())
        );

        // Native grounded wins over a world group that reports no wheel
        // contact with world geometry.
        row.car_records[slot].is_on_ground = true;
        for wheel in row.replay_state.as_mut().unwrap().cars[slot]
            .wheel_contacts
            .as_mut()
            .unwrap()
        {
            wheel.has_contact_world_geometry = false;
        }
        let history = rows_from_ticks(std::slice::from_ref(&row), slot);
        assert!(history[0].is_grounded());

        // Native airborne wins over a world group that reports every wheel
        // in contact with world geometry.
        let mut row = ticks[start].clone();
        row.car_records[slot].is_on_ground = false;
        for wheel in row.replay_state.as_mut().unwrap().cars[slot]
            .wheel_contacts
            .as_mut()
            .unwrap()
        {
            wheel.has_contact_world_geometry = true;
        }
        let history = rows_from_ticks(std::slice::from_ref(&row), slot);
        assert!(!history[0].is_grounded());
    }

    #[test]
    fn car_history_frame_after_unusable_row_resolves() {
        let ticks = recording_ticks();
        let slot = 0;
        let start = usable_window(ticks, slot, 4);
        let mut rows = ticks[start..start + 4].to_vec();
        rows[1].car_records[slot].physics_available = 0;
        rows[1].car_records[slot].phys.physics_frame = u32::MAX;

        let table = CarHistoryTable::from_ticks(&rows, slot);
        assert!(table.is_valid());
        assert!(table.invalid_reason().is_none());
        assert_eq!(table.frames.len(), 3);
        assert!(table.frames.windows(2).all(|pair| pair[0] < pair[1]));

        let frame3 = rows[2].car_records[slot].phys.physics_frame;
        let frame4 = rows[3].car_records[slot].phys.physics_frame;
        assert!(table.entry_for(frame3).is_some());
        assert!(table.entry_for(frame4).is_some());

        let ext = rows[2].replay_state.as_ref().unwrap().cars[slot].clone();
        let mut state = CarState::DEFAULT;
        let report = apply_car_replay_history(&mut state, &ext, &table, frame3);
        assert_ne!(report.autoflip, AutoFlipApply::Absent);
    }

    #[test]
    fn other_car_mask_change_keeps_slot_history() {
        let ticks = recording_ticks();
        let slot = 0;
        let other = 1;
        let index = usable_pair_with_both(ticks, slot, other);
        let mut rows = ticks[index - 1..=index].to_vec();
        rows[1].car_records[other].physics_available = 0;
        rows[1].car_records[other].phys.physics_frame = u32::MAX;

        let history = rows_from_ticks(&rows, slot);
        assert!(!history[1].new_run);

        let mut rows = ticks[index - 1..=index].to_vec();
        rows[1].car_records[slot].physics_available = 0;
        rows[1].car_records[slot].phys.physics_frame = u32::MAX;
        let history = rows_from_ticks(&rows, slot);
        assert!(history[1].new_run);
    }

    #[test]
    fn duplicate_usable_frame_invalidates_table() {
        let rows = vec![
            row(10, None, None, Mat3A::IDENTITY),
            row(10, None, None, Mat3A::IDENTITY),
        ];
        let table = CarHistoryTable::build(&rows);
        assert!(!table.is_valid());
        assert_eq!(
            table.invalid_reason(),
            Some(CarHistoryInvalid::DuplicateFrame(10))
        );
        assert_eq!(
            table.invalid_reason().unwrap().to_string(),
            "duplicate usable frame 10"
        );
        assert!(table.entry_for(10).is_none());

        let mut state = CarState::DEFAULT;
        let report = apply_car_replay_history(&mut state, &CarExt::default(), &table, 10);
        assert_eq!(report.air_time, ClockApply::PartialZero);
        assert_eq!(report.air_time_since_jump, ClockApply::PartialZero);
        assert_eq!(state.air_time, 0.0);
        assert_eq!(state.air_time_since_jump, 0.0);
    }

    #[test]
    fn decreasing_usable_frame_invalidates_table() {
        let rows = vec![
            row(10, None, None, Mat3A::IDENTITY),
            row(12, None, None, Mat3A::IDENTITY),
            row(11, None, None, Mat3A::IDENTITY),
        ];
        let table = CarHistoryTable::build(&rows);
        assert!(!table.is_valid());
        assert_eq!(
            table.invalid_reason(),
            Some(CarHistoryInvalid::DecreasingFrame {
                previous: 12,
                current: 11,
            })
        );
        assert_eq!(
            table.invalid_reason().unwrap().to_string(),
            "usable frame decreased from 12 to 11"
        );
        assert!(table.entry_for(10).is_none());
        assert!(table.entry_for(11).is_none());
        assert!(table.entry_for(12).is_none());
    }

    #[test]
    fn autoflip_remaining_subtraction_drift_is_below_one_tick() {
        let duration = 0.4f32;
        let mut by_subtraction = duration;
        for step in 1..=48u32 {
            by_subtraction -= TICK_TIME;
            let by_elapsed = duration - step as f32 * TICK_TIME;
            assert!((by_elapsed - by_subtraction).abs() < TICK_TIME);
        }
    }

    fn recording_ticks() -> &'static [TickRecord] {
        static TICKS: OnceLock<Vec<TickRecord>> = OnceLock::new();
        TICKS.get_or_init(|| {
            let root = std::env::var_os("ROCKETSIM_TEST_DIR")
                .map(PathBuf::from)
                .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")));
            let path = root.join("recordings/nexto_1v1_300s.rlpr.zst");
            let recording = Recording::from_file(&path)
                .unwrap_or_else(|error| panic!("load {}: {error}", path.display()));
            crate::common::collapse_duplicate_ticks(&recording.ticks)
        })
    }

    fn usable_window(ticks: &[TickRecord], slot: usize, count: usize) -> usize {
        assert!(ticks.len() >= count, "recording is too short");
        for start in 0..=ticks.len() - count {
            let window = &ticks[start..start + count];
            if window
                .iter()
                .all(|tick| tick.car_available(slot) && tick.has_physics() && tick.ball_available())
                && window
                    .windows(2)
                    .all(|pair| same_slot_row(&pair[0], &pair[1], slot))
            {
                return start;
            }
        }
        panic!("no contiguous usable window for slot {slot}");
    }

    fn usable_pair_with_both(ticks: &[TickRecord], slot: usize, other: usize) -> usize {
        for index in 1..ticks.len() {
            let prev = &ticks[index - 1];
            let cur = &ticks[index];
            if prev.car_available(slot)
                && cur.car_available(slot)
                && prev.car_available(other)
                && cur.car_available(other)
                && same_slot_row(prev, cur, slot)
            {
                return index;
            }
        }
        panic!("no contiguous pair for slots {slot} and {other}");
    }

    fn same_slot_row(prev: &TickRecord, cur: &TickRecord, slot: usize) -> bool {
        prev.has_physics()
            && cur.has_physics()
            && prev.ball_available()
            && cur.ball_available()
            && prev.car_available(slot)
            && cur.car_available(slot)
            && prev.car_records[slot].phys.physics_frame.wrapping_add(1)
                == cur.car_records[slot].phys.physics_frame
            && prev.ball_record.physics_frame.wrapping_add(1) == cur.ball_record.physics_frame
            && !crate::common::tick_is_frozen(prev, cur)
            && !crate::common::any_teleport(prev, cur)
    }
}
