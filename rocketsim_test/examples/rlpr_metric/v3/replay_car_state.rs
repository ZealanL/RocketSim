//! Proven RLPR v12 car mappings and the no-history fallback.
//!
//! [`apply_car_replay_state`] maps fields that need no prefix history. It
//! cannot reconstruct the remaining autoflip timer or the core airborne
//! clocks. Use [`super::replay_car_history`] at segment starts to rebuild
//! those values from the recording prefix.
//!
//! The no-history fallback never leaves an active autoflip flag with a stale
//! timer when the field is present. It clears the flag and the timer together.
//! When the field is absent, the live simulator value is preserved because
//! there is no recorded value. The report records that the value was not
//! reconstructed.
//!
//! The existing `v3::apply_v12_car_state` function applies supersonic state.
//! Wheel world contact is captured but not applied: the core wheel
//! `has_contact` and native `is_on_ground` stay authoritative. The v11 row
//! record applies the jump and dodge state. This module does not repeat those
//! mappings.
//!
//! ## Autoflip
//!
//! The car group bit 2 is the native `FlipCar` component. These fields map to
//! the core auto-flip state:
//!
//! - `autoflip_active` is the native `bActive` flag. It maps to
//!   [`CarState::is_auto_flipping`].
//! - `autoflip_right` is the native `bFlipRight` latch. It maps to
//!   [`CarState::auto_flip_torque_scale`]: `true` is `+1.0` and `false` is
//!   `-1.0`.
//!
//! `autoflip_activity_time_s` is an elapsed clock. It is not a remaining
//! timer, so it does not map to [`CarState::auto_flip_timer`] without the
//! prefix history.
//! `autoflip_active_time_s` and `autoflip_inactive_time_s` are the active and
//! inactive views of the same elapsed clock. They add no state.
//!
//! ## Jump
//!
//! The car group bit 3 is the native `JumpComponent`. Its `bActive` and
//! `ActivityTime` values are the same getters as the v11 row `is_jumping` and
//! `jump_time` fields. The v11 path already restores them. This module does
//! not write the jump latch.
//!
//! ## Wheel contact
//!
//! The v12 wheel record holds `had_contact` and `has_contact_change_time`.
//! The current [`CarState`] has no per-wheel previous-contact latch and no
//! per-wheel contact-change clock. This module does not write wheel state.
//! The v12 `has_contact_world_geometry` flag is distinct unsupported data and
//! does not replace the core wheel contact or `is_on_ground`.
//!
//! [`CarState::is_auto_flipping`]: rocketsim::CarState::is_auto_flipping
//! [`CarState::auto_flip_torque_scale`]: rocketsim::CarState::auto_flip_torque_scale
//! [`CarState::auto_flip_timer`]: rocketsim::CarState::auto_flip_timer

use rocketsim::CarState;
use rocketsim_test::rlpr::row_ext::CarExt;

/// Result of one car-state restore.
///
/// The caller can count partial restores for coverage output.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) struct CarRestoreReport {
    /// Autoflip restore status.
    pub autoflip: AutoFlipApply,
    /// Total airborne clock status.
    pub air_time: ClockApply,
    /// Clock since the jump ended status.
    pub air_time_since_jump: ClockApply,
}

/// Autoflip restore status.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) enum AutoFlipApply {
    /// The row has no autoflip field.
    #[default]
    Absent,
    /// The native `FlipCar` flag is false. The timer is zero.
    Inactive,
    /// The remaining timer is known.
    Timed { duration: f32, remaining: f32 },
    /// The row is active, but the remaining timer is not known.
    Unknown,
}

/// Airborne clock restore status.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) enum ClockApply {
    /// The row has no clock entry.
    #[default]
    Absent,
    /// The value is exact.
    Exact(f32),
    /// The value is zero because the prefix is unknown.
    PartialZero,
}

/// Apply the RLPR v12 car fields that need no prefix history.
///
/// The caller must set the v11 row fields first. This function does not apply
/// supersonic state, wheel contact, jump latch state, or dodge state.
///
/// An absent field leaves the current value unchanged. An active autoflip
/// without a prefix history clears the flag and the timer together. An absent
/// autoflip field keeps the live simulator value.
pub(super) fn apply_car_replay_state(state: &mut CarState, ext: &CarExt) -> CarRestoreReport {
    let mut report = CarRestoreReport::default();

    if let Some(active) = ext.autoflip_active {
        state.is_auto_flipping = false;
        state.auto_flip_timer = 0.0;
        report.autoflip = if active {
            AutoFlipApply::Unknown
        } else {
            AutoFlipApply::Inactive
        };
    }

    if let Some(flip_right) = ext.autoflip_right {
        state.auto_flip_torque_scale = auto_flip_scale(flip_right);
    }

    let (air_time, air_time_since_jump) = apply_jump_reset_zeroes(state);
    report.air_time = air_time;
    report.air_time_since_jump = air_time_since_jump;
    report
}

/// Zero the airborne clocks on the rows where the core resets them.
///
/// The core resets `air_time` and `air_time_since_jump` on the ground. It
/// resets `air_time_since_jump` while jumping and when `has_jumped` is false.
/// The other airborne values need the prefix history.
///
/// This function writes zero for an unknown clock. The report marks that
/// value [`ClockApply::PartialZero`].
pub(super) fn apply_jump_reset_zeroes(state: &mut CarState) -> (ClockApply, ClockApply) {
    if state.is_on_ground() {
        state.air_time = 0.0;
        state.air_time_since_jump = 0.0;
        return (ClockApply::Exact(0.0), ClockApply::Exact(0.0));
    }

    state.air_time = 0.0;
    if state.is_jumping || !state.has_jumped {
        state.air_time_since_jump = 0.0;
        (ClockApply::PartialZero, ClockApply::Exact(0.0))
    } else {
        state.air_time_since_jump = 0.0;
        (ClockApply::PartialZero, ClockApply::PartialZero)
    }
}

/// Torque scale for the native `bFlipRight` latch.
///
/// The ecbc static evidence shows that `InitFlip` writes
/// `bFlipRight = Roll > 0`. The native sustain body adds the component torque
/// with `+1.0` when the bit is set and `-1.0` when the bit is clear.
/// [`CarState::auto_flip_torque_scale`] uses the same two values.
///
/// Use the recorded latch. Do not recompute the sign from a later row. The car
/// rotates during the recovery, so the roll at a restore row is not the roll
/// at activation.
#[inline]
pub(super) const fn auto_flip_scale(flip_right: bool) -> f32 {
    if flip_right { 1.0 } else { -1.0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn autoflip_ext(active: bool, right: Option<bool>) -> CarExt {
        CarExt {
            autoflip_active: Some(active),
            autoflip_right: right,
            autoflip_activity_time_s: Some(0.2),
            autoflip_active_time_s: Some(if active { 0.2 } else { 0.0 }),
            autoflip_inactive_time_s: Some(if active { 0.0 } else { 0.2 }),
            ..CarExt::default()
        }
    }

    #[test]
    fn active_autoflip_without_history_is_unknown() {
        let mut state = CarState::DEFAULT;
        state.auto_flip_timer = 0.125;

        let report = apply_car_replay_state(&mut state, &autoflip_ext(true, Some(true)));

        assert!(!state.is_auto_flipping);
        assert_eq!(state.auto_flip_timer, 0.0);
        assert_eq!(state.auto_flip_torque_scale, 1.0);
        assert_eq!(report.autoflip, AutoFlipApply::Unknown);
    }

    #[test]
    fn left_autoflip_sets_negative_scale() {
        let mut state = CarState::DEFAULT;

        let report = apply_car_replay_state(&mut state, &autoflip_ext(true, Some(false)));

        assert!(!state.is_auto_flipping);
        assert_eq!(state.auto_flip_torque_scale, -1.0);
        assert_eq!(report.autoflip, AutoFlipApply::Unknown);
    }

    #[test]
    fn inactive_autoflip_clears_flag_and_timer() {
        let mut state = CarState::DEFAULT;
        state.is_auto_flipping = true;
        state.auto_flip_timer = 0.25;

        let report = apply_car_replay_state(&mut state, &autoflip_ext(false, Some(true)));

        assert!(!state.is_auto_flipping);
        assert_eq!(state.auto_flip_timer, 0.0);
        assert_eq!(state.auto_flip_torque_scale, 1.0);
        assert_eq!(report.autoflip, AutoFlipApply::Inactive);
    }

    #[test]
    fn absent_autoflip_keeps_live_value() {
        let mut state = CarState::DEFAULT;
        state.is_auto_flipping = true;
        state.auto_flip_timer = 0.1;
        state.auto_flip_torque_scale = 1.0;

        let report = apply_car_replay_state(&mut state, &CarExt::default());

        assert!(state.is_auto_flipping);
        assert_eq!(state.auto_flip_timer, 0.1);
        assert_eq!(state.auto_flip_torque_scale, 1.0);
        assert_eq!(report.autoflip, AutoFlipApply::Absent);
    }

    #[test]
    fn absent_right_preserves_scale() {
        let mut state = CarState::DEFAULT;
        state.auto_flip_torque_scale = -1.0;

        apply_car_replay_state(&mut state, &autoflip_ext(true, None));

        assert_eq!(state.auto_flip_torque_scale, -1.0);
    }

    #[test]
    fn jump_group_does_not_change_jump_latch() {
        let mut state = CarState::DEFAULT;
        state.is_jumping = true;
        state.jump_ticks = 5;
        let ext = CarExt {
            jump_active: Some(false),
            jump_activity_time_s: Some(0.5),
            jump_inactive_time_s: Some(0.5),
            ..CarExt::default()
        };

        apply_car_replay_state(&mut state, &ext);

        assert!(state.is_jumping);
        assert_eq!(state.jump_ticks, 5);
    }

    #[test]
    fn grounded_state_zeroes_both_airborne_clocks() {
        let mut state = CarState::DEFAULT;
        state.set_contact_flags(true, [true; 4]);
        state.air_time = 1.0;
        state.air_time_since_jump = 2.0;

        let report = apply_car_replay_state(&mut state, &CarExt::default());

        assert_eq!(state.air_time, 0.0);
        assert_eq!(state.air_time_since_jump, 0.0);
        assert_eq!(report.air_time, ClockApply::Exact(0.0));
        assert_eq!(report.air_time_since_jump, ClockApply::Exact(0.0));
    }

    #[test]
    fn native_base_ground_wins_over_wheel_ext_for_air_clocks() {
        use rocketsim_test::rlpr::row_ext::WheelContactExt;

        let no_world = WheelContactExt {
            has_contact: false,
            has_contact_world_geometry: false,
            ..WheelContactExt::default()
        };
        let world = WheelContactExt {
            has_contact: true,
            has_contact_world_geometry: true,
            ..WheelContactExt::default()
        };

        // The base cache says grounded while the world group says no wheel
        // touches world geometry. The clocks must still reset.
        let mut state = CarState::DEFAULT;
        state.set_contact_flags(true, [true; 4]);
        state.air_time = 1.0;
        state.air_time_since_jump = 2.0;
        let report = apply_car_replay_state(
            &mut state,
            &CarExt {
                wheel_contacts: Some([no_world; 4]),
                ..CarExt::default()
            },
        );
        assert_eq!(state.air_time, 0.0);
        assert_eq!(state.air_time_since_jump, 0.0);
        assert_eq!(report.air_time, ClockApply::Exact(0.0));
        assert_eq!(report.air_time_since_jump, ClockApply::Exact(0.0));

        // The base cache says airborne while the world group says every
        // wheel touches world geometry. The world group must not ground it.
        let mut state = CarState::DEFAULT;
        state.set_contact_flags(false, [false; 4]);
        state.air_time = 1.0;
        state.air_time_since_jump = 2.0;
        state.has_jumped = true;
        let report = apply_car_replay_state(
            &mut state,
            &CarExt {
                wheel_contacts: Some([world; 4]),
                ..CarExt::default()
            },
        );
        assert_eq!(state.air_time, 0.0);
        assert_eq!(state.air_time_since_jump, 0.0);
        assert_eq!(report.air_time, ClockApply::PartialZero);
        assert_eq!(report.air_time_since_jump, ClockApply::PartialZero);
    }

    #[test]
    fn jumping_state_zeroes_air_time_since_jump() {
        let mut state = CarState::DEFAULT;
        state.set_contact_flags(false, [false; 4]);
        state.is_jumping = true;
        state.has_jumped = true;
        state.air_time = 1.0;
        state.air_time_since_jump = 2.0;

        let report = apply_car_replay_state(&mut state, &CarExt::default());

        assert_eq!(state.air_time, 0.0);
        assert_eq!(state.air_time_since_jump, 0.0);
        assert_eq!(report.air_time, ClockApply::PartialZero);
        assert_eq!(report.air_time_since_jump, ClockApply::Exact(0.0));
    }

    #[test]
    fn airborne_after_jump_without_history_is_partial() {
        let mut state = CarState::DEFAULT;
        state.set_contact_flags(false, [false; 4]);
        state.has_jumped = true;
        state.air_time = 1.0;
        state.air_time_since_jump = 2.0;

        let report = apply_car_replay_state(&mut state, &CarExt::default());

        assert_eq!(state.air_time, 0.0);
        assert_eq!(state.air_time_since_jump, 0.0);
        assert_eq!(report.air_time, ClockApply::PartialZero);
        assert_eq!(report.air_time_since_jump, ClockApply::PartialZero);
    }

    #[test]
    fn turtle_capture_row_without_history_is_unknown() {
        let mut state = CarState::DEFAULT;
        state.auto_flip_timer = 0.0;
        state.is_flipping = true;
        state.flip_time = 0.5;
        let ext = CarExt {
            autoflip_active: Some(true),
            autoflip_right: Some(true),
            autoflip_activity_time_s: Some(0.008_333_333),
            autoflip_active_time_s: Some(0.008_333_333),
            autoflip_inactive_time_s: Some(0.0),
            ..CarExt::default()
        };

        let report = apply_car_replay_state(&mut state, &ext);

        assert!(!state.is_auto_flipping);
        assert_eq!(state.auto_flip_timer, 0.0);
        assert_eq!(state.auto_flip_torque_scale, 1.0);
        assert_eq!(report.autoflip, AutoFlipApply::Unknown);
        // The FlipCar group is not the Dodge state. The v11 row owns those.
        assert!(state.is_flipping);
        assert_eq!(state.flip_time, 0.5);
    }
}
