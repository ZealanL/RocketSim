use glam::Vec3A;
use rocketsim_test::rlpr::{cpp_records::ControlsRecord, tick_record::TickRecord};

/// State restoration mode. Motion-only seeds fully once per segment,
/// then preserves orientation and logical history across transitions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResetMode {
    Segmented,
    ResetEachTick,
    ResetMotionEachTick,
}

impl ResetMode {
    /// Map CLI flags to a mode. Panics when both reset flags are set: the
    /// CLI declares them conflicting, so this is a backup guard.
    pub fn from_flags(reset_each_tick: bool, reset_motion_each_tick: bool) -> Self {
        match (reset_each_tick, reset_motion_each_tick) {
            (false, false) => Self::Segmented,
            (true, false) => Self::ResetEachTick,
            (false, true) => Self::ResetMotionEachTick,
            (true, true) => {
                panic!("--reset-each-tick and --reset-motion-each-tick are mutually exclusive")
            }
        }
    }
}
/// Plain body state. Axes are unit vectors. Uses [`Vec3A`].
#[derive(Clone, Copy, Debug)]
pub struct BodySnapshot {
    pub pos: Vec3A,
    pub vel: Vec3A,
    pub ang_vel: Vec3A,
    pub forward: Vec3A,
    pub up: Vec3A,
}

/// Plain car plus ball state.
#[derive(Clone, Copy, Debug)]
pub struct Snapshot {
    pub car: BodySnapshot,
    pub ball: BodySnapshot,
}

/// Backend adapter. Holds the sim with one arena car per recorded car.
/// `step` reports the contacts the sim itself observed that tick, one
/// entry per arena car in recording order.
pub trait ReplayBackend {
    /// Build prefix history from the same deduplicated rows that are scored.
    /// This does not step the simulation or change the scoring rules.
    fn prepare_replay(&mut self, _ticks: &[TickRecord]) {}

    fn reset(&mut self, start: &TickRecord);
    fn set_state(&mut self, state: &TickRecord);

    /// Motion-only reset: overwrite exactly car/ball `pos`/`vel`/`ang_vel`.
    ///
    /// Preserve orientation and logical history. Do not explicitly clear
    /// wheel or manifold caches. State setters can clear transient caches.
    /// Covers all cars plus ball. Default panics; no `set_state` fallback.
    fn reset_motion(&mut self, _state: &TickRecord) {
        panic!("reset_motion not implemented for this backend");
    }

    /// Restore the handbrake integrator from the recording.
    fn set_handbrake_value(&mut self, _car_idx: usize, _value: f32) {}

    /// Restore recorded boost latch state.
    fn set_boost_state(&mut self, _car_idx: usize, _armed: bool, _time: f32) {}

    /// Whether this backend can restore extra-hit gating from RL truth.
    ///
    /// Only the v3 backend exposes the cooldown. Others keep the sim's own
    /// gating evolution and replay firing ticks with live gating.
    fn supports_cooldown_restore(&self) -> bool {
        false
    }

    /// Allow or suppress the ball extra impulse on the backend's next step.
    ///
    /// `suppress = true` spends RL's recorded extra firing so the replay does
    /// not apply it a second time; `false` clears stale sim gating so a real
    /// RL kick is not wrongly suppressed. Default is a no-op (see
    /// [`ReplayBackend::supports_cooldown_restore`]).
    fn suppress_next_extra_hit(&mut self, _car_idx: usize, _suppress: bool) {}

    /// Whether this backend can restore bump-cooldown gating from RL truth.
    ///
    /// Backends without support keep the sim's own cooldown evolution.
    fn supports_bump_restore(&self) -> bool {
        false
    }

    /// Set one car's bump-cooldown timer for one victim pair, in seconds.
    ///
    /// `victim_idx` is the recording slot of the bumped car; backends map
    /// both slots to arena cars. `0.0` means the cooldown is expired (bumps
    /// allowed); larger values suppress the next bump on that pair for that
    /// long. The sim scopes its live cooldown per pair, and the restore
    /// follows truth pairs so a different-victim refire is not wrongly
    /// suppressed. Default is a no-op (see
    /// [`ReplayBackend::supports_bump_restore`]).
    fn set_bump_cooldown(&mut self, _car_idx: usize, _victim_idx: usize, _seconds: f32) {}

    /// Restore jump-hold continuity after a state reset.
    ///
    /// Only the v3 backend tracks the release latch. Others keep the sim's
    /// own sustain evolution.
    fn set_jump_hold_broken(&mut self, _car_idx: usize, _broken: bool) {}

    /// Refresh hidden prior-tick wheel state without advancing dynamics.
    fn refresh_sticky_gates(&mut self) {}

    /// Apply native body internals after the normal state setters and the
    /// optional restores. The normal setters recompute inertia tensors and
    /// reset the ball interpolation basis. Ball activation belongs to the
    /// normal recorded-state restore. Call this after the last
    /// car state edit of one reset. Invalid values are rejected by
    /// [`validate_v12_body_state`] before evaluation.
    fn restore_body_state(&mut self, _state: &TickRecord) {}

    /// Restore supported pad and collision history from the state row.
    /// Call after state editors and body restoration. Never read target-row
    /// contacts to initialize the preceding state.
    fn restore_auxiliary_state(&mut self, _ticks: &[TickRecord], _row: usize) {}

    fn step(&mut self, controls: &[ControlsRecord]) -> Vec<SimContactEvents>;
    fn snapshot(&mut self, car_idx: usize) -> Snapshot;
}

/// Contacts the sim observed during one stepped tick, for the scored car.
/// Ball labels are shared; car labels describe the scored car only.
#[derive(Clone, Copy, Debug, Default)]
pub struct SimContactEvents {
    pub car_ball: bool,
    pub car_car: bool,
    pub ball_world: bool,
    pub chassis_world: bool,
}
/// Read one body from parts.
fn body_from_parts(
    pos: Vec3A,
    vel: Vec3A,
    ang_vel: Vec3A,
    forward: Vec3A,
    up: Vec3A,
) -> BodySnapshot {
    BodySnapshot {
        pos,
        vel,
        ang_vel,
        forward,
        up,
    }
}

/// Read ground truth for one car from a tick. `None` without that car.
///
/// A no-physics row, an unavailable car, or an unavailable ball yields
/// `None`. Callers must never score a placeholder.
pub fn snapshot_from_tick(tick: &TickRecord, car_idx: usize) -> Option<Snapshot> {
    if !tick.has_physics() || !tick.ball_available() {
        return None;
    }
    let car = tick.car_records.get(car_idx)?;
    if !car.is_available() {
        return None;
    }
    let car_forward = Vec3A::from(car.phys.rot.column(0));
    let car_up = Vec3A::from(car.phys.rot.column(2));
    let ball_forward = Vec3A::from(tick.ball_record.rot.column(0));
    let ball_up = Vec3A::from(tick.ball_record.rot.column(2));
    Some(Snapshot {
        car: body_from_parts(
            car.phys.pos.into(),
            car.phys.lin_vel.into(),
            car.phys.ang_vel.into(),
            car_forward,
            car_up,
        ),
        ball: body_from_parts(
            tick.ball_record.pos.into(),
            tick.ball_record.lin_vel.into(),
            tick.ball_record.ang_vel.into(),
            ball_forward,
            ball_up,
        ),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::test_support::*;

    #[test]
    fn snapshot_from_tick_skips_unavailable_slots() {
        let tick = multi_tick(5, 0.0, 6, 0b11_0111);
        assert!(snapshot_from_tick(&tick, 2).is_some());
        assert!(snapshot_from_tick(&tick, 3).is_none());
        assert!(snapshot_from_tick(&tick, 5).is_some());
        assert!(snapshot_from_tick(&guard_tick(6, 6), 0).is_none());
        let mut no_ball = multi_tick(7, 0.0, 6, 0b11_1111);
        no_ball.row_flags = 0;
        assert!(snapshot_from_tick(&no_ball, 0).is_none());
    }
    #[test]
    fn reset_mode_from_flags_maps() {
        assert_eq!(ResetMode::from_flags(false, false), ResetMode::Segmented);
        assert_eq!(ResetMode::from_flags(true, false), ResetMode::ResetEachTick);
        assert_eq!(
            ResetMode::from_flags(false, true),
            ResetMode::ResetMotionEachTick
        );
    }

    #[test]
    fn reset_mode_from_flags_rejects_both() {
        let result = std::panic::catch_unwind(|| ResetMode::from_flags(true, true));
        assert!(result.is_err());
    }
    #[test]
    fn reset_motion_default_panics_without_fallback() {
        struct NoMotion {
            snaps: Vec<Snapshot>,
            cursor: usize,
        }
        impl ReplayBackend for NoMotion {
            fn reset(&mut self, _start: &TickRecord) {}
            fn set_state(&mut self, _state: &TickRecord) {}
            fn step(&mut self, _controls: &[ControlsRecord]) -> Vec<SimContactEvents> {
                vec![]
            }
            fn snapshot(&mut self, _car_idx: usize) -> Snapshot {
                self.snaps[self.cursor]
            }
        }
        let body = BodySnapshot {
            pos: Vec3A::ZERO,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
            forward: Vec3A::X,
            up: Vec3A::Z,
        };
        let mut backend = NoMotion {
            snaps: vec![Snapshot {
                car: body,
                ball: body,
            }],
            cursor: 0,
        };
        let tick = quiet_tick(0, 0.0);
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            backend.reset_motion(&tick);
        }));
        assert!(result.is_err());
    }
}
