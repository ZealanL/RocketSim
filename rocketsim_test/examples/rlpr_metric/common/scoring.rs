use rocketsim_test::rlpr::tick_record::TickRecord;

use super::backend::{ResetMode, SimContactEvents, Snapshot};

/// Per-component tolerances for one scoring mode. See [`normalized_error`].
///
/// Every budget is tighter in one-tick replay than in open-loop segments.
/// A one-tick replay reproduces a tick far inside the segment budget, so
/// tighter budgets keep the gate sensitive enough to catch a regression;
/// open-loop segments diverge over 120 ticks, so looser budgets there avoid
/// measuring chaos rather than engine fidelity. Velocity dominates every
/// observed failure, so its gap is the widest.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tolerances {
    pub pos_uu: f32,
    pub vel_uu_s: f32,
    pub ang_vel_rad_s: f32,
    pub axis: f32,
}

impl Tolerances {
    /// Budgets for open-loop segment scoring.
    pub const SEGMENT: Self = Self {
        pos_uu: 2.0,
        vel_uu_s: 3.0,
        ang_vel_rad_s: 0.05,
        axis: 0.05,
    };

    /// Budgets for one-tick replay scoring.
    pub const RESET_EACH_TICK: Self = Self {
        pos_uu: 0.02,
        vel_uu_s: 2.0,
        ang_vel_rad_s: 0.02,
        axis: 0.02,
    };

    /// Tolerances for the given replay mode.
    pub fn for_mode(reset_each_tick: bool) -> Self {
        if reset_each_tick {
            Self::RESET_EACH_TICK
        } else {
            Self::SEGMENT
        }
    }

    /// Tolerances for a reset mode. Motion-only uses one-tick budgets.
    pub fn for_reset_mode(mode: ResetMode) -> Self {
        match mode {
            ResetMode::Segmented => Self::SEGMENT,
            ResetMode::ResetEachTick | ResetMode::ResetMotionEachTick => Self::RESET_EACH_TICK,
        }
    }
}
/// Overlapping contact categories plus the `Total` aggregate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ContactCategory {
    CarBall,
    CarCar,
    BallWorld,
    ChassisWorld,
    WheelWorld,
    /// No labeled contact anywhere in this tick, including the shared ball.
    /// Do not read this as a per-car quiet category.
    NoContact,
    Total,
}

impl ContactCategory {
    /// All categories in CLI column order.
    pub const ALL: [ContactCategory; 7] = [
        ContactCategory::CarBall,
        ContactCategory::CarCar,
        ContactCategory::BallWorld,
        ContactCategory::ChassisWorld,
        ContactCategory::WheelWorld,
        ContactCategory::NoContact,
        ContactCategory::Total,
    ];

    /// Short CLI column name.
    pub fn as_str(&self) -> &'static str {
        match self {
            ContactCategory::CarBall => "car_ball",
            ContactCategory::CarCar => "car_car",
            ContactCategory::BallWorld => "ball_world",
            ContactCategory::ChassisWorld => "chassis_world",
            ContactCategory::WheelWorld => "wheel_world",
            ContactCategory::NoContact => "no_contact",
            ContactCategory::Total => "total",
        }
    }
}

/// Per-tick contact labels. Only `NoContact` is exclusive. `ball_world`
/// describes the shared ball, so `NoContact` means no labeled contact
/// anywhere in the tick, not that the scored car touched nothing.
#[derive(Clone, Copy, Debug, Default)]
pub struct ContactLabels {
    pub car_ball: bool,
    pub car_car: bool,
    pub ball_world: bool,
    pub chassis_world: bool,
    pub wheel_world: bool,
}

impl ContactLabels {
    /// True when no label is set, including the shared-ball label.
    pub fn is_quiet(&self) -> bool {
        !(self.car_ball
            || self.car_car
            || self.ball_world
            || self.chassis_world
            || self.wheel_world)
    }

    /// Membership. `NoContact` holds only when all labels are false.
    pub fn contains(&self, category: ContactCategory) -> bool {
        match category {
            ContactCategory::CarBall => self.car_ball,
            ContactCategory::CarCar => self.car_car,
            ContactCategory::BallWorld => self.ball_world,
            ContactCategory::ChassisWorld => self.chassis_world,
            ContactCategory::WheelWorld => self.wheel_world,
            ContactCategory::NoContact => self.is_quiet(),
            ContactCategory::Total => true,
        }
    }
}

/// Label one target tick for one car.
///
/// Each label is true when the RL recording flags it OR the sim observed
/// it while replaying the tick. Sim-observed contacts are exact engine
/// events, never trajectory guesses: the sim cannot miss a contact the
/// way flag-based inference can, and a contact only the sim sees is a
/// real divergence worth scoring. `ball_world` comes from the shared ball.
/// `NoContact` therefore excludes every tick with a shared ball contact.
pub fn classify_tick(tick: &TickRecord, car_idx: usize, sim: SimContactEvents) -> ContactLabels {
    let Some(car) = tick.car_records.get(car_idx) else {
        return ContactLabels::default();
    };
    let car_ball = car.is_touching_ball || sim.car_ball;
    let car_car = car.is_touching_car || sim.car_car;
    let wheel_world = car.wheels.iter().any(|wheel| wheel.has_contact);
    let ball_world = tick.ball_record.has_world_contact || sim.ball_world;
    let chassis_world = car.phys.has_world_contact || sim.chassis_world;
    ContactLabels {
        car_ball,
        car_car,
        ball_world,
        chassis_world,
        wheel_world,
    }
}

/// Per-term normalized errors. Each term is already divided by its tol.
/// Use it to find which body part dominates a `wheel_world` fail.
#[derive(Clone, Copy, Debug, Default)]
pub struct ComponentErrors {
    pub car_pos: f32,
    pub ball_pos: f32,
    pub car_vel: f32,
    pub ball_vel: f32,
    pub car_ang: f32,
    pub ball_ang: f32,
    pub car_fwd: f32,
    pub car_up: f32,
    pub ball_fwd: f32,
    pub ball_up: f32,
}

impl ComponentErrors {
    /// Combined norm. Same value as [`normalized_error`].
    pub fn norm(&self) -> f32 {
        (self.car_pos * self.car_pos
            + self.ball_pos * self.ball_pos
            + self.car_vel * self.car_vel
            + self.ball_vel * self.ball_vel
            + self.car_ang * self.car_ang
            + self.ball_ang * self.ball_ang
            + self.car_fwd * self.car_fwd
            + self.car_up * self.car_up
            + self.ball_fwd * self.ball_fwd
            + self.ball_up * self.ball_up)
            .sqrt()
    }
}

/// Per-term errors for one sim vs truth pair.
///
/// Each term is already divided by its tolerance from `tol`.
pub fn component_errors(sim: &Snapshot, truth: &Snapshot, tol: &Tolerances) -> ComponentErrors {
    ComponentErrors {
        car_pos: (sim.car.pos - truth.car.pos).length() / tol.pos_uu,
        ball_pos: (sim.ball.pos - truth.ball.pos).length() / tol.pos_uu,
        car_vel: (sim.car.vel - truth.car.vel).length() / tol.vel_uu_s,
        ball_vel: (sim.ball.vel - truth.ball.vel).length() / tol.vel_uu_s,
        car_ang: (sim.car.ang_vel - truth.car.ang_vel).length() / tol.ang_vel_rad_s,
        ball_ang: (sim.ball.ang_vel - truth.ball.ang_vel).length() / tol.ang_vel_rad_s,
        car_fwd: (sim.car.forward - truth.car.forward).length() / tol.axis,
        car_up: (sim.car.up - truth.car.up).length() / tol.axis,
        ball_fwd: (sim.ball.forward - truth.ball.forward).length() / tol.axis,
        ball_up: (sim.ball.up - truth.ball.up).length() / tol.axis,
    }
}

/// Normalized physics error over car and ball.
///
/// Each component is divided by its tolerance from `tol`, then combined as
/// a norm. Passes when norm < 1. Tolerances are mode-dependent; see
/// [`Tolerances`].
pub fn normalized_error(sim: &Snapshot, truth: &Snapshot, tol: &Tolerances) -> f32 {
    component_errors(sim, truth, tol).norm()
}

/// Strict pass rule.
pub fn passes(norm_error: f32) -> bool {
    norm_error < 1.0
}

/// Pass rate in percent. 0 with no support.
pub fn pass_rate(passed: usize, support: usize) -> f64 {
    if support == 0 {
        0.0
    } else {
        100.0 * passed as f64 / support as f64
    }
}

/// Aggregate stats for one category.
#[derive(Clone, Debug, Default)]
pub struct CategoryStats {
    pub support: usize,
    pub passed: usize,
    /// Non-finite simulated errors. They fail and contribute infinite error.
    pub non_finite: usize,
    sum_norm: f64,
    pub max_norm: f32,
    pub first_fail_tick: Option<usize>,
}

impl CategoryStats {
    /// Record one scored tick.
    pub fn add(&mut self, tick_index: usize, norm_error: f32) {
        let norm_error = if norm_error.is_finite() {
            norm_error
        } else {
            self.non_finite += 1;
            f32::INFINITY
        };
        self.support += 1;
        self.sum_norm += norm_error as f64;
        self.max_norm = self.max_norm.max(norm_error);
        if passes(norm_error) {
            self.passed += 1;
        } else if self.first_fail_tick.is_none() {
            self.first_fail_tick = Some(tick_index);
        }
    }

    /// Mean norm error. 0 with no support.
    pub fn mean_norm(&self) -> f64 {
        if self.support == 0 {
            0.0
        } else {
            self.sum_norm / self.support as f64
        }
    }

    /// Pass rate in percent.
    pub fn rate(&self) -> f64 {
        pass_rate(self.passed, self.support)
    }

    /// Fold another aggregate into this one. Support, passes, error mass,
    /// and max merge exactly; `first_fail_tick` is left untouched because
    /// tick indices are per-recording and meaningless once combined.
    pub fn merge(&mut self, other: &CategoryStats) {
        self.support += other.support;
        self.passed += other.passed;
        self.non_finite += other.non_finite;
        self.sum_norm += other.sum_norm;
        self.max_norm = self.max_norm.max(other.max_norm);
    }
}

/// Per-category report. Contact categories overlap.
#[derive(Clone, Debug, Default)]
pub struct EvalReport {
    pub total: CategoryStats,
    pub car_ball: CategoryStats,
    pub car_car: CategoryStats,
    pub ball_world: CategoryStats,
    pub chassis_world: CategoryStats,
    pub wheel_world: CategoryStats,
    pub no_contact: CategoryStats,
}

impl EvalReport {
    /// Read one category.
    pub fn for_category(&self, category: ContactCategory) -> &CategoryStats {
        match category {
            ContactCategory::CarBall => &self.car_ball,
            ContactCategory::CarCar => &self.car_car,
            ContactCategory::BallWorld => &self.ball_world,
            ContactCategory::ChassisWorld => &self.chassis_world,
            ContactCategory::WheelWorld => &self.wheel_world,
            ContactCategory::NoContact => &self.no_contact,
            ContactCategory::Total => &self.total,
        }
    }

    /// Update one category.
    fn for_category_mut(&mut self, category: ContactCategory) -> &mut CategoryStats {
        match category {
            ContactCategory::CarBall => &mut self.car_ball,
            ContactCategory::CarCar => &mut self.car_car,
            ContactCategory::BallWorld => &mut self.ball_world,
            ContactCategory::ChassisWorld => &mut self.chassis_world,
            ContactCategory::WheelWorld => &mut self.wheel_world,
            ContactCategory::NoContact => &mut self.no_contact,
            ContactCategory::Total => &mut self.total,
        }
    }

    /// Record one tick in each matching category.
    pub(super) fn add(&mut self, labels: ContactLabels, tick_index: usize, norm_error: f32) {
        for category in ContactCategory::ALL {
            if labels.contains(category) {
                self.for_category_mut(category).add(tick_index, norm_error);
            }
        }
    }

    /// Fold another recording's report into this one for a cross-recording
    /// aggregate. Rates and means stay exact (support-weighted); max takes
    /// the worst. `first_fail_tick` is cleared: tick indices are
    /// per-recording, so a combined first-fail tick would be meaningless and
    /// the combined table drops the column entirely.
    pub fn merge(&mut self, other: &EvalReport) {
        for category in ContactCategory::ALL {
            let stats = self.for_category_mut(category);
            stats.merge(other.for_category(category));
            stats.first_fail_tick = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::Vec3A;

    use super::*;
    use crate::common::{snapshot_from_tick, test_support::*};

    #[test]
    fn categories_overlap() {
        let tick = make_tick(
            0,
            (0.0, 0.0, 100.0),
            (50.0, 0.0, 100.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            true,
            false,
            false,
            true,
        );
        let labels = classify_tick(&tick, 0, SimContactEvents::default());
        assert!(labels.car_ball && labels.wheel_world);
        assert!(labels.contains(ContactCategory::CarBall));
        assert!(labels.contains(ContactCategory::WheelWorld));
        assert!(labels.contains(ContactCategory::Total));
        assert!(!labels.contains(ContactCategory::NoContact));
    }

    #[test]
    fn no_contact_only_when_all_labels_false() {
        let quiet = quiet_tick(0, 0.0);
        let labels = classify_tick(&quiet, 0, SimContactEvents::default());
        assert!(labels.is_quiet());
        assert!(labels.contains(ContactCategory::NoContact));
        let noisy = make_tick(
            0,
            (0.0, 0.0, 100.0),
            (0.0, 0.0, 500.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            false,
            true,
            false,
            false,
        );
        let labels = classify_tick(&noisy, 0, SimContactEvents::default());
        assert!(labels.ball_world);
        assert!(!labels.contains(ContactCategory::NoContact));
    }

    #[test]
    fn norm_error_matches_strict_thresholds() {
        let tick = quiet_tick(0, 0.0);
        let truth = snapshot_from_tick(&tick, 0).unwrap();
        let tol = Tolerances::SEGMENT;
        assert_eq!(normalized_error(&truth, &truth, &tol), 0.0);
        assert!(passes(0.0));
        let mut moved = truth;
        moved.car.pos.x += tol.pos_uu;
        assert!((normalized_error(&moved, &truth, &tol) - 1.0).abs() < 1e-5);
        assert!(!passes(normalized_error(&moved, &truth, &tol)));
        moved.car.pos.x -= tol.pos_uu / 2.0;
        assert!(passes(normalized_error(&moved, &truth, &tol)));
        let mut ball_moved = truth;
        ball_moved.ball.forward = Vec3A::new(0.0, 1.0, 0.0);
        assert!(!passes(normalized_error(&ball_moved, &truth, &tol)));
    }

    #[test]
    fn reset_each_tick_tightens_budgets() {
        // One-tick replay tightens every budget relative to open-loop
        // segments (see `Tolerances`): a replayed tick sits far inside the
        // segment divergence budget.
        let seg = Tolerances::for_mode(false);
        let reset = Tolerances::for_mode(true);
        assert!(reset.pos_uu < seg.pos_uu);
        assert!(reset.vel_uu_s < seg.vel_uu_s);
        assert!(reset.ang_vel_rad_s < seg.ang_vel_rad_s);
        assert!(reset.axis < seg.axis);
    }

    #[test]
    fn reset_each_tick_velocity_error_fails_earlier() {
        let tick = quiet_tick(0, 0.0);
        let truth = snapshot_from_tick(&tick, 0).unwrap();
        let mut moved = truth;
        // Between the two velocity budgets: passes on segments, fails on
        // one-tick replay.
        moved.car.vel.y =
            (Tolerances::SEGMENT.vel_uu_s + Tolerances::RESET_EACH_TICK.vel_uu_s) / 2.0;
        assert!(passes(normalized_error(
            &moved,
            &truth,
            &Tolerances::SEGMENT
        )));
        assert!(!passes(normalized_error(
            &moved,
            &truth,
            &Tolerances::RESET_EACH_TICK
        )));
    }

    #[test]
    fn aggregates_percentages_and_first_failure() {
        assert_eq!(pass_rate(1, 2), 50.0);
        assert_eq!(pass_rate(0, 0), 0.0);
        let mut stats = CategoryStats::default();
        stats.add(7, 0.5);
        stats.add(9, 2.0);
        assert_eq!(stats.support, 2);
        assert_eq!(stats.passed, 1);
        assert_eq!(stats.rate(), 50.0);
        assert!((stats.mean_norm() - 1.25).abs() < 1e-9);
        assert_eq!(stats.max_norm, 2.0);
        assert_eq!(stats.first_fail_tick, Some(9));
    }
    #[test]
    fn labels_sim_observed_contacts() {
        // Sim-observed contacts label the tick even when RL flags are clear.
        let tick = quiet_tick(1, 10.0);
        let sim = SimContactEvents {
            car_ball: false,
            car_car: true,
            ball_world: true,
            chassis_world: false,
        };
        let labels = classify_tick(&tick, 0, sim);
        assert!(labels.car_car);
        assert!(labels.ball_world);
        assert!(!labels.car_ball);
        assert!(!labels.contains(ContactCategory::NoContact));
        // No sim events and clear flags: quiet.
        let quiet = classify_tick(&tick, 0, SimContactEvents::default());
        assert!(quiet.is_quiet());
        // RL flags still label without sim events.
        let mut touch = quiet_tick(1, 10.0);
        touch.car_records[0].is_touching_ball = true;
        let labels = classify_tick(&touch, 0, SimContactEvents::default());
        assert!(labels.car_ball);
    }

    #[test]
    fn car_car_labels_recorded_flag_without_sim() {
        // Recorded-events default path: empty sim events still get RL support.
        let mut touch = quiet_tick(1, 10.0);
        touch.car_records[0].is_touching_car = true;
        let labels = classify_tick(&touch, 0, SimContactEvents::default());
        assert!(labels.car_car);
        assert!(!labels.is_quiet());
        assert!(labels.contains(ContactCategory::CarCar));
    }

    #[test]
    fn car_car_labels_either_source() {
        for (recorded, sim_flag, want) in [
            (false, false, false),
            (true, false, true),
            (false, true, true),
            (true, true, true),
        ] {
            let mut tick = quiet_tick(1, 10.0);
            tick.car_records[0].is_touching_car = recorded;
            let sim = SimContactEvents {
                car_car: sim_flag,
                ..SimContactEvents::default()
            };
            assert_eq!(
                classify_tick(&tick, 0, sim).car_car,
                want,
                "recorded={recorded} sim_flag={sim_flag}",
            );
        }
    }

    #[test]
    fn car_car_labels_are_per_car_asymmetric() {
        let mut tick = quiet_tick(1, 10.0);
        let mut other = blank_car();
        other.phys.physics_frame = 1;
        tick.car_records.push(other);
        tick.car_records[0].is_touching_car = false;
        tick.car_records[1].is_touching_car = true;
        let car0 = classify_tick(&tick, 0, SimContactEvents::default());
        let car1 = classify_tick(&tick, 1, SimContactEvents::default());
        assert!(!car0.car_car);
        assert!(car1.car_car);
        assert!(car0.is_quiet());
        assert!(!car1.is_quiet());
        // Sim events also apply per scored car.
        let sim = SimContactEvents {
            car_car: true,
            ..SimContactEvents::default()
        };
        assert!(classify_tick(&tick, 0, sim).car_car);
    }
    #[test]
    fn motion_tolerances_share_one_tick_budgets() {
        assert_eq!(
            Tolerances::for_reset_mode(ResetMode::ResetMotionEachTick),
            Tolerances::RESET_EACH_TICK
        );
        assert_eq!(
            Tolerances::for_reset_mode(ResetMode::ResetEachTick),
            Tolerances::for_mode(true)
        );
        assert_eq!(
            Tolerances::for_reset_mode(ResetMode::Segmented),
            Tolerances::for_mode(false)
        );
    }
    #[test]
    fn invalid_simulated_error_is_visible_in_merged_stats() {
        let mut finite = CategoryStats::default();
        finite.add(1, 0.25);
        let mut invalid = CategoryStats::default();
        invalid.add(2, f32::NAN);
        invalid.add(3, f32::INFINITY);
        finite.merge(&invalid);
        assert_eq!(
            (finite.support, finite.passed, finite.non_finite),
            (3, 1, 2)
        );
        assert_eq!(finite.mean_norm(), f64::INFINITY);
        assert_eq!(finite.max_norm, f32::INFINITY);
        assert_eq!(invalid.first_fail_tick, Some(2));
    }
}
