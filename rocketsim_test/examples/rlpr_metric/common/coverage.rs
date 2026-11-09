use rocketsim_test::rlpr::tick_record::TickRecord;

/// One requested RLPR v12 field.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum V12Field {
    Supersonic,
    WheelWorldContact,
    Autoflip,
    Jump,
    UpdateTag,
    CarInvInertia,
    BallInvInertia,
    BallInterp,
    BallSleep,
    Pads,
    PadTouch,
    BallHasBeenHit,
    RoundActive,
    CountdownRemaining,
    CountdownTime,
    CollisionSnapshot,
}

impl V12Field {
    /// Requested fields, in report order.
    pub const ALL: [Self; 16] = [
        Self::Supersonic,
        Self::WheelWorldContact,
        Self::Autoflip,
        Self::Jump,
        Self::UpdateTag,
        Self::CarInvInertia,
        Self::BallInvInertia,
        Self::BallInterp,
        Self::BallSleep,
        Self::Pads,
        Self::PadTouch,
        Self::BallHasBeenHit,
        Self::RoundActive,
        Self::CountdownRemaining,
        Self::CountdownTime,
        Self::CollisionSnapshot,
    ];

    /// Stable report name.
    pub fn name(self) -> &'static str {
        match self {
            Self::Supersonic => "supersonic",
            Self::WheelWorldContact => "wheel world contact",
            Self::Autoflip => "autoflip component",
            Self::Jump => "jump component",
            Self::UpdateTag => "update tag",
            Self::CarInvInertia => "car inverse inertia",
            Self::BallInvInertia => "ball inverse inertia",
            Self::BallInterp => "ball interpolation basis",
            Self::BallSleep => "ball sleep",
            Self::Pads => "pads",
            Self::PadTouch => "pad touch",
            Self::BallHasBeenHit => "ball has been hit",
            Self::RoundActive => "round active",
            Self::CountdownRemaining => "countdown remaining",
            Self::CountdownTime => "countdown time",
            Self::CollisionSnapshot => "collision snapshot",
        }
    }

    /// True when the metric applies the supported fields at a state reset.
    pub fn applied(self) -> bool {
        matches!(
            self,
            Self::Supersonic
                | Self::CarInvInertia
                | Self::BallInvInertia
                | Self::BallInterp
                | Self::BallSleep
        )
    }

    /// True when a group has a partial or prefix-derived restore.
    pub fn partially_applied(self) -> bool {
        matches!(self, Self::Autoflip | Self::Pads | Self::CollisionSnapshot)
    }
}

/// RLPR v12 group 4 carries a bounded collision snapshot.
const V12_COLLISION_GROUP_BIT: u32 = 1 << 4;

/// Per-recording v12 field coverage.
///
/// `rows` counts every recording row. `ext_rows` counts rows with a v12
/// extension. `car_rows` counts every available car slot. Car field counts
/// are available car-slot rows. Ball, pad, kickoff, and collision counts are
/// rows. `collision_unreadable_rows` counts group 4 rows whose schema the
/// parser did not recognize.
#[derive(Clone, Debug, Default)]
pub struct V12Report {
    pub rows: usize,
    pub ext_rows: usize,
    pub car_rows: usize,
    collision_unreadable_rows: usize,
    counts: [usize; 16],
}

impl V12Report {
    /// True when this recording carries at least one v12 extension.
    pub fn has_extension(&self) -> bool {
        self.ext_rows > 0
    }

    /// Count for one requested field.
    pub fn count(&self, field: V12Field) -> usize {
        self.counts[field as usize]
    }

    /// Group 4 rows whose schema the parser did not recognize.
    pub fn collision_unreadable_rows(&self) -> usize {
        self.collision_unreadable_rows
    }

    /// One-line field coverage. It includes zero counts on purpose.
    pub fn coverage_line(&self) -> String {
        let mut line = format!(
            "v12 coverage: ext rows {}/{}, car rows {};",
            self.ext_rows, self.rows, self.car_rows
        );
        for field in V12Field::ALL {
            line.push_str(&format!(" {} {}", field.name(), self.count(field)));
        }
        line.push_str(&format!(
            "; collision unreadable {}",
            self.collision_unreadable_rows
        ));
        line
    }

    /// Applied fields that are present in this recording.
    pub fn applied_names(&self) -> Vec<&'static str> {
        V12Field::ALL
            .iter()
            .filter(|field| field.applied() && self.count(**field) > 0)
            .map(|field| field.name())
            .collect()
    }

    /// Recorded fields that the harness does not apply.
    pub fn not_applied(&self) -> Vec<(&'static str, usize)> {
        V12Field::ALL
            .iter()
            .filter(|field| {
                !field.applied() && !field.partially_applied() && self.count(**field) > 0
            })
            .map(|field| (field.name(), self.count(*field)))
            .collect()
    }

    /// Requested fields that this recording does not carry.
    pub fn absent_names(&self) -> Vec<&'static str> {
        V12Field::ALL
            .iter()
            .filter(|field| self.count(**field) == 0)
            .map(|field| field.name())
            .collect()
    }
}

/// Count v12 field coverage across one recording.
pub fn v12_report(ticks: &[TickRecord]) -> V12Report {
    let mut report = V12Report::default();
    for tick in ticks {
        report.rows += 1;
        report.car_rows += tick
            .car_records
            .iter()
            .filter(|car| car.is_available())
            .count();
        let Some(ext) = &tick.replay_state else {
            continue;
        };
        report.ext_rows += 1;
        for (slot, car) in tick.car_records.iter().enumerate() {
            if !car.is_available() {
                continue;
            }
            let Some(car_ext) = ext.cars.get(slot) else {
                continue;
            };
            if car_ext.b_supersonic.is_some() && car_ext.time_below_supersonic_s.is_some() {
                report.counts[V12Field::Supersonic as usize] += 1;
            }
            if car_ext.wheel_contacts.is_some() {
                report.counts[V12Field::WheelWorldContact as usize] += 1;
            }
            if car_ext.autoflip_active.is_some() {
                report.counts[V12Field::Autoflip as usize] += 1;
            }
            if car_ext.jump_active.is_some() {
                report.counts[V12Field::Jump as usize] += 1;
            }
            if car_ext.update_tag.is_some() {
                report.counts[V12Field::UpdateTag as usize] += 1;
            }
            if car_ext.inv_inertia_world.is_some() {
                report.counts[V12Field::CarInvInertia as usize] += 1;
            }
        }
        if let Some(ball) = &ext.ball {
            if ball.inv_inertia_world.is_some() {
                report.counts[V12Field::BallInvInertia as usize] += 1;
            }
            if ball.interp_basis.is_some() {
                report.counts[V12Field::BallInterp as usize] += 1;
            }
            if ball.activation_state.is_some()
                && ball.deactivation_time_s.is_some()
                && ball.linear_sleeping_threshold.is_some()
                && ball.angular_sleeping_threshold.is_some()
            {
                report.counts[V12Field::BallSleep as usize] += 1;
            }
        }
        if !ext.pads.is_empty() {
            report.counts[V12Field::Pads as usize] += 1;
        }
        if ext.pads.iter().any(|pad| pad.is_touching_vehicle.is_some()) {
            report.counts[V12Field::PadTouch as usize] += 1;
        }
        if let Some(global) = &ext.global {
            if global.ball_has_been_hit.is_some() {
                report.counts[V12Field::BallHasBeenHit as usize] += 1;
            }
            if global.round_active.is_some() {
                report.counts[V12Field::RoundActive as usize] += 1;
            }
            if global.seconds_remaining_countdown.is_some() {
                report.counts[V12Field::CountdownRemaining as usize] += 1;
            }
            if global.countdown_time.is_some() {
                report.counts[V12Field::CountdownTime as usize] += 1;
            }
        }
        if ext.collision_snapshot.is_some() {
            report.counts[V12Field::CollisionSnapshot as usize] += 1;
        } else if ext.group_mask & V12_COLLISION_GROUP_BIT != 0 {
            report.collision_unreadable_rows += 1;
        }
    }
    report
}

/// Validate recorded v12 body state before evaluation.
///
/// The native body setters return errors. Check the applied fields here so
/// a bad recording fails before the metric starts a tick. Activation values
/// other than 1, 2, and 3 are rejected. Do not replace them with a default.
pub fn validate_v12_body_state(ticks: &[TickRecord]) -> Result<(), String> {
    for (row, tick) in ticks.iter().enumerate() {
        let Some(ext) = &tick.replay_state else {
            continue;
        };
        for (slot, car) in ext.cars.iter().enumerate() {
            if let Some(rows) = car.inv_inertia_world
                && !matrix_is_finite(rows)
            {
                return Err(format!(
                    "RLPR v12 row {row} car slot {slot}: inverse inertia tensor is not finite"
                ));
            }
        }
        let Some(ball) = &ext.ball else {
            continue;
        };
        if let Some(rows) = ball.inv_inertia_world
            && !matrix_is_finite(rows)
        {
            return Err(format!(
                "RLPR v12 row {row} ball: inverse inertia tensor is not finite"
            ));
        }
        if let Some(rows) = ball.interp_basis
            && !matrix_is_finite(rows)
        {
            return Err(format!(
                "RLPR v12 row {row} ball: interpolation basis is not finite"
            ));
        }
        if let Some(value) = ball.activation_state
            && !matches!(value, 1..=3)
        {
            return Err(format!(
                "RLPR v12 row {row} ball: unsupported activation state {value}; expected 1, 2, or 3"
            ));
        }
        for (name, value) in [
            ("deactivation time", ball.deactivation_time_s),
            ("linear sleeping threshold", ball.linear_sleeping_threshold),
            (
                "angular sleeping threshold",
                ball.angular_sleeping_threshold,
            ),
        ] {
            if let Some(value) = value {
                if !value.is_finite() {
                    return Err(format!("RLPR v12 row {row} ball: {name} is not finite"));
                }
                if value < 0.0 {
                    return Err(format!("RLPR v12 row {row} ball: {name} is negative"));
                }
            }
        }
    }
    Ok(())
}

fn matrix_is_finite(rows: [[f32; 3]; 3]) -> bool {
    rows.iter().flatten().all(|value| value.is_finite())
}
/// Capture-coverage counts for one recording's rows.
///
/// These describe what the file contains. They are not scoring support:
/// scoring still counts one sample per available car-tick, and
/// [`snapshot_from_tick`] drops every unavailable slot. v2-v10 rows carry
/// every car and the ball, so the unavailable, no-physics, and
/// ball-unavailable counts stay zero and the old output is unchanged.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AvailabilityCounts {
    /// Rows the metric read, after duplicate collapse.
    pub rows: usize,
    /// Car-tick pairs with real physics.
    pub available_car_ticks: usize,
    /// Car-tick pairs marked unavailable (placeholder physics).
    pub unavailable_car_ticks: usize,
    /// Rows that carry a timestamp but no physics at all.
    pub no_physics_rows: usize,
    /// Rows whose shared ball is unavailable.
    pub ball_unavailable_rows: usize,
    /// Changes of the per-car availability mask between rows.
    pub mask_changes: usize,
}

impl AvailabilityCounts {
    /// Fold another recording's counts into this one.
    pub fn merge(&mut self, other: &AvailabilityCounts) {
        self.rows += other.rows;
        self.available_car_ticks += other.available_car_ticks;
        self.unavailable_car_ticks += other.unavailable_car_ticks;
        self.no_physics_rows += other.no_physics_rows;
        self.ball_unavailable_rows += other.ball_unavailable_rows;
        self.mask_changes += other.mask_changes;
    }

    /// Available car-tick share, in percent. Zero when there are no car-ticks.
    pub fn coverage_percent(&self) -> f64 {
        let total = self.available_car_ticks + self.unavailable_car_ticks;
        if total == 0 {
            0.0
        } else {
            100.0 * self.available_car_ticks as f64 / total as f64
        }
    }
}

/// Count row availability across one recording's ticks.
pub fn availability_counts(ticks: &[TickRecord]) -> AvailabilityCounts {
    let mut counts = AvailabilityCounts::default();
    let mut prev_mask: Option<u32> = None;
    for tick in ticks {
        counts.rows += 1;
        let mask = tick.availability_mask();
        for car in &tick.car_records {
            if car.is_available() {
                counts.available_car_ticks += 1;
            } else {
                counts.unavailable_car_ticks += 1;
            }
        }
        if !tick.has_physics() {
            counts.no_physics_rows += 1;
        }
        if !tick.ball_available() {
            counts.ball_unavailable_rows += 1;
        }
        if prev_mask.is_some_and(|prev| prev != mask) {
            counts.mask_changes += 1;
        }
        prev_mask = Some(mask);
    }
    counts
}

#[cfg(test)]
mod tests {
    use rocketsim_test::rlpr::{
        BallExt, CarExt, CollisionSnapshot, GlobalExt, PadExt, RowExt, WheelContactExt,
    };

    use super::*;
    use crate::common::{test_support::*, validation::validate_recorded_state};

    #[test]
    fn availability_counts_track_mask_and_guard_rows() {
        let ticks = vec![
            multi_tick(0, 0.0, 6, 0b11_1111),
            guard_tick(1, 6),
            multi_tick(2, 60.0, 6, 0b11_0111),
            multi_tick(3, 70.0, 6, 0b11_1111),
        ];
        let counts = availability_counts(&ticks);
        assert_eq!(counts.rows, 4);
        assert_eq!(counts.available_car_ticks, 6 + 0 + 5 + 6);
        assert_eq!(counts.unavailable_car_ticks, 0 + 6 + 1 + 0);
        assert_eq!(counts.no_physics_rows, 1);
        assert_eq!(counts.ball_unavailable_rows, 1);
        assert_eq!(counts.mask_changes, 3);
        assert!((counts.coverage_percent() - 100.0 * 17.0 / 24.0).abs() < 1e-9);
    }
    #[test]
    fn v12_report_counts_applied_and_skipped_fields() {
        let mut plain = quiet_tick(0, 0.0);
        plain.replay_state = None;
        let mut extended = quiet_tick(1, 10.0);
        let wheel = WheelContactExt {
            has_contact_world_geometry: true,
            ..WheelContactExt::default()
        };
        extended.replay_state = Some(RowExt {
            version: 1,
            group_mask: 0x1f,
            cars: vec![CarExt {
                b_supersonic: Some(true),
                time_below_supersonic_s: Some(0.5),
                wheel_contacts: Some([wheel; 4]),
                autoflip_active: Some(true),
                jump_active: Some(true),
                update_tag: Some(7),
                inv_inertia_world: Some([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
                ..CarExt::default()
            }],
            ball: Some(BallExt {
                inv_inertia_world: Some([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
                interp_basis: Some([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
                activation_state: Some(2),
                deactivation_time_s: Some(1.0),
                linear_sleeping_threshold: Some(0.8),
                angular_sleeping_threshold: Some(1.0),
                ..BallExt::default()
            }),
            pads: vec![PadExt {
                is_touching_vehicle: Some(false),
                ..PadExt::default()
            }],
            global: Some(GlobalExt {
                ball_has_been_hit: Some(false),
                round_active: Some(true),
                seconds_remaining_countdown: Some(1),
                countdown_time: Some(2),
                ..GlobalExt::default()
            }),
            collision_snapshot: Some(CollisionSnapshot::default()),
            ..RowExt::default()
        });

        let report = v12_report(&[plain, extended]);
        assert_eq!(report.rows, 2);
        assert!(report.has_extension());
        assert_eq!(report.ext_rows, 1);
        assert_eq!(report.car_rows, 2);
        for field in V12Field::ALL {
            assert_eq!(report.count(field), 1, "{}", field.name());
        }
        assert_eq!(
            report.applied_names(),
            vec![
                "supersonic",
                "car inverse inertia",
                "ball inverse inertia",
                "ball interpolation basis",
                "ball sleep",
            ]
        );
        assert_eq!(
            report.not_applied(),
            vec![
                ("wheel world contact", 1),
                ("jump component", 1),
                ("update tag", 1),
                ("pad touch", 1),
                ("ball has been hit", 1),
                ("round active", 1),
                ("countdown remaining", 1),
                ("countdown time", 1),
            ]
        );
        assert_eq!(report.collision_unreadable_rows(), 0);
        assert!(report.absent_names().is_empty());
    }

    #[test]
    fn v12_report_distinguishes_unreadable_collision_schema() {
        let mut tick = quiet_tick(0, 0.0);
        tick.replay_state = Some(RowExt {
            version: 1,
            group_mask: V12_COLLISION_GROUP_BIT,
            ..RowExt::default()
        });
        let report = v12_report(&[tick]);
        assert_eq!(report.count(V12Field::CollisionSnapshot), 0);
        assert_eq!(report.collision_unreadable_rows(), 1);
        assert!(
            report.coverage_line().contains("collision unreadable 1"),
            "{}",
            report.coverage_line()
        );
        assert!(
            report.absent_names().contains(&"collision snapshot"),
            "unreadable collision must not count as recorded"
        );
    }

    #[test]
    fn v12_report_without_extension_names_every_field_absent() {
        let report = v12_report(&[quiet_tick(0, 0.0)]);
        assert!(!report.has_extension());
        assert_eq!(report.ext_rows, 0);
        assert!(report.applied_names().is_empty());
        assert!(report.not_applied().is_empty());
        assert_eq!(report.absent_names(), V12Field::ALL.map(V12Field::name));
    }

    #[test]
    fn v12_report_needs_both_supersonic_values() {
        let mut tick = quiet_tick(0, 0.0);
        tick.replay_state = Some(RowExt {
            version: 1,
            group_mask: 1,
            cars: vec![CarExt {
                b_supersonic: Some(true),
                ..CarExt::default()
            }],
            ..RowExt::default()
        });
        let report = v12_report(&[tick]);
        assert_eq!(report.count(V12Field::Supersonic), 0);
    }
    #[test]
    fn recording_validation_rejects_invalid_available_state_with_context() {
        let good = quiet_tick(42, 0.0);
        assert_eq!(validate_recorded_state(std::slice::from_ref(&good)), Ok(()));
        let mut cases = Vec::new();
        let mut tick = good.clone();
        tick.ball_record.pos.x = f32::NAN;
        cases.push((tick, "ball native frame 42: position is not finite"));
        let mut tick = good.clone();
        tick.car_records[0].phys.lin_vel.y = f32::INFINITY;
        cases.push((
            tick,
            "car slot 0 native frame 42: linear velocity is not finite",
        ));
        let mut tick = good.clone();
        tick.car_records[0].phys.rot.rows[0].x = 2.0;
        cases.push((tick, "rotation row 0 is not a unit vector"));
        let mut tick = good.clone();
        tick.car_records[0].phys.rot.rows[0].x = -1.0;
        cases.push((tick, "rotation is not right-handed"));
        let mut tick = good.clone();
        tick.car_records[0].prev_controls.throttle = f32::NAN;
        cases.push((tick, "throttle is not finite"));
        for (tick, expected) in cases {
            let error = validate_recorded_state(&[good.clone(), tick]).unwrap_err();
            assert!(error.contains("RLPR row 1"), "{error}");
            assert!(error.contains(expected), "{error}");
        }
    }

    #[test]
    fn recording_validation_ignores_unavailable_placeholders() {
        let mut tick = quiet_tick(42, 0.0);
        tick.car_records[0].physics_available = 0;
        tick.car_records[0].phys.rot.rows = [vec(0.0, 0.0, 0.0); 3];
        tick.car_records[0].phys.pos.x = f32::NAN;
        tick.ball_record.pos.x = f32::NAN;
        tick.row_flags &= !rocketsim_test::rlpr::ROW_FLAGS_BALL_AVAILABLE;
        assert_eq!(validate_recorded_state(&[tick]), Ok(()));
    }

    fn ball_ext_tick(ball: BallExt) -> TickRecord {
        let mut tick = quiet_tick(0, 0.0);
        tick.replay_state = Some(RowExt {
            version: 1,
            group_mask: 1 << 1,
            ball: Some(ball),
            ..RowExt::default()
        });
        tick
    }

    #[test]
    fn v12_validation_accepts_valid_and_rejects_invalid_body_state() {
        let valid = ball_ext_tick(BallExt {
            inv_inertia_world: Some([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
            interp_basis: Some([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
            activation_state: Some(2),
            deactivation_time_s: Some(1.0),
            linear_sleeping_threshold: Some(0.8),
            angular_sleeping_threshold: Some(1.0),
            ..BallExt::default()
        });
        assert_eq!(validate_v12_body_state(&[valid]), Ok(()));

        let mut nan_car = quiet_tick(0, 0.0);
        nan_car.replay_state = Some(RowExt {
            version: 1,
            group_mask: 1,
            cars: vec![CarExt {
                inv_inertia_world: Some([[f32::NAN, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
                ..CarExt::default()
            }],
            ..RowExt::default()
        });
        let cases = [
            (
                ball_ext_tick(BallExt {
                    activation_state: Some(4),
                    ..BallExt::default()
                }),
                "unsupported activation state 4",
            ),
            (
                ball_ext_tick(BallExt {
                    deactivation_time_s: Some(-0.1),
                    ..BallExt::default()
                }),
                "deactivation time is negative",
            ),
            (
                ball_ext_tick(BallExt {
                    interp_basis: Some([[f32::INFINITY; 3]; 3]),
                    ..BallExt::default()
                }),
                "interpolation basis is not finite",
            ),
            (
                ball_ext_tick(BallExt {
                    inv_inertia_world: Some([[f32::NAN; 3]; 3]),
                    ..BallExt::default()
                }),
                "inverse inertia tensor is not finite",
            ),
            (nan_car, "inverse inertia tensor is not finite"),
        ];
        for (tick, expected) in cases {
            let error = validate_v12_body_state(&[tick]).unwrap_err();
            assert!(error.contains(expected), "{error}");
        }
    }
}
