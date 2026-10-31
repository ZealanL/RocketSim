use std::time::{Duration, Instant};

use rocketsim_test::rlpr::{Recording, cpp_records::ControlsRecord, tick_record::TickRecord};

use crate::common::{any_teleport, frame_is_contiguous, run_start, tick_car_count, tick_is_frozen};

pub const THREE_V_THREE_CARS: usize = 6;

#[derive(Clone, Copy, Debug)]
pub struct ReplayReset {
    pub state_index: usize,
    pub run_start: usize,
}

pub struct ReplayPlan {
    pub controls: Vec<Vec<ControlsRecord>>,
    pub resets: Vec<Option<ReplayReset>>,
    pub simulation_ticks: usize,
    pub skipped_ticks: usize,
}

impl ReplayPlan {
    pub fn from_recording(recording: &Recording) -> Result<Self, String> {
        Self::from_ticks(&recording.ticks)
    }

    fn from_ticks(ticks: &[TickRecord]) -> Result<Self, String> {
        if ticks.len() < 2 {
            return Err("recording must contain at least two ticks".to_string());
        }
        if ticks
            .iter()
            .any(|tick| tick_car_count(tick) != THREE_V_THREE_CARS)
        {
            return Err(
                "RLPR replay mode requires exactly six cars (3v3) in every tick".to_string(),
            );
        }

        let mut controls = vec![Vec::new(); ticks.len()];
        let mut supported = vec![false; ticks.len()];
        for (index, tick) in ticks.iter().enumerate() {
            if replay_tick_supported(tick) {
                supported[index] = true;
                controls[index] = tick
                    .car_records
                    .iter()
                    .map(|car| car.prev_controls)
                    .collect();
            }
        }

        let mut resets = vec![None; ticks.len()];
        if let Some(first_supported) = supported.iter().position(|&available| available) {
            resets[first_supported] = Some(ReplayReset {
                state_index: first_supported,
                run_start: first_supported,
            });
        }

        for (offset, (reset_slot, window)) in
            resets.iter_mut().skip(1).zip(ticks.windows(2)).enumerate()
        {
            let index = offset + 1;
            let from = &window[0];
            let to = &window[1];
            if !supported[index] {
                continue;
            }
            let needs_state = !supported[index - 1]
                || !frame_is_contiguous(from, to)
                || tick_is_frozen(from, to)
                || any_teleport(from, to);
            if needs_state {
                *reset_slot = Some(ReplayReset {
                    state_index: index,
                    run_start: run_start(ticks, index),
                });
            }
        }

        let skipped_ticks = supported.iter().filter(|&&available| !available).count();
        if !supported.iter().any(|&available| available) {
            return Err("RLPR replay has no fully available physics ticks".to_string());
        }
        let simulation_ticks = (1..ticks.len())
            .filter(|&index| supported[index] && resets[index].is_none())
            .count();
        Ok(Self {
            controls,
            resets,
            simulation_ticks,
            skipped_ticks,
        })
    }

    pub fn state_reset_count(&self) -> usize {
        self.resets.iter().filter(|reset| reset.is_some()).count()
    }
}

fn replay_tick_supported(tick: &TickRecord) -> bool {
    tick.has_physics()
        && tick.ball_available()
        && tick.car_records.len() == THREE_V_THREE_CARS
        && tick.car_records.iter().all(|car| car.is_available())
}

pub trait ReplayBenchmarkBackend {
    type Control: Copy;

    fn control_from_record(record: ControlsRecord) -> Self::Control;

    fn restore_replay_state(
        &mut self,
        ticks: &[TickRecord],
        state_index: usize,
        run_start: usize,
        recording_version: u32,
    );

    /// Apply preconverted controls and advance the physics by one tick.
    /// Do not collect metrics or inspect events in this method.
    fn step_simulation(&mut self, controls: &[Self::Control]);
}

#[derive(Debug)]
pub struct ReplayStats {
    pub elapsed: Duration,
    pub simulation_ticks: usize,
    pub timed_state_restores: usize,
    pub skipped_ticks: usize,
    pub arena_count: usize,
}

impl ReplayStats {
    pub fn total_simulation_ticks(&self) -> usize {
        self.simulation_ticks * self.arena_count
    }

    pub fn ticks_per_second(&self) -> f64 {
        self.total_simulation_ticks() as f64 / self.elapsed.as_secs_f64()
    }
}

pub fn run_replay<B: ReplayBenchmarkBackend>(
    backends: &mut [B],
    recording: &Recording,
    plan: &ReplayPlan,
) -> Result<ReplayStats, String> {
    if backends.is_empty() {
        return Err("--num-arenas must be greater than zero in RLPR replay mode".to_string());
    }

    // Convert controls and build all replay metadata before timing. The timed
    // loop must measure arena stepping, not RLPR parsing or adapter work.
    let prepared_controls = plan
        .controls
        .iter()
        .map(|tick| {
            tick.iter()
                .copied()
                .map(B::control_from_record)
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();

    let first_reset = *plan
        .resets
        .iter()
        .flatten()
        .next()
        .expect("replay plan has no supported ticks");
    for backend in backends.iter_mut() {
        backend.restore_replay_state(
            &recording.ticks,
            first_reset.state_index,
            first_reset.run_start,
            recording.version,
        );
    }

    let start = Instant::now();
    let mut timed_state_restores = 0;
    for (index, (reset, controls)) in plan.resets.iter().zip(prepared_controls.iter()).enumerate() {
        if index == first_reset.state_index || !replay_tick_supported(&recording.ticks[index]) {
            continue;
        }
        if let Some(reset) = reset {
            timed_state_restores += backends.len();
            for backend in backends.iter_mut() {
                backend.restore_replay_state(
                    &recording.ticks,
                    reset.state_index,
                    reset.run_start,
                    recording.version,
                );
            }
        } else {
            for backend in backends.iter_mut() {
                backend.step_simulation(controls);
            }
        }
    }

    Ok(ReplayStats {
        elapsed: start.elapsed(),
        simulation_ticks: plan.simulation_ticks,
        timed_state_restores,
        skipped_ticks: plan.skipped_ticks,
        arena_count: backends.len(),
    })
}

#[cfg(test)]
mod tests {
    use rocketsim_test::rlpr::{
        ROW_FLAGS_BALL_AVAILABLE, ROW_FLAGS_NO_PHYSICS,
        cpp_records::{CarRecord, ControlsRecord, PhysRecord, VecRecord},
    };

    use super::*;

    fn available_phys(frame: u32) -> PhysRecord {
        let mut phys: PhysRecord = unsafe { std::mem::zeroed() };
        phys.physics_frame = frame;
        phys.pos = VecRecord::new(frame as f32, 0.0, 0.0);
        phys.lin_vel = VecRecord::new(1.0, 0.0, 0.0);
        phys.rot.rows = [
            VecRecord::new(1.0, 0.0, 0.0),
            VecRecord::new(0.0, 1.0, 0.0),
            VecRecord::new(0.0, 0.0, 1.0),
        ];
        phys
    }

    fn tick(frame: u32, available_mask: u8, ball_available: bool) -> TickRecord {
        let mut cars = Vec::with_capacity(THREE_V_THREE_CARS);
        for slot in 0..THREE_V_THREE_CARS {
            let available = available_mask & (1 << slot) != 0;
            let mut car: CarRecord = unsafe { std::mem::zeroed() };
            car.physics_available = u8::from(available);
            car.phys = if available {
                available_phys(frame)
            } else {
                let mut placeholder = available_phys(u32::MAX);
                placeholder.rot.rows = [VecRecord::new(0.0, 0.0, 0.0); 3];
                placeholder
            };
            car.prev_controls = ControlsRecord {
                throttle: slot as f32,
                ..unsafe { std::mem::zeroed() }
            };
            cars.push(car);
        }
        let mut ball = if ball_available {
            available_phys(frame)
        } else {
            let mut placeholder = available_phys(u32::MAX);
            placeholder.rot.rows = [VecRecord::new(0.0, 0.0, 0.0); 3];
            placeholder
        };
        if !ball_available {
            ball.physics_frame = u32::MAX;
        }
        TickRecord {
            level_tick: frame as u64,
            row_flags: if ball_available {
                ROW_FLAGS_BALL_AVAILABLE
            } else {
                0
            },
            reserved0: 0,
            car_records: cars,
            ball_record: ball,
        }
    }

    #[test]
    fn sparse_car_and_ball_rows_are_skipped_and_split_replay_ranges() {
        let ticks = [
            tick(10, 0b11_1111, true),
            tick(11, 0b11_1011, true),
            tick(12, 0b11_1111, false),
            tick(13, 0b11_1111, true),
            tick(14, 0b11_1111, true),
        ];

        let plan = ReplayPlan::from_ticks(&ticks).unwrap();

        assert_eq!(plan.skipped_ticks, 2);
        assert_eq!(plan.simulation_ticks, 1);
        assert_eq!(plan.state_reset_count(), 2);
        assert!(plan.controls[1].is_empty());
        assert!(plan.controls[2].is_empty());
        assert_eq!(plan.controls[3].len(), THREE_V_THREE_CARS);
        assert!(plan.resets[0].is_some());
        assert!(plan.resets[3].is_some());
        assert!(plan.resets[4].is_none());
    }

    #[test]
    fn unavailable_placeholder_rows_are_never_replay_states() {
        let ticks = [
            tick(20, 0, false),
            tick(21, 0b11_1111, true),
            tick(22, 0, false),
        ];

        let plan = ReplayPlan::from_ticks(&ticks).unwrap();
        assert_eq!(plan.skipped_ticks, 2);
        assert_eq!(plan.simulation_ticks, 0);
        assert!(plan.resets[0].is_none());
        assert!(plan.resets[1].is_some());
        assert!(plan.controls[0].is_empty());
        assert!(plan.controls[2].is_empty());
    }

    #[test]
    fn replay_rejects_recordings_without_a_fully_available_tick() {
        let ticks = [tick(30, 0b11_1110, true), tick(31, 0b11_1110, false)];
        assert!(matches!(
            ReplayPlan::from_ticks(&ticks),
            Err(message) if message == "RLPR replay has no fully available physics ticks"
        ));
    }

    #[test]
    fn no_physics_rows_are_skipped_without_reading_placeholder_transforms() {
        let mut guard = tick(40, 0, false);
        guard.row_flags = ROW_FLAGS_NO_PHYSICS;
        let ticks = [tick(39, 0b11_1111, true), guard, tick(41, 0b11_1111, true)];

        let plan = ReplayPlan::from_ticks(&ticks).unwrap();
        assert_eq!(plan.skipped_ticks, 1);
        assert!(plan.controls[1].is_empty());
        assert!(plan.resets[2].is_some());
    }
}
