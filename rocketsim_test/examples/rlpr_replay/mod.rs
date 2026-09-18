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
}

impl ReplayPlan {
    pub fn from_recording(recording: &Recording) -> Result<Self, String> {
        if recording.ticks.len() < 2 {
            return Err("recording must contain at least two ticks".to_string());
        }
        if recording
            .ticks
            .iter()
            .any(|tick| tick_car_count(tick) != THREE_V_THREE_CARS)
        {
            return Err(
                "RLPR replay mode requires exactly six cars (3v3) in every tick".to_string(),
            );
        }

        let controls = recording
            .ticks
            .iter()
            .map(|tick| {
                tick.car_records
                    .iter()
                    .map(|car| car.prev_controls)
                    .collect()
            })
            .collect::<Vec<Vec<_>>>();

        let mut resets = vec![None; recording.ticks.len()];
        resets[0] = Some(ReplayReset {
            state_index: 0,
            run_start: 0,
        });

        for index in 1..recording.ticks.len() {
            let from = &recording.ticks[index - 1];
            let to = &recording.ticks[index];
            let needs_state = !frame_is_contiguous(from, to)
                || tick_is_frozen(from, to)
                || any_teleport(from, to);
            if needs_state {
                resets[index] = Some(ReplayReset {
                    state_index: index,
                    run_start: run_start(&recording.ticks, index),
                });
            }
        }

        let simulation_ticks = resets
            .iter()
            .skip(1)
            .filter(|reset| reset.is_none())
            .count();
        Ok(Self {
            controls,
            resets,
            simulation_ticks,
        })
    }

    pub fn state_reset_count(&self) -> usize {
        self.resets.iter().filter(|reset| reset.is_some()).count()
    }
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

    let first_reset = plan.resets[0].expect("replay plan always has an initial reset");
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
    for target_index in 1..recording.ticks.len() {
        if let Some(reset) = plan.resets[target_index] {
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
            let controls = &prepared_controls[target_index];
            for backend in backends.iter_mut() {
                backend.step_simulation(controls);
            }
        }
    }

    Ok(ReplayStats {
        elapsed: start.elapsed(),
        simulation_ticks: plan.simulation_ticks,
        timed_state_restores,
        arena_count: backends.len(),
    })
}
