//! Measure one-tick position parity for an RLPR corpus.
//!
//! `car_sum_pass_rate` is the primary metric. It sums the Euclidean position
//! error for all cars in a tick. The tick passes when the sum is less than
//! `--threshold-uu`.
//!
//! Position samples are excluded when a car record is absent or when either
//! endpoint is part of a frozen run or a kickoff/reset boundary.

use std::{
    collections::HashSet,
    fs, io,
    path::{Path, PathBuf},
};

use clap::Parser;
use glam::Vec3A;
use rocketsim::{
    Arena, CarBodyConfig, CarControls, CarState, GameMode, PhysState, Team,
    consts::{BT_TO_UU, car::spawn::RESPAWN_TIME},
    rlpr::{
        Recording,
        cpp_records::{CarRecord, PhysRecord, RecordingInfo, VecRecord},
        tick_record::TickRecord,
    },
};

/// Measure one-tick RocketSim position parity against RLPR recordings.
#[derive(Parser)]
struct Args {
    /// Directory that contains RLPR files and an optional manifest.csv.
    directory: PathBuf,

    /// Position error threshold in Unreal units.
    #[arg(long, default_value_t = 0.03)]
    threshold_uu: f32,

    /// Include recordings that manifest.csv does not mark as scored_corpus.
    #[arg(long)]
    all: bool,
}

#[derive(Default)]
struct Metrics {
    eligible_ticks: usize,
    max_passed_ticks: usize,
    sum_passed_ticks: usize,
    eligible_cars: usize,
    passed_cars: usize,
    eligible_ball_ticks: usize,
    passed_ball_ticks: usize,
    frame_gaps: usize,
    missing_car_ticks: usize,
    /// Comparisons excluded because either endpoint is frozen.
    frozen_ticks: usize,
    /// Comparisons excluded because either endpoint is a reset state.
    reset_ticks: usize,
    max_error_uu: f32,
}

impl Metrics {
    fn add(&mut self, other: &Self) {
        self.eligible_ticks += other.eligible_ticks;
        self.max_passed_ticks += other.max_passed_ticks;
        self.sum_passed_ticks += other.sum_passed_ticks;
        self.eligible_cars += other.eligible_cars;
        self.passed_cars += other.passed_cars;
        self.eligible_ball_ticks += other.eligible_ball_ticks;
        self.passed_ball_ticks += other.passed_ball_ticks;
        self.frame_gaps += other.frame_gaps;
        self.missing_car_ticks += other.missing_car_ticks;
        self.frozen_ticks += other.frozen_ticks;
        self.reset_ticks += other.reset_ticks;
        self.max_error_uu = self.max_error_uu.max(other.max_error_uu);
    }

    fn tick_max_rate(&self) -> f64 {
        percent(self.max_passed_ticks, self.eligible_ticks)
    }

    fn tick_sum_rate(&self) -> f64 {
        percent(self.sum_passed_ticks, self.eligible_ticks)
    }

    fn car_rate(&self) -> f64 {
        percent(self.passed_cars, self.eligible_cars)
    }

    fn ball_rate(&self) -> f64 {
        percent(self.passed_ball_ticks, self.eligible_ball_ticks)
    }
}

fn percent(numerator: usize, denominator: usize) -> f64 {
    if denominator == 0 {
        0.0
    } else {
        100.0 * numerator as f64 / denominator as f64
    }
}

fn collect_rlpr_files(directory: &Path, files: &mut Vec<PathBuf>) -> io::Result<()> {
    for entry in fs::read_dir(directory)? {
        let path = entry?.path();
        if path.is_dir() {
            collect_rlpr_files(&path, files)?;
        } else if path
            .extension()
            .is_some_and(|extension| extension.eq_ignore_ascii_case("rlpr"))
        {
            files.push(path);
        }
    }
    Ok(())
}

fn scored_recording_names(directory: &Path) -> io::Result<Option<HashSet<String>>> {
    let path = directory.join("manifest.csv");
    let Ok(csv) = fs::read_to_string(path) else {
        return Ok(None);
    };

    let mut names = HashSet::new();
    for line in csv.lines().skip(1) {
        let columns: Vec<_> = line.split(',').collect();
        if columns.len() >= 4 && columns[3] == "1" {
            names.insert(columns[0].to_string());
        }
    }
    Ok(Some(names))
}

fn set_state_to_tick(
    arena: &mut Arena,
    car_ids: &[usize],
    tick: &TickRecord,
    controls: &[CarControls],
) {
    for ((&car_id, car_record), &controls) in car_ids.iter().zip(&tick.car_records).zip(controls) {
        let mut state = *arena.get_car_state(car_id);
        let recorded: CarState = (*car_record).into();
        state.phys = recorded.phys;
        state.is_on_ground = recorded.is_on_ground;
        state.wheels_with_contact = recorded.wheels_with_contact;
        state.is_jumping = recorded.is_jumping;
        state.is_flipping = recorded.is_flipping;
        state.jump_ticks = recorded.jump_ticks;
        state.flip_time = recorded.flip_time;
        state.has_jumped = recorded.has_jumped;
        state.prev_controls = car_record.prev_controls.into();
        state.flip_rel_torque = recorded.flip_rel_torque;
        state.boost = recorded.boost;
        state.is_demoed = false;
        state.demo_respawn_timer = 0.0;

        if car_record.has_flip {
            state.has_double_jumped = false;
            state.has_flipped = false;
        } else if car_record.is_flipping {
            state.has_double_jumped = false;
            state.has_flipped = true;
        } else if car_record.double_jumped_or_flipped && !state.has_flipped {
            state.has_double_jumped = true;
        }

        state.controls = controls;
        arena.set_car_state(car_id, state);
    }

    let recorded_ball: PhysState = tick.ball_record.into();
    let mut ball = *arena.get_ball_state();
    ball.phys = recorded_ball;
    arena.set_ball_state(ball);
}

fn closest_car_config(info: &RecordingInfo) -> CarBodyConfig {
    let min: Vec3A = info.hitbox_rel_min_bt.into();
    let max: Vec3A = info.hitbox_rel_max_bt.into();
    let recorded_size = (max - min) * BT_TO_UU;
    let recorded_offset = (max + min) * (0.5 * BT_TO_UU);
    let configs = [
        CarBodyConfig::OCTANE,
        CarBodyConfig::DOMINUS,
        CarBodyConfig::PLANK,
        CarBodyConfig::BREAKOUT,
        CarBodyConfig::HYBRID,
        CarBodyConfig::MERC,
        CarBodyConfig::PSYCLOPS,
    ];

    configs
        .into_iter()
        .min_by(|a, b| {
            let error = |config: &CarBodyConfig| {
                (config.hitbox_size - recorded_size).length_squared()
                    + (config.hitbox_pos_offset - recorded_offset).length_squared()
            };
            error(a).total_cmp(&error(b))
        })
        .unwrap()
}

fn frame_is_contiguous(from: &TickRecord, to: &TickRecord) -> bool {
    from.car_records
        .iter()
        .zip(&to.car_records)
        .all(|(from, to)| to.phys.physics_frame == from.phys.physics_frame + 1)
        && to.ball_record.physics_frame == from.ball_record.physics_frame + 1
}

fn phys_is_unchanged(from: &PhysRecord, to: &PhysRecord) -> bool {
    from.pos == to.pos && from.lin_vel == to.lin_vel && from.ang_vel == to.ang_vel
}

fn tick_is_frozen(from: &TickRecord, to: &TickRecord) -> bool {
    phys_is_unchanged(&from.ball_record, &to.ball_record)
        && from
            .car_records
            .iter()
            .zip(&to.car_records)
            .all(|(from, to)| phys_is_unchanged(&from.phys, &to.phys))
}

fn frozen_tick_flags(ticks: &[TickRecord]) -> Vec<bool> {
    let mut flags = vec![false; ticks.len()];
    for (index, window) in ticks.windows(2).enumerate() {
        let [from, to] = window else { unreachable!() };
        if tick_is_frozen(from, to) {
            flags[index] = true;
            flags[index + 1] = true;
        }
    }
    flags
}

fn zero_vector(vector: VecRecord) -> bool {
    let vector = Vec3A::from(vector);
    vector.length_squared() <= 1e-6
}

fn car_is_at_kickoff_spawn(car: &CarRecord) -> bool {
    let pos = Vec3A::from(car.phys.pos);
    let matches_spawn = |x: f32, y: f32| (pos.x - x).abs() <= 0.1 && (pos.y - y).abs() <= 0.1;
    let at_spawn = rocketsim::consts::car::spawn::get_kickoff_spawn_locations(GameMode::Soccar)
        .iter()
        .any(|spawn| matches_spawn(spawn.x, spawn.y) || matches_spawn(-spawn.x, -spawn.y));
    at_spawn && zero_vector(car.phys.lin_vel) && zero_vector(car.phys.ang_vel)
}

fn tick_is_reset(tick: &TickRecord) -> bool {
    let ball = &tick.ball_record;
    // RLPR records the standard kickoff ball height as 100.49 UU.
    let ball_at_center =
        ball.pos.x == 0.0 && ball.pos.y == 0.0 && (ball.pos.z - 100.49).abs() <= 0.01;
    (ball_at_center && zero_vector(ball.lin_vel) && zero_vector(ball.ang_vel))
        || tick.car_records.iter().any(car_is_at_kickoff_spawn)
}

fn reset_tick_flags(ticks: &[TickRecord]) -> Vec<bool> {
    let mut flags = vec![false; ticks.len()];
    for (index, tick) in ticks.iter().enumerate() {
        if tick_is_reset(tick) {
            flags[index] = true;
        }
    }
    flags
}

// RLPR omits a car while it is demolished. Keep it out of the simulation
// until the recorded respawn state is available.
fn mark_cars_demoed(arena: &mut Arena, car_ids: &[usize]) {
    for &car_id in car_ids {
        let mut state = *arena.get_car_state(car_id);
        state.is_demoed = true;
        state.demo_respawn_timer = RESPAWN_TIME;
        arena.set_car_state(car_id, state);
    }
}

fn measure_recording(recording: &Recording, threshold_uu: f32) -> Metrics {
    let num_cars = recording
        .ticks
        .iter()
        .map(|tick| tick.car_records.len())
        .max()
        .unwrap_or(0);
    if num_cars == 0 {
        return Metrics::default();
    }

    let mut arena = Arena::new(GameMode::Soccar);
    let car_config = closest_car_config(&recording.info);
    let car_ids: Vec<_> = (0..num_cars)
        .map(|idx| arena.add_car(Team::try_from((idx % 2) as u8).unwrap(), car_config))
        .collect();

    if let Some(warmup_tick) = recording
        .ticks
        .iter()
        .find(|tick| tick.car_records.len() == num_cars)
    {
        let warmup_controls: Vec<CarControls> = warmup_tick
            .car_records
            .iter()
            .map(|record| record.prev_controls.into())
            .collect();
        set_state_to_tick(&mut arena, &car_ids, warmup_tick, &warmup_controls);
        arena.step_tick();
    }

    let frozen_ticks = frozen_tick_flags(&recording.ticks);
    let reset_ticks = reset_tick_flags(&recording.ticks);
    let mut metrics = Metrics::default();
    for (index, ticks) in recording.ticks.windows(2).enumerate() {
        let [from, to] = ticks else { unreachable!() };
        if index == 0 {
            continue;
        }
        if from.car_records.len() != num_cars || to.car_records.len() != num_cars {
            metrics.missing_car_ticks += 1;
            if from.car_records.len() == num_cars && to.car_records.len() != num_cars {
                mark_cars_demoed(&mut arena, &car_ids);
            } else if from.car_records.len() != num_cars && to.car_records.len() == num_cars {
                let controls: Vec<CarControls> = to
                    .car_records
                    .iter()
                    .map(|record| record.prev_controls.into())
                    .collect();
                set_state_to_tick(&mut arena, &car_ids, to, &controls);
            }
            continue;
        }
        if frozen_ticks[index] || frozen_ticks[index + 1] {
            metrics.frozen_ticks += 1;
            continue;
        }
        if reset_ticks[index] || reset_ticks[index + 1] {
            metrics.reset_ticks += 1;
            continue;
        }
        if !frame_is_contiguous(from, to) {
            metrics.frame_gaps += 1;
            continue;
        }

        let controls: Vec<CarControls> = to
            .car_records
            .iter()
            .map(|record| record.prev_controls.into())
            .collect();
        set_state_to_tick(&mut arena, &car_ids, from, &controls);
        arena.step_tick();

        let ball_error =
            (arena.get_ball_state().phys.pos - Vec3A::from(to.ball_record.pos)).length();
        metrics.eligible_ball_ticks += 1;
        metrics.passed_ball_ticks += usize::from(ball_error < threshold_uu);

        let mut max_error = 0.0_f32;
        let mut sum_error = 0.0_f32;
        for (&car_id, to_record) in car_ids.iter().zip(&to.car_records) {
            let error =
                (arena.get_car_state(car_id).phys.pos - Vec3A::from(to_record.phys.pos)).length();
            max_error = max_error.max(error);
            sum_error += error;
            metrics.eligible_cars += 1;
            metrics.passed_cars += usize::from(error < threshold_uu);
        }

        metrics.eligible_ticks += 1;
        metrics.max_passed_ticks += usize::from(max_error < threshold_uu);
        metrics.sum_passed_ticks += usize::from(sum_error < threshold_uu);
        metrics.max_error_uu = metrics.max_error_uu.max(max_error);
    }

    metrics
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    if !(args.threshold_uu.is_finite() && args.threshold_uu > 0.0) {
        return Err("--threshold-uu must be a positive finite value".into());
    }

    rocketsim::init_from_default(true)?;

    let scored_names = scored_recording_names(&args.directory)?;
    if !args.all && scored_names.is_none() {
        return Err("manifest.csv is required unless --all is set".into());
    }

    let mut files = Vec::new();
    collect_rlpr_files(&args.directory, &mut files)?;
    files.sort();

    println!(
        "file,version,ticks,eligible,frame_gaps,missing_car_ticks,frozen,reset,car_max_pass_rate,car_sum_pass_rate,car_object_pass_rate,ball_pass_rate,max_car_error_uu"
    );

    let mut total = Metrics::default();
    let mut measured_files = 0usize;
    let mut skipped_files = 0usize;
    for path in files {
        let name = path
            .file_stem()
            .and_then(|stem| stem.to_str())
            .unwrap_or("");
        if !args.all
            && scored_names
                .as_ref()
                .is_some_and(|names| !names.contains(name))
        {
            continue;
        }

        let recording = match Recording::from_file(&path) {
            Ok(recording) => recording,
            Err(error) => {
                eprintln!("skip {}: {error}", path.display());
                skipped_files += 1;
                continue;
            }
        };
        if recording.ticks.len() < 2 {
            eprintln!(
                "skip {}: recording has fewer than two ticks",
                path.display()
            );
            skipped_files += 1;
            continue;
        }

        let metrics = measure_recording(&recording, args.threshold_uu);
        println!(
            "{},{},{},{},{},{},{},{},{:.6},{:.6},{:.6},{:.6},{:.9}",
            recording.name,
            recording.version,
            recording.ticks.len(),
            metrics.eligible_ticks,
            metrics.frame_gaps,
            metrics.missing_car_ticks,
            metrics.frozen_ticks,
            metrics.reset_ticks,
            metrics.tick_max_rate(),
            metrics.tick_sum_rate(),
            metrics.car_rate(),
            metrics.ball_rate(),
            metrics.max_error_uu,
        );
        total.add(&metrics);
        measured_files += 1;
    }

    println!(
        "TOTAL,-,{},{},{},{},{},{},{:.6},{:.6},{:.6},{:.6},{:.9}",
        measured_files,
        total.eligible_ticks,
        total.frame_gaps,
        total.missing_car_ticks,
        total.frozen_ticks,
        total.reset_ticks,
        total.tick_max_rate(),
        total.tick_sum_rate(),
        total.car_rate(),
        total.ball_rate(),
        total.max_error_uu,
    );
    eprintln!(
        "measured_files={measured_files}, skipped_files={skipped_files}, threshold_uu={}",
        args.threshold_uu
    );

    Ok(())
}
