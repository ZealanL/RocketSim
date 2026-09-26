//! Segmented replay metric for RLPR recordings.
//!
//! Reset at each segment start, run open-loop, score ticks after warmup.
//! Print one table row per backend and contact category.

use std::path::PathBuf;

use clap::Parser;
use rocketsim_test::rlpr::Recording;

#[path = "rlpr_metric/common.rs"]
mod common;
#[cfg(feature = "v2")]
#[path = "rlpr_metric/v2.rs"]
mod v2;
#[path = "rlpr_metric/v3.rs"]
mod v3;

/// Segmented RocketSim replay metric against RLPR recordings.
/// Scores each car against its own trajectory. All cars share the sim.
/// Accepts several recordings in one run (e.g. the bundled Wisp 3v3 plus
/// Daizen 2v2 and PartyCannon 3v3 captures) and prints a single aggregate
/// table with all car-ticks added up.
#[derive(Parser)]
struct Args {
    /// RLPR recording files. Uses every bundled capture present in
    /// `rocketsim_test/recordings` by default.
    rlpr_files: Vec<PathBuf>,

    /// Ticks per segment.
    #[arg(long, default_value_t = 120)]
    segment_ticks: usize,

    /// Warmup ticks per segment that advance the sim without scoring.
    #[arg(long, default_value_t = 2)]
    warmup_ticks: usize,

    /// Reset to the prior RL state before each scored tick.
    #[arg(long)]
    reset_each_tick: bool,

    /// Refresh prior-state wheel rays at each segment start. This settles the
    /// sticky-wheel gate without advancing dynamics.
    /// Applies only with `--reset-each-tick`, at the first tick of each segment.
    #[arg(long)]
    reset_warmup: bool,

    /// Label ticks from recorded RL flags by default; also include contacts the sim observed.
    #[arg(long)]
    use_sim_events: bool,

    /// Dodge deadzone (|yaw| + |pitch| + |roll| needed to flip).
    /// Match this to the account the recording was made on.
    #[arg(long, default_value_t = 0.5)]
    dodge_deadzone: f32,
}

/// Bundled captures evaluated when no file is passed on the CLI, in order.
/// Missing files are skipped with a warning so the metric keeps working
/// before every capture has been recorded.
const DEFAULT_RECORDINGS: [&str; 3] = [
    "wisp_3v3_300s.rlpr.zst",
    "daizen_2v2_300s.rlpr.zst",
    "partycannon_3v3_300s.rlpr.zst",
];

/// Resolve the recordings to evaluate: explicit CLI files as-is, or every
/// bundled capture that exists on disk.
fn resolve_recordings(cli_files: &[PathBuf]) -> Result<Vec<PathBuf>, Box<dyn std::error::Error>> {
    if !cli_files.is_empty() {
        return Ok(cli_files.to_vec());
    }
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("recordings");
    let mut files = Vec::new();
    for name in DEFAULT_RECORDINGS {
        let path = dir.join(name);
        if path.is_file() {
            files.push(path);
        } else {
            println!("Skipping missing bundled capture: {}", path.display());
        }
    }
    if files.is_empty() {
        return Err("no RLPR recordings found; pass files explicitly or capture the bundled ones".into());
    }
    Ok(files)
}

fn metric_value(support: usize, value: f64) -> String {
    if support == 0 {
        "-".to_string()
    } else {
        format!("{value:.6}")
    }
}

fn print_report(backend: &str, report: &common::EvalReport) {
    for category in common::ContactCategory::ALL {
        let stats = report.for_category(category);
        let first_fail = stats
            .first_fail_tick
            .map(|tick| tick.to_string())
            .unwrap_or_else(|| "-".to_string());
        let pass_pct = metric_value(stats.support, stats.rate());
        let mean_norm = metric_value(stats.support, stats.mean_norm());
        let max_norm = metric_value(stats.support, f64::from(stats.max_norm));
        println!(
            "{backend:<7} {:<15} {:>9} {:>9} {pass_pct:>9} {mean_norm:>12} {max_norm:>12} {first_fail:>12}",
            category.as_str(),
            stats.support,
            stats.passed,
        );
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    if args.segment_ticks <= 1 {
        return Err("--segment-ticks must be greater than 1".into());
    }
    if !(0.0..=1.0).contains(&args.dodge_deadzone) {
        return Err("--dodge-deadzone must be within 0.0..=1.0".into());
    }
    if args.warmup_ticks >= args.segment_ticks {
        return Err("--warmup-ticks must be less than --segment-ticks".into());
    }

    let rlpr_files = resolve_recordings(&args.rlpr_files)?;
    println!(
        "Recordings: {} file{}",
        rlpr_files.len(),
        if rlpr_files.len() == 1 { "" } else { "s" },
    );
    for rlpr_file in &rlpr_files {
        println!("File: {}", rlpr_file.display());
    }

    let config = common::SegmentConfig {
        segment_ticks: args.segment_ticks,
        warmup_ticks: args.warmup_ticks,
    };
    if args.reset_each_tick {
        println!("Mode: one-tick replay with a state reset before each tick");
    } else {
        println!(
            "Segments: {} ticks ({} warmup ticks)",
            config.segment_ticks,
            config.warmup_ticks,
        );
    }
    println!("Categories overlap. Support counts car-ticks (cars x ticks).");
    println!("Dodge deadzone: {:.2}", args.dodge_deadzone);

    v3::init();
    let mut v3_backend = v3::V3Backend::with_dodge_deadzone(args.dodge_deadzone);
    #[cfg(feature = "v2")]
    let mut v2_backend = {
        v2::init();
        v2::V2Backend::with_dodge_deadzone(args.dodge_deadzone)
    };
    let mut combined_v3 = common::EvalReport::default();
    #[cfg(feature = "v2")]
    let mut combined_v2 = common::EvalReport::default();
    let mut skipped_transitions = 0usize;

    for rlpr_file in rlpr_files.iter() {
        let recording = Recording::from_file(rlpr_file)
            .map_err(|err| format!("{}: {err}", rlpr_file.display()))?;
        let num_cars = recording
            .ticks
            .first()
            .map(common::tick_car_count)
            .unwrap_or(0);
        if num_cars == 0 || num_cars > common::MAX_SCORED_CARS {
            return Err(format!("{}: recording must hold 1-8 cars in every tick", rlpr_file.display()).into());
        }
        if !recording
            .ticks
            .iter()
            .all(|tick| common::tick_car_count(tick) == num_cars)
        {
            return Err(format!("{}: recording car count must be constant", rlpr_file.display()).into());
        }
        if recording.ticks.len() <= args.warmup_ticks {
            return Err(format!(
                "{}: recording has too few ticks for the warmup length",
                rlpr_file.display()
            )
            .into());
        }

        let segments = common::split_segments(&recording.ticks, config);
        if segments.is_empty() {
            return Err(format!(
                "{}: split_segments returned no segments for this recording and config",
                rlpr_file.display()
            )
            .into());
        }

        // One backend serves every recording; `reset` rebuilds the arena
        // when the car count changes between recordings.
        let v3_outcome = common::evaluate(
            &mut v3_backend,
            &recording.ticks,
            &segments,
            config.warmup_ticks,
            args.reset_each_tick,
            args.reset_warmup,
            args.use_sim_events,
            rocketsim_test::rlpr::recording_has_boost_state(recording.version),
            rocketsim_test::rlpr::recording_has_handbrake_state(recording.version),
        );
        combined_v3.merge(&v3_outcome.report);
        skipped_transitions += v3_outcome.skipped_transitions;

        #[cfg(feature = "v2")]
        {
            let v2_outcome = common::evaluate(
                &mut v2_backend,
                &recording.ticks,
                &segments,
                config.warmup_ticks,
                args.reset_each_tick,
                args.reset_warmup,
                args.use_sim_events,
                rocketsim_test::rlpr::recording_has_boost_state(recording.version),
                rocketsim_test::rlpr::recording_has_handbrake_state(recording.version),
            );
            combined_v2.merge(&v2_outcome.report);
        }
    }

    println!();
    println!(
        "Kickoff stasis ({}): {} transitions stepped but unscored.",
        common::KICKOFF_STASIS_RULE,
        skipped_transitions,
    );
    println!(
        "{:<7} {:<15} {:>9} {:>9} {:>9} {:>12} {:>12} {:>12}",
        "Backend", "Category", "Support", "Passed", "Pass %", "Mean norm", "Max norm", "First fail"
    );
    println!("{}", "-".repeat(102));
    print_report("v3", &combined_v3);
    #[cfg(feature = "v2")]
    {
        println!();
        print_report("v2", &combined_v2);
    }

    Ok(())
}
