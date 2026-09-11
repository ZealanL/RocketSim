//! Segmented replay metric for one-car RLPR recordings.
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

/// Segmented RocketSim replay metric against one RLPR recording.
/// Scores each car independently against its own trajectory; the opponent
/// is absent from the sim.
#[derive(Parser)]
struct Args {
    /// RLPR recording file. Uses the bundled Wisp 1v1 capture by default.
    rlpr_file: Option<PathBuf>,

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

    /// Label ticks from RL flags only; ignore contacts the sim observed.
    /// Isolates physics fidelity from sim event sensitivity.
    #[arg(long)]
    ignore_sim_events: bool,

    /// Dodge deadzone (|yaw| + |pitch| + |roll| needed to flip).
    /// Match this to the account the recording was made on.
    #[arg(long, default_value_t = 0.5)]
    dodge_deadzone: f32,
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

    let rlpr_file = args.rlpr_file.unwrap_or_else(|| {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("recordings")
            .join("wisp_1v1_300s.rlpr")
    });
    let recording = Recording::from_file(&rlpr_file)?;
    let num_cars = recording
        .ticks
        .first()
        .map(common::tick_car_count)
        .unwrap_or(0);
    if num_cars == 0 || num_cars > common::MAX_SCORED_CARS {
        return Err("recording must hold 1-8 cars in every tick".into());
    }
    if !recording
        .ticks
        .iter()
        .all(|tick| common::tick_car_count(tick) == num_cars)
    {
        return Err("recording car count must be constant".into());
    }
    if recording.ticks.len() <= args.warmup_ticks {
        return Err("recording has too few ticks for the warmup length".into());
    }

    let config = common::SegmentConfig {
        segment_ticks: args.segment_ticks,
        warmup_ticks: args.warmup_ticks,
    };
    let segments = common::split_segments(&recording.ticks, config);
    if segments.is_empty() {
        return Err("split_segments returned no segments for this recording and config".into());
    }

    println!(
        "Recording: {} (RLPR v{}, {} car{})",
        recording.name,
        recording.version,
        num_cars,
        if num_cars == 1 { "" } else { "s" },
    );
    println!("File: {}", rlpr_file.display());
    println!("Ticks: {}", recording.ticks.len());
    if args.reset_each_tick {
        println!("Mode: one-tick replay with a state reset before each tick");
    } else {
        println!(
            "Segments: {} x {} ticks ({} warmup ticks)",
            segments.len(),
            config.segment_ticks,
            config.warmup_ticks,
        );
    }
    println!("Categories overlap. Support counts car-ticks (cars x ticks).");
    if num_cars > 1 {
        println!("All cars share the sim; car-car contacts are real sim events.");
    }
    println!("Dodge deadzone: {:.2}", args.dodge_deadzone);
    println!();
    println!(
        "{:<7} {:<15} {:>9} {:>9} {:>9} {:>12} {:>12} {:>12}",
        "Backend", "Category", "Support", "Passed", "Pass %", "Mean norm", "Max norm", "First fail"
    );
    println!("{}", "-".repeat(102));

    v3::init();
    let mut v3_backend = v3::V3Backend::with_dodge_deadzone(args.dodge_deadzone);
    let v3_report = common::evaluate(
        &mut v3_backend,
        &recording.ticks,
        &segments,
        config.warmup_ticks,
        args.reset_each_tick,
        args.reset_warmup,
        !args.ignore_sim_events,
    );
    print_report("v3", &v3_report);

    #[cfg(feature = "v2")]
    {
        v2::init();
        let mut v2_backend = v2::V2Backend::with_dodge_deadzone(args.dodge_deadzone);
        let v2_report = common::evaluate(
            &mut v2_backend,
            &recording.ticks,
            &segments,
            config.warmup_ticks,
            args.reset_each_tick,
            args.reset_warmup,
            !args.ignore_sim_events,
        );
        println!();
        print_report("v2", &v2_report);
    }

    Ok(())
}
