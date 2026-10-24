//! Segmented replay metric for RLPR recordings.
//!
//! Reset at each segment start, run open-loop, score every tick after.
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
/// Daizen 2v2 and PartyCannon 3v3 captures) and prints one aggregate
/// table with all car-ticks added up. Use `--per-recording` to also print
/// a table for each file.
#[derive(Parser)]
struct Args {
    /// RLPR recording files. Uses every bundled capture present in
    /// `rocketsim_test/recordings` by default.
    rlpr_files: Vec<PathBuf>,

    /// Print one table per recording in addition to the aggregate table.
    #[arg(long)]
    per_recording: bool,

    /// Explicit body preset for each car slot, in recording order.
    /// Use comma-separated names: octane,dominus,plank,breakout,hybrid,merc,psyclops.
    /// The list must contain exactly one preset per car in every selected recording.
    #[arg(long, value_name = "PRESET,PRESET,...")]
    car_bodies: Option<String>,

    /// Ticks per segment.
    #[arg(long, default_value_t = 120)]
    segment_ticks: usize,

    /// Reset to the prior RL state before each scored tick.
    #[arg(long)]
    reset_each_tick: bool,

    /// Reset only car/ball pos/vel/ang_vel before each scored tick.
    ///
    /// Motion-only diagnostic: full seed once per segment, then preserve
    /// orientation and logical history. Setters still clear transient
    /// accumulators and car impulse cache. Conflicts with `--reset-each-tick`.
    #[arg(long, conflicts_with = "reset_each_tick")]
    reset_motion_each_tick: bool,

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
const DEFAULT_RECORDINGS: [&str; 5] = [
    "wisp_3v3_300s.rlpr.zst",
    "daizen_2v2_300s.rlpr.zst",
    "partycannon_3v3_300s.rlpr.zst",
    "nexto_1v1_300s.rlpr.zst",
    "veai_3v3_320wall.rlpr.zst",
];

fn parse_car_bodies(value: &str) -> Result<Vec<usize>, String> {
    value
        .split(',')
        .map(|name| {
            common::BODY_PRESET_NAMES
                .iter()
                .position(|preset| preset.eq_ignore_ascii_case(name.trim()))
                .ok_or_else(|| {
                    format!(
                        "unknown body preset {name:?}; expected one of {}",
                        common::BODY_PRESET_NAMES.join(", ")
                    )
                })
        })
        .collect()
}

fn validate_roster_len(
    recording: &std::path::Path,
    roster_len: usize,
    car_slots: usize,
) -> Result<(), String> {
    if roster_len != car_slots {
        return Err(format!(
            "{}: --car-bodies has {roster_len} entries but the recording has {car_slots} car slots",
            recording.display()
        ));
    }
    Ok(())
}

fn validate_roster_for_recordings(recordings: &[PathBuf], roster: &[usize]) -> Result<(), String> {
    for path in recordings {
        let recording =
            Recording::from_file(path).map_err(|err| format!("{}: {err}", path.display()))?;
        let ticks = common::collapse_duplicate_ticks(&recording.ticks);
        let car_slots = ticks.first().map(common::tick_car_count).unwrap_or(0);
        validate_roster_len(path, roster.len(), car_slots)?;
    }
    Ok(())
}

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
        return Err(
            "no RLPR recordings found; pass files explicitly or capture the bundled ones".into(),
        );
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

/// Table rule widths, matching the header formats in [`print_table_header`].
const WIDE_RULE: usize = 92;
/// Narrow rule: the 12-wide "First fail" column plus its separating space.
const NARROW_RULE: usize = WIDE_RULE - 13;

/// Print skipped-transition counts. Kickoff stasis is stepped but unscored;
/// see [`common::KICKOFF_STASIS_RULE`].
fn print_skip_counts(kickoff_stasis: usize) {
    println!(
        "Kickoff stasis ({}): {} transitions stepped but unscored.",
        common::KICKOFF_STASIS_RULE,
        kickoff_stasis,
    );
}

/// Print the table header. `first_fail` adds the "First fail" column, which
/// only means something for a single recording: tick indices are
/// per-recording, so a combined report has none to show.
fn print_table_header(first_fail: bool) {
    if first_fail {
        println!(
            "{:<7} {:<15} {:>9} {:>9} {:>9} {:>12} {:>12} {:>12}",
            "Backend",
            "Category",
            "Support",
            "Passed",
            "Pass %",
            "Mean norm",
            "Max norm",
            "First fail"
        );
        println!("{}", "-".repeat(WIDE_RULE));
    } else {
        println!(
            "{:<7} {:<15} {:>9} {:>9} {:>9} {:>12} {:>12}",
            "Backend", "Category", "Support", "Passed", "Pass %", "Mean norm", "Max norm"
        );
        println!("{}", "-".repeat(NARROW_RULE));
    }
}

/// Print one row per category. `first_fail` matches [`print_table_header`]:
/// pass when it is set, drop the column when it is not.
fn print_report(backend: &str, report: &common::EvalReport, first_fail: bool) {
    for category in common::ContactCategory::ALL {
        let stats = report.for_category(category);
        let pass_pct = metric_value(stats.support, stats.rate());
        let mean_norm = metric_value(stats.support, stats.mean_norm());
        let max_norm = metric_value(stats.support, f64::from(stats.max_norm));
        if first_fail {
            let first_fail_tick = stats
                .first_fail_tick
                .map(|tick| tick.to_string())
                .unwrap_or_else(|| "-".to_string());
            println!(
                "{backend:<7} {:<15} {:>9} {:>9} {pass_pct:>9} {mean_norm:>12} {max_norm:>12} {first_fail_tick:>12}",
                category.as_str(),
                stats.support,
                stats.passed,
            );
        } else {
            println!(
                "{backend:<7} {:<15} {:>9} {:>9} {pass_pct:>9} {mean_norm:>12} {max_norm:>12}",
                category.as_str(),
                stats.support,
                stats.passed,
            );
        }
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

    let rlpr_files = resolve_recordings(&args.rlpr_files)?;
    let explicit_bodies = args
        .car_bodies
        .as_deref()
        .map(parse_car_bodies)
        .transpose()?;
    if let Some(bodies) = &explicit_bodies {
        validate_roster_for_recordings(&rlpr_files, bodies)?;
    }
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
    };
    let reset_mode =
        common::ResetMode::from_flags(args.reset_each_tick, args.reset_motion_each_tick);
    match reset_mode {
        common::ResetMode::ResetEachTick => {
            println!("Mode: one-tick replay with a state reset before each tick");
        }
        common::ResetMode::ResetMotionEachTick => {
            println!(
                "Mode: one-tick replay with motion-only reset before each tick (pos/vel/ang_vel; orientation and logical history preserved; transients cleared); segments: {} ticks; full seed once per segment",
                config.segment_ticks,
            );
        }
        common::ResetMode::Segmented => {
            println!("Segments: {} ticks", config.segment_ticks,);
        }
    }
    println!("Categories overlap. Support counts car-ticks (cars x ticks).");
    println!("Dodge deadzone: {:.2}", args.dodge_deadzone);
    let tol = common::Tolerances::for_reset_mode(reset_mode);
    println!(
        "Tolerances: pos {:.1} uu, vel {:.1} uu/s, ang vel {:.2} rad/s, axis {:.2}{}",
        tol.pos_uu,
        tol.vel_uu_s,
        tol.ang_vel_rad_s,
        tol.axis,
        if reset_mode != common::ResetMode::Segmented {
            " (velocity tightened for one-tick replay)"
        } else {
            ""
        },
    );
    v3::init();
    let settled_heights = v3::preset_settled_heights();
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
        // Collapse recorder-duplicated rows first: some files repeat a tick
        // verbatim, which shatters runs and voids neighboring transitions
        // for contact-timing checks. Everything below scores physics steps,
        // so it runs on the deduplicated ticks.
        let ticks = common::collapse_duplicate_ticks(&recording.ticks);
        let num_cars = ticks.first().map(common::tick_car_count).unwrap_or(0);
        if num_cars == 0 || num_cars > common::MAX_SCORED_CARS {
            return Err(format!(
                "{}: recording must hold 1-8 cars in every tick",
                rlpr_file.display()
            )
            .into());
        }

        let bodies = match &explicit_bodies {
            Some(bodies) => bodies.clone(),
            None => {
                // The header carries one hitbox for the whole file, so it only
                // describes car 0. Settled heights detect wheel geometry but
                // cannot separate bodies with shared wheel geometry.
                let header_body = v3::body_preset_index(&recording.info)
                    .map_err(|err| format!("{}: {err}", rlpr_file.display()))?;
                common::bodies_from_ticks(&ticks, header_body, &settled_heights)
                    .map_err(|err| format!("{}: {err}", rlpr_file.display()))?
            }
        };
        v3_backend.set_bodies(&bodies);
        #[cfg(feature = "v2")]
        v2_backend.set_bodies(&bodies);
        println!(
            "{}: bodies {:?}",
            rlpr_file.display(),
            bodies
                .iter()
                .map(|&index| common::BODY_PRESET_NAMES[index])
                .collect::<Vec<_>>()
        );
        if !ticks
            .iter()
            .all(|tick| common::tick_car_count(tick) == num_cars)
        {
            return Err(format!(
                "{}: recording car count must be constant",
                rlpr_file.display()
            )
            .into());
        }
        if ticks.len() <= 1 {
            return Err(format!(
                "{}: recording has too few ticks to score a transition",
                rlpr_file.display()
            )
            .into());
        }

        let segments = common::split_segments(&ticks, config);
        if segments.is_empty() {
            return Err(format!(
                "{}: split_segments returned no segments for this recording and config",
                rlpr_file.display()
            )
            .into());
        }

        // One backend serves every recording. The detected body roster plus
        // `reset` rebuild the arena when the bodies or the car count change
        // between recordings.
        let v3_outcome = if reset_mode == common::ResetMode::ResetMotionEachTick {
            common::evaluate_motion_each_tick(
                &mut v3_backend,
                &ticks,
                &segments,
                args.use_sim_events,
                rocketsim_test::rlpr::recording_has_boost_state(recording.version),
                rocketsim_test::rlpr::recording_has_handbrake_state(recording.version),
                rocketsim_test::rlpr::recording_has_touch_frames(recording.version),
            )
        } else {
            common::evaluate(
                &mut v3_backend,
                &ticks,
                &segments,
                reset_mode == common::ResetMode::ResetEachTick,
                args.use_sim_events,
                rocketsim_test::rlpr::recording_has_boost_state(recording.version),
                rocketsim_test::rlpr::recording_has_handbrake_state(recording.version),
                rocketsim_test::rlpr::recording_has_touch_frames(recording.version),
            )
        };
        combined_v3.merge(&v3_outcome.report);
        skipped_transitions += v3_outcome.skipped_transitions;

        #[cfg(feature = "v2")]
        let v2_outcome = if reset_mode == common::ResetMode::ResetMotionEachTick {
            common::evaluate_motion_each_tick(
                &mut v2_backend,
                &ticks,
                &segments,
                args.use_sim_events,
                rocketsim_test::rlpr::recording_has_boost_state(recording.version),
                rocketsim_test::rlpr::recording_has_handbrake_state(recording.version),
                rocketsim_test::rlpr::recording_has_touch_frames(recording.version),
            )
        } else {
            common::evaluate(
                &mut v2_backend,
                &ticks,
                &segments,
                reset_mode == common::ResetMode::ResetEachTick,
                args.use_sim_events,
                rocketsim_test::rlpr::recording_has_boost_state(recording.version),
                rocketsim_test::rlpr::recording_has_handbrake_state(recording.version),
                rocketsim_test::rlpr::recording_has_touch_frames(recording.version),
            )
        };
        #[cfg(feature = "v2")]
        combined_v2.merge(&v2_outcome.report);

        if args.per_recording {
            println!("\nRecording: {}", rlpr_file.display());
            print_skip_counts(v3_outcome.skipped_transitions);
            print_table_header(true);
            print_report("v3", &v3_outcome.report, true);
            #[cfg(feature = "v2")]
            {
                println!();
                print_report("v2", &v2_outcome.report, true);
            }
        }
    }

    println!();
    if args.per_recording {
        println!("Combined recordings:");
    }
    print_skip_counts(skipped_transitions);
    print_table_header(false);
    print_report("v3", &combined_v3, false);
    #[cfg(feature = "v2")]
    {
        println!();
        print_report("v2", &combined_v2, false);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn explicit_car_bodies_parse_case_insensitively_and_require_exact_slots() {
        let valid = Args::try_parse_from([
            "rlpr_metric",
            "--car-bodies",
            "OCTANE,Dominus,plank,BREAKOUT,hybrid,Merc,PSYCLOPS",
        ])
        .unwrap();
        let roster = parse_car_bodies(valid.car_bodies.as_deref().unwrap()).unwrap();
        assert_eq!(roster, (0..common::NUM_BODY_PRESETS).collect::<Vec<_>>());

        for (input, expected_error) in [
            ("octane,,hybrid", "unknown body preset \"\""),
            ("octane,unknown", "unknown body preset \"unknown\""),
            ("octane, hybrid,", "unknown body preset \"\""),
        ] {
            let parsed = parse_car_bodies(input).unwrap_err();
            assert!(parsed.contains(expected_error), "{input:?}: {parsed}");
        }

        let mut errors = Vec::new();
        for (recording, roster_len, car_slots) in [
            ("three-cars.rlpr", 3, 3),
            ("three-cars.rlpr", 2, 3),
            ("two-cars.rlpr", 3, 2),
        ] {
            let result =
                validate_roster_len(std::path::Path::new(recording), roster_len, car_slots);
            errors.push(result.map_err(|error| error));
        }
        assert_eq!(errors[0], Ok(()));
        assert!(errors[1].as_ref().unwrap_err().contains("three-cars.rlpr"));
        assert!(errors[2].as_ref().unwrap_err().contains("two-cars.rlpr"));
    }

    #[test]
    fn roster_preflight_checks_each_selected_recording() {
        let recordings = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("recordings");
        let selected = [
            recordings.join("wisp_3v3_300s.rlpr.zst"),
            recordings.join("nexto_1v1_300s.rlpr.zst"),
        ];
        let six_car_roster = vec![0; 6];

        let error = validate_roster_for_recordings(&selected, &six_car_roster).unwrap_err();
        assert!(error.contains("nexto_1v1_300s.rlpr.zst"), "{error}");
        assert!(error.contains("2 car slots"), "{error}");
    }

    #[test]
    fn explicit_roster_resolves_shared_octane_hybrid_height_ambiguity() {
        let references: [f32; common::NUM_BODY_PRESETS] =
            [100.0, 102.0, 105.0, 108.0, 100.0, 112.0, 115.0];
        let hybrid_height = references[4];
        let candidates: Vec<_> = references
            .iter()
            .enumerate()
            .filter_map(|(index, height)| {
                ((hybrid_height - height).abs() <= common::SETTLED_HEIGHT_TOL_UU).then_some(index)
            })
            .collect();
        let default_selection =
            common::body_from_settled_height(hybrid_height, &references, Some(0)).unwrap();
        let explicit_roster = parse_car_bodies("octane,hybrid").unwrap();

        assert_eq!(candidates, vec![0, 4]);
        assert_eq!(default_selection, 0);
        assert_eq!(explicit_roster[1], 4);
    }

    #[test]
    fn explicit_car_bodies_are_opt_in() {
        let defaults = Args::try_parse_from(["rlpr_metric"]).unwrap();
        assert!(defaults.car_bodies.is_none());
    }

    #[test]
    fn per_recording_is_opt_in() {
        let defaults = Args::try_parse_from(["rlpr_metric"]).unwrap();
        assert!(!defaults.per_recording);

        let selected =
            Args::try_parse_from(["rlpr_metric", "--per-recording", "wisp.rlpr", "daizen.rlpr"])
                .unwrap();
        assert!(selected.per_recording);
        assert_eq!(selected.rlpr_files.len(), 2);
    }

    #[test]
    fn motion_reset_defaults_off() {
        let defaults = Args::try_parse_from(["rlpr_metric"]).unwrap();
        assert!(!defaults.reset_each_tick);
        assert!(!defaults.reset_motion_each_tick);
        assert_eq!(
            common::ResetMode::from_flags(
                defaults.reset_each_tick,
                defaults.reset_motion_each_tick
            ),
            common::ResetMode::Segmented
        );
    }

    #[test]
    fn motion_reset_flag_maps_to_mode() {
        let full = Args::try_parse_from(["rlpr_metric", "--reset-each-tick"]).unwrap();
        assert!(full.reset_each_tick);
        assert!(!full.reset_motion_each_tick);
        assert_eq!(
            common::ResetMode::from_flags(full.reset_each_tick, full.reset_motion_each_tick),
            common::ResetMode::ResetEachTick
        );
        let motion = Args::try_parse_from(["rlpr_metric", "--reset-motion-each-tick"]).unwrap();
        assert!(!motion.reset_each_tick);
        assert!(motion.reset_motion_each_tick);
        assert_eq!(
            common::ResetMode::from_flags(motion.reset_each_tick, motion.reset_motion_each_tick),
            common::ResetMode::ResetMotionEachTick
        );
    }

    #[test]
    fn reset_flags_conflict() {
        let result = Args::try_parse_from([
            "rlpr_metric",
            "--reset-each-tick",
            "--reset-motion-each-tick",
        ]);
        assert!(result.is_err());
        let err = result.err().unwrap();
        let message = err.to_string();
        assert!(
            message.contains("--reset-each-tick") && message.contains("--reset-motion-each-tick"),
            "conflict error names both flags: {message}"
        );
    }
}
