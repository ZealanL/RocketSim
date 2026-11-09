//! Segmented replay metric for RLPR recordings.
//!
//! Reset at each segment start, run open-loop, score every tick after.
//! Print one table row per backend and contact category.

use std::path::PathBuf;

use clap::Parser;
use rocketsim_test::rlpr::Recording;

#[path = "rlpr_metric/common/mod.rs"]
mod common;
#[path = "rlpr_metric/metadata.rs"]
mod metadata;
#[cfg(feature = "v2")]
#[path = "rlpr_metric/v2.rs"]
mod v2;
#[path = "rlpr_metric/v3/mod.rs"]
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

    /// Explicit team for each car slot, in recording order.
    /// Use comma-separated names: blue,orange. The values 0 and 1 are also accepted.
    /// The list must contain exactly one team per car in every selected recording.
    #[arg(long, value_name = "TEAM,TEAM,...")]
    car_teams: Option<String>,

    /// Ticks per segment.
    #[arg(long, default_value_t = 120)]
    segment_ticks: usize,

    /// Reset to the recorded state of the prior tick before each scored tick.
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
///
/// Both backends replay the same set, including sparse v11 captures.
/// Unavailable physics is never scored.
const DEFAULT_RECORDINGS: [&str; 6] = [
    "wisp_3v3_300s.rlpr.zst",
    "daizen_2v2_300s.rlpr.zst",
    "partycannon_3v3_300s.rlpr.zst",
    "nexto_1v1_300s.rlpr.zst",
    "veai_3v3_320s_rl278.rlpr.zst",
    "ripple_3v3_600s_rl278.rlpr.zst",
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

fn validate_teams_len(
    recording: &std::path::Path,
    roster_len: usize,
    car_slots: usize,
) -> Result<(), String> {
    if roster_len != car_slots {
        return Err(format!(
            "{}: --car-teams has {roster_len} entries but the recording has {car_slots} car slots",
            recording.display()
        ));
    }
    Ok(())
}

fn validate_teams_for_recordings(
    recordings: &[PathBuf],
    teams: &[rocketsim::Team],
) -> Result<(), String> {
    for path in recordings {
        let recording =
            Recording::from_file(path).map_err(|err| format!("{}: {err}", path.display()))?;
        let ticks = common::collapse_duplicate_ticks(&recording.ticks);
        let car_slots = ticks.first().map(common::tick_car_count).unwrap_or(0);
        validate_teams_len(path, teams.len(), car_slots)?;
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
fn print_skip_counts(kickoff_stasis: usize, skipped_car_ticks: usize) {
    println!("{}", skip_counts_line(kickoff_stasis, skipped_car_ticks));
}

/// One skip line with transitions and their car-tick cost.
fn skip_counts_line(kickoff_stasis: usize, skipped_car_ticks: usize) -> String {
    format!(
        "Kickoff stasis ({}): {} transitions ({} car-ticks) stepped but unscored.",
        common::KICKOFF_STASIS_RULE,
        kickoff_stasis,
        skipped_car_ticks,
    )
}

/// One tolerance line. Position uses two decimals so one-tick budgets show
/// their actual 0.02 uu value.
fn tolerance_line(mode: common::ResetMode, tol: &common::Tolerances) -> String {
    format!(
        "Tolerances: pos {:.2} uu, vel {:.1} uu/s, ang vel {:.2} rad/s, axis {:.2}{}",
        tol.pos_uu,
        tol.vel_uu_s,
        tol.ang_vel_rad_s,
        tol.axis,
        if mode != common::ResetMode::Segmented {
            " (one-tick budgets)"
        } else {
            ""
        },
    )
}

/// Print one availability line with an explicit unit name.
///
/// `kind` is `capture` for the recording rows before duplicate collapse, or
/// `evaluated` for the deduplicated ticks that segmentation sees. v2-v10
/// never call this, so their output is unchanged.
fn print_availability_counts(kind: &str, label: &str, counts: &common::AvailabilityCounts) {
    let car_ticks = counts.available_car_ticks + counts.unavailable_car_ticks;
    println!(
        "{label}: {kind} availability rows {} car-ticks {} available {} unavailable {} no-physics {} ball-unavailable {} mask-changes {} coverage {:.4}%",
        counts.rows,
        car_ticks,
        counts.available_car_ticks,
        counts.unavailable_car_ticks,
        counts.no_physics_rows,
        counts.ball_unavailable_rows,
        counts.mask_changes,
        counts.coverage_percent(),
    );
}

/// Print raw capture availability for an RLPR v11 recording.
///
/// These counts come from the recording rows before duplicate collapse, so
/// they describe the file. Coverage is available car-ticks over all raw
/// car-ticks. Do not report this value as evaluated or scored coverage.
fn print_capture_availability(label: &str, counts: &common::AvailabilityCounts) {
    print_availability_counts("capture", label, counts);
}

/// Print evaluated availability for an RLPR v11 recording.
///
/// These counts come from the deduplicated ticks that segmentation sees.
/// Coverage is available car-ticks over all evaluated car-ticks. This value
/// is not the capture coverage.
fn print_evaluated_availability(label: &str, counts: &common::AvailabilityCounts) {
    print_availability_counts("evaluated", label, counts);
}

/// Print the v12 field coverage for one recording.
///
/// The count line includes zero fields. The applied and skipped lines name
/// the requested fields that the harness uses or leaves out.
fn print_v12_restoration(report: &common::V12Report) {
    if !report.has_extension() {
        return;
    }
    println!("{}", report.coverage_line());
    if report.collision_unreadable_rows() > 0 {
        println!(
            "v3 v12 collision snapshot present but unreadable: {} rows (unknown schema; not restored)",
            report.collision_unreadable_rows()
        );
    }
    let applied = report.applied_names();
    println!(
        "v3 v12 applied: {}",
        if applied.is_empty() {
            "none".to_string()
        } else {
            applied.join(", ")
        }
    );
    let partial: Vec<_> = common::V12Field::ALL
        .iter()
        .filter(|field| field.partially_applied() && report.count(**field) > 0)
        .map(|field| field.name())
        .collect();
    if !partial.is_empty() {
        println!(
            "v3 v12 partial or derived: {} (write counts follow evaluation)",
            partial.join(", ")
        );
        println!(
            "Jump active/time use the base row. Inactive time has no separate core field. Auto-flip timers and airborne clocks need a valid prefix. Pad cooldowns are derived. Collision restoration covers dynamic-body geometry only."
        );
    }
    let not_applied = report.not_applied();
    if !not_applied.is_empty() {
        let names: Vec<String> = not_applied
            .iter()
            .map(|(name, count)| format!("{name} ({count})"))
            .collect();
        println!("v3 v12 recorded but not applied: {}", names.join(", "));
    }
    let absent = report.absent_names();
    if !absent.is_empty() {
        println!("v3 v12 absent: {}", absent.join(", "));
    }
    #[cfg(feature = "v2")]
    println!("v2 v12 applied: none (v2 mapping not implemented)");
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
    if report.total.non_finite > 0 {
        println!(
            "{backend}: non-finite simulated errors {} (failed samples; mean and maximum are infinite)",
            report.total.non_finite
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

    let rlpr_files = resolve_recordings(&args.rlpr_files)?;
    let explicit_bodies = args
        .car_bodies
        .as_deref()
        .map(parse_car_bodies)
        .transpose()?;
    if let Some(bodies) = &explicit_bodies {
        validate_roster_for_recordings(&rlpr_files, bodies)?;
    }
    let explicit_teams = args
        .car_teams
        .as_deref()
        .map(metadata::parse_teams)
        .transpose()?;
    if let Some(teams) = &explicit_teams {
        validate_teams_for_recordings(&rlpr_files, teams)?;
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
    if reset_mode == common::ResetMode::ResetMotionEachTick {
        println!(
            "The v12 report describes the segment seed. Per-tick motion resets restore motion only; the seed body state does not survive them."
        );
    }
    println!("Checkpoint: partial recorded state, not a full native checkpoint.");
    println!("History restores use state-row or earlier evidence, never a target outcome.");
    println!("Categories overlap. Support counts car-ticks (cars x ticks).");
    println!(
        "no_contact means no labeled contact anywhere in the tick, including the shared ball."
    );
    println!("Dodge deadzone: {:.2}", args.dodge_deadzone);
    let tol = common::Tolerances::for_reset_mode(reset_mode);
    println!("{}", tolerance_line(reset_mode, &tol));
    // Mesh loading and settled heights are process-wide. Do this once.
    v3::init();
    let settled_heights = v3::preset_settled_heights();
    #[cfg(feature = "v2")]
    v2::init();
    let mut combined_v3 = common::EvalReport::default();
    #[cfg(feature = "v2")]
    let mut combined_v2 = common::EvalReport::default();
    let mut skipped_transitions = 0usize;
    let mut skipped_car_ticks = 0usize;
    let mut combined_availability = common::AvailabilityCounts::default();
    let mut combined_capture_availability = common::AvailabilityCounts::default();
    let mut any_availability = false;

    for rlpr_file in rlpr_files.iter() {
        let recording = Recording::from_file(rlpr_file)
            .map_err(|err| format!("{}: {err}", rlpr_file.display()))?;
        println!(
            "{}: RLPR v{} SHA-256 {}",
            rlpr_file.display(),
            recording.version,
            metadata::file_sha256(rlpr_file)?,
        );
        common::validation::validate_recorded_state(&recording.ticks)
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

        let loaded_metadata = metadata::load(rlpr_file, num_cars)
            .map_err(|err| format!("{}: {err}", rlpr_file.display()))?;
        let (bodies, body_source, header_body) = match (&explicit_bodies, &loaded_metadata) {
            (Some(bodies), _) => (bodies.clone(), "cli".to_string(), None),
            (None, Some(loaded)) => (
                loaded.car_bodies.clone(),
                format!("metadata {}", loaded.path.display()),
                None,
            ),
            (None, None) => {
                // The header carries one hitbox for the whole file, so it only
                // describes car 0. Settled heights detect wheel geometry but
                // cannot separate bodies with shared wheel geometry.
                let header_body = v3::body_preset_index(&recording.info)
                    .map_err(|err| format!("{}: {err}", rlpr_file.display()))?;
                let bodies = common::bodies_from_ticks(&ticks, header_body, &settled_heights)
                    .map_err(|err| format!("{}: {err}", rlpr_file.display()))?;
                (bodies, "inferred".to_string(), Some(header_body))
            }
        };
        let (teams, team_source) = match (&explicit_teams, &loaded_metadata) {
            (Some(teams), _) => (teams.clone(), "cli".to_string()),
            (None, Some(loaded)) => (
                loaded.car_teams.clone(),
                format!("metadata {}", loaded.path.display()),
            ),
            (None, None) => (
                metadata::alternating_teams(num_cars),
                "alternating fallback".to_string(),
            ),
        };
        // Build the arena from scratch for each recording. A shared arena
        // would carry boost pads, contact manifolds, and tick history from
        // the previous file even when the car roster stays the same.
        let mut v3_backend = v3::V3Backend::with_dodge_deadzone(args.dodge_deadzone);
        v3_backend.set_bodies(&bodies);
        v3_backend.set_teams(&teams);
        #[cfg(feature = "v2")]
        let mut v2_backend = {
            let mut backend = v2::V2Backend::with_dodge_deadzone(args.dodge_deadzone);
            backend.set_bodies(&bodies);
            backend.set_teams_from_sides(&teams);
            backend
        };
        println!(
            "{}: bodies {:?} (source: {})",
            rlpr_file.display(),
            bodies
                .iter()
                .map(|&index| common::BODY_PRESET_NAMES[index])
                .collect::<Vec<_>>(),
            body_source,
        );
        println!(
            "{}: teams {:?} (source: {})",
            rlpr_file.display(),
            metadata::team_names(&teams),
            team_source,
        );
        if explicit_teams.is_none() && loaded_metadata.is_none() {
            println!(
                "{}: WARNING: teams are unverified; using alternating Blue/Orange by slot. Add {} with car_teams or pass --car-teams.",
                rlpr_file.display(),
                metadata::metadata_path(rlpr_file).display(),
            );
        }
        if let Some(header_body) = header_body {
            let fallback = common::header_fallback_slots(&ticks);
            if !fallback.is_empty() {
                println!(
                    "{}: WARNING: car slot(s) {:?} never settled; body {} comes from the car-0 header, not a measured per-car body",
                    rlpr_file.display(),
                    fallback,
                    common::BODY_PRESET_NAMES[header_body],
                );
            }
        }
        print_v12_restoration(&common::v12_report(&ticks));
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

        // Reporting only. The counts do not read the backend and do not
        // change evaluation order or scoring.
        println!(
            "{}: {}",
            rlpr_file.display(),
            common::transition_report(&ticks, &segments)
        );

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
        v3_backend.print_restore_counts();
        combined_v3.merge(&v3_outcome.report);
        skipped_transitions += v3_outcome.skipped_transitions;
        skipped_car_ticks += v3_outcome.skipped_car_ticks;
        if rocketsim_test::rlpr::recording_has_availability(recording.version) {
            let label = rlpr_file.display().to_string();
            // Raw capture counts: the rows as written, before duplicate
            // collapse. These describe the file, not the scored set.
            let capture_availability = common::availability_counts(&recording.ticks);
            print_capture_availability(&label, &capture_availability);
            // Evaluated counts: the deduplicated ticks segmentation sees.
            print_evaluated_availability(&label, &v3_outcome.availability);
            combined_capture_availability.merge(&capture_availability);
            combined_availability.merge(&v3_outcome.availability);
            any_availability = true;
        }

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
            print_skip_counts(v3_outcome.skipped_transitions, v3_outcome.skipped_car_ticks);
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
    print_skip_counts(skipped_transitions, skipped_car_ticks);
    if any_availability {
        print_capture_availability("Combined", &combined_capture_availability);
        print_evaluated_availability("Combined", &combined_availability);
    }
    print_table_header(false);
    print_report("v3", &combined_v3, false);
    #[cfg(feature = "v2")]
    {
        println!();
        print_report("v2", &combined_v2, false);
    }

    if combined_v3.total.non_finite > 0 {
        return Err("v3 produced non-finite simulated errors; see failed-sample counts".into());
    }
    #[cfg(feature = "v2")]
    if combined_v2.total.non_finite > 0 {
        return Err("v2 produced non-finite simulated errors; see failed-sample counts".into());
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
    fn default_recordings_match_backend_capability() {
        let has_v11 = DEFAULT_RECORDINGS
            .iter()
            .any(|name| name.starts_with("ripple_3v3_600s"));
        assert_eq!(DEFAULT_RECORDINGS.len(), 6);
        assert!(has_v11, "both backends include the sparse v11 capture");
        let explicit = vec![PathBuf::from("selected.rlpr")];
        assert_eq!(resolve_recordings(&explicit).unwrap(), explicit);
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

    #[test]
    fn tolerance_line_shows_the_actual_one_tick_position_budget() {
        let reset = tolerance_line(
            common::ResetMode::ResetEachTick,
            &common::Tolerances::RESET_EACH_TICK,
        );
        assert!(reset.contains("pos 0.02 uu"), "{reset}");
        assert!(reset.contains("one-tick budgets"), "{reset}");

        let segmented = tolerance_line(common::ResetMode::Segmented, &common::Tolerances::SEGMENT);
        assert!(segmented.contains("pos 2.00 uu"), "{segmented}");
        assert!(!segmented.contains("one-tick budgets"), "{segmented}");
    }

    #[test]
    fn skip_line_reports_stasis_car_ticks() {
        let line = skip_counts_line(3, 9);
        assert!(line.contains("3 transitions"), "{line}");
        assert!(line.contains("9 car-ticks"), "{line}");
        assert!(line.contains("stepped but unscored"), "{line}");
    }

    #[test]
    fn car_teams_parse_and_require_exact_slots() {
        assert_eq!(
            metadata::parse_teams("blue,ORANGE,1,0").unwrap(),
            vec![
                rocketsim::Team::Blue,
                rocketsim::Team::Orange,
                rocketsim::Team::Orange,
                rocketsim::Team::Blue,
            ]
        );
        let error = validate_teams_len(std::path::Path::new("x.rlpr"), 3, 4).unwrap_err();
        assert!(error.contains("--car-teams"), "{error}");
        assert!(error.contains("4 car slots"), "{error}");
    }
}
