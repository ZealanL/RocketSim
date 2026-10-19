//! Retained debug-render API demo. Use `--drive` to explore a Soccar arena.
//!
//! Example: `cargo run --locked -p rocketsim_vis --example debug_render -- --addr 127.0.0.1:34254 --seconds 20 --mode showcase`

#[path = "support/debug_render/mod.rs"]
mod debug_render_support;
#[path = "support/drive.rs"]
mod drive_controls;
#[path = "support/photo.rs"]
mod photo_support;
mod support;

use std::{
    fs,
    net::SocketAddr,
    path::PathBuf,
    time::{Duration, Instant},
};

use clap::{Parser, ValueEnum};
use drive_controls::Driver;
use glam::Vec3A;
use photo_support::init_meshes;
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, GameMode, Team};
use rocketsim_vis::{ArenaVisExt, pace::TickPacer};

#[derive(Clone, Copy, Debug, ValueEnum)]
enum Mode {
    Wire,
    Solid,
    Showcase,
    Stress,
}

#[derive(Parser)]
#[command(name = "debug_render", version)]
struct Cli {
    /// Connect to an existing viewer. Optional with --drive.
    #[arg(long, required_unless_present = "drive")]
    addr: Option<SocketAddr>,
    /// Stop after this many seconds. Default: 15, or no limit with --drive.
    #[arg(long, value_parser = parse_duration)]
    seconds: Option<Duration>,
    /// Add a drivable Octane and use a Soccar arena. Requires collision meshes.
    #[arg(long)]
    drive: bool,
    #[arg(long, value_enum, default_value_t = Mode::Showcase)]
    mode: Mode,
    #[arg(long, default_value_t = 10000)]
    count: usize,
    #[arg(long)]
    obj: Option<PathBuf>,
    #[arg(long = "static")]
    static_scene: bool,
    #[arg(long)]
    original_placement: bool,
    #[arg(long)]
    benchmark_output: Option<PathBuf>,
}

impl Cli {
    fn run_duration(&self) -> Option<Duration> {
        self.seconds
            .or_else(|| (!self.drive).then(|| Duration::from_secs(15)))
    }
}

fn parse_duration(text: &str) -> Result<Duration, String> {
    let seconds = text
        .parse::<f64>()
        .map_err(|_| "--seconds must be positive and finite".to_owned())?;
    Duration::try_from_secs_f64(seconds)
        .ok()
        .filter(|duration| !duration.is_zero())
        .ok_or_else(|| "--seconds must be positive, finite, and within range".to_owned())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    if cli.count > rocketsim_vis::debug::MAX_TOTAL_ITEMS {
        return Err(format!(
            "--count cannot exceed {}",
            rocketsim_vis::debug::MAX_TOTAL_ITEMS
        )
        .into());
    }
    let _viewer = if cli.addr.is_none() {
        support::prepare_viewer()?
    } else {
        None
    };
    let mode = if cli.drive {
        init_meshes();
        GameMode::Soccar
    } else {
        GameMode::TheVoid
    };
    let mut arena = Arena::new_with_config(ArenaConfig::new(mode).with_rng_seed(0));
    let mut driver = cli.drive.then(|| {
        let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        arena.reset_to_random_kickoff(None);
        Driver::new(car)
    });
    if let Some(addr) = cli.addr {
        arena.try_set_net_vis(Some(&addr.to_string()))?;
    } else {
        arena.try_set_vis_enabled(true)?;
    }
    if cli.drive {
        println!("Close settings with Escape. Press 1 to follow and drive the Octane.");
        println!("Press F for free camera. Use W/A/S/D and the mouse to move.");
        println!("Reset, dribble, and launch use Backspace, F2, and F4 by default.");
        println!("Debug geometry has no collision. Press Ctrl+C in this terminal to stop.");
    }
    let renderer = arena
        .debug_renderer()
        .ok_or("debug renderer unavailable after attach")?;
    let build_start = Instant::now();
    let mut mesh_submit_ms = 0.0;
    match cli.mode {
        Mode::Showcase => debug_render_support::showcase::showcase(&renderer, &mut mesh_submit_ms)?,
        Mode::Wire | Mode::Solid => {
            if let Some(path) = &cli.obj {
                debug_render_support::obj::render_obj(
                    &renderer,
                    path,
                    matches!(cli.mode, Mode::Solid),
                    cli.original_placement,
                    &mut mesh_submit_ms,
                )?;
            } else {
                return Err("--obj PATH is required for wire and solid modes".into());
            }
        }
        Mode::Stress => {
            debug_render_support::stress::render(&renderer, cli.count, &mut mesh_submit_ms)?
        }
    }
    let build_ms = build_start.elapsed().as_secs_f64() * 1000.0;
    let scene = renderer.snapshot();
    let scene_items: usize = scene.groups.values().map(|g| g.items.len()).sum();
    let estimated_bytes = scene
        .meshes
        .values()
        .map(|mesh| mesh.estimated_bytes())
        .sum::<usize>()
        + scene
            .textures
            .values()
            .map(|texture| texture.estimated_bytes())
            .sum::<usize>()
        + scene
            .groups
            .values()
            .map(|g| g.estimated_bytes())
            .sum::<usize>();
    let mesh_bytes: usize = scene
        .meshes
        .values()
        .map(|mesh| mesh.estimated_bytes())
        .sum();
    let texture_bytes: usize = scene
        .textures
        .values()
        .map(|texture| texture.estimated_bytes())
        .sum();
    let mut animated_groups: Vec<String> = match cli.mode {
        Mode::Showcase => vec!["orbiters".into(), "knot".into()],
        Mode::Solid => vec!["obj_solid".into()],
        Mode::Wire => scene
            .groups
            .keys()
            .filter(|name| name.starts_with("obj_wire_"))
            .cloned()
            .collect(),
        Mode::Stress => scene
            .groups
            .keys()
            .filter(|name| name.starts_with("stress_instances_"))
            .cloned()
            .collect(),
    };
    animated_groups.sort();
    eprintln!("READY {} groups, {scene_items} items", scene.groups.len());
    let start = Instant::now();
    let deadline = cli
        .run_duration()
        .map(|duration| start.checked_add(duration).ok_or("--seconds is too large"))
        .transpose()?;
    let mut pacer = TickPacer::at_120hz();
    let mut frames = 0u64;
    let mut update_latencies_ms = Vec::new();
    let mut update_batches = 0u64;
    let mut last_update = Instant::now();
    let mut angle = 0.0f32;
    while deadline.is_none_or(|deadline| Instant::now() < deadline) {
        for _ in 0..pacer.wait() {
            if deadline.is_some_and(|deadline| Instant::now() >= deadline) {
                break;
            }
            if let Some(driver) = &mut driver {
                driver.tick(&mut arena);
            } else {
                arena.edit_ball_state(|ball| {
                    ball.phys.pos = Vec3A::new(0.0, 0.0, 900.0);
                    ball.phys.vel = Vec3A::ZERO;
                });
                arena.step_tick();
            }
            frames += 1;
        }
        if !cli.static_scene
            && last_update.elapsed() >= Duration::from_millis(33)
            && !animated_groups.is_empty()
        {
            angle += 0.035;
            let rotation = debug_render_support::meshes::rotation_z(angle);
            let transform = rocketsim_vis::debug::GroupTransform {
                position: [0.0; 3],
                rotation,
                scale: [1.0; 3],
            };
            let t = Instant::now();
            for name in &animated_groups {
                renderer.set_group_transform(name, transform.clone())?;
            }
            update_latencies_ms.push(t.elapsed().as_secs_f64() * 1000.0);
            update_batches += 1;
            last_update = Instant::now();
        }
    }
    let run_ms = start.elapsed().as_secs_f64() * 1000.0;
    update_latencies_ms.sort_by(f64::total_cmp);
    let percentile = |p: f64| -> f64 {
        if update_latencies_ms.is_empty() {
            0.0
        } else {
            update_latencies_ms[((update_latencies_ms.len() - 1) as f64 * p).ceil() as usize]
        }
    };
    let update_mean_ms = if update_latencies_ms.is_empty() {
        0.0
    } else {
        update_latencies_ms.iter().sum::<f64>() / update_latencies_ms.len() as f64
    };
    let summary = format!(
        "{{\"mode\":\"{:?}\",\"frames\":{},\"groups\":{},\"items\":{},\"mesh_bytes\":{},\"texture_bytes\":{},\"resource_bytes\":{},\"build_ms\":{:.3},\"mesh_submit_ms\":{:.3},\"run_ms\":{:.3},\"update_batches\":{},\"update_mean_ms\":{:.3},\"update_p50_ms\":{:.3},\"update_p95_ms\":{:.3},\"update_max_ms\":{:.3},\"static\":{},\"drive\":{}}}",
        cli.mode,
        frames,
        scene.groups.len(),
        scene_items,
        mesh_bytes,
        texture_bytes,
        estimated_bytes,
        build_ms,
        mesh_submit_ms,
        run_ms,
        update_batches,
        update_mean_ms,
        percentile(0.50),
        percentile(0.95),
        update_latencies_ms.last().copied().unwrap_or(0.0),
        cli.static_scene,
        cli.drive
    );
    println!("SUMMARY {summary}");
    if let Some(path) = cli.benchmark_output {
        fs::write(path, format!("{summary}\n"))?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_modes_set_viewer_and_duration_defaults_and_reject_bad_times() {
        assert!(Cli::try_parse_from(["debug_render"]).is_err());
        let debug = Cli::try_parse_from(["debug_render", "--addr", "127.0.0.1:34254"]).unwrap();
        assert!(!debug.drive);
        assert_eq!(debug.run_duration(), Some(Duration::from_secs(15)));
        let drive = Cli::try_parse_from(["debug_render", "--drive"]).unwrap();
        assert!(drive.addr.is_none());
        assert_eq!(drive.run_duration(), None);
        let timed = Cli::try_parse_from([
            "debug_render",
            "--drive",
            "--addr",
            "127.0.0.1:34254",
            "--seconds",
            "1.5",
            "--static",
        ])
        .unwrap();
        assert!(timed.static_scene);
        assert_eq!(timed.run_duration(), Some(Duration::from_millis(1500)));
        for seconds in ["0", "-1", "NaN", "inf", "1e100", "invalid"] {
            assert!(parse_duration(seconds).is_err(), "{seconds}");
        }
    }
}
