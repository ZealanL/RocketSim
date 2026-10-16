//! Viewer per-frame CPU microbenchmark (headless, no GL).
//!
//! Mirrors the `rocketsim-viewer` update body: frame clone, ball/boost trail
//! ribbons, ground line, [`CarFx`](rocketsim_viewer::fx::CarFx) effects,
//! follow-camera smoothing, view/projection, frustum planes, pad instance
//! build, car draw grouping, and CPU line expansion. Runs entirely on
//! snapshots so results are comparable across machines and commits.
//!
//! Run with: `cargo run -r -p rocketsim_viewer --example bench_frame`
//! (`perf record` this binary for hotspot traces.)

use std::{hint::black_box, time::Instant};

use glam::{Mat3, Vec3, Vec3A};
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};
use rocketsim_viewer::{
    camera::{CameraConfig, CarCamSmoothing},
    frustum::frustum_planes,
    fx::CarFx,
    lines::{RibbonConfig, RibbonEmitter, Segment, expand_segment, ground_line},
    main_gl::{car_model_name, car_texture_name, group_car_draws, model_matrix},
    pads::{build_instances, load_pad_meshes, variant_radius},
    view::view_proj,
};
use rocketsim_vis::{photo::init_meshes, proto::VisFrame};

/// 3v3 with mixed bodies (matches a real match for grouping/culling).
const BODIES: [CarBodyConfig; 6] = [
    CarBodyConfig::OCTANE,
    CarBodyConfig::DOMINUS,
    CarBodyConfig::BREAKOUT,
    CarBodyConfig::MERC,
    CarBodyConfig::OCTANE,
    CarBodyConfig::DOMINUS,
];

/// Ticks of pre-roll play before capturing (cars spread out, ball moving).
const PREROLL_TICKS: usize = 600;
/// Consecutive frames captured for the cyclic workload.
const CAPTURED_FRAMES: usize = 240;
/// Benchmark iterations (each = one viewer frame of CPU work).
const ITERS: usize = 3000;
/// 720p aspect (the CPU-limited case from the report).
const ASPECT: f32 = 1280.0 / 720.0;

/// Floats per packed corner in [`LinesGl`](rocketsim_viewer::lines_gl::LinesGl)
/// upload: world(3) + pa(3) + pb(3) + color(4) + width(1).
const PACK_STRIDE: usize = 14;
/// Segment cap mirrored from the lines pipeline.
const PACK_MAX: usize = 1024;

/// Mirrors the CPU half of `LinesGl::upload` (no GL): skip invisible,
/// truncate past the cap, expand every kept segment, pack corners into a
/// float buffer. Returns (floats, truncated, checksum).
fn pack_lines(segs: &[Segment], eye: Vec3A, view: &glam::Mat4, proj11: f32) -> (usize, usize, f64) {
    let mut buf = Vec::new();
    let mut kept = 0usize;
    for seg in segs {
        if seg.is_invisible() {
            continue;
        }
        if kept >= PACK_MAX {
            break;
        }
        for corner in expand_segment(seg, eye, view, proj11, 720.0) {
            buf.extend_from_slice(&corner.world.to_array());
            buf.extend_from_slice(&seg.a.to_array());
            buf.extend_from_slice(&seg.b.to_array());
            buf.extend_from_slice(&corner.color.to_array());
            buf.push(corner.width);
        }
        kept += 1;
    }
    let over = segs
        .iter()
        .filter(|s| !s.is_invisible())
        .count()
        .saturating_sub(kept);
    let mut acc = 0.0f64;
    for (i, v) in buf.iter().enumerate() {
        if i % PACK_STRIDE == 0 {
            acc += *v as f64;
        }
    }
    black_box(&buf);
    (buf.len(), over, acc)
}

fn controls_for_tick(car: usize, tick: usize) -> CarControls {
    CarControls {
        throttle: if tick % 240 < 200 { 1.0 } else { -0.5 },
        steer: if car.is_multiple_of(2) { 0.4 } else { -0.4 },
        boost: tick % 120 < 60,
        jump: tick % 97 == car * 13 % 97,
        ..CarControls::default()
    }
}

fn build_frames() -> Vec<VisFrame> {
    init_meshes();
    let mut arena = Arena::new_with_config(ArenaConfig {
        rng_seed: Some(0),
        ..ArenaConfig::new(GameMode::Soccar)
    });
    for (i, body) in BODIES.iter().enumerate() {
        arena.add_car(if i % 2 == 0 { Team::Blue } else { Team::Orange }, *body);
    }
    arena.reset_to_random_kickoff(Some(0));
    for tick in 0..PREROLL_TICKS {
        for (i, _) in BODIES.iter().enumerate() {
            arena.set_car_controls(i, controls_for_tick(i, tick));
        }
        arena.step_tick();
    }
    let mut frames = Vec::with_capacity(CAPTURED_FRAMES);
    for tick in 0..CAPTURED_FRAMES {
        for (i, _) in BODIES.iter().enumerate() {
            arena.set_car_controls(i, controls_for_tick(i, PREROLL_TICKS + tick));
        }
        arena.step_tick();
        frames.push(VisFrame::new(arena.get_arena_state(), 1.0 / 120.0));
    }
    frames
}

/// One viewer frame of CPU work; returns a checksum sink.
#[allow(clippy::too_many_arguments)]
fn frame_work(
    frame: &VisFrame,
    ball_ribbon: &mut RibbonEmitter,
    boost_ribbons: &mut Vec<RibbonEmitter>,
    fx: &mut CarFx,
    smooth: &mut CarCamSmoothing,
    cfg: &CameraConfig,
    pad_radii: &[f32; 4],
    eye_in: Vec3A,
) -> (f64, Vec3A) {
    // Trail replay (viewer replays 1 step; missed ticks clamp at 10).
    let ball_pos = frame.state.ball.phys.pos;
    ball_ribbon.update(true, ball_pos, Vec3A::ZERO, frame.dt);
    let mut segs = ball_ribbon.segments(ball_pos);
    segs.push(ground_line(ball_pos, -100.0));
    segs.extend(fx.update(&frame.state, frame.dt));
    for (i, (_, car)) in frame.state.cars.iter().enumerate() {
        while boost_ribbons.len() <= i {
            boost_ribbons.push(RibbonEmitter::new(RibbonConfig::boost()));
        }
        let fwd = car.phys.get_forward_dir();
        let emit_pos = car.phys.pos - fwd * 60.0;
        let emit_vel = -fwd * 300.0;
        boost_ribbons[i].update(car.is_boosting, emit_pos, emit_vel, frame.dt);
        segs.extend(boost_ribbons[i].segments(emit_pos));
    }
    // Follow camera + projection + frustum.
    let mut pos = eye_in;
    let mut dir = Vec3A::X;
    smooth.update(0, true, &frame.state, cfg, &mut pos, &mut dir, 1.0 / 60.0);
    let (view, proj) = view_proj(
        Vec3::from(pos),
        Vec3::from(pos + dir),
        smooth.cur_fov_h(),
        ASPECT,
    );
    let planes = frustum_planes(proj * view);
    let (instances, _culled) = build_instances(&frame.state.boost_pads, &planes, pad_radii);
    let inst_total: usize = instances.iter().map(|v| v.len()).sum();
    // Car draw grouping (per body+team batches + model matrices).
    let mut placements = Vec::with_capacity(frame.state.cars.len());
    for (info, state) in &frame.state.cars {
        if state.is_demoed {
            continue;
        }
        placements.push((
            car_model_name(&info.config),
            car_texture_name(info.team),
            model_matrix(Vec3::from(state.phys.pos), Mat3::from(state.phys.rot_mat)),
        ));
    }
    let groups = group_car_draws(&placements);
    // Lines overlay path: expand every segment + pack the upload buffer
    // (mirrors the CPU half of `LinesGl::upload` — the viewer's only
    // expansion of these segments).
    let (packed, _trunc, pack_acc) = pack_lines(&segs, pos, &view, proj.y_axis.y);
    let mut acc = pack_acc;
    // Sink the batching work too (else the optimizer deletes it).
    acc += (inst_total + packed) as f64
        + groups.iter().map(|g| g.matrices.len()).sum::<usize>() as f64;
    black_box(acc);
    (acc, pos)
}

/// Forces visible effects edges into a cloned frame (jump/dodge/latch),
/// so the fx-heavy phase exercises dust, rings, and the latched glow.
fn force_fx_edges(frame: &mut VisFrame, k: usize) {
    use rocketsim::{RaycastHitInfo, UserInfoType};
    let n = frame.state.cars.len();
    if n == 0 {
        return;
    }
    // Car 0: jump edge every 30 iters (dust burst).
    frame.state.cars[0].1.is_jumping = k % 30 < 15;
    // Car 1: dodge edge every 60 iters (orange ring).
    if n > 1 {
        frame.state.cars[1].1.is_flipping = k % 60 < 30;
    }
    // Car 2: latched reset glow every 120 iters (4 wheel rings + flash).
    if n > 2 {
        let touch = k % 120 < 60;
        let hit = touch.then_some(RaycastHitInfo {
            user_info: UserInfoType::Ball,
            ..Default::default()
        });
        frame.state.cars[2].1.wheels_with_contact = [hit; 4];
    }
}

/// Times one workload phase; prints per-frame ns and fps.
#[allow(clippy::too_many_arguments)]
fn run_phase(
    name: &str,
    frames: &[VisFrame],
    iters: usize,
    fx_heavy: bool,
    ball_ribbon: &mut RibbonEmitter,
    boost_ribbons: &mut Vec<RibbonEmitter>,
    fx: &mut CarFx,
    smooth: &mut CarCamSmoothing,
    cfg: &CameraConfig,
    pad_radii: &[f32; 4],
    eye: &mut Vec3A,
) {
    let mut sink = 0.0f64;
    let t0 = Instant::now();
    for k in 0..iters {
        // Mirror `FrameClient::latest()`: one full clone per viewer frame.
        let mut frame = black_box(frames[k % frames.len()].clone());
        if fx_heavy {
            force_fx_edges(&mut frame, k);
        }
        let (a, e) = frame_work(
            &frame,
            ball_ribbon,
            boost_ribbons,
            fx,
            smooth,
            cfg,
            pad_radii,
            *eye,
        );
        sink += a;
        *eye = e;
    }
    let dt = t0.elapsed();
    let ns = dt.as_nanos() as f64 / iters as f64;
    println!(
        "bench_frame[{name}]: iters={iters} total={dt:?} per_frame={ns:.0}ns fps={:.0}",
        1e9 / ns
    );
    println!("bench_frame[{name}]: sink={sink:.3} (do not optimize away)");
}

fn main() {
    let iters: usize = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(ITERS);
    let frames = build_frames();
    let pad_meshes = load_pad_meshes();
    let pad_radii = [
        variant_radius(&pad_meshes[0]),
        variant_radius(&pad_meshes[1]),
        variant_radius(&pad_meshes[2]),
        variant_radius(&pad_meshes[3]),
    ];
    let cfg = CameraConfig::default();

    // Warm up (ribbons reach steady state, like a live viewer).
    let mut ball_ribbon = RibbonEmitter::new(RibbonConfig::ball_trail());
    let mut boost_ribbons = Vec::new();
    let mut fx = CarFx::default();
    let mut smooth = CarCamSmoothing::new();
    let mut eye = Vec3A::new(0.0, -3000.0, 1500.0);
    let mut sink = 0.0f64;
    for f in &frames {
        let (a, e) = frame_work(
            f,
            &mut ball_ribbon,
            &mut boost_ribbons,
            &mut fx,
            &mut smooth,
            &cfg,
            &pad_radii,
            eye,
        );
        sink += a;
        eye = e;
    }
    black_box(sink);
    // Steady-state segment load (what each timed frame chews through).
    let probe = frames[frames.len() - 1].clone();
    let mut segs_probe: Vec<Segment> = ball_ribbon.segments(probe.state.ball.phys.pos);
    for (i, (_, car)) in probe.state.cars.iter().enumerate() {
        let fwd = car.phys.get_forward_dir();
        segs_probe.extend(boost_ribbons[i].segments(car.phys.pos - fwd * 60.0));
    }
    eprintln!(
        "bench_frame: warm segments/frame={} cars={}",
        segs_probe.len(),
        probe.state.cars.len()
    );

    run_phase(
        "steady",
        &frames,
        iters,
        false,
        &mut ball_ribbon,
        &mut boost_ribbons,
        &mut fx,
        &mut smooth,
        &cfg,
        &pad_radii,
        &mut eye,
    );
    run_phase(
        "fxheavy",
        &frames,
        iters.max(3000) / 3,
        true,
        &mut ball_ribbon,
        &mut boost_ribbons,
        &mut fx,
        &mut smooth,
        &cfg,
        &pad_radii,
        &mut eye,
    );
}
