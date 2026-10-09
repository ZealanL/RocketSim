//! Deterministic scene used for screenshot review and determinism gates.
//!
//! Fixed seed-0 Soccar scene (blue drives, orange idles), 600 ticks, streams
//! to the detached viewer and prints ball/car0 checksums.
//!
//! Run with: `cargo run -p rocketsim_vis --example photo`

use rocketsim_vis::{
    ArenaVisExt, ensure_viewer_running,
    photo::{build_arena, init_meshes, step_ticks},
};

fn main() {
    init_meshes();
    let mut arena = build_arena();
    ensure_viewer_running("127.0.0.1:39201");
    arena.set_net_vis_enabled("127.0.0.1:39201");
    for _ in 0..600 {
        step_ticks(&mut arena, 1);
        // Pacing only (does not affect sim): lets the viewer sample ~every
        // other tick at 60Hz against the 120Hz stream.
        std::thread::sleep(std::time::Duration::from_millis(2));
    }
    let ball = arena.get_ball_state().phys.pos;
    let car = arena.get_car_state(0).phys.pos;
    println!(
        "photo tick={} ball=({:.1},{:.1},{:.1}) car0=({:.1},{:.1},{:.1})",
        arena.tick_count(),
        ball.x,
        ball.y,
        ball.z,
        car.x,
        car.y,
        car.z
    );
    // Keep streaming live after the checksum (the viewer holds the last
    // frame, so any capture sees this deterministic scene).
    loop {
        step_ticks(&mut arena, 1);
        std::thread::sleep(std::time::Duration::from_millis(8));
    }
}
