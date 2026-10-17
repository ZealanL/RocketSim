//! Deterministic scene used for screenshot review and determinism gates.
//!
//! Fixed seed-0 Soccar scene (blue drives, orange idles), 600 ticks,
//! streams to the viewer and prints ball/car0 checksums.
//!
//! Run with: `cargo run -p rocketsim_viewer --example photo`

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
        // Pacing only (not sim): lets the 60Hz viewer sample the 120Hz stream.
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
    // Keep streaming after the checksum so the viewer remains attached.
    loop {
        step_ticks(&mut arena, 1);
        std::thread::sleep(std::time::Duration::from_millis(8));
    }
}
