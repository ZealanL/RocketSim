//! Deterministic scene used for screenshot review and determinism gates.
//!
//! Fixed seed-0 Soccar scene (blue drives, orange idles), 600 ticks, prints
//! ball/car0 checksums. Later stages attach the NetVis streamer here.
//!
//! Run with: `cargo run -p rocketsim_vis --example photo`

use rocketsim_vis::photo::{build_arena, init_meshes, step_ticks};

fn main() {
    init_meshes();
    let mut arena = build_arena();
    for _ in 0..600 {
        step_ticks(&mut arena, 1);
        // Pacing only (does not affect sim): lets a future viewer keep up.
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
}
