//! Headless performance gates for the detached vis transport.
//!
//! No window, no viewer, no sleeps: synthetic [`ArenaState`]s exercise
//! [`proto`](rocketsim_vis_net::proto) framing sizes/speed plus the
//! [`NetVis`](rocketsim_vis_net::NetVis) hot path (`update` with nobody listening).
//! Everything is deterministic (fixed tick, default states, const pad layout).
//!
//! States include the real 34-pad Soccar layout from
//! [`rocketsim::consts::boost_pads`] (pure const data — no collision meshes
//! needed) so payloads stay representative of real frames: the fixed pad/ball
//! overhead is what makes per-car growth sublinear, which is exactly what the
//! size-ratio gate below pins down.

use std::time::Instant;

use rocketsim::{
    ArenaState, BoostPadConfig, BoostPadState, CarInfo, CarState, GameMode, TileStates, Vis,
    consts::boost_pads::get_locations,
};
use rocketsim_vis_net::{
    NetVis,
    proto::{VisFrame, decode_frame, encode_frame},
};

/// Tick dt, matches `rocketsim::consts::TICK_TIME`.
const DT: f32 = 1.0 / 120.0;
/// Fixed tick stamped on every synthetic state (deterministic varint cost).
const TICK: u64 = 7;
/// Performance gates (µs, mean over `ITERS_*` iterations).
const MAX_UPDATE_US: f64 = 100.0;
const MAX_ENCODE_US: f64 = 200.0;
const ITERS_UPDATE: usize = 1000;
const ITERS_ENCODE: usize = 200;

/// Synthetic Soccar snapshot: `n_cars` default cars + the 34-pad layout.
///
/// `new_empty` gives no cars/pads, so both are filled from defaults/constants.
/// No `Arena` (and no collision meshes) required.
fn dummy_state(n_cars: usize) -> ArenaState {
    let mut s = ArenaState::new_empty(GameMode::Soccar);
    s.tick_count = TICK;
    for i in 0..n_cars {
        s.cars.push((
            CarInfo {
                idx: i,
                ..Default::default()
            },
            CarState::default(),
        ));
    }
    for &pos in get_locations(GameMode::Soccar, false) {
        s.boost_pads.push((
            BoostPadConfig { pos, is_big: false },
            BoostPadState::default(),
        ));
    }
    for &pos in get_locations(GameMode::Soccar, true) {
        s.boost_pads.push((
            BoostPadConfig { pos, is_big: true },
            BoostPadState::default(),
        ));
    }
    s
}

/// Postcard payload bytes for an `n_cars` frame (wire bytes minus the 4-byte
/// length prefix).
fn payload_len(n_cars: usize) -> usize {
    let frame = VisFrame::new(dummy_state(n_cars), DT);
    encode_frame(&frame).len() - 4
}

#[test]
fn payload_size_gates() {
    let p1 = payload_len(1);
    let p2 = payload_len(2);
    let p6 = payload_len(6);
    // Absolute cap: real 6-car frames are ~3 KB; 16 KB is generous headroom
    // for format overhead + future fields.
    assert!(p6 < 16 * 1024, "6-car payload too big: {p6} B");
    // Growth sanity: fixed ball/pad overhead dominates, so tripling the cars
    // (2 -> 6) must cost well under 2x the 2-car payload.
    assert!(
        p6 < 2 * p2,
        "payload grows too fast with car count: 6-car {p6} B vs 2x 2-car {} B",
        2 * p2,
    );
    // Monotonic: more cars => more bytes.
    assert!(
        p1 < p2 && p2 < p6,
        "payloads not monotonic: {p1}, {p2}, {p6}"
    );
}

#[test]
fn netvis_update_no_viewer_is_cheap() {
    let state = dummy_state(6);
    // Port 1 never accepts: the sender thread retries off-thread, so `update`
    // must stay a clone + `try_lock` regardless.
    let mut vis = NetVis::connect("127.0.0.1:1");
    vis.update(&state, DT); // warm up (slot alloc, thread spawn)
    let t0 = Instant::now();
    for _ in 0..ITERS_UPDATE {
        vis.update(&state, DT);
    }
    let mean_us = t0.elapsed().as_secs_f64() * 1e6 / ITERS_UPDATE as f64;
    assert!(
        mean_us < MAX_UPDATE_US,
        "NetVis::update mean too slow with no viewer: {mean_us:.2} µs (gate: {MAX_UPDATE_US} µs)"
    );
    // `vis` drops here; the sender thread is mid-backoff so join is fast.
}

#[test]
fn encode_frame_6car_is_fast() {
    let frame = VisFrame::new(dummy_state(6), DT);
    let _ = encode_frame(&frame); // warm up (allocator, first serialize)
    let mut total_len = 0usize;
    let t0 = Instant::now();
    for _ in 0..ITERS_ENCODE {
        total_len += encode_frame(std::hint::black_box(&frame)).len();
    }
    std::hint::black_box(total_len);
    let mean_us = t0.elapsed().as_secs_f64() * 1e6 / ITERS_ENCODE as f64;
    assert!(
        mean_us < MAX_ENCODE_US,
        "encode_frame @6 cars mean too slow: {mean_us:.2} µs (gate: {MAX_ENCODE_US} µs)"
    );
}

#[test]
fn dropshot_tile_states_round_trip() {
    let mut s = ArenaState::new_empty(GameMode::Dropshot);
    s.tick_count = TICK;
    for i in 0..2 {
        s.cars.push((
            CarInfo {
                idx: i,
                ..Default::default()
            },
            CarState::default(),
        ));
    }
    // `new_empty` leaves this `None` even for Dropshot; a real Dropshot frame
    // always carries the grid.
    s.tile_states = Some(TileStates::DEFAULT);

    let frame = VisFrame::new(s, DT);
    let bytes = encode_frame(&frame);
    let (back, consumed) = decode_frame(&bytes)
        .expect("valid framing")
        .expect("complete frame");
    assert_eq!(consumed, bytes.len());
    assert_eq!(back.dt, DT);
    assert_eq!(back.state.tick_count, TICK);
    assert_eq!(back.state.game_mode(), GameMode::Dropshot);
    assert_eq!(back.state.num_cars(), 2);
    assert_eq!(back.state.tile_states, Some(TileStates::DEFAULT));
}
