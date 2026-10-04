//! Consumer-local throttle and steering quantization.
//!
//! Compare independent arenas only.
//! Keep controls raw. Quantize only at force consumers.
//! Keep sticky and auto-roll gates raw. Avoid gate effects in force tests.

use std::sync::Once;

use glam::{Mat3A, Vec3A};
use rocketsim::{Arena, CarBodyConfig, CarControls, CarState, GameMode, PhysState, Team};

fn ensure_init() {
    static INIT: Once = Once::new();
    INIT.call_once(|| {
        rocketsim::init(
            concat!(env!("CARGO_MANIFEST_DIR"), "/../collision_meshes"),
            true,
        )
        .unwrap();
    });
}

/// Independent scalar form of the known asymmetric 8-bit rule.
///
/// Negatives use 128. Positives use 127. Zero stays zero.
/// Mirror `shared/quantize.rs` without calling it.
/// Use explicit fixtures below to check this helper.
fn expected_quantized_axis(x: f32) -> f32 {
    let clamped = x.clamp(-1.0, 1.0);
    let scale = if clamped < 0.0 { 128.0 } else { 127.0 };
    let biased = clamped * scale + 128.0;
    let w = biased + biased + 0.5;
    let rounded = w.round();
    let byte = ((rounded as i32 >> 1) & 0xFF) as f32;
    let s = byte - 128.0;
    if s < 0.0 { s / 128.0 } else { s / 127.0 }
}

fn air_start(pos: Vec3A, vel: Vec3A) -> CarState {
    CarState {
        phys: PhysState {
            pos,
            rot_mat: Mat3A::IDENTITY,
            vel,
            ang_vel: Vec3A::ZERO,
        },
        boost: 50.0,
        is_on_ground: false,
        wheels_with_contact: [None; 4],
        world_contact_normal: None,
        is_flipping: false,
        is_auto_flipping: false,
        ..CarState::DEFAULT
    }
}

fn ground_start(pos: Vec3A, vel: Vec3A) -> CarState {
    CarState {
        phys: PhysState {
            pos,
            rot_mat: Mat3A::IDENTITY,
            vel,
            ang_vel: Vec3A::ZERO,
        },
        boost: 50.0,
        is_on_ground: true,
        ..CarState::DEFAULT
    }
}

fn run_air_case(start: CarState, controls: CarControls, steps: usize) -> CarState {
    ensure_init();
    let mut arena = Arena::new(GameMode::Soccar);
    let idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.set_car_state(idx, start);
    arena.reset_car_transient_contacts(idx);
    arena.clear_persistent_manifolds();
    for _ in 0..steps {
        arena.set_car_controls(idx, controls);
        arena.step_tick();
    }
    *arena.get_car_state(idx)
}

fn run_ground_case(start: CarState, controls: CarControls, steps: usize) -> CarState {
    ensure_init();
    let mut arena = Arena::new(GameMode::Soccar);
    let idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.set_car_state(idx, start);
    arena.reset_car_transient_contacts(idx);
    arena.clear_persistent_manifolds();
    // Warm up wheels on flat ground with neutral inputs.
    // Both arenas use the same warmup. Test inputs start after warmup.
    for _ in 0..30 {
        arena.set_car_controls(idx, CarControls::DEFAULT);
        arena.step_tick();
    }
    for _ in 0..steps {
        arena.set_car_controls(idx, controls);
        arena.step_tick();
    }
    *arena.get_car_state(idx)
}

fn pos_diff(a: &CarState, b: &CarState) -> f32 {
    (a.pos - b.pos).length()
}

fn vel_diff(a: &CarState, b: &CarState) -> f32 {
    (a.vel - b.vel).length()
}

#[test]
fn quant_rule_fixtures_match_asymmetric_levels() {
    // Explicit levels. Positives use k/127. Negatives use k/128.
    let cases: &[(f32, f32)] = &[
        (0.0, 0.0),
        (1.0, 1.0),
        (-1.0, -1.0),
        (0.002, 0.0),
        (-0.002, 0.0),
        (0.003, 0.0),
        (-0.003, 0.0),
        (0.5, 64.0 / 127.0),
        (-0.5, -64.0 / 128.0),
        (0.6, 76.0 / 127.0),
        (-0.6, -77.0 / 128.0),
        (0.3, 38.0 / 127.0),
        (-0.3, -38.0 / 128.0),
        // Same-bin pairs share one level.
        (0.5005, 64.0 / 127.0),
        (0.507, 64.0 / 127.0),
        (0.503, 64.0 / 127.0),
        (0.505, 64.0 / 127.0),
        (-0.5005, -64.0 / 128.0),
        (-0.503, -64.0 / 128.0),
    ];
    for (raw, expected) in cases {
        let got = expected_quantized_axis(*raw);
        assert!(
            (got - expected).abs() < 1e-7,
            "raw={raw} got={got} expected={expected}"
        );
    }
}

#[test]
fn controls_stay_raw_after_tick() {
    ensure_init();
    let mut arena = Arena::new(GameMode::Soccar);
    let idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let start = ground_start(Vec3A::new(0.0, 0.0, 17.0), Vec3A::ZERO);
    arena.set_car_state(idx, start);
    arena.reset_car_transient_contacts(idx);
    arena.clear_persistent_manifolds();

    let controls = CarControls {
        throttle: 0.5,
        steer: 0.503,
        ..CarControls::DEFAULT
    };
    arena.set_car_controls(idx, controls);
    arena.step_tick();

    let stored = *arena.get_car_controls(idx);
    assert!(
        (stored.throttle - 0.5).abs() < 1e-9,
        "throttle must stay raw, got {}",
        stored.throttle
    );
    assert!(
        (stored.steer - 0.503).abs() < 1e-9,
        "steer must stay raw, got {}",
        stored.steer
    );
    // Quantized levels differ from raw. Stored values must not equal them.
    let q_throttle = expected_quantized_axis(0.5);
    assert!(
        (q_throttle - 0.5).abs() > 1e-4,
        "fixture needs raw != quantized for throttle"
    );
    assert!(
        (stored.throttle - q_throttle).abs() > 1e-4,
        "stored throttle must not be quantized"
    );
}

#[test]
fn air_tiny_throttle_rounds_to_zero() {
    // Airborne force path. No sticky. No auto-roll. No boost.
    let start = air_start(Vec3A::new(0.0, 0.0, 1500.0), Vec3A::ZERO);
    let zero = CarControls::DEFAULT;
    let tiny_pos = CarControls {
        throttle: 0.002,
        ..CarControls::DEFAULT
    };
    let tiny_neg = CarControls {
        throttle: -0.002,
        ..CarControls::DEFAULT
    };

    let base = run_air_case(start, zero, 60);
    let got_pos = run_air_case(start, tiny_pos, 60);
    let got_neg = run_air_case(start, tiny_neg, 60);

    assert!(
        pos_diff(&got_pos, &base) < 1e-4 && vel_diff(&got_pos, &base) < 1e-4,
        "tiny positive throttle must match zero in air"
    );
    assert!(
        pos_diff(&got_neg, &base) < 1e-4 && vel_diff(&got_neg, &base) < 1e-4,
        "tiny negative throttle must match zero in air"
    );

    // Sanity: full throttle must move the car forward.
    // This proves the test can see air throttle force.
    let full = run_air_case(
        start,
        CarControls {
            throttle: 1.0,
            ..CarControls::DEFAULT
        },
        60,
    );
    assert!(
        (full.vel.x - base.vel.x) > 5.0,
        "full throttle must add forward speed in air"
    );
}

#[test]
fn air_nonzero_throttle_matches_quantized_level() {
    // Raw inputs must match their explicit quantized levels.
    // Both signs are covered. Boost stays off.
    let start = air_start(Vec3A::new(0.0, 0.0, 1500.0), Vec3A::ZERO);

    let raw_pos = 0.5_f32;
    let q_pos = 64.0_f32 / 127.0_f32;
    assert!((expected_quantized_axis(raw_pos) - q_pos).abs() < 1e-7);

    let a = run_air_case(
        start,
        CarControls {
            throttle: raw_pos,
            ..CarControls::DEFAULT
        },
        60,
    );
    let b = run_air_case(
        start,
        CarControls {
            throttle: q_pos,
            ..CarControls::DEFAULT
        },
        60,
    );
    assert!(
        pos_diff(&a, &b) < 1e-4 && vel_diff(&a, &b) < 1e-4,
        "raw 0.5 must match 64/127 in air"
    );

    let raw_neg = -0.6_f32;
    let q_neg = -77.0_f32 / 128.0_f32;
    assert!((expected_quantized_axis(raw_neg) - q_neg).abs() < 1e-7);

    let c = run_air_case(
        start,
        CarControls {
            throttle: raw_neg,
            ..CarControls::DEFAULT
        },
        60,
    );
    let d = run_air_case(
        start,
        CarControls {
            throttle: q_neg,
            ..CarControls::DEFAULT
        },
        60,
    );
    assert!(
        pos_diff(&c, &d) < 1e-4 && vel_diff(&c, &d) < 1e-4,
        "raw -0.6 must match -77/128 in air"
    );
}

#[test]
fn ground_tiny_throttle_rounds_to_zero_when_moving() {
    // Grounded drive and brake paths. Use forward speed above the
    // stopping threshold. Then sticky stays active for both zero
    // and tiny inputs. This isolates force from gate effects.
    // Auto-roll stays off with four-wheel contact.
    let start = ground_start(Vec3A::new(0.0, 0.0, 17.0), Vec3A::new(800.0, 0.0, 0.0));

    let zero = CarControls::DEFAULT;
    let tiny_pos = CarControls {
        throttle: 0.002,
        ..CarControls::DEFAULT
    };
    let tiny_neg = CarControls {
        throttle: -0.002,
        ..CarControls::DEFAULT
    };

    let base = run_ground_case(start, zero, 60);
    let got_pos = run_ground_case(start, tiny_pos, 60);
    let got_neg = run_ground_case(start, tiny_neg, 60);

    assert!(
        pos_diff(&got_pos, &base) < 1e-3 && vel_diff(&got_pos, &base) < 1e-3,
        "tiny positive throttle must match zero on ground when moving"
    );
    assert!(
        pos_diff(&got_neg, &base) < 1e-3 && vel_diff(&got_neg, &base) < 1e-3,
        "tiny negative throttle must match zero on ground when moving"
    );

    // Sanity: full throttle must change speed on ground.
    let full = run_ground_case(
        start,
        CarControls {
            throttle: 1.0,
            ..CarControls::DEFAULT
        },
        60,
    );
    assert!(
        (full.vel.x - base.vel.x).abs() > 10.0,
        "full throttle must change ground speed"
    );
}

#[test]
fn ground_nonzero_throttle_matches_quantized_level() {
    // Drive path with positive input. Brake path with negative
    // input while moving forward. Four-wheel contact keeps
    // auto-roll off. Both inputs are nonzero, so sticky gates match.
    let drive_start = ground_start(Vec3A::new(0.0, 0.0, 17.0), Vec3A::new(500.0, 0.0, 0.0));

    let raw_pos = 0.5_f32;
    let q_pos = 64.0_f32 / 127.0_f32;
    let a = run_ground_case(
        drive_start,
        CarControls {
            throttle: raw_pos,
            ..CarControls::DEFAULT
        },
        60,
    );
    let b = run_ground_case(
        drive_start,
        CarControls {
            throttle: q_pos,
            ..CarControls::DEFAULT
        },
        60,
    );
    assert!(
        pos_diff(&a, &b) < 1e-3 && vel_diff(&a, &b) < 1e-3,
        "raw 0.5 must match 64/127 on ground"
    );

    let raw_neg = -0.6_f32;
    let q_neg = -77.0_f32 / 128.0_f32;
    let c = run_ground_case(
        drive_start,
        CarControls {
            throttle: raw_neg,
            ..CarControls::DEFAULT
        },
        60,
    );
    let d = run_ground_case(
        drive_start,
        CarControls {
            throttle: q_neg,
            ..CarControls::DEFAULT
        },
        60,
    );
    assert!(
        pos_diff(&c, &d) < 1e-3 && vel_diff(&c, &d) < 1e-3,
        "raw -0.6 must match -77/128 on ground brake path"
    );
}

#[test]
fn steer_same_bin_matches_on_ground() {
    // Steering force path. Throttle stays at exact 1.0.
    // Both steers in each pair share one quantization bin.
    // Four-wheel contact keeps auto-roll off.
    let start = ground_start(Vec3A::new(0.0, -2000.0, 17.0), Vec3A::new(1000.0, 0.0, 0.0));

    let mk = |steer: f32| CarControls {
        throttle: 1.0,
        steer,
        ..CarControls::DEFAULT
    };

    let a = run_ground_case(start, mk(0.5005), 90);
    let b = run_ground_case(start, mk(0.507), 90);
    assert!(
        pos_diff(&a, &b) < 1e-3 && vel_diff(&a, &b) < 1e-3,
        "steers in same positive bin must match"
    );

    let c = run_ground_case(start, mk(-0.5005), 90);
    let d = run_ground_case(start, mk(-0.503), 90);
    assert!(
        pos_diff(&c, &d) < 1e-3 && vel_diff(&c, &d) < 1e-3,
        "steers in same negative bin must match"
    );

    // Sanity: left and right steers must diverge.
    // This proves the test can see steering.
    let left = run_ground_case(start, mk(0.5), 90);
    let right = run_ground_case(start, mk(-0.5), 90);
    assert!(
        pos_diff(&left, &right) > 10.0,
        "opposite steers must diverge on ground"
    );
}

#[test]
fn boost_overrides_tiny_throttle_in_air() {
    // Boost latch forces throttle scale to 1.0 in air.
    // Tiny, zero, and half inputs must match with boost held.
    let start = air_start(Vec3A::new(0.0, 0.0, 1500.0), Vec3A::ZERO);

    let mk = |throttle: f32| CarControls {
        throttle,
        boost: true,
        ..CarControls::DEFAULT
    };

    let tiny = run_air_case(start, mk(0.002), 60);
    let zero = run_air_case(start, mk(0.0), 60);
    let half = run_air_case(start, mk(0.5), 60);

    assert!(
        pos_diff(&tiny, &zero) < 1e-4 && vel_diff(&tiny, &zero) < 1e-4,
        "boost must override tiny vs zero throttle in air"
    );
    assert!(
        pos_diff(&half, &zero) < 1e-4 && vel_diff(&half, &zero) < 1e-4,
        "boost must override half vs zero throttle in air"
    );

    // Sanity: boost must add speed compared with no boost.
    let no_boost = run_air_case(start, CarControls::DEFAULT, 60);
    assert!(
        (zero.vel.x - no_boost.vel.x) > 5.0,
        "boost must add forward speed in air"
    );
}

#[test]
fn boost_overrides_tiny_throttle_on_ground_when_moving() {
    // Grounded friction and drive paths force real throttle to 1.0
    // with boost held and fuel left. Use speed above the stopping
    // threshold so sticky gates match for zero and tiny inputs.
    let start = ground_start(Vec3A::new(0.0, 0.0, 17.0), Vec3A::new(800.0, 0.0, 0.0));

    let mk = |throttle: f32| CarControls {
        throttle,
        boost: true,
        ..CarControls::DEFAULT
    };

    let tiny = run_ground_case(start, mk(0.002), 60);
    let zero = run_ground_case(start, mk(0.0), 60);
    let full = run_ground_case(start, mk(1.0), 60);

    assert!(
        pos_diff(&tiny, &zero) < 1e-3 && vel_diff(&tiny, &zero) < 1e-3,
        "boost must override tiny vs zero throttle on ground"
    );
    assert!(
        pos_diff(&full, &zero) < 1e-3 && vel_diff(&full, &zero) < 1e-3,
        "boost must override full vs zero throttle on ground"
    );
}
