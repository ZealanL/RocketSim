use std::sync::Once;

use glam::{Mat3A, Quat, Vec3A};
use rocketsim::{
    Arena, BallState, CarBodyConfig, CarControls, CarState, GameMode, PhysState, Team,
};

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

fn car_state_at(pos: Vec3A, vel: Vec3A) -> CarState {
    CarState {
        phys: PhysState {
            pos,
            rot_mat: Mat3A::IDENTITY,
            vel,
            ang_vel: Vec3A::ZERO,
        },
        boost: 50.0,
        ..CarState::DEFAULT
    }
}

fn step_car_case(
    arena: &mut Arena,
    car_idx: usize,
    start: CarState,
    controls: CarControls,
    steps: usize,
) -> CarState {
    // Planning pattern: teleport, then opt into fresh-arena matching.
    // Plain `set_car_state` preserves contacts for replay continuity.
    arena.set_car_state(car_idx, start);
    arena.reset_car_transient_contacts(car_idx);
    arena.clear_persistent_manifolds();
    for _ in 0..steps {
        arena.set_car_controls(car_idx, controls);
        arena.step_tick();
    }
    *arena.get_car_state(car_idx)
}

/// Planning-style reuse: warm wheel/sticky contacts on the ground, then
/// teleport to air with the planning opt-in. Must match a fresh arena.
/// This is the RocketSim port of rocketsim-utils `reused_arena` regression.
#[test]
fn reused_arena_matches_fresh_after_ground_then_air() {
    ensure_init();

    let ground_start = car_state_at(Vec3A::new(0.0, 0.0, 17.0), Vec3A::new(1000.0, 0.0, 0.0));
    let air_target = car_state_at(Vec3A::new(0.0, 0.0, 500.0), Vec3A::new(500.0, 0.0, 0.0));
    let controls = CarControls::DEFAULT;

    let mut reused = Arena::new(GameMode::Soccar);
    let car_idx = reused.add_car(Team::Blue, CarBodyConfig::OCTANE);
    reused.set_car_state(car_idx, ground_start);
    for _ in 0..30 {
        reused.set_car_controls(car_idx, CarControls::DEFAULT);
        reused.step_tick();
    }
    assert!(
        reused.get_car_state(car_idx).is_on_ground,
        "warmup should have wheel contact"
    );

    // Teleport to air with the planning opt-in (see step_car_case).
    let got = step_car_case(&mut reused, car_idx, air_target, controls, 30);

    let mut fresh = Arena::new(GameMode::Soccar);
    let fresh_idx = fresh.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let expected = step_car_case(&mut fresh, fresh_idx, air_target, controls, 30);

    for v in [
        got.pos.x, got.pos.y, got.pos.z, got.vel.x, got.vel.y, got.vel.z,
    ] {
        assert!(v.is_finite(), "non-finite reused output: {got:?}");
    }
    let pos_diff = (got.pos - expected.pos).length();
    let vel_diff = (got.vel - expected.vel).length();
    assert!(
        pos_diff < 1e-3 && vel_diff < 1e-3,
        "reused arena diverged: pos_diff={pos_diff} vel_diff={vel_diff}\nreused {got:?}\nfresh {expected:?}"
    );
}

#[test]
fn explicit_clear_api_works() {
    ensure_init();

    // Ball floor contact creates manifolds.
    let warm = BallState {
        phys: PhysState {
            pos: Vec3A::new(0.0, 0.0, 93.15),
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
        },
        ..Default::default()
    };
    let target = BallState {
        phys: PhysState {
            pos: Vec3A::new(800.0, -1000.0, 500.0),
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::new(500.0, 300.0, -200.0),
            ang_vel: Vec3A::ZERO,
        },
        ..Default::default()
    };

    let mut arena = Arena::new(GameMode::Soccar);
    arena.set_ball_state(warm);
    for _ in 0..30 {
        arena.step_tick();
    }
    assert!(
        arena.num_persistent_manifolds() > 0,
        "warmup created no manifold"
    );

    // Explicit opt-in clear for planning code that wants fresh matching.
    arena.clear_persistent_manifolds();
    assert_eq!(arena.num_persistent_manifolds(), 0);

    // Planning teleport with the opt-in: reused must match fresh.
    arena.set_ball_state(target);
    arena.clear_persistent_manifolds();
    for _ in 0..60 {
        arena.step_tick();
    }
    let got = arena.get_ball_state().phys;

    let mut fresh = Arena::new(GameMode::Soccar);
    fresh.set_ball_state(target);
    for _ in 0..60 {
        fresh.step_tick();
    }
    let expected = fresh.get_ball_state().phys;
    let pos_diff = (got.pos - expected.pos).length();
    let vel_diff = (got.vel - expected.vel).length();
    assert!(
        pos_diff < 1e-3 && vel_diff < 1e-3,
        "ball reused diverged: pos_diff={pos_diff} vel_diff={vel_diff}"
    );
}

fn varied_car_states() -> Vec<CarState> {
    let yaws = [0.0, 0.6, -1.2, 2.5, -2.9, 1.57];
    let positions = [
        Vec3A::new(0.0, -2000.0, 17.0),
        Vec3A::new(1500.0, 1000.0, 17.0),
        Vec3A::new(-2500.0, -1000.0, 17.0),
        Vec3A::new(0.0, 0.0, 100.0),
        Vec3A::new(-1800.0, 2500.0, 300.0),
        Vec3A::new(3000.0, -3000.0, 17.0),
    ];
    let vels = [
        Vec3A::ZERO,
        Vec3A::new(800.0, 0.0, 0.0),
        Vec3A::new(-500.0, 1200.0, 100.0),
        Vec3A::new(0.0, -1500.0, 0.0),
        Vec3A::new(2000.0, 500.0, -200.0),
        Vec3A::new(100.0, 100.0, 300.0),
    ];

    yaws.into_iter()
        .zip(positions)
        .zip(vels)
        .enumerate()
        .map(|(i, ((yaw, pos), vel))| {
            let rot_mat = Mat3A::from_quat(Quat::from_rotation_z(yaw));
            CarState {
                phys: PhysState {
                    pos,
                    rot_mat,
                    vel,
                    ang_vel: Vec3A::ZERO,
                },
                boost: 20.0 + i as f32 * 12.0,
                ..CarState::DEFAULT
            }
        })
        .collect()
}

#[test]
fn reused_arena_matches_fresh_after_varied_poses() {
    ensure_init();

    let starts = varied_car_states();
    let controls = [
        CarControls {
            throttle: 1.0,
            steer: 0.3,
            ..CarControls::DEFAULT
        },
        CarControls {
            throttle: -0.5,
            steer: -0.8,
            handbrake: true,
            ..CarControls::DEFAULT
        },
        CarControls::DEFAULT,
    ];

    let mut reused = Arena::new(GameMode::Soccar);
    let reused_idx = reused.add_car(Team::Blue, CarBodyConfig::OCTANE);

    for (case_idx, start) in starts.iter().enumerate() {
        let ctrls = controls[case_idx % controls.len()];
        let steps = 30 + case_idx * 5;

        let mut fresh = Arena::new(GameMode::Soccar);
        let fresh_idx = fresh.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let expected = step_car_case(&mut fresh, fresh_idx, *start, ctrls, steps);
        let got = step_car_case(&mut reused, reused_idx, *start, ctrls, steps);

        assert!(
            got.pos.is_finite() && got.vel.is_finite(),
            "non-finite output in case {case_idx}: {got:?}"
        );
        let pos_diff = (got.pos - expected.pos).length();
        let vel_diff = (got.vel - expected.vel).length();
        assert!(
            pos_diff < 1e-3 && vel_diff < 1e-3,
            "reused arena diverged in case {case_idx}: pos_diff={pos_diff} vel_diff={vel_diff}"
        );
    }
}
