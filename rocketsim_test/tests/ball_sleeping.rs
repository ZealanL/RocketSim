use glam::{Mat3A, Vec3A};
use rocketsim::{
    ActivationState, Arena, ArenaConfig, BallState, CarBodyConfig, GameMode, MutatorConfig, Team,
};

#[test]
fn sleep_preserves_pose_and_explicit_wake_restores_motion() {
    let mut arena = Arena::new(GameMode::TheVoid);

    let cases = [
        (
            Vec3A::new(0.0, 0.0, 1000.0),
            Mat3A::IDENTITY,
            Vec3A::new(100.0, -50.0, 25.0),
            Vec3A::new(1.0, 2.0, 3.0),
        ),
        (
            Vec3A::new(300.0, -200.0, 1200.0),
            Mat3A::from_rotation_z(0.7),
            Vec3A::new(-80.0, 40.0, -10.0),
            Vec3A::new(-2.0, 1.0, 0.5),
        ),
    ];

    for (pos, rot, vel, ang_vel) in cases {
        let mut state = BallState::DEFAULT;
        state.phys.pos = pos;
        state.phys.rot_mat = rot;
        state.phys.vel = vel;
        state.phys.ang_vel = ang_vel;
        state.wake();
        arena.set_ball_state(state);
        arena.sleep_ball();

        let sleeping_state = arena.get_ball_state();
        assert_eq!(sleeping_state.pos, pos);
        assert_eq!(sleeping_state.rot_mat, rot);
        assert_eq!(sleeping_state.vel, Vec3A::ZERO);
        assert_eq!(sleeping_state.ang_vel, Vec3A::ZERO);
        assert_eq!(sleeping_state.activation_state, ActivationState::Sleeping);
        arena.step_tick();
        assert_eq!(arena.get_ball_state().pos, pos);
        assert_eq!(arena.get_ball_state().vel, Vec3A::ZERO);
        assert_eq!(arena.get_ball_state().ang_vel, Vec3A::ZERO);
        assert_eq!(
            arena.get_ball_state().activation_state,
            ActivationState::Sleeping
        );
    }

    // A zero-velocity write needs an explicit wake to fall under gravity.
    let mut state = *arena.get_ball_state();
    state.phys.pos = Vec3A::new(0.0, 0.0, 1500.0);
    state.phys.vel = Vec3A::ZERO;
    state.phys.ang_vel = Vec3A::ZERO;
    state.wake();
    arena.set_ball_state(state);
    arena.step_tick();
    assert!(
        arena.get_ball_state().vel.z < 0.0,
        "explicit wake must restore gravity response"
    );

    arena.sleep_ball();
    let mut moving_state = *arena.get_ball_state();
    moving_state.phys.vel = Vec3A::new(100.0, -50.0, 25.0);
    moving_state.phys.ang_vel = Vec3A::new(1.0, 2.0, 3.0);
    moving_state.wake();
    arena.set_ball_state(moving_state);
    arena.step_tick();
    assert!(arena.get_ball_state().vel.x > 90.0);
    assert!(arena.get_ball_state().vel.y < -40.0);
    assert!(arena.get_ball_state().vel.z > 10.0);
    assert_ne!(arena.get_ball_state().ang_vel, Vec3A::ZERO);
}

#[test]
fn sleep_ball_zeroes_velocity_and_retains_the_timer() {
    let mut arena = Arena::new(GameMode::TheVoid);

    let mut ball = BallState::DEFAULT;
    ball.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    ball.phys.vel = Vec3A::new(200.0, 0.0, 0.0);
    ball.phys.ang_vel = Vec3A::new(0.0, 0.0, 2.0);
    ball.activation_state = ActivationState::Sleeping;
    ball.deactivation_time = 1.25;
    arena.set_ball_state(ball);

    arena.sleep_ball();

    let state = arena.get_ball_state();
    assert_eq!(state.activation_state, ActivationState::Sleeping);
    assert_eq!(state.deactivation_time.to_bits(), 1.25f32.to_bits());
    assert_eq!(state.vel, Vec3A::ZERO);
    assert_eq!(state.ang_vel, Vec3A::ZERO);
}

#[test]
fn activation_roundtrip_and_continuation_preserve_physics() {
    // No meshes are loaded. Each ball is in free flight without contact
    // history. A restored arena must therefore continue identically.
    let cases = [
        (
            ActivationState::Active,
            0.0f32,
            Vec3A::new(300.0, -200.0, 50.0),
            0.0f32,
        ),
        (
            ActivationState::Sleeping,
            1.25,
            Vec3A::new(120.0, 80.0, -40.0),
            0.5,
        ),
        (
            ActivationState::WantsDeactivation,
            1.75,
            Vec3A::new(-30.0, 10.0, 5.0),
            0.1,
        ),
    ];

    for (idx, (activation, timer, vel, ang_scale)) in cases.into_iter().enumerate() {
        let mut arena = Arena::new(GameMode::TheVoid);

        let pos = Vec3A::new(100.0 * idx as f32, -50.0, 1500.0);
        let ang_vel = Vec3A::new(ang_scale, -ang_scale, 0.5 * ang_scale);

        let mut ball = BallState::DEFAULT;
        ball.phys.pos = pos;
        ball.phys.vel = vel;
        ball.phys.ang_vel = ang_vel;
        ball.activation_state = activation;
        ball.deactivation_time = timer;
        arena.set_ball_state(ball);

        // The getter agrees immediately. It does not wait for a tick.
        let read = arena.get_ball_state();
        assert_eq!(read.activation_state, activation, "case {idx}");
        assert_eq!(
            read.deactivation_time.to_bits(),
            timer.to_bits(),
            "case {idx}"
        );
        assert_eq!(read.phys.pos, pos, "case {idx}");
        assert_eq!(read.phys.vel, vel, "case {idx}");
        assert_eq!(read.phys.ang_vel, ang_vel, "case {idx}");

        // Advance the source, then restore the checkpoint into a fresh arena.
        for _ in 0..10 {
            arena.step_tick();
        }
        let checkpoint = *arena.get_ball_state();

        let mut continued = Arena::new(GameMode::TheVoid);
        continued.set_ball_state(checkpoint);

        let restored = continued.get_ball_state();
        assert_eq!(
            restored.activation_state, checkpoint.activation_state,
            "case {idx}"
        );
        assert_eq!(
            restored.deactivation_time.to_bits(),
            checkpoint.deactivation_time.to_bits(),
            "case {idx}"
        );
        assert_eq!(restored.phys.pos, checkpoint.phys.pos, "case {idx}");
        assert_eq!(restored.phys.vel, checkpoint.phys.vel, "case {idx}");
        if activation == ActivationState::Sleeping {
            assert_ne!(
                restored.phys.vel,
                Vec3A::ZERO,
                "a sleeping restore must keep the supplied velocity (case {idx})"
            );
        }

        for _ in 0..20 {
            arena.step_tick();
            continued.step_tick();
        }

        let a = arena.get_ball_state();
        let b = continued.get_ball_state();
        assert_eq!(a.activation_state, b.activation_state, "case {idx}");
        assert_eq!(
            a.deactivation_time.to_bits(),
            b.deactivation_time.to_bits(),
            "timer diverged in case {idx}: {} vs {}",
            a.deactivation_time,
            b.deactivation_time
        );
        assert_eq!(
            a.phys, b.phys,
            "physics diverged in case {idx}: {:?} vs {:?}",
            a.phys, b.phys
        );
    }
}

#[test]
fn unrelated_edit_preserves_activation_and_timer() {
    let mut arena = Arena::new(GameMode::TheVoid);

    let mut ball = BallState::DEFAULT;
    ball.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    ball.phys.vel = Vec3A::new(50.0, 0.0, 0.0);
    ball.activation_state = ActivationState::Sleeping;
    ball.deactivation_time = 0.75;
    arena.set_ball_state(ball);

    let new_pos = Vec3A::new(10.0, 20.0, 1300.0);
    arena.edit_ball_state(|state| state.phys.pos = new_pos);

    let edited = arena.get_ball_state();
    assert_eq!(edited.phys.pos, new_pos);
    assert_eq!(edited.phys.vel, ball.phys.vel);
    assert_eq!(edited.activation_state, ActivationState::Sleeping);
    assert_eq!(edited.deactivation_time.to_bits(), 0.75f32.to_bits());
}

#[test]
fn explicit_wake_responds_to_gravity_after_a_sleep() {
    let mut arena = Arena::new(GameMode::TheVoid);

    let mut ball = BallState::DEFAULT;
    ball.phys.pos = Vec3A::new(0.0, 0.0, 1500.0);
    ball.activation_state = ActivationState::Sleeping;
    ball.deactivation_time = 1.0;
    arena.set_ball_state(ball);

    arena.step_tick();
    assert_eq!(arena.get_ball_state().phys.pos, ball.phys.pos);
    assert_eq!(arena.get_ball_state().phys.vel, Vec3A::ZERO);
    assert_eq!(
        arena.get_ball_state().activation_state,
        ActivationState::Sleeping
    );

    // The state method wakes without a pose or velocity change.
    let mut snapshot = *arena.get_ball_state();
    snapshot.phys.vel = Vec3A::new(40.0, -20.0, 10.0);
    snapshot.phys.ang_vel = Vec3A::new(0.5, -0.25, 0.1);
    let expected_phys = snapshot.phys;
    snapshot.wake();
    assert_eq!(snapshot.activation_state, ActivationState::Active);
    assert_eq!(snapshot.deactivation_time.to_bits(), 0.0f32.to_bits());
    assert_eq!(snapshot.phys, expected_phys);

    // Compare with the stored pose after the previous tick's quantization.
    let sleeping_phys = arena.get_ball_state().phys;
    arena.wake_ball();
    let woken = arena.get_ball_state();
    assert_eq!(woken.activation_state, ActivationState::Active);
    assert_eq!(woken.deactivation_time.to_bits(), 0.0f32.to_bits());
    assert_eq!(woken.phys, sleeping_phys);

    arena.step_tick();
    assert!(arena.get_ball_state().vel.z < 0.0);
}

#[test]
fn deactivation_timer_updates_after_ticks() {
    // Zero gravity keeps a still ball inside the sleep threshold.
    let mut mutators = MutatorConfig::new(GameMode::TheVoid);
    mutators.gravity = Vec3A::ZERO;
    let mut arena =
        Arena::new_with_config(ArenaConfig::new(GameMode::TheVoid).with_mutators(mutators));

    let mut ball = BallState::DEFAULT;
    ball.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    ball.wake();
    arena.set_ball_state(ball);

    for _ in 0..60 {
        arena.step_tick();
    }

    let state = arena.get_ball_state();
    assert_eq!(state.activation_state, ActivationState::Active);
    assert!(
        (state.deactivation_time - 0.5).abs() < 0.02,
        "expected about 0.5 s of deactivation time, got {}",
        state.deactivation_time
    );
}

#[test]
fn mutator_ball_property_changes_wake_the_sleeping_ball() {
    // Zero gravity keeps the ball below the sleep threshold after a wake.
    let mut base = MutatorConfig::new(GameMode::TheVoid);
    base.gravity = Vec3A::ZERO;

    type MutatorChange = fn(&mut MutatorConfig);
    let cases: [(&str, MutatorChange); 3] = [
        ("mass", |m| m.ball_mass += 5.0),
        ("radius", |m| m.ball_radius += 10.0),
        ("drag", |m| m.ball_drag += 0.1),
    ];

    for (name, change) in cases {
        let mut arena =
            Arena::new_with_config(ArenaConfig::new(GameMode::TheVoid).with_mutators(base));

        let mut ball = BallState::DEFAULT;
        ball.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
        ball.deactivation_time = 1.0;
        arena.set_ball_state(ball);
        arena.sleep_ball();
        assert_eq!(
            arena.get_ball_state().activation_state,
            ActivationState::Sleeping,
            "{name} case setup"
        );
        assert_eq!(
            arena.get_ball_state().deactivation_time.to_bits(),
            1.0f32.to_bits(),
            "{name} case must keep the timer"
        );

        let mut mutators = *arena.mutator_config();
        change(&mut mutators);
        arena.set_mutator_config(mutators);

        // The wake must be visible before the next tick.
        let woken = arena.get_ball_state();
        assert_eq!(
            woken.activation_state,
            ActivationState::Active,
            "a ball {name} change must wake the ball"
        );
        assert_eq!(
            woken.deactivation_time.to_bits(),
            0.0f32.to_bits(),
            "a ball {name} change must clear the timer"
        );
    }
}

#[test]
fn serde_defaults_missing_sleep_fields_to_active_and_zero() {
    let mut old = serde_value::to_value(BallState::DEFAULT).unwrap();
    let serde_value::Value::Map(fields) = &mut old else {
        panic!("expected map serialization");
    };
    fields.remove(&serde_value::Value::String("activation_state".to_owned()));
    fields.remove(&serde_value::Value::String("deactivation_time".to_owned()));

    let decoded: BallState = old.deserialize_into().unwrap();
    assert_eq!(decoded.activation_state, ActivationState::Active);
    assert_eq!(decoded.deactivation_time.to_bits(), 0.0f32.to_bits());
}

#[test]
fn serde_roundtrip_preserves_sleep_state() {
    let mut ball = BallState::DEFAULT;
    ball.activation_state = ActivationState::Sleeping;
    ball.deactivation_time = 1.75;

    let text = serde_json::to_string(&ball).unwrap();
    let decoded: BallState = serde_json::from_str(&text).unwrap();

    assert_eq!(decoded.activation_state, ActivationState::Sleeping);
    assert_eq!(decoded.deactivation_time.to_bits(), 1.75f32.to_bits());
}

#[test]
fn no_ball_rot_keeps_identity_without_removing_angular_velocity() {
    for no_ball_rot in [false, true] {
        let mut arena = Arena::new_with_config(
            ArenaConfig::new(GameMode::TheVoid).with_no_ball_rot(no_ball_rot),
        );
        let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let mut car = *arena.get_car_state(car_idx);
        car.phys.pos = Vec3A::new(1000.0, 1000.0, 1000.0);
        car.phys.ang_vel = Vec3A::new(1.0, 2.0, 3.0);
        arena.set_car_state_full(car_idx, car);

        let mut ball = BallState::DEFAULT;
        ball.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
        ball.phys.rot_mat = Mat3A::from_rotation_z(0.7);
        ball.phys.ang_vel = Vec3A::new(1.0, 2.0, 3.0);
        ball.wake();
        arena.set_ball_state(ball);
        let expected_rotation = if no_ball_rot {
            Mat3A::IDENTITY
        } else {
            ball.phys.rot_mat
        };
        assert_eq!(arena.get_ball_state().rot_mat, expected_rotation);
        assert_eq!(arena.get_ball_state().ang_vel, ball.phys.ang_vel);

        for _ in 0..8 {
            arena.step_tick();
            assert_ne!(arena.get_ball_state().ang_vel, Vec3A::ZERO);
            assert_eq!(
                arena.get_ball_state().rot_mat == Mat3A::IDENTITY,
                no_ball_rot
            );
        }
        assert_ne!(arena.get_car_state(car_idx).phys.rot_mat, Mat3A::IDENTITY);
    }
}

#[test]
fn car_contact_wakes_sleeping_ball_without_meshes() {
    let mut arena = Arena::new(GameMode::TheVoid);
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);

    let mut ball = BallState::DEFAULT;
    ball.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    ball.wake();
    arena.set_ball_state(ball);
    arena.sleep_ball();

    let mut car = *arena.get_car_state(car_idx);
    car.phys.pos = Vec3A::new(0.0, -170.0, 1000.0);
    car.phys.vel = Vec3A::new(0.0, 2300.0, 0.0);
    arena.set_car_state_full(car_idx, car);

    for _ in 0..4 {
        arena.step_tick();
        let state = arena.get_ball_state();
        if state.vel.length_squared() > 0.0 {
            assert_ne!(
                state.activation_state,
                ActivationState::Sleeping,
                "contact must wake the ball"
            );
            return;
        }
    }
    panic!("car contact did not wake the sleeping ball");
}
