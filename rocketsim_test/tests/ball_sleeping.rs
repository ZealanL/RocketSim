use glam::{Mat3A, Vec3A};
use rocketsim::{Arena, BallState, CarBodyConfig, GameMode, Team};

#[test]
fn sleep_preserves_pose_and_zero_velocity_state_writes_wake_ball() {
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
        arena.set_ball_state(state);
        arena.sleep_ball();

        let sleeping_state = arena.get_ball_state();
        assert_eq!(sleeping_state.pos, pos);
        assert_eq!(sleeping_state.rot_mat, rot);
        assert_eq!(sleeping_state.vel, Vec3A::ZERO);
        assert_eq!(sleeping_state.ang_vel, Vec3A::ZERO);
        arena.step_tick();
        assert_eq!(arena.get_ball_state().pos, pos);
        assert_eq!(arena.get_ball_state().vel, Vec3A::ZERO);
        assert_eq!(arena.get_ball_state().ang_vel, Vec3A::ZERO);
    }

    let mut state = *arena.get_ball_state();
    state.phys.pos = Vec3A::new(0.0, 0.0, 1500.0);
    state.phys.vel = Vec3A::ZERO;
    state.phys.ang_vel = Vec3A::ZERO;
    arena.sleep_ball();
    arena.set_ball_state(state);
    arena.step_tick();
    assert!(
        arena.get_ball_state().vel.z < 0.0,
        "zero-velocity state write must wake the ball under gravity"
    );

    arena.sleep_ball();
    let mut moving_state = *arena.get_ball_state();
    moving_state.phys.vel = Vec3A::new(100.0, -50.0, 25.0);
    moving_state.phys.ang_vel = Vec3A::new(1.0, 2.0, 3.0);
    arena.set_ball_state(moving_state);
    arena.step_tick();
    assert!(arena.get_ball_state().vel.x > 90.0);
    assert!(arena.get_ball_state().vel.y < -40.0);
    assert!(arena.get_ball_state().vel.z > 10.0);
    assert_ne!(arena.get_ball_state().ang_vel, Vec3A::ZERO);
}

#[test]
fn car_contact_wakes_sleeping_ball_without_meshes() {
    let mut arena = Arena::new(GameMode::TheVoid);
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);

    let mut ball = BallState::DEFAULT;
    ball.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    arena.set_ball_state(ball);
    arena.sleep_ball();

    let mut car = *arena.get_car_state(car_idx);
    car.phys.pos = Vec3A::new(0.0, -170.0, 1000.0);
    car.phys.vel = Vec3A::new(0.0, 2300.0, 0.0);
    arena.set_car_state_full(car_idx, car);

    for _ in 0..4 {
        arena.step_tick();
        if arena.get_ball_state().vel.length_squared() > 0.0 {
            return;
        }
    }
    panic!("car contact did not wake the sleeping ball");
}
