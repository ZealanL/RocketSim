use std::{panic::AssertUnwindSafe, sync::Once};

use glam::Vec3A;
use rocketsim::{
    Arena, ArenaConfig, BoostPadConfig, BoostPadState, CarBodyConfig, CarControls, CarState,
    GameMode, MutatorConfig, RaycastQuery, Team, UserInfoType,
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

#[test]
fn old_mutator_maps_default_boost_pad_checks_to_enabled() {
    let mut old = serde_value::to_value(MutatorConfig::new(GameMode::Soccar)).unwrap();
    let serde_value::Value::Map(fields) = &mut old else {
        panic!("expected map serialization");
    };
    fields.remove(&serde_value::Value::String("boost_pads_enabled".to_owned()));

    let decoded: MutatorConfig = old.deserialize_into().unwrap();
    assert!(decoded.boost_pads_enabled);
}

#[test]
fn refresh_updates_existing_physics_and_future_spawns() {
    ensure_init();
    let mut arena = Arena::new(GameMode::TheVoid);
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);

    let mut ball = *arena.get_ball_state();
    ball.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    ball.phys.vel = Vec3A::new(120.0, 10.0, 20.0);
    arena.set_ball_state(ball);

    let mut car = CarState::DEFAULT;
    car.phys.pos = Vec3A::new(145.0, 0.0, 1000.0);
    car.boost = 37.0;
    arena.set_car_state_full(car_idx, car);
    let tick_count = arena.tick_count();

    let mut mutators = *arena.mutator_config();
    mutators.gravity = Vec3A::new(0.0, 0.0, -325.0);
    mutators.car_mass = 225.0;
    mutators.ball_mass = 45.0;
    mutators.ball_radius = 120.0;
    mutators.ball_drag = 0.2;
    mutators.car_spawn_boost_amount = 25.0;
    arena.set_mutator_config(mutators);

    assert_eq!(arena.tick_count(), tick_count);
    assert_eq!(arena.get_ball_state().pos, ball.phys.pos);
    assert_eq!(arena.get_ball_state().vel, ball.phys.vel);
    assert_eq!(arena.get_car_state(car_idx).pos, car.phys.pos);
    assert_eq!(arena.get_car_state(car_idx).boost, 37.0);
    assert_eq!(arena.mutator_config().gravity, mutators.gravity);
    assert_eq!(arena.mutator_config().car_mass, mutators.car_mass);
    assert_eq!(arena.mutator_config().ball_mass, mutators.ball_mass);
    assert_eq!(arena.mutator_config().ball_radius, mutators.ball_radius);
    let [ball_hit] = arena.cast_rays([RaycastQuery {
        from: Vec3A::new(-130.0, 0.0, 1000.0),
        to: Vec3A::new(-100.0, 0.0, 1000.0),
        include_dynamics: true,
    }]);
    let ball_hit = ball_hit.expect("ray through the expanded shell must hit the ball");
    assert_eq!(ball_hit.user_info, UserInfoType::Ball);
    assert!((ball_hit.hit_point.x + 120.0).abs() < 2.0);
    assert!(ball_hit.hit_normal.x < -0.99);

    let mut reference =
        Arena::new_with_config(ArenaConfig::new(GameMode::TheVoid).with_mutators(mutators));
    let reference_car = reference.add_car(Team::Blue, CarBodyConfig::OCTANE);
    reference.set_ball_state(ball);
    reference.set_car_state_full(car_idx, car);
    arena.step_tick();
    reference.step_tick();
    assert!((arena.get_ball_state().vel - reference.get_ball_state().vel).length() < 1e-4);
    assert_ne!(arena.get_ball_state().vel.x, ball.phys.vel.x);
    let new_car = arena.add_car(Team::Orange, CarBodyConfig::DOMINUS);
    assert_eq!(arena.get_car_state(new_car).boost, 25.0);
    assert!(
        (arena.get_car_state(car_idx).vel - reference.get_car_state(reference_car).vel).length()
            < 1e-4
    );
}

#[test]
fn grounded_jump_survives_runtime_car_mass_change() {
    ensure_init();
    let mut original = Arena::new(GameMode::Soccar);
    let car_idx = original.add_car(Team::Blue, CarBodyConfig::OCTANE);
    for _ in 0..30 {
        original.set_car_controls(car_idx, CarControls::DEFAULT);
        original.step_tick();
    }
    assert!(original.get_car_state(car_idx).is_on_ground());

    let mut unchanged = original.clone();
    let mut changed = original.clone();
    let mut updated_config = *changed.mutator_config();
    updated_config.car_mass = 225.0;
    changed.set_mutator_config(updated_config);

    let mut fresh =
        Arena::new_with_config(ArenaConfig::new(GameMode::Soccar).with_mutators(updated_config));
    let fresh_car = fresh.add_car(Team::Blue, CarBodyConfig::OCTANE);
    fresh.set_car_state_full(fresh_car, *original.get_car_state(car_idx));
    fresh.set_car_controls(fresh_car, CarControls::DEFAULT);
    fresh.step_tick();

    assert!(changed.get_car_state(car_idx).is_on_ground());
    assert!(fresh.get_car_state(fresh_car).is_on_ground());
    for (arena, idx) in [
        (&mut unchanged, car_idx),
        (&mut changed, car_idx),
        (&mut fresh, fresh_car),
    ] {
        arena.set_car_controls(
            idx,
            CarControls {
                jump: true,
                ..CarControls::DEFAULT
            },
        );
        arena.step_tick();
        assert!(arena.get_car_state(idx).is_jumping);
        assert!(arena.get_car_state(idx).vel.z > 0.0);
    }
}

#[test]
fn config_changes_without_ball_properties_keep_a_sleeping_ball_asleep() {
    ensure_init();
    let mut arena = Arena::new(GameMode::TheVoid);
    arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let mut ball = *arena.get_ball_state();
    ball.phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    arena.set_ball_state(ball);
    arena.sleep_ball();

    let original_pos = arena.get_ball_state().pos;
    let unchanged = *arena.mutator_config();
    arena.set_mutator_config(unchanged);
    arena.step_tick();
    assert_eq!(arena.get_ball_state().vel, Vec3A::ZERO);
    assert_eq!(arena.get_ball_state().pos, original_pos);

    let mut pad_only = unchanged;
    pad_only.boost_pads_enabled = false;
    arena.set_mutator_config(pad_only);
    arena.step_tick();
    assert_eq!(arena.get_ball_state().vel, Vec3A::ZERO);
    assert_eq!(arena.get_ball_state().pos, original_pos);

    let mut gravity_only = pad_only;
    gravity_only.gravity.z = -300.0;
    arena.set_mutator_config(gravity_only);
    arena.step_tick();
    assert_eq!(arena.get_ball_state().vel, Vec3A::ZERO);
    assert_eq!(arena.get_ball_state().pos, original_pos);

    gravity_only.car_mass += 10.0;
    arena.set_mutator_config(gravity_only);
    arena.step_tick();
    assert_eq!(arena.get_ball_state().vel, Vec3A::ZERO);
    assert_eq!(arena.get_ball_state().pos, original_pos);
}

#[test]
fn runtime_mass_updates_require_finite_positive_values() {
    let mut arena = Arena::new(GameMode::TheVoid);
    let initial = *arena.mutator_config();

    for invalid_mass in [0.0, -1.0, f32::NAN, f32::INFINITY] {
        let mut bad_car_mass = initial;
        bad_car_mass.car_mass = invalid_mass;
        assert!(
            std::panic::catch_unwind(AssertUnwindSafe(|| {
                arena.set_mutator_config(bad_car_mass);
            }))
            .is_err()
        );
        assert_eq!(arena.mutator_config().car_mass, initial.car_mass);

        let mut bad_ball_mass = initial;
        bad_ball_mass.ball_mass = invalid_mass;
        assert!(
            std::panic::catch_unwind(AssertUnwindSafe(|| {
                arena.set_mutator_config(bad_ball_mass);
            }))
            .is_err()
        );
        assert_eq!(arena.mutator_config().ball_mass, initial.ball_mass);
    }
}

#[test]
fn boost_pad_checks_can_be_disabled_and_reenabled_without_resetting_pads() {
    ensure_init();
    let mut mutators = MutatorConfig::new(GameMode::Soccar);
    mutators.boost_pads_enabled = false;
    mutators.boost_pad_cooldown_small = 1.0;
    mutators.boost_pad_amount_small = 25.0;
    let pad_pos = Vec3A::new(0.0, 0.0, 1200.0);
    let config = ArenaConfig::new(GameMode::Soccar)
        .with_mutators(mutators)
        .with_custom_boost_pads(vec![BoostPadConfig {
            pos: pad_pos,
            is_big: false,
        }]);
    let mut arena = Arena::new_with_config(config);
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let mut ball = *arena.get_ball_state();
    ball.phys.pos = Vec3A::new(0.0, 3000.0, 1500.0);
    arena.set_ball_state(ball);
    arena.sleep_ball();
    let sleeping_ball_pos = arena.get_ball_state().pos;

    let mut car = CarState::DEFAULT;
    car.phys.pos = pad_pos;
    car.boost = 0.0;
    arena.set_car_state_full(car_idx, car);

    arena.step_tick();
    assert_eq!(arena.get_car_state(car_idx).boost, 0.0);
    assert!(arena.get_boost_pad_state(0).is_active());
    assert_eq!(arena.get_ball_state().vel, Vec3A::ZERO);
    assert_eq!(arena.get_ball_state().pos, sleeping_ball_pos);

    mutators.boost_pads_enabled = true;
    arena.set_mutator_config(mutators);
    arena.step_tick();
    assert_eq!(arena.get_car_state(car_idx).boost, 25.0);
    assert!(arena.get_boost_pad_state(0).cooldown > 0.9);
    assert_eq!(arena.get_ball_state().vel, Vec3A::ZERO);
    assert_eq!(arena.get_ball_state().pos, sleeping_ball_pos);

    let mut disabled = *arena.mutator_config();
    disabled.boost_pads_enabled = false;
    disabled.boost_pad_cooldown_small = 3.0;
    arena.set_mutator_config(disabled);
    let cooldown_before = arena.get_boost_pad_state(0).cooldown;
    arena.step_tick();
    assert!(arena.get_boost_pad_state(0).cooldown > 2.9);
    assert!(cooldown_before > 0.9);

    let mut empty_boost = car;
    empty_boost.boost = 0.0;
    arena.set_car_state_full(car_idx, empty_boost);
    disabled.boost_pads_enabled = true;
    arena.set_mutator_config(disabled);
    arena.step_tick();
    assert_eq!(arena.get_car_state(car_idx).boost, 0.0);
    assert!(arena.get_boost_pad_state(0).cooldown > 2.9);

    arena.set_boost_pad_state(0, BoostPadState::DEFAULT);
    arena.step_tick();
    assert_eq!(arena.get_car_state(car_idx).boost, 25.0);
}
