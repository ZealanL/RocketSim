use std::sync::Once;

use glam::{Mat3A, Vec3A};
use rocketsim::{
    Arena, ArenaConfig, BallState, BoostPadState, CarBodyConfig, CarControls, CarState, GameMode,
    PhysState, Team,
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

fn f32_bits_eq(a: f32, b: f32) -> bool {
    a.to_bits() == b.to_bits()
}

fn vec3a_bits_eq(a: Vec3A, b: Vec3A) -> bool {
    let aa = a.to_array();
    let bb = b.to_array();
    aa[0].to_bits() == bb[0].to_bits()
        && aa[1].to_bits() == bb[1].to_bits()
        && aa[2].to_bits() == bb[2].to_bits()
}

fn mat3a_bits_eq(a: Mat3A, b: Mat3A) -> bool {
    vec3a_bits_eq(a.x_axis, b.x_axis)
        && vec3a_bits_eq(a.y_axis, b.y_axis)
        && vec3a_bits_eq(a.z_axis, b.z_axis)
}

fn phys_bits_eq(a: &PhysState, b: &PhysState) -> bool {
    vec3a_bits_eq(a.pos, b.pos)
        && mat3a_bits_eq(a.rot_mat, b.rot_mat)
        && vec3a_bits_eq(a.vel, b.vel)
        && vec3a_bits_eq(a.ang_vel, b.ang_vel)
}

fn ball_bits_eq(a: &BallState, b: &BallState) -> bool {
    phys_bits_eq(&a.phys, &b.phys)
        && a.hs_info.y_target_dir == b.hs_info.y_target_dir
        && f32_bits_eq(a.hs_info.cur_target_speed, b.hs_info.cur_target_speed)
        && f32_bits_eq(a.hs_info.time_since_hit, b.hs_info.time_since_hit)
        && a.ds_info.charge_level == b.ds_info.charge_level
        && f32_bits_eq(
            a.ds_info.accumulated_hit_force,
            b.ds_info.accumulated_hit_force,
        )
        && a.ds_info.y_target_dir == b.ds_info.y_target_dir
        && a.ds_info.last_damage_tick == b.ds_info.last_damage_tick
        && a.tick_count_since_kickoff == b.tick_count_since_kickoff
}

fn car_bits_eq(a: &CarState, b: &CarState) -> bool {
    phys_bits_eq(&a.phys, &b.phys)
        && a.controls == b.controls
        && a.prev_controls == b.prev_controls
        && a.is_on_ground == b.is_on_ground
        && a.wheels_with_contact == b.wheels_with_contact
        && a.has_jumped == b.has_jumped
        && a.has_double_jumped == b.has_double_jumped
        && a.has_flipped == b.has_flipped
        && vec3a_bits_eq(a.flip_rel_torque, b.flip_rel_torque)
        && a.jump_ticks == b.jump_ticks
        && f32_bits_eq(a.flip_time, b.flip_time)
        && a.is_flipping == b.is_flipping
        && a.is_jumping == b.is_jumping
        && f32_bits_eq(a.air_time, b.air_time)
        && f32_bits_eq(a.air_time_since_jump, b.air_time_since_jump)
        && f32_bits_eq(a.boost, b.boost)
        && f32_bits_eq(a.time_since_boosted, b.time_since_boosted)
        && a.is_boosting == b.is_boosting
        && f32_bits_eq(a.boosting_time, b.boosting_time)
        && a.is_supersonic == b.is_supersonic
        && f32_bits_eq(a.supersonic_grace_timer, b.supersonic_grace_timer)
        && f32_bits_eq(a.handbrake_val, b.handbrake_val)
        && a.is_auto_flipping == b.is_auto_flipping
        && f32_bits_eq(a.auto_flip_timer, b.auto_flip_timer)
        && f32_bits_eq(a.auto_flip_torque_scale, b.auto_flip_torque_scale)
        && f32_bits_eq(a.bump_cooldown_timer, b.bump_cooldown_timer)
        && a.last_extra_hit_tick == b.last_extra_hit_tick
        && match (a.world_contact_normal, b.world_contact_normal) {
            (Some(x), Some(y)) => vec3a_bits_eq(x, y),
            (None, None) => true,
            _ => false,
        }
        && a.is_demoed == b.is_demoed
        && f32_bits_eq(a.demo_respawn_timer, b.demo_respawn_timer)
}

fn pad_bits_eq(a: &BoostPadState, b: &BoostPadState) -> bool {
    f32_bits_eq(a.cooldown, b.cooldown)
}

fn assert_arenas_equal(a: &Arena, b: &Arena) {
    assert_eq!(a.tick_count(), b.tick_count(), "tick_count diverged");
    assert_eq!(a.num_cars(), b.num_cars(), "num_cars diverged");

    let ab = a.get_ball_state();
    let bb = b.get_ball_state();
    assert!(ball_bits_eq(ab, bb), "ball diverged\na={ab:?}\nb={bb:?}");

    for i in 0..a.num_cars() {
        let ca = a.get_car_state(i);
        let cb = b.get_car_state(i);
        assert!(car_bits_eq(ca, cb), "car {i} diverged\na={ca:?}\nb={cb:?}");
        let ia = a.get_car_info(i);
        let ib = b.get_car_info(i);
        assert!(
            ia.idx == ib.idx && ia.team == ib.team && ia.config == ib.config,
            "car {i} info diverged: {ia:?} vs {ib:?}"
        );
    }

    assert_eq!(a.num_boost_pads(), b.num_boost_pads());
    for i in 0..a.num_boost_pads() {
        let pa = a.get_boost_pad_state(i);
        let pb = b.get_boost_pad_state(i);
        assert!(pad_bits_eq(&pa, &pb), "pad {i} diverged: {pa:?} vs {pb:?}");
        let ba = a.get_boost_pad_config(i);
        let bb2 = b.get_boost_pad_config(i);
        assert!(
            vec3a_bits_eq(ba.pos, bb2.pos) && ba.is_big == bb2.is_big,
            "pad {i} config diverged"
        );
    }

    if a.get_config().game_mode == GameMode::Dropshot {
        assert_eq!(a.get_tile_states(), b.get_tile_states(), "tiles diverged");
    }

    assert_eq!(
        a.num_persistent_manifolds(),
        b.num_persistent_manifolds(),
        "manifold count diverged"
    );
    assert_eq!(
        a.get_last_step_events().len(),
        b.get_last_step_events().len()
    );
}

#[test]
fn arena_clone_steps_identically_soccar() {
    ensure_init();

    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar).with_rng_seed(1234));
    let c0 = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let c1 = arena.add_car(Team::Orange, CarBodyConfig::DOMINUS);
    arena.reset_to_random_kickoff(Some(42));

    let ctrls = [
        CarControls {
            throttle: 1.0,
            steer: 0.3,
            boost: true,
            ..CarControls::DEFAULT
        },
        CarControls {
            throttle: 0.5,
            steer: -0.5,
            jump: true,
            ..CarControls::DEFAULT
        },
    ];

    // Warm up: drive into contact so manifolds/broadphase are non-trivial
    for tick in 0..120 {
        arena.set_car_controls(c0, ctrls[(tick / 10) % 2]);
        arena.set_car_controls(c1, ctrls[(tick / 7) % 2]);
        arena.step_tick();
    }

    let mut clone = arena.clone();
    assert!(clone.vis.is_none(), "clone vis must be None");
    assert_arenas_equal(&arena, &clone);

    // Step both identically for 240 ticks, compare every tick
    for tick in 0..240 {
        let cc0 = ctrls[(tick / 13) % 2];
        let cc1 = ctrls[(tick / 11) % 2];
        arena.set_car_controls(c0, cc0);
        arena.set_car_controls(c1, cc1);
        clone.set_car_controls(c0, cc0);
        clone.set_car_controls(c1, cc1);
        arena.step_tick();
        clone.step_tick();
        assert_arenas_equal(&arena, &clone);
    }
}

#[test]
fn arena_clone_preserves_rng_and_pads() {
    ensure_init();

    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar).with_rng_seed(999));
    arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff(Some(7));
    for _ in 0..30 {
        arena.step_tick();
    }
    let mut clone = arena.clone();

    // RNG state cloned: next random kickoffs must match exactly
    arena.reset_to_random_kickoff(None);
    clone.reset_to_random_kickoff(None);
    // Note: reset_to_random_kickoff(None) consumes rng in both; states must still match
    assert_arenas_equal(&arena, &clone);

    // Boost pad timers cloned: drain a pad, clone, verify cooldown matches
    let mut arena2 = Arena::new(GameMode::Soccar);
    let car = arena2.add_car(Team::Blue, CarBodyConfig::OCTANE);
    // Teleport onto pad 0 to collect it
    let pad_config = arena2.get_boost_pad_config(0);
    let mut st = *arena2.get_car_state(car);
    st.phys.pos = pad_config.pos;
    st.boost = 0.0;
    arena2.set_car_state(car, st);
    arena2.step_tick();
    assert!(
        arena2.get_car_state(car).boost > 0.0,
        "should have collected pad"
    );
    let clone2 = arena2.clone();
    assert_arenas_equal(&arena2, &clone2);
    // Step 60 ticks, pads must stay in sync
    for _ in 0..60 {
        arena2.step_tick();
    }
    let mut clone2 = clone2;
    for _ in 0..60 {
        clone2.step_tick();
    }
    assert_arenas_equal(&arena2, &clone2);
}

#[test]
fn arena_clone_dropshot_tiles() {
    ensure_init();
    let mut arena = Arena::new(GameMode::Dropshot);
    arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff(Some(3));
    for _ in 0..60 {
        arena.step_tick();
    }
    let mut clone = arena.clone();
    assert_arenas_equal(&arena, &clone);
    for _ in 0..120 {
        arena.step_tick();
        clone.step_tick();
        assert_arenas_equal(&arena, &clone);
    }
    // Explicit user change diverges (allowed): change only one arena
    let mut tiles = *clone.get_tile_states();
    tiles.states[0][0] = rocketsim::TileDamageState::Broken;
    clone.set_tile_states(tiles);
    assert_ne!(arena.get_tile_states(), clone.get_tile_states());
}

#[test]
fn arena_clone_ball_only_and_mid_contact() {
    ensure_init();
    // Ball resting on floor -> persistent manifold present
    let mut arena = Arena::new(GameMode::Soccar);
    arena.set_ball_state(rocketsim::BallState {
        phys: PhysState {
            pos: Vec3A::new(0.0, 0.0, 93.15),
            rot_mat: Mat3A::IDENTITY,
            vel: Vec3A::ZERO,
            ang_vel: Vec3A::ZERO,
        },
        ..Default::default()
    });
    for _ in 0..30 {
        arena.step_tick();
    }
    assert!(
        arena.num_persistent_manifolds() > 0,
        "expected contact manifold"
    );
    let mut clone = arena.clone();
    assert_arenas_equal(&arena, &clone);
    for _ in 0..60 {
        arena.step_tick();
        clone.step_tick();
        assert_arenas_equal(&arena, &clone);
    }
}
