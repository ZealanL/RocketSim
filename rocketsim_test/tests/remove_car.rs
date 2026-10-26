use std::sync::Once;

use glam::{Mat3A, Vec3A};
use rocketsim::{
    Arena, ArenaConfig, BoostPadConfig, CarBodyConfig, CarControls, CarState, GameMode,
    MutatorConfig, RaycastQuery, Team,
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

fn place(arena: &mut Arena, idx: usize, pos: Vec3A, boost: f32) {
    let mut state = CarState::DEFAULT;
    state.phys.pos = pos;
    state.phys.rot_mat = Mat3A::IDENTITY;
    state.boost = boost;
    arena.set_car_state_full(idx, state);
}

#[test]
fn dense_removal_preserves_survivors_and_proxy_queries() {
    for removed in [0, 1, 2] {
        ensure_init();
        let mut arena = Arena::new(GameMode::Soccar);
        for i in 0..3 {
            let idx = arena.add_car(
                if i % 2 == 0 { Team::Blue } else { Team::Orange },
                CarBodyConfig::OCTANE,
            );
            place(
                &mut arena,
                idx,
                Vec3A::new(1000.0 + i as f32 * 800.0, 0.0, 100.0),
                10.0 + i as f32,
            );
            arena.set_car_controls(
                idx,
                CarControls {
                    throttle: i as f32 / 2.0,
                    ..CarControls::DEFAULT
                },
            );
        }
        let before: Vec<_> = (0..3).map(|i| *arena.get_car_state(i)).collect();
        let tick = arena.tick_count();
        let query = |x| RaycastQuery {
            from: Vec3A::new(x, 0.0, 300.0),
            to: Vec3A::new(x, 0.0, 0.0),
            include_dynamics: true,
        };
        arena.refresh_car_contact_state(0);
        let short_hits = arena.cast_rays([query(1000.0), query(1800.0), query(2600.0)]);
        assert!(
            short_hits
                .iter()
                .all(|hit| hit.is_some_and(|hit| hit.user_info == rocketsim::UserInfoType::Car))
        );
        let long_query = |x| RaycastQuery {
            from: Vec3A::new(x, 0.0, 700.0),
            to: Vec3A::new(x, 0.0, -1000.0),
            include_dynamics: true,
        };
        assert!(
            arena
                .cast_rays([long_query(1000.0), long_query(1800.0), long_query(2600.0)])
                .iter()
                .all(|hit| hit.is_some_and(|hit| hit.user_info == rocketsim::UserInfoType::Car))
        );
        arena.remove_car(removed);
        assert_eq!(arena.num_cars(), 2);
        assert_eq!(arena.tick_count(), tick);
        for new_idx in 0..2 {
            let old_idx = if new_idx < removed {
                new_idx
            } else {
                new_idx + 1
            };
            let state = arena.get_car_state(new_idx);
            assert_eq!(state.phys, before[old_idx].phys);
            assert_eq!(state.controls, before[old_idx].controls);
            assert_eq!(arena.get_car_info(new_idx).idx, new_idx);
        }
        arena.refresh_car_contact_state(0);
        let hits = arena.cast_rays([query(1000.0), query(1800.0), query(2600.0)]);
        assert!(
            hits[removed].is_none(),
            "removed proxy still hit at slot {removed}: {hits:?}"
        );
        assert!(
            hits.iter().enumerate().filter(|(i, _)| *i != removed).all(
                |(_, hit)| hit.is_some_and(|hit| hit.user_info == rocketsim::UserInfoType::Car)
            )
        );
        let long_hits =
            arena.cast_rays([long_query(1000.0), long_query(1800.0), long_query(2600.0)]);
        assert!(
            !long_hits[removed].is_some_and(|hit| hit.user_info == rocketsim::UserInfoType::Car),
            "removed proxy still hit on long ray: {long_hits:?}"
        );
        assert!(
            long_hits
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != removed)
                .all(
                    |(_, hit)| hit.is_some_and(|hit| hit.user_info == rocketsim::UserInfoType::Car)
                )
        );
        let added = arena.add_car(Team::Blue, CarBodyConfig::DOMINUS);
        assert_eq!(added, 2);
        arena.step_tick();
    }
}

#[test]
fn soccar_pad_membership_and_winner_follow_remapped_indices() {
    ensure_init();
    let mut arena = Arena::new(GameMode::Soccar);
    let removed = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let owner = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    let waiting = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let pad_idx = (0..arena.num_boost_pads())
        .find(|&i| arena.get_boost_pad_config(i).is_big)
        .unwrap();
    let pad = arena.get_boost_pad_config(pad_idx).pos;
    // The owner collects first. The waiting car stays eligible during cooldown.
    place(&mut arena, owner, pad + Vec3A::new(150.0, 0.0, 17.01), 0.0);
    place(
        &mut arena,
        waiting,
        pad + Vec3A::new(-150.0, 0.0, 17.01),
        0.0,
    );
    place(
        &mut arena,
        removed,
        pad + Vec3A::new(900.0, 0.0, 17.01),
        0.0,
    );
    let _ = arena.step_tick();
    let state = arena.get_boost_pad_state(pad_idx);
    assert_eq!(state.touching_car_idxs, vec![owner, waiting]);
    // Remove either an unrelated lower-index car or the current pad owner.
    let mut owner_removed = arena.clone();
    owner_removed.remove_car(owner);
    assert_eq!(
        owner_removed.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![1]
    );
    assert_eq!(
        owner_removed.get_boost_pad_state(pad_idx).cooldown,
        state.cooldown
    );
    owner_removed.edit_boost_pad_state(pad_idx, |state| state.cooldown = 0.0);
    assert!(owner_removed.step_tick().iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == 1
    )));

    arena.remove_car(removed);
    assert_eq!(
        arena.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![0, 1]
    );
    assert_eq!(arena.get_boost_pad_state(pad_idx).cooldown, state.cooldown);
    let mut pad_state = arena.get_boost_pad_state(pad_idx);
    pad_state.cooldown = 0.0;
    arena.set_boost_pad_state(pad_idx, pad_state);
    let mut waiting_state = *arena.get_car_state(1);
    waiting_state.boost = 0.0;
    arena.set_car_state_full(1, waiting_state);
    let events = arena.step_tick();
    assert!(events.iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == 1
    )));
}

#[test]
fn invalid_removal_is_atomic_and_removal_clears_step_events() {
    ensure_init();
    let mut arena = Arena::new(GameMode::Soccar);
    let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let mut car_state = *arena.get_car_state(car);
    car_state.phys.pos = Vec3A::new(0.0, 0.0, 100.0);
    car_state.phys.vel = Vec3A::new(0.0, 0.0, -100.0);
    arena.set_car_state_full(car, car_state);
    let mut ball = *arena.get_ball_state();
    ball.phys.pos = Vec3A::new(0.0, 0.0, 150.0);
    arena.set_ball_state(ball);
    arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    let snapshot = arena.clone();
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| arena.remove_car(2)));
    assert!(result.is_err());
    assert_eq!(arena.num_cars(), snapshot.num_cars());
    assert_eq!(arena.tick_count(), snapshot.tick_count());
    for i in 0..arena.num_cars() {
        assert_eq!(arena.get_car_state(i).phys, snapshot.get_car_state(i).phys);
    }
    let events = arena.step_tick();
    assert!(
        !events.is_empty(),
        "setup must produce a real contact event"
    );
    arena.remove_car(0);
    assert!(arena.get_last_step_events().is_empty());
}

#[test]
fn removing_final_car_keeps_ball_path_and_clone_determinism() {
    let mut arena = Arena::new(GameMode::TheVoid);
    let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let mut car_state = *arena.get_car_state(car);
    car_state.phys.pos = Vec3A::new(0.0, 0.0, 100.0);
    arena.set_car_state_full(car, car_state);
    let mut ball = *arena.get_ball_state();
    ball.phys.pos = Vec3A::new(0.0, 0.0, 100.0);
    arena.set_ball_state(ball);
    let contacts = arena.step_tick();
    assert!(
        contacts
            .iter()
            .any(|event| matches!(event, rocketsim::ArenaEvent::CarHitBall(_))),
        "setup must create a car-ball contact"
    );

    arena.remove_car(car);
    assert_eq!(arena.num_cars(), 0);
    assert!(arena.get_last_step_events().is_empty());
    let mut no_car_reference = Arena::new(GameMode::TheVoid);
    no_car_reference.set_ball_state(*arena.get_ball_state());
    let tick = arena.tick_count();
    for step in 0..10 {
        arena.step_tick();
        no_car_reference.step_tick();
        assert_eq!(arena.tick_count(), tick + step + 1);
        assert_eq!(
            arena.get_ball_state().phys,
            no_car_reference.get_ball_state().phys
        );
        assert_eq!(
            arena.get_last_step_events().len(),
            no_car_reference.get_last_step_events().len()
        );
    }
    let mut clone = arena.clone();
    arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    clone.add_car(Team::Orange, CarBodyConfig::OCTANE);
    arena.step_tick();
    clone.step_tick();
    assert_eq!(arena.get_ball_state().phys, clone.get_ball_state().phys);
    assert_eq!(arena.get_car_state(0).phys, clone.get_car_state(0).phys);
}

#[test]
fn cooldown_victims_remap_and_removed_victim_clears_global_gate() {
    for (victim, expected_victim, expected_timer) in [
        (Some(0), None, 0.0),
        (Some(2), Some(1), 1.25),
        (None, None, 1.25),
    ] {
        let mut arena = Arena::new(GameMode::TheVoid);
        for _ in 0..3 {
            arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        }
        let mut attacker = *arena.get_car_state(2);
        attacker.bump_cooldown_victim = victim;
        attacker.bump_cooldown_timer = 1.25;
        arena.set_car_state_full(2, attacker);
        arena.remove_car(0);
        let survivor = arena.get_car_state(1);
        assert_eq!(survivor.bump_cooldown_victim, expected_victim);
        assert_eq!(survivor.bump_cooldown_timer, expected_timer);

        // Continue through a real tick after dense body deletion and index repair.
        let mut left = *arena.get_car_state(0);
        let mut right = *arena.get_car_state(1);
        left.phys.pos = Vec3A::new(-40.0, 0.0, 100.0);
        right.phys.pos = Vec3A::new(40.0, 0.0, 100.0);
        left.phys.vel = Vec3A::new(500.0, 0.0, 0.0);
        right.phys.vel = Vec3A::new(-500.0, 0.0, 0.0);
        arena.set_car_state_full(0, left);
        arena.set_car_state_full(1, right);
        arena.step_tick();
        assert_eq!(arena.tick_count(), 1);
        assert!(arena.get_car_state(0).phys.pos.is_finite());
        assert!(arena.get_car_state(1).phys.pos.is_finite());
        assert!(arena.get_ball_state().phys.pos.is_finite());
    }
}

#[test]
fn disabled_pad_removal_keeps_ordered_membership_for_reenabled_award() {
    ensure_init();
    let mut mutators = MutatorConfig::new(GameMode::Soccar);
    let pad = Vec3A::new(0.0, 0.0, 1200.0);
    let mut arena = Arena::new_with_config(
        ArenaConfig::new(GameMode::Soccar)
            .with_mutators(mutators)
            .with_custom_boost_pads(vec![BoostPadConfig {
                pos: pad,
                is_big: false,
            }]),
    );
    let removed = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let later = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    let first = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    place(
        &mut arena,
        removed,
        pad + Vec3A::new(900.0, 0.0, 17.01),
        0.0,
    );
    place(
        &mut arena,
        later,
        pad + Vec3A::new(-110.0, 0.0, 17.01),
        50.0,
    );
    place(&mut arena, first, pad + Vec3A::new(110.0, 0.0, 17.01), 50.0);
    let mut pad_state = arena.get_boost_pad_state(0);
    pad_state.touching_car_idxs = vec![first, later];
    arena.set_boost_pad_state(0, pad_state);
    assert_eq!(
        arena.get_boost_pad_state(0).touching_car_idxs,
        vec![first, later]
    );

    mutators.boost_pads_enabled = false;
    arena.set_mutator_config(mutators);
    let membership = arena.get_boost_pad_state(0).touching_car_idxs;
    arena.remove_car(removed);
    assert_eq!(arena.get_boost_pad_state(0).touching_car_idxs, vec![1, 0]);
    assert_eq!(
        arena.get_boost_pad_state(0).touching_car_idxs.len(),
        membership.len()
    );
    mutators.boost_pads_enabled = true;
    arena.set_mutator_config(mutators);
    arena.edit_boost_pad_state(0, |state| state.cooldown = 0.0);
    let mut waiting = *arena.get_car_state(1);
    waiting.boost = 0.0;
    arena.set_car_state_full(1, waiting);
    let events = arena.step_tick();
    assert!(events.iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == 1
    )));
    assert!(arena.get_car_state(1).boost > 0.0);
}

#[test]
fn wheel_support_contacts_refresh_and_grounded_survivor_jumps_after_removal() {
    ensure_init();
    for (remove_support, query_only_refresh) in
        [(false, false), (false, true), (true, false), (true, true)]
    {
        let mut arena = Arena::new(GameMode::Soccar);
        let mut ball = *arena.get_ball_state();
        ball.phys.pos = Vec3A::new(4000.0, 3000.0, 1500.0);
        arena.set_ball_state(ball);
        let unrelated = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
        let support = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let rider = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        place(&mut arena, unrelated, Vec3A::new(2500.0, 0.0, 17.01), 50.0);
        place(&mut arena, support, Vec3A::new(0.0, 0.0, 17.01), 50.0);
        place(&mut arena, rider, Vec3A::new(0.0, 0.0, 75.0), 50.0);
        if !query_only_refresh {
            arena.step_tick();
        }
        arena.refresh_car_contact_state(rider);
        let has_car_support = arena
            .get_car_state(rider)
            .wheels_with_contact()
            .iter()
            .flatten()
            .any(|hit| hit.user_info == rocketsim::UserInfoType::Car);
        assert!(
            has_car_support,
            "expected wheel ray to hit support car: {:?}",
            arena.get_car_state(rider).wheels_with_contact()
        );

        let removed_idx = if remove_support { support } else { unrelated };
        arena.remove_car(removed_idx);
        let rider_idx = if rider > removed_idx {
            rider - 1
        } else {
            rider
        };
        let refreshed = arena
            .get_car_state(rider_idx)
            .wheels_with_contact()
            .iter()
            .flatten()
            .any(|hit| hit.user_info == rocketsim::UserInfoType::Car);
        assert_eq!(refreshed, !remove_support);
        assert!(arena.get_car_state(0).is_on_ground());
        arena.set_car_controls(
            0,
            CarControls {
                jump: true,
                ..CarControls::DEFAULT
            },
        );
        arena.step_tick();
        assert!(
            arena.get_car_state(0).has_jumped,
            "grounded surviving car did not jump"
        );
        assert!(arena.get_ball_state().phys.pos.is_finite());
    }
}
