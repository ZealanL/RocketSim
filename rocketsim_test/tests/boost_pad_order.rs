use std::sync::Once;

use glam::{Mat3A, Vec3A};
use rocketsim::{Arena, CarBodyConfig, CarState, GameMode, Team};

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

fn place(arena: &mut Arena, car_idx: usize, pos: Vec3A, boost: f32) {
    place_rotated(arena, car_idx, pos, Mat3A::IDENTITY, boost);
}

fn place_rotated(arena: &mut Arena, car_idx: usize, pos: Vec3A, rot: Mat3A, boost: f32) {
    let mut state = CarState::DEFAULT;
    state.phys.pos = pos;
    state.phys.rot_mat = rot;
    state.boost = boost;
    arena.set_car_state_full(car_idx, state);
}

#[test]
fn first_arrival_wins_after_a_later_lower_index_car_enters_during_cooldown() {
    ensure_init();
    let mut arena = Arena::new(GameMode::Soccar);
    let car0 = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let car1 = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    let pad_idx = (0..arena.num_boost_pads())
        .find(|&idx| {
            let pad = arena.get_boost_pad_config(idx);
            pad.is_big
                && pad
                    .pos
                    .truncate()
                    .distance(Vec3A::new(-3584.0, 0.0, 0.0).truncate())
                    < 1.0
        })
        .expect("default big pad");
    let pad_pos = arena.get_boost_pad_config(pad_idx).pos;

    place(
        &mut arena,
        car0,
        Vec3A::new(pad_pos.x + 800.0, pad_pos.y, 17.01),
        0.0,
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pad_pos.x + 150.0, pad_pos.y, 17.01),
        0.0,
    );
    let first_events = arena.step_tick().to_vec();
    assert!(first_events.iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == car1
    )));

    place(
        &mut arena,
        car0,
        Vec3A::new(pad_pos.x - 150.0, pad_pos.y, 17.01),
        0.0,
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pad_pos.x + 150.0, pad_pos.y, 17.01),
        0.0,
    );
    arena.edit_boost_pad_state(pad_idx, |state| state.cooldown = 0.0);
    let events = arena.step_tick();
    let pickups: Vec<_> = events
        .iter()
        .filter_map(|event| match event {
            rocketsim::ArenaEvent::CarPickupBoost(pickup) => Some(pickup.car_idx),
            _ => None,
        })
        .collect();
    assert_eq!(pickups, vec![car1]);
    assert_eq!(
        arena.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![car1, car0]
    );
}

#[test]
fn cached_winner_moves_to_next_member_after_owner_leaves() {
    ensure_init();
    use rocketsim::{ArenaConfig, MutatorConfig};

    let mut mutators = MutatorConfig::new(GameMode::Soccar);
    mutators.boost_pad_cooldown_big = 2.0 / 120.0;
    let mut arena =
        Arena::new_with_config(ArenaConfig::new(GameMode::Soccar).with_mutators(mutators));
    let first = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let next = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    let pad_idx = (0..arena.num_boost_pads())
        .find(|&idx| {
            let pad = arena.get_boost_pad_config(idx);
            pad.is_big
                && pad
                    .pos
                    .truncate()
                    .distance(Vec3A::new(-3584.0, 0.0, 0.0).truncate())
                    < 1.0
        })
        .expect("default big pad");
    let pad = *arena.get_boost_pad_config(pad_idx);

    place(
        &mut arena,
        first,
        pad.pos + Vec3A::new(150.0, 0.0, 17.01),
        0.0,
    );
    place(
        &mut arena,
        next,
        pad.pos + Vec3A::new(800.0, 0.0, 17.01),
        0.0,
    );
    assert!(arena.step_tick().iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == first
    )));

    place(
        &mut arena,
        first,
        pad.pos + Vec3A::new(800.0, 0.0, 17.01),
        0.0,
    );
    place(
        &mut arena,
        next,
        pad.pos + Vec3A::new(-150.0, 0.0, 17.01),
        0.0,
    );
    assert!(
        arena
            .step_tick()
            .iter()
            .all(|event| !matches!(event, rocketsim::ArenaEvent::CarPickupBoost(_)))
    );
    assert_eq!(
        arena.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![next]
    );

    let at_expiry = arena.step_tick();
    assert!(at_expiry.iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == next
    )));
}

#[test]
fn lower_index_first_arrival_stays_ahead_of_later_car() {
    ensure_init();
    let mut arena = Arena::new(GameMode::Soccar);
    let car0 = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let car1 = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    let pad_idx = (0..arena.num_boost_pads())
        .find(|&idx| {
            let pad = arena.get_boost_pad_config(idx);
            pad.is_big
                && pad
                    .pos
                    .truncate()
                    .distance(Vec3A::new(-3584.0, 0.0, 0.0).truncate())
                    < 1.0
        })
        .expect("default big pad");
    let pad_pos = arena.get_boost_pad_config(pad_idx).pos;

    place(
        &mut arena,
        car0,
        Vec3A::new(pad_pos.x - 150.0, pad_pos.y, 17.01),
        0.0,
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pad_pos.x + 800.0, pad_pos.y, 17.01),
        0.0,
    );
    let first_events = arena.step_tick().to_vec();
    assert!(first_events.iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == car0
    )));

    place(
        &mut arena,
        car0,
        Vec3A::new(pad_pos.x - 150.0, pad_pos.y, 17.01),
        0.0,
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pad_pos.x + 150.0, pad_pos.y, 17.01),
        0.0,
    );
    arena.edit_boost_pad_state(pad_idx, |state| state.cooldown = 0.0);
    let events = arena.step_tick();
    let pickups: Vec<_> = events
        .iter()
        .filter_map(|event| match event {
            rocketsim::ArenaEvent::CarPickupBoost(pickup) => Some(pickup.car_idx),
            _ => None,
        })
        .collect();
    assert_eq!(pickups, vec![car0]);
    assert_eq!(
        arena.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![car0, car1]
    );
}

#[test]
fn small_pad_first_arrival_wins_during_cooldown() {
    ensure_init();
    let mut arena = Arena::new(GameMode::Soccar);
    let car0 = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let car1 = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    let pad_idx = (0..arena.num_boost_pads())
        .find(|&idx| {
            let pad = arena.get_boost_pad_config(idx);
            !pad.is_big
        })
        .expect("default small pad");
    let pad_pos = arena.get_boost_pad_config(pad_idx).pos;

    place(
        &mut arena,
        car0,
        Vec3A::new(pad_pos.x + 400.0, pad_pos.y, 17.01),
        0.0,
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pad_pos.x + 40.0, pad_pos.y, 17.01),
        0.0,
    );
    let first_events = arena.step_tick().to_vec();
    assert!(first_events.iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == car1
    )));

    place(
        &mut arena,
        car0,
        Vec3A::new(pad_pos.x - 40.0, pad_pos.y, 17.01),
        0.0,
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pad_pos.x + 40.0, pad_pos.y, 17.01),
        0.0,
    );
    arena.edit_boost_pad_state(pad_idx, |state| state.cooldown = 0.0);
    let events = arena.step_tick();
    let pickups: Vec<_> = events
        .iter()
        .filter_map(|event| match event {
            rocketsim::ArenaEvent::CarPickupBoost(pickup) => Some(pickup.car_idx),
            _ => None,
        })
        .collect();
    assert_eq!(pickups, vec![car1]);
    assert_eq!(
        arena.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![car1, car0]
    );
}

#[test]
fn custom_separated_pads_use_ordered_member_arbitration() {
    ensure_init();
    use rocketsim::{ArenaConfig, BoostPadConfig, MutatorConfig};

    let first_pos = Vec3A::new(0.0, 0.0, 73.0);
    let second_pos = Vec3A::new(1000.0, 0.0, 73.0);
    let mut mutators = MutatorConfig::new(GameMode::Soccar);
    mutators.boost_pad_cooldown_small = 2.0 / 120.0;
    let config = ArenaConfig::new(GameMode::Soccar)
        .with_mutators(mutators)
        .with_custom_boost_pads(vec![
            BoostPadConfig {
                pos: first_pos,
                is_big: false,
            },
            BoostPadConfig {
                pos: second_pos,
                is_big: false,
            },
        ]);
    let mut arena = Arena::new_with_config(config);
    let car0 = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let car1 = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    let first_pad = (0..arena.num_boost_pads())
        .find(|&idx| arena.get_boost_pad_config(idx).pos == first_pos)
        .unwrap();
    assert!(
        first_pos.truncate().distance(second_pos.truncate())
            > 2.0 * rocketsim::consts::boost_pads::CYL_RAD_SMALL
    );

    place(&mut arena, car0, Vec3A::new(-800.0, 0.0, 17.01), 0.0);
    place(&mut arena, car1, Vec3A::new(40.0, 0.0, 17.01), 0.0);
    assert!(arena.step_tick().iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup)
            if pickup.car_idx == car1 && pickup.boost_pad_idx == first_pad
    )));

    place(&mut arena, car0, Vec3A::new(-40.0, 0.0, 17.01), 0.0);
    place(&mut arena, car1, Vec3A::new(40.0, 0.0, 17.01), 0.0);
    assert!(
        arena
            .step_tick()
            .iter()
            .all(|event| !matches!(event, rocketsim::ArenaEvent::CarPickupBoost(_)))
    );
    assert_eq!(
        arena.get_boost_pad_state(first_pad).touching_car_idxs,
        vec![car1, car0]
    );

    arena.edit_boost_pad_state(first_pad, |state| state.cooldown = 0.0);
    let pickups: Vec<_> = arena
        .step_tick()
        .iter()
        .filter_map(|event| match event {
            rocketsim::ArenaEvent::CarPickupBoost(pickup) => {
                Some((pickup.car_idx, pickup.boost_pad_idx))
            }
            _ => None,
        })
        .collect();
    assert_eq!(pickups, vec![(car1, first_pad)]);
}

#[test]
fn full_winner_refreshes_pad_for_next_eligible_member() {
    ensure_init();
    use rocketsim::{ArenaConfig, BoostPadConfig, MutatorConfig};

    let pad_pos = Vec3A::new(0.0, 0.0, 73.0);
    let mut mutators = MutatorConfig::new(GameMode::Soccar);
    mutators.car_max_boost_amount = 12.0;
    let config = ArenaConfig::new(GameMode::Soccar)
        .with_mutators(mutators)
        .with_custom_boost_pads(vec![BoostPadConfig {
            pos: pad_pos,
            is_big: false,
        }]);
    let mut arena = Arena::new_with_config(config);
    let first = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let next = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    place(
        &mut arena,
        first,
        pad_pos + Vec3A::new(40.0, 0.0, -55.99),
        0.0,
    );
    place(
        &mut arena,
        next,
        pad_pos + Vec3A::new(-40.0, 0.0, -55.99),
        0.0,
    );

    let first_events = arena.step_tick().to_vec();
    assert!(first_events.iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == first
    )));
    assert_eq!(arena.get_car_state(first).boost, 12.0);
    assert!(first_events.iter().all(|event| !matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == next
    )));

    arena.edit_boost_pad_state(0, |state| state.cooldown = 0.0);
    let next_events = arena.step_tick();
    assert!(next_events.iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == next
    )));
}

#[test]
fn restored_cross_pad_membership_keeps_only_the_lowest_pad_index() {
    ensure_init();
    use rocketsim::{ArenaConfig, BoostPadConfig, BoostPadState};

    let first_pos = Vec3A::new(0.0, 0.0, 73.0);
    let second_pos = Vec3A::new(1000.0, 0.0, 73.0);
    let config = ArenaConfig::new(GameMode::Soccar).with_custom_boost_pads(vec![
        BoostPadConfig {
            pos: first_pos,
            is_big: false,
        },
        BoostPadConfig {
            pos: second_pos,
            is_big: false,
        },
    ]);
    let mut arena = Arena::new_with_config(config);
    let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let first_pad = (0..arena.num_boost_pads())
        .find(|&idx| arena.get_boost_pad_config(idx).pos == first_pos)
        .unwrap();
    let second_pad = (0..arena.num_boost_pads())
        .find(|&idx| arena.get_boost_pad_config(idx).pos == second_pos)
        .unwrap();
    place(
        &mut arena,
        car,
        first_pos + Vec3A::new(100.0, 0.0, -55.99),
        0.0,
    );

    for pad_idx in [first_pad, second_pad] {
        arena.set_boost_pad_state(
            pad_idx,
            BoostPadState {
                cooldown: 10.0,
                touching_car_idxs: vec![car],
            },
        );
    }
    arena.step_tick();

    assert_eq!(
        arena.get_boost_pad_state(first_pad).touching_car_idxs,
        vec![car]
    );
    assert!(
        arena
            .get_boost_pad_state(second_pad)
            .touching_car_idxs
            .is_empty()
    );
}

#[test]
fn full_membership_stable_removal_reentry_demo_clone_serde_and_reset() {
    ensure_init();
    let mut arena = Arena::new(GameMode::Soccar);
    let car0 = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let car1 = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    let pad_idx = (0..arena.num_boost_pads())
        .find(|&idx| arena.get_boost_pad_config(idx).is_big)
        .unwrap();
    let pos = arena.get_boost_pad_config(pad_idx).pos;

    place(
        &mut arena,
        car0,
        Vec3A::new(pos.x + 500.0, pos.y, 17.01),
        0.0,
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pos.x + 100.0, pos.y, 17.01),
        100.0,
    );
    let full_first_events = arena.step_tick();
    assert!(!full_first_events.iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == car1
    )));
    assert_eq!(arena.get_car_state(car1).boost, 100.0);
    place(
        &mut arena,
        car0,
        Vec3A::new(pos.x - 100.0, pos.y, 17.01),
        0.0,
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pos.x + 100.0, pos.y, 17.01),
        100.0,
    );
    let full_skip_events = arena.step_tick();
    let pickups: Vec<_> = full_skip_events
        .iter()
        .filter_map(|event| match event {
            rocketsim::ArenaEvent::CarPickupBoost(pickup) => Some(pickup.car_idx),
            _ => None,
        })
        .collect();
    assert_eq!(pickups, vec![car0]);
    assert!(arena.get_car_state(car0).boost > 0.0);
    assert_eq!(arena.get_car_state(car1).boost, 100.0);
    assert_eq!(
        arena.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![car1, car0]
    );

    let clone = arena.clone();
    assert_eq!(
        clone.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![car1, car0]
    );
    let mut restored = arena.clone();
    restored.set_boost_pad_state(pad_idx, arena.get_boost_pad_state(pad_idx));
    assert_eq!(
        restored.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![car1, car0]
    );
    let snapshot = arena.get_arena_state();
    assert_eq!(
        snapshot.boost_pads[pad_idx].1.touching_car_idxs,
        vec![car1, car0]
    );
    let serialized = serde_value::to_value(arena.get_boost_pad_state(pad_idx)).unwrap();
    let decoded: rocketsim::BoostPadState = serialized.deserialize_into().unwrap();
    assert_eq!(decoded.touching_car_idxs, vec![car1, car0]);

    place(
        &mut arena,
        car1,
        Vec3A::new(pos.x + 500.0, pos.y, 17.01),
        0.0,
    );
    arena.step_tick();
    assert_eq!(
        arena.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![car0]
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pos.x + 100.0, pos.y, 17.01),
        0.0,
    );
    arena.step_tick();
    assert_eq!(
        arena.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![car0, car1]
    );

    place(
        &mut arena,
        car0,
        Vec3A::new(pos.x + 500.0, pos.y, 17.01),
        0.0,
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pos.x + 100.0, pos.y, 17.01),
        0.0,
    );
    arena.edit_boost_pad_state(pad_idx, |state| state.cooldown = 0.0);
    arena.demolish_car(car1);
    let demo_events = arena.step_tick();
    assert!(
        !demo_events
            .iter()
            .any(|event| matches!(event, rocketsim::ArenaEvent::CarPickupBoost(_)))
    );
    assert!(
        arena
            .get_boost_pad_state(pad_idx)
            .touching_car_idxs
            .is_empty()
    );
    arena.reset_to_random_kickoff(Some(7));
    assert!(
        arena
            .get_boost_pad_state(pad_idx)
            .touching_car_idxs
            .is_empty()
    );
}

#[test]
fn natural_big_pad_cooldown_expiry_preserves_the_first_entrant() {
    ensure_init();
    use rocketsim::{ArenaConfig, MutatorConfig};
    let mut mutators = MutatorConfig::new(GameMode::Soccar);
    mutators.boost_pad_cooldown_big = 2.0 / 120.0;
    let mut arena =
        Arena::new_with_config(ArenaConfig::new(GameMode::Soccar).with_mutators(mutators));
    let car0 = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let car1 = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    let pad_idx = (0..arena.num_boost_pads())
        .find(|&idx| {
            let pad = arena.get_boost_pad_config(idx);
            pad.is_big
                && pad
                    .pos
                    .truncate()
                    .distance(Vec3A::new(-3584.0, 0.0, 0.0).truncate())
                    < 1.0
        })
        .expect("default big pad");
    let pos = arena.get_boost_pad_config(pad_idx).pos;
    place(
        &mut arena,
        car0,
        Vec3A::new(pos.x + 800.0, pos.y, 17.01),
        0.0,
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pos.x + 150.0, pos.y, 17.01),
        0.0,
    );
    assert!(arena.step_tick().iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == car1
    )));

    place(
        &mut arena,
        car0,
        Vec3A::new(pos.x - 150.0, pos.y, 17.01),
        0.0,
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pos.x + 150.0, pos.y, 17.01),
        0.0,
    );
    assert!(arena.get_boost_pad_state(pad_idx).cooldown > 0.0);
    let during_cooldown = arena.step_tick();
    assert!(
        !during_cooldown
            .iter()
            .any(|event| matches!(event, rocketsim::ArenaEvent::CarPickupBoost(_)))
    );

    place(
        &mut arena,
        car0,
        Vec3A::new(pos.x - 150.0, pos.y, 17.01),
        0.0,
    );
    place(
        &mut arena,
        car1,
        Vec3A::new(pos.x + 150.0, pos.y, 17.01),
        0.0,
    );
    let at_expiry = arena.step_tick();
    let pickups: Vec<_> = at_expiry
        .iter()
        .filter_map(|event| match event {
            rocketsim::ArenaEvent::CarPickupBoost(pickup) => Some(pickup.car_idx),
            _ => None,
        })
        .collect();
    assert_eq!(pickups, vec![car1]);
}

#[test]
fn old_map_pad_state_defaults_membership_to_empty() {
    use std::collections::BTreeMap;
    let old_state = serde_value::Value::Map(BTreeMap::from([(
        serde_value::Value::String("cooldown".to_owned()),
        serde_value::Value::F32(2.5),
    )]));
    let state: rocketsim::BoostPadState = old_state.deserialize_into().unwrap();
    assert_eq!(state.cooldown, 2.5);
    assert!(state.touching_car_idxs.is_empty());
}

#[test]
fn default_soccar_and_hoops_pads_are_separated_for_ordered_awards() {
    ensure_init();
    for mode in [GameMode::Soccar, GameMode::Hoops] {
        let arena = Arena::new(mode);
        for first in 0..arena.num_boost_pads() {
            let first_pad = arena.get_boost_pad_config(first);
            for second in first + 1..arena.num_boost_pads() {
                let second_pad = arena.get_boost_pad_config(second);
                let separation = if first_pad.is_big || second_pad.is_big {
                    rocketsim::consts::boost_pads::CYL_RAD_BIG * 2.0
                } else {
                    rocketsim::consts::boost_pads::CYL_RAD_SMALL * 2.0
                };
                assert!(
                    first_pad
                        .pos
                        .truncate()
                        .distance_squared(second_pad.pos.truncate())
                        >= separation * separation,
                    "{mode:?} pads {first} and {second} overlap"
                );
            }
        }
    }
}

#[test]
fn default_pad_owner_stays_locked_by_rotated_hitbox_until_cooldown_expiry() {
    ensure_init();
    use rocketsim::{ArenaConfig, MutatorConfig};

    for (is_big, offset, yaw) in [
        (true, 209.85, 0.0),
        (false, 145.0, std::f32::consts::FRAC_PI_2),
    ] {
        let mut mutators = MutatorConfig::new(GameMode::Soccar);
        mutators.boost_pad_cooldown_big = 2.0 / 120.0;
        mutators.boost_pad_cooldown_small = 2.0 / 120.0;
        let mut arena =
            Arena::new_with_config(ArenaConfig::new(GameMode::Soccar).with_mutators(mutators));
        let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let pad_idx = (0..arena.num_boost_pads())
            .find(|&idx| arena.get_boost_pad_config(idx).is_big == is_big)
            .expect("pad of requested size");
        let pad = *arena.get_boost_pad_config(pad_idx);
        let rot = Mat3A::from_rotation_z(yaw);
        place_rotated(
            &mut arena,
            car,
            pad.pos + Vec3A::new(100.0, 0.0, 17.01),
            rot,
            0.0,
        );
        assert!(arena.step_tick().iter().any(|event| matches!(event,
            rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == car
        )));

        let retained_pos = if yaw == 0.0 {
            pad.pos + Vec3A::new(-offset, 0.0, 17.01)
        } else {
            pad.pos + Vec3A::new(0.0, -offset, 17.01)
        };
        place_rotated(&mut arena, car, retained_pos, rot, 0.0);
        assert!(
            arena
                .step_tick()
                .iter()
                .all(|event| !matches!(event, rocketsim::ArenaEvent::CarPickupBoost(_)))
        );
        assert_eq!(
            arena.get_boost_pad_state(pad_idx).touching_car_idxs,
            vec![car]
        );

        place_rotated(&mut arena, car, retained_pos, rot, 0.0);
        let at_expiry = arena.step_tick();
        assert!(at_expiry.iter().any(|event| matches!(event,
            rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == car
        )));
    }
}

#[test]
fn pad_retention_drops_nonoverlapping_car_and_does_not_widen_acquisition() {
    ensure_init();
    let mut arena = Arena::new(GameMode::Soccar);
    let owner = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let fresh = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    let pad_idx = (0..arena.num_boost_pads())
        .find(|&idx| {
            let pad = arena.get_boost_pad_config(idx);
            pad.is_big
                && pad
                    .pos
                    .truncate()
                    .distance(Vec3A::new(-3584.0, 0.0, 0.0).truncate())
                    < 1.0
        })
        .expect("default big pad");
    let pad = *arena.get_boost_pad_config(pad_idx);
    place(
        &mut arena,
        owner,
        pad.pos + Vec3A::new(150.0, 0.0, 17.01),
        0.0,
    );
    place(
        &mut arena,
        fresh,
        pad.pos + Vec3A::new(220.16, 0.0, 17.01),
        0.0,
    );
    assert!(arena.step_tick().iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == owner
    )));
    assert_eq!(
        arena.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![owner]
    );

    place(
        &mut arena,
        owner,
        pad.pos + Vec3A::new(220.16, 0.0, 17.01),
        0.0,
    );
    let events = arena.step_tick();
    assert!(
        !events
            .iter()
            .any(|event| matches!(event, rocketsim::ArenaEvent::CarPickupBoost(_)))
    );
    assert!(
        arena
            .get_boost_pad_state(pad_idx)
            .touching_car_idxs
            .is_empty()
    );
}

#[test]
fn retained_locked_owner_above_pad_grid_z_limit_can_collect() {
    ensure_init();
    use rocketsim::{ArenaConfig, MutatorConfig};

    let mut mutators = MutatorConfig::new(GameMode::Soccar);
    mutators.boost_pad_cooldown_big = 2.0 / 120.0;
    let mut arena =
        Arena::new_with_config(ArenaConfig::new(GameMode::Soccar).with_mutators(mutators));
    let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let pad_idx = (0..arena.num_boost_pads())
        .find(|&idx| {
            let pad = arena.get_boost_pad_config(idx);
            pad.is_big
                && pad
                    .pos
                    .truncate()
                    .distance(Vec3A::new(-3584.0, 0.0, 0.0).truncate())
                    < 1.0
        })
        .expect("default big pad");
    let pad = *arena.get_boost_pad_config(pad_idx);
    let max_pad_z = (0..arena.num_boost_pads())
        .map(|idx| {
            arena.get_boost_pad_config(idx).pos.z + rocketsim::consts::boost_pads::CYL_HEIGHT
        })
        .fold(f32::NEG_INFINITY, f32::max);

    place(
        &mut arena,
        car,
        pad.pos + Vec3A::new(100.0, 0.0, 17.01),
        0.0,
    );
    assert!(arena.step_tick().iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == car
    )));

    let above_grid_pos = pad.pos + Vec3A::new(0.0, 0.0, 100.0);
    assert!(above_grid_pos.z > max_pad_z);
    place_rotated(
        &mut arena,
        car,
        above_grid_pos,
        Mat3A::from_rotation_y(-std::f32::consts::FRAC_PI_2),
        0.0,
    );
    assert!(
        arena
            .step_tick()
            .iter()
            .all(|event| !matches!(event, rocketsim::ArenaEvent::CarPickupBoost(_)))
    );
    assert_eq!(
        arena.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![car]
    );

    let at_expiry = arena.step_tick();
    assert!(at_expiry.iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == car
    )));
}

#[test]
fn all_default_pad_layouts_use_ordered_membership_awards() {
    ensure_init();
    for mode in [
        GameMode::Soccar,
        GameMode::Hoops,
        GameMode::Snowday,
        GameMode::Heatseeker,
    ] {
        let mut arena = Arena::new(mode);
        let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        let pad = *arena.get_boost_pad_config(0);
        place(
            &mut arena,
            car,
            pad.pos + Vec3A::new(100.0, 0.0, 17.01),
            0.0,
        );
        assert!(
            arena.step_tick().iter().any(|event| matches!(event,
                rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == car
            )),
            "{mode:?}"
        );
        assert_eq!(arena.get_boost_pad_state(0).touching_car_idxs, vec![car]);
    }
}

#[test]
fn exact_cylinder_edges_keep_strict_xy_and_inclusive_z_rules() {
    ensure_init();
    let mut horizontal = Arena::new(GameMode::Soccar);
    let car = horizontal.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let pad_idx = (0..horizontal.num_boost_pads())
        .find(|&idx| {
            let pad = horizontal.get_boost_pad_config(idx);
            pad.is_big
                && pad
                    .pos
                    .truncate()
                    .distance(Vec3A::new(-3584.0, 0.0, 0.0).truncate())
                    < 1.0
        })
        .unwrap();
    let pad = *horizontal.get_boost_pad_config(pad_idx);
    place(
        &mut horizontal,
        car,
        Vec3A::new(
            pad.pos.x + rocketsim::consts::boost_pads::CYL_RAD_BIG,
            pad.pos.y,
            pad.pos.z,
        ),
        0.0,
    );
    let events = horizontal.step_tick();
    assert!(
        !events
            .iter()
            .any(|event| matches!(event, rocketsim::ArenaEvent::CarPickupBoost(_)))
    );
    assert!(
        horizontal
            .get_boost_pad_state(pad_idx)
            .touching_car_idxs
            .is_empty()
    );

    let mut vertical = Arena::new(GameMode::Soccar);
    let car = vertical.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let pad_idx = (0..vertical.num_boost_pads())
        .find(|&idx| {
            let candidate = vertical.get_boost_pad_config(idx);
            candidate.is_big
                && candidate
                    .pos
                    .truncate()
                    .distance(Vec3A::new(-3584.0, 0.0, 0.0).truncate())
                    < 1.0
        })
        .unwrap();
    let pad = *vertical.get_boost_pad_config(pad_idx);
    place(
        &mut vertical,
        car,
        Vec3A::new(
            pad.pos.x,
            pad.pos.y,
            pad.pos.z + rocketsim::consts::boost_pads::CYL_HEIGHT,
        ),
        0.0,
    );
    let events = vertical.step_tick();
    assert!(events.iter().any(|event| matches!(event,
        rocketsim::ArenaEvent::CarPickupBoost(pickup) if pickup.car_idx == car
    )));
    assert_eq!(
        vertical.get_boost_pad_state(pad_idx).touching_car_idxs,
        vec![car]
    );
}
