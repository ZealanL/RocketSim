//! Focused bump and demolition behavior through the public `Arena` API.

use glam::{Mat3A, Vec3A};
use rocketsim::{
    Arena, ArenaEvent, CarBodyConfig, CarControls, CarCore, CarState, GameMode, PhysState, Team,
    consts,
};

const START_Z: f32 = 100.0;
const TICK_TIME: f32 = consts::TICK_TIME;

fn car_state(pos: Vec3A, yaw: f32, vel: Vec3A) -> CarState {
    CarState::from(CarCore::grounded(
        PhysState {
            pos,
            rot_mat: Mat3A::from_euler(glam::EulerRot::ZYX, yaw, 0.0, 0.0),
            vel,
            ang_vel: Vec3A::ZERO,
        },
        0.0,
    ))
}

fn two_car_arena() -> (Arena, usize, usize) {
    let mut arena = Arena::new(GameMode::TheVoid);
    let attacker = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let victim = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    (arena, attacker, victim)
}

fn bump_events(arena: &Arena) -> Vec<(usize, usize, bool)> {
    arena
        .get_last_step_events()
        .iter()
        .filter_map(|event| match event {
            ArenaEvent::CarHitCar(hit) => {
                Some((hit.bumper_car_idx, hit.victim_car_idx, hit.is_demo))
            }
            _ => None,
        })
        .collect()
}

/// A front hit bumps and starts the half-tick cooldown. A hit outside the
/// 70 degree cone gives no impulse but still starts the cooldown.
#[test]
fn bump_needs_the_native_cone_but_always_starts_the_cooldown() {
    let close = consts::car::bump::COOLDOWN_TIME - TICK_TIME * 0.5;

    for (victim_offset, expect_bump) in [
        (Vec3A::new(100.0, 0.0, 0.0), true),
        (Vec3A::new(10.0, 75.0, 0.0), false),
    ] {
        let (mut arena, attacker, victim) = two_car_arena();
        arena.set_car_state(
            attacker,
            CarCore::grounded(
                PhysState {
                    pos: Vec3A::new(0.0, 0.0, START_Z),
                    rot_mat: Mat3A::IDENTITY,
                    vel: Vec3A::new(1000.0, 0.0, 0.0),
                    ang_vel: Vec3A::ZERO,
                },
                0.0,
            ),
        );
        arena.set_car_state(
            victim,
            CarCore::grounded(
                PhysState {
                    pos: Vec3A::new(0.0, 0.0, START_Z) + victim_offset,
                    rot_mat: Mat3A::IDENTITY,
                    vel: Vec3A::ZERO,
                    ang_vel: Vec3A::ZERO,
                },
                0.0,
            ),
        );

        arena.step_tick();

        let events = bump_events(&arena);
        let bump = events
            .iter()
            .find(|(bumper, _, is_demo)| *bumper == attacker && !*is_demo);
        assert_eq!(bump.is_some(), expect_bump, "events {events:?}");
        if expect_bump {
            assert_eq!(bump.map(|(_, v, _)| *v), Some(victim));
        }

        let attacker_state = arena.get_car_state(attacker);
        assert_eq!(attacker_state.bump_cooldown_victim, Some(victim));
        assert!(
            (attacker_state.bump_cooldown_timer - close).abs() < 1e-5,
            "cooldown {}",
            attacker_state.bump_cooldown_timer
        );
        assert!(!arena.get_car_state(victim).is_demoed);
    }
}

/// A demolition is checked before the bump cooldown, so an active cooldown
/// does not block it.
#[test]
fn demolition_ignores_an_active_bump_cooldown() {
    let (mut arena, attacker, victim) = two_car_arena();
    arena.set_car_state(
        attacker,
        CarCore::grounded(
            PhysState {
                pos: Vec3A::new(0.0, 0.0, START_Z),
                rot_mat: Mat3A::IDENTITY,
                vel: Vec3A::new(2300.0, 0.0, 0.0),
                ang_vel: Vec3A::ZERO,
            },
            0.0,
        ),
    );
    arena.set_car_state(
        victim,
        CarCore::grounded(
            PhysState {
                pos: Vec3A::new(110.0, 0.0, START_Z),
                rot_mat: Mat3A::IDENTITY,
                vel: Vec3A::ZERO,
                ang_vel: Vec3A::ZERO,
            },
            0.0,
        ),
    );
    arena.edit_car_state_full(attacker, |state| {
        state.is_supersonic = true;
        state.controls = CarControls::default();
        state.bump_cooldown_timer = consts::car::bump::COOLDOWN_TIME;
        state.bump_cooldown_victim = Some(victim);
    });

    arena.step_tick();

    assert!(arena.get_car_state(victim).is_demoed);
    assert!(
        bump_events(&arena)
            .iter()
            .any(|(bumper, target, is_demo)| *bumper == attacker && *target == victim && *is_demo)
    );
}

/// Two cars that demolish each other on one tick both end up demolished.
///
/// This is a behavior guard. It also passes on the baseline, because the
/// baseline's early return is outside the two-way loop. The same-tick rule
/// itself is covered by `same_tick_demo_gate_rejects_a_different_demoer_and_blocks_bumps`.
#[test]
fn same_tick_mutual_demolition_sets_both_cars() {
    let (mut arena, first, second) = two_car_arena();
    arena.set_car_state_full(
        first,
        car_state(
            Vec3A::new(-55.0, 0.0, START_Z),
            0.0,
            Vec3A::new(2300.0, 0.0, 0.0),
        ),
    );
    arena.set_car_state_full(
        second,
        car_state(
            Vec3A::new(55.0, 0.0, START_Z),
            std::f32::consts::PI,
            Vec3A::new(-2300.0, 0.0, 0.0),
        ),
    );
    for car in [first, second] {
        arena.edit_car_state_full(car, |state| state.is_supersonic = true);
    }

    arena.step_tick();

    assert!(arena.get_car_state(first).is_demoed);
    assert!(arena.get_car_state(second).is_demoed);
}
