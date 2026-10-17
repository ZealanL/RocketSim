//! Retained car-world contacts must supply the next auto-roll update.
//!
//! These Arena tests use poses from `partycannon_3v3_300s.rlpr.zst`, v9.
//! Recording SHA-256:
//! `d31f18f13cd4e0567b39ee2329faedea5b3ce3a35e00d056f265a24e9a86c88e`
//! The executable build for this recording is unknown.
//! The tests do not load the recording or change contact geometry.

use glam::{Mat3A, Vec3A};
use rocketsim::{
    Arena, ArenaEvent, CarBodyConfig, CarControls, CarCore, CarState, GameMode, Team, init,
};
use rocketsim_test::rlpr::cpp_records::copy_recorded_contact_cache;

fn apply_recorded_pose(live: &mut CarState, recorded: &CarState) {
    *live = copy_recorded_contact_cache(live, recorded);
    live.set_prev_controls(*recorded.prev_controls());
    live.pos = recorded.pos;
    live.rot_mat = recorded.rot_mat;
    live.vel = recorded.vel;
    live.ang_vel = recorded.ang_vel;
    live.controls = recorded.controls;
    live.has_jumped = recorded.has_jumped;
    live.has_double_jumped = recorded.has_double_jumped;
    live.has_flipped = recorded.has_flipped;
    live.flip_rel_torque = recorded.flip_rel_torque;
    live.jump_ticks = recorded.jump_ticks;
    live.flip_time = recorded.flip_time;
    live.is_flipping = recorded.is_flipping;
    live.is_jumping = recorded.is_jumping;
    live.jump_hold_broken = recorded.jump_hold_broken;
    live.air_time_since_jump = recorded.air_time_since_jump;
    live.boost = recorded.boost;
    live.is_auto_flipping = recorded.is_auto_flipping;
    live.auto_flip_timer = recorded.auto_flip_timer;
    live.auto_flip_torque_scale = recorded.auto_flip_torque_scale;
    live.bump_cooldown_timer = recorded.bump_cooldown_timer;
    live.bump_cooldown_victim = recorded.bump_cooldown_victim;
    live.last_extra_hit_tick = recorded.last_extra_hit_tick;
    live.is_demoed = recorded.is_demoed;
    live.demo_respawn_timer = recorded.demo_respawn_timer;
    live.handbrake_val = recorded.handbrake_val;
    live.is_supersonic = recorded.is_supersonic;
    live.supersonic_grace_timer = recorded.supersonic_grace_timer;
    live.time_since_boosted = recorded.time_since_boosted;
    live.boosting_time = recorded.boosting_time;
    live.is_boosting = recorded.is_boosting;
    live.air_time = recorded.air_time;
}

fn set_controls_and_previous(state: &mut CarState, controls: CarControls) {
    state.controls = controls;
    state.set_prev_controls(controls);
}

fn init_meshes() {
    init(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../collision_meshes"),
        true,
    )
    .expect("load collision meshes");
}

/// Recorded tick-2997 car0 state (restored pose for the 2997->2998 step).
fn pose_2997() -> CarState {
    // rot rows (recorder Mat3Record rows); columns rebuild rot_mat exactly
    // like PhysState::from (from_cols(column(0), column(1), column(2))).
    let rot_mat = Mat3A::from_cols(
        Vec3A::new(-0.986590, 0.061216, 0.151302),
        Vec3A::new(0.081757, 0.987671, 0.133501),
        Vec3A::new(-0.141264, 0.144080, -0.979431),
    );
    let mut state = CarState::from(CarCore::aerial(CarState::DEFAULT.core().phys));
    state.pos = Vec3A::new(1_550.600_8, 5_071.119, 772.524_7);
    state.rot_mat = rot_mat;
    state.vel = Vec3A::new(249.703_31, 543.539_1, 313.766_3);
    state.ang_vel = Vec3A::new(-4.736583, -1.521454, -3.436491);
    state.has_jumped = true;
    state.has_double_jumped = true;
    state.has_flipped = false;
    state.is_flipping = false;
    state.is_jumping = false;
    state.jump_ticks = 177;
    state.flip_time = 12.008236;
    // Boost button is false on these ticks, so the amount is inert; the
    // value mirrors the neighboring recorded amount.
    state.boost = 69.33;
    state.handbrake_val = 0.0;
    state.air_time = 5.0;
    state.air_time_since_jump = 5.0;
    state
}

/// Recorded tick-2998 car0 state (restored pose for the 2998->2999 step).
fn pose_2998() -> CarState {
    let rot_mat = Mat3A::from_cols(
        Vec3A::new(-0.989788, 0.021477, 0.140922),
        Vec3A::new(0.028721, 0.998358, 0.049572),
        Vec3A::new(-0.139626, 0.053113, -0.988779),
    );
    let mut state = CarState::from(CarCore::aerial(CarState::DEFAULT.core().phys));
    state.pos = Vec3A::new(1_553.067_9, 5_072.542_5, 775.086_7);
    state.rot_mat = rot_mat;
    state.vel = Vec3A::new(296.154_14, 245.674_64, 308.010_04);
    state.ang_vel = Vec3A::new(-9.909379, -0.863449, 6.000502);
    state.has_jumped = true;
    state.has_double_jumped = true;
    state.has_flipped = false;
    state.is_flipping = false;
    state.is_jumping = false;
    state.jump_ticks = 178;
    state.flip_time = 12.016569;
    state.boost = 69.33;
    state.handbrake_val = 0.0;
    state.air_time = 5.0;
    state.air_time_since_jump = 5.0;
    state
}

/// Recorded tick-2999 car0 state (restored pose for the 2999->3000 step).
fn pose_2999() -> CarState {
    let rot_mat = Mat3A::from_cols(
        Vec3A::new(-0.990587, 0.001369, 0.136878),
        Vec3A::new(0.001816, 0.999993, 0.003136),
        Vec3A::new(-0.136873, 0.003355, -0.990583),
    );
    let mut state = CarState::from(CarCore::aerial(CarState::DEFAULT.core().phys));
    state.pos = Vec3A::new(1_555.501_5, 5_074.479_5, 777.599_1);
    state.rot_mat = rot_mat;
    state.vel = Vec3A::new(291.794_25, 232.792_19, 301.104_34);
    state.ang_vel = Vec3A::new(-5.582328, -0.425622, 3.218727);
    state.has_jumped = true;
    state.has_double_jumped = true;
    state.has_flipped = false;
    state.is_flipping = false;
    state.is_jumping = false;
    state.jump_ticks = 179;
    state.flip_time = 12.024902;
    state.boost = 69.33;
    state.handbrake_val = 0.0;
    state.air_time = 5.0;
    state.air_time_since_jump = 5.0;
    state
}

/// Recorded tick-3000 car0 state (restored pose for the 3000->3001 step).
fn pose_3000() -> CarState {
    let rot_mat = Mat3A::from_cols(
        Vec3A::new(-0.990956, -0.015286, 0.133313),
        Vec3A::new(-0.020495, 0.999076, -0.037784),
        Vec3A::new(-0.132612, -0.040175, -0.990354),
    );
    let mut state = CarState::from(CarCore::aerial(CarState::DEFAULT.core().phys));
    state.pos = Vec3A::new(1_557.927, 5_076.427_2, 780.064_64);
    state.rot_mat = rot_mat;
    state.vel = Vec3A::new(291.240_7, 233.625_08, 295.760_4);
    state.ang_vel = Vec3A::new(-4.907515, -0.465823, 2.685506);
    state.has_jumped = true;
    state.has_double_jumped = true;
    state.has_flipped = false;
    state.is_flipping = false;
    state.is_jumping = false;
    state.jump_ticks = 180;
    state.flip_time = 12.033236;
    state.boost = 69.33;
    state.handbrake_val = 0.0;
    state.air_time = 5.0;
    state.air_time_since_jump = 5.0;
    state
}

/// Recorded tick-3001 car0 state (restored pose for the heldout 3001->3002 step).
fn pose_3001() -> CarState {
    let rot_mat = Mat3A::from_cols(
        Vec3A::new(-0.990156, -0.012000, 0.139451),
        Vec3A::new(-0.028433, 0.992789, -0.116456),
        Vec3A::new(-0.137048, -0.119275, -0.983357),
    );
    let mut state = CarState::from(CarCore::aerial(CarState::DEFAULT.core().phys));
    state.pos = Vec3A::new(1_559.802_2, 5_076.241, 782.210_4);
    state.rot_mat = rot_mat;
    state.vel = Vec3A::new(224.687_53, -11.776859, 258.051_03);
    state.ang_vel = Vec3A::new(-9.476849, 0.613153, 0.932627);
    state.has_jumped = true;
    state.has_double_jumped = true;
    state.has_flipped = false;
    state.is_flipping = false;
    state.is_jumping = false;
    state.jump_ticks = 181;
    state.flip_time = 12.041569;
    state.boost = 69.33;
    state.handbrake_val = 0.0;
    state.air_time = 5.0;
    state.air_time_since_jump = 5.0;
    state
}

/// Recorded tick-3002 car0 state (restored pose for the heldout 3002->3003 step).
fn pose_3002() -> CarState {
    let rot_mat = Mat3A::from_cols(
        Vec3A::new(-0.989832, -0.009496, 0.141925),
        Vec3A::new(-0.033414, 0.985372, -0.167106),
        Vec3A::new(-0.138262, -0.170149, -0.975670),
    );
    let mut state = CarState::from(CarCore::aerial(CarState::DEFAULT.core().phys));
    state.pos = Vec3A::new(1_561.667_6, 5_076.148_4, 784.315_86);
    state.rot_mat = rot_mat;
    state.vel = Vec3A::new(224.130_92, -10.944331, 252.711_79);
    state.ang_vel = Vec3A::new(-6.150_74, 0.233085, 0.570879);
    state.has_jumped = true;
    state.has_double_jumped = true;
    state.has_flipped = false;
    state.is_flipping = false;
    state.is_jumping = false;
    state.jump_ticks = 182;
    state.flip_time = 12.049902;
    state.boost = 69.33;
    state.handbrake_val = 0.0;
    state.air_time = 5.0;
    state.air_time_since_jump = 5.0;
    state
}

/// Recorded tick-2999 `prev_controls` (drives the 2998->2999 transition).
fn controls_2999() -> CarControls {
    CarControls {
        throttle: 1.0,
        steer: -1.0,
        pitch: 1.0,
        yaw: -1.0,
        roll: 1.0,
        jump: false,
        boost: false,
        handbrake: false,
    }
}

/// Recorded tick-3000 `prev_controls` (drives the 2999->3000 transition).
fn controls_3000() -> CarControls {
    CarControls {
        throttle: 1.0,
        steer: 1.0,
        pitch: 1.0,
        yaw: 1.0,
        roll: 1.0,
        jump: false,
        boost: false,
        handbrake: false,
    }
}

/// Recorded tick-3001 `prev_controls` (drives the 3000->3001 transition).
fn controls_3001() -> CarControls {
    CarControls {
        throttle: 1.0,
        steer: 1.0,
        pitch: 1.0,
        yaw: 1.0,
        roll: -1.0,
        jump: false,
        boost: false,
        handbrake: false,
    }
}

/// Recorded tick-3002 `prev_controls` (drives the heldout 3001->3002 transition).
fn controls_3002() -> CarControls {
    CarControls {
        throttle: 1.0,
        steer: 1.0,
        pitch: 1.0,
        yaw: 1.0,
        roll: -1.0,
        jump: false,
        boost: false,
        handbrake: false,
    }
}

/// Recorded tick-3003 `prev_controls` (drives the heldout 3002->3003 transition).
fn controls_3003() -> CarControls {
    CarControls {
        throttle: 1.0,
        steer: 1.0,
        pitch: 1.0,
        yaw: 1.0,
        roll: -1.0,
        jump: false,
        boost: false,
        handbrake: false,
    }
}

fn has_car_world(events: &[ArenaEvent], car: usize) -> bool {
    events
        .iter()
        .any(|e| matches!(e, ArenaEvent::CarHitWorld(hit) if hit.car_idx == car))
}

fn count_car_world(events: &[ArenaEvent], car: usize) -> usize {
    events
        .iter()
        .filter(|e| matches!(e, ArenaEvent::CarHitWorld(hit) if hit.car_idx == car))
        .count()
}

fn new_soccar_one_octane() -> (Arena, usize) {
    let mut arena = Arena::new(GameMode::Soccar);
    let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    (arena, car)
}

/// Primary chain: fresh contact tick, then retained-only tick reports world
/// state, then the auto-roll repair tick matches recorded truth.
///
/// Canonical 2997->2998 (fresh) / 2998->2999 (retained) / 2999->3000 (repair).
/// Prepatch: retained step reports nothing (world None), repair step misses
/// auto-roll (vel.y off by 0.833, ang.x off by 0.660). Postpatch: retained
/// step reports world, repair step matches truth.
#[test]
fn retained_world_report_repairs_primary_chain() {
    init_meshes();
    let (mut arena, car) = new_soccar_one_octane();

    // Fresh leg (passes pre and post): initial wall contact generates.
    let mut s = pose_2997();
    set_controls_and_previous(&mut s, controls_2999());
    arena.set_car_state_full(car, s);
    arena.set_car_controls(car, controls_2999());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(
        has_car_world(&events, car),
        "fresh wall contact at pose2997 must report an event"
    );

    // Retained-only leg: no fresh point at pose2998 (BVH visits nothing new),
    // but the cache retains survivors. Must report world state postpatch.
    let recorded = pose_2998();
    let mut s = *arena.get_car_state(car);
    apply_recorded_pose(&mut s, &recorded);
    set_controls_and_previous(&mut s, controls_2999());
    arena.set_car_state_full(car, s);
    arena.set_car_controls(car, controls_2999());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(
        has_car_world(&events, car),
        "retained wall contact at pose2998 must report an event (processed callback)"
    );
    let world = arena.get_car_state(car).world_contact_normal();
    assert!(
        matches!(world, Some(n) if (n - Vec3A::new(0.0, -1.0, 0.0)).length() < 1e-3),
        "retained wall contact must set world normal (0,-1,0), got {:?}",
        world
    );

    // Repair leg: carry world across set_state exactly like V3Backend (which
    // preserves the field), then auto-roll must fire and match truth3000.
    let recorded = pose_2999();
    let mut s = *arena.get_car_state(car);
    apply_recorded_pose(&mut s, &recorded);
    set_controls_and_previous(&mut s, controls_3000());
    arena.set_car_state_full(car, s);
    arena.set_car_controls(car, controls_3000());
    arena.step_tick();
    let after = *arena.get_car_state(car);
    assert!(
        (after.vel.y - 233.625_08).abs() < 0.25,
        "auto-roll linear must apply (vel.y near 233.63), got {}",
        after.vel.y
    );
    assert!(
        (after.ang_vel.x + 4.907515).abs() < 0.2,
        "auto-roll torque must apply (ang.x near -4.9075), got {}",
        after.ang_vel.x
    );
}

/// Heldout chain: same three-leg shape on ticks 3000->3001->3002->3003
/// (retained pair, roll sign differs). Truth3003 vel.y is -10.112940 and
/// ang.x is -6.126513; prepatch misses by 0.833 and 0.548.
#[test]
fn retained_world_report_repairs_heldout_chain() {
    init_meshes();
    let (mut arena, car) = new_soccar_one_octane();

    // Fresh leg at pose3000 (passes pre and post): initial wall contact.
    let mut s = pose_3000();
    set_controls_and_previous(&mut s, controls_3001());
    arena.set_car_state_full(car, s);
    arena.set_car_controls(car, controls_3001());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(
        has_car_world(&events, car),
        "fresh wall contact at pose3000 must report an event"
    );

    // Retained-only leg at pose3001.
    let recorded = pose_3001();
    let mut s = *arena.get_car_state(car);
    apply_recorded_pose(&mut s, &recorded);
    set_controls_and_previous(&mut s, controls_3002());
    arena.set_car_state_full(car, s);
    arena.set_car_controls(car, controls_3002());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(
        has_car_world(&events, car),
        "retained wall contact at pose3001 must report an event (processed callback)"
    );
    assert!(
        arena.get_car_state(car).world_contact_normal().is_some(),
        "retained wall contact must set world state"
    );

    // Repair leg at pose3002 with carried world.
    let recorded = pose_3002();
    let mut s = *arena.get_car_state(car);
    apply_recorded_pose(&mut s, &recorded);
    set_controls_and_previous(&mut s, controls_3003());
    arena.set_car_state_full(car, s);
    arena.set_car_controls(car, controls_3003());
    arena.step_tick();
    let after = *arena.get_car_state(car);
    assert!(
        (after.vel.y + 10.112_94).abs() < 0.25,
        "auto-roll linear must apply (vel.y near -10.11), got {}",
        after.vel.y
    );
    assert!(
        (after.ang_vel.x + 6.126513).abs() < 0.2,
        "auto-roll torque must apply (ang.x near -6.1265), got {}",
        after.ang_vel.x
    );
}

/// Non-activation: far from any wall, no event and no world pre and postpatch.
#[test]
fn retained_report_stays_quiet_far_from_wall() {
    init_meshes();
    let (mut arena, car) = new_soccar_one_octane();
    let recorded = pose_2998();
    let mut s = *arena.get_car_state(car);
    apply_recorded_pose(&mut s, &recorded);
    s.pos -= Vec3A::new(0.0, 60.0, 0.0);
    set_controls_and_previous(&mut s, controls_2999());
    arena.set_car_state_full(car, s);
    arena.set_car_controls(car, controls_2999());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(
        !has_car_world(&events, car),
        "far pose (105 UU from wall) must report no contact"
    );
    assert!(
        arena.get_car_state(car).world_contact_normal().is_none(),
        "far pose must leave world state empty"
    );
}

/// All-dropped: a warmed cache fully culled at a far pose stays silent.
/// Guards the refresh-cull path against the new reporting call.
#[test]
fn retained_report_stays_quiet_when_cache_culls() {
    init_meshes();
    let (mut arena, car) = new_soccar_one_octane();

    // Warm the cache with a fresh wall contact.
    let mut s = pose_2997();
    set_controls_and_previous(&mut s, controls_2999());
    arena.set_car_state_full(car, s);
    arena.set_car_controls(car, controls_2999());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(has_car_world(&events, car), "warmup contact must exist");

    // Teleport far: refresh must cull everything, no report either way.
    let recorded = pose_2998();
    let mut s = *arena.get_car_state(car);
    apply_recorded_pose(&mut s, &recorded);
    s.pos -= Vec3A::new(0.0, 60.0, 0.0);
    set_controls_and_previous(&mut s, controls_2999());
    arena.set_car_state_full(car, s);
    arena.set_car_controls(car, controls_2999());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(
        !has_car_world(&events, car),
        "fully culled cache must stay silent"
    );
    assert!(
        count_car_world(&events, car) == 0,
        "no duplicate or retained event may appear after a full cull"
    );
    assert!(
        arena.get_car_state(car).world_contact_normal().is_none(),
        "fully culled cache must leave world state empty"
    );
}

/// Car-car contacts must not supply a world-contact normal.
#[test]
fn car_car_contacts_do_not_report_world_contact() {
    init_meshes();
    let mut arena = Arena::new(GameMode::Soccar);
    let a = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let b = arena.add_car(Team::Orange, CarBodyConfig::OCTANE);
    // Keep the overlapping cars away from the world geometry.
    let mut sa = CarState::from(CarCore::aerial(CarState::DEFAULT.core().phys));
    sa.pos = Vec3A::new(1400.0, 4000.0, 500.0);
    sa.vel = Vec3A::ZERO;
    sa.ang_vel = Vec3A::ZERO;
    let mut sb = CarState::from(CarCore::aerial(CarState::DEFAULT.core().phys));
    sb.pos = Vec3A::new(1490.5, 4000.0, 500.0);
    sb.vel = Vec3A::ZERO;
    sb.ang_vel = Vec3A::ZERO;
    arena.set_car_state(a, sa.core());
    arena.set_car_state(b, sb.core());
    let neutral = CarControls::DEFAULT;
    arena.set_car_controls(a, neutral);
    arena.set_car_controls(b, neutral);

    let first: Vec<ArenaEvent> = arena.step_tick().to_vec();
    for car in [a, b] {
        assert!(!has_car_world(&first, car));
        assert!(arena.get_car_state(car).world_contact_normal().is_none());
    }
    // Check that the solver processed the overlapping pair.
    let pa = arena.get_car_state(a).pos;
    let pb = arena.get_car_state(b).pos;
    assert!(
        (pa - pb).length() > 105.0,
        "solver rows must have separated the overlap, distance {}",
        (pa - pb).length()
    );
}
