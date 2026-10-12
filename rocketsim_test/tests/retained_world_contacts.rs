//! Retained car-world contacts must supply the next auto-roll update.
//!
//! These Arena tests use poses from `partycannon_3v3_300s.rlpr.zst`, v9.
//! Recording SHA-256:
//! `d31f18f13cd4e0567b39ee2329faedea5b3ce3a35e00d056f265a24e9a86c88e`
//! The executable build for this recording is unknown.
//! The tests do not load the recording or change contact geometry.

use glam::{Mat3A, Vec3A};
use rocketsim::{Arena, ArenaEvent, CarBodyConfig, CarControls, CarState, GameMode, Team, init};

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
    let mut state = CarState::DEFAULT;
    state.phys.pos = Vec3A::new(1550.600830, 5071.119141, 772.524719);
    state.phys.rot_mat = rot_mat;
    state.phys.vel = Vec3A::new(249.703308, 543.539124, 313.766296);
    state.phys.ang_vel = Vec3A::new(-4.736583, -1.521454, -3.436491);
    state.is_on_ground = false;
    state.wheels_with_contact = [None, None, None, None];
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
    state.world_contact_normal = None;
    state
}

/// Recorded tick-2998 car0 state (restored pose for the 2998->2999 step).
fn pose_2998() -> CarState {
    let rot_mat = Mat3A::from_cols(
        Vec3A::new(-0.989788, 0.021477, 0.140922),
        Vec3A::new(0.028721, 0.998358, 0.049572),
        Vec3A::new(-0.139626, 0.053113, -0.988779),
    );
    let mut state = CarState::DEFAULT;
    state.phys.pos = Vec3A::new(1553.067871, 5072.542480, 775.086670);
    state.phys.rot_mat = rot_mat;
    state.phys.vel = Vec3A::new(296.154144, 245.674637, 308.010040);
    state.phys.ang_vel = Vec3A::new(-9.909379, -0.863449, 6.000502);
    state.is_on_ground = false;
    state.wheels_with_contact = [None, None, None, None];
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
    state.world_contact_normal = None;
    state
}

/// Recorded tick-2999 car0 state (restored pose for the 2999->3000 step).
fn pose_2999() -> CarState {
    let rot_mat = Mat3A::from_cols(
        Vec3A::new(-0.990587, 0.001369, 0.136878),
        Vec3A::new(0.001816, 0.999993, 0.003136),
        Vec3A::new(-0.136873, 0.003355, -0.990583),
    );
    let mut state = CarState::DEFAULT;
    state.phys.pos = Vec3A::new(1555.501465, 5074.479492, 777.599121);
    state.phys.rot_mat = rot_mat;
    state.phys.vel = Vec3A::new(291.794250, 232.792191, 301.104340);
    state.phys.ang_vel = Vec3A::new(-5.582328, -0.425622, 3.218727);
    state.is_on_ground = false;
    state.wheels_with_contact = [None, None, None, None];
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
    state.world_contact_normal = None;
    state
}

/// Recorded tick-3000 car0 state (restored pose for the 3000->3001 step).
fn pose_3000() -> CarState {
    let rot_mat = Mat3A::from_cols(
        Vec3A::new(-0.990956, -0.015286, 0.133313),
        Vec3A::new(-0.020495, 0.999076, -0.037784),
        Vec3A::new(-0.132612, -0.040175, -0.990354),
    );
    let mut state = CarState::DEFAULT;
    state.phys.pos = Vec3A::new(1557.927002, 5076.427246, 780.064636);
    state.phys.rot_mat = rot_mat;
    state.phys.vel = Vec3A::new(291.240692, 233.625076, 295.760406);
    state.phys.ang_vel = Vec3A::new(-4.907515, -0.465823, 2.685506);
    state.is_on_ground = false;
    state.wheels_with_contact = [None, None, None, None];
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
    state.world_contact_normal = None;
    state
}

/// Recorded tick-3001 car0 state (restored pose for the heldout 3001->3002 step).
fn pose_3001() -> CarState {
    let rot_mat = Mat3A::from_cols(
        Vec3A::new(-0.990156, -0.012000, 0.139451),
        Vec3A::new(-0.028433, 0.992789, -0.116456),
        Vec3A::new(-0.137048, -0.119275, -0.983357),
    );
    let mut state = CarState::DEFAULT;
    state.phys.pos = Vec3A::new(1559.802246, 5076.241211, 782.210388);
    state.phys.rot_mat = rot_mat;
    state.phys.vel = Vec3A::new(224.687531, -11.776859, 258.051025);
    state.phys.ang_vel = Vec3A::new(-9.476849, 0.613153, 0.932627);
    state.is_on_ground = false;
    state.wheels_with_contact = [None, None, None, None];
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
    state.world_contact_normal = None;
    state
}

/// Recorded tick-3002 car0 state (restored pose for the heldout 3002->3003 step).
fn pose_3002() -> CarState {
    let rot_mat = Mat3A::from_cols(
        Vec3A::new(-0.989832, -0.009496, 0.141925),
        Vec3A::new(-0.033414, 0.985372, -0.167106),
        Vec3A::new(-0.138262, -0.170149, -0.975670),
    );
    let mut state = CarState::DEFAULT;
    state.phys.pos = Vec3A::new(1561.667603, 5076.148438, 784.315857);
    state.phys.rot_mat = rot_mat;
    state.phys.vel = Vec3A::new(224.130920, -10.944331, 252.711792);
    state.phys.ang_vel = Vec3A::new(-6.150740, 0.233085, 0.570879);
    state.is_on_ground = false;
    state.wheels_with_contact = [None, None, None, None];
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
    state.world_contact_normal = None;
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
    s.controls = controls_2999();
    s.prev_controls = controls_2999();
    arena.set_car_state(car, s);
    arena.set_car_controls(car, controls_2999());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(
        has_car_world(&events, car),
        "fresh wall contact at pose2997 must report an event"
    );

    // Retained-only leg: no fresh point at pose2998 (BVH visits nothing new),
    // but the cache retains survivors. Must report world state postpatch.
    let mut s = pose_2998();
    s.controls = controls_2999();
    s.prev_controls = controls_2999();
    arena.set_car_state(car, s);
    arena.set_car_controls(car, controls_2999());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(
        has_car_world(&events, car),
        "retained wall contact at pose2998 must report an event (processed callback)"
    );
    let world = arena.get_car_state(car).world_contact_normal;
    assert!(
        matches!(world, Some(n) if (n - Vec3A::new(0.0, -1.0, 0.0)).length() < 1e-3),
        "retained wall contact must set world normal (0,-1,0), got {:?}",
        world
    );

    // Repair leg: carry world across set_state exactly like V3Backend (which
    // preserves the field), then auto-roll must fire and match truth3000.
    let mut s = pose_2999();
    s.world_contact_normal = arena.get_car_state(car).world_contact_normal;
    s.controls = controls_3000();
    s.prev_controls = controls_3000();
    arena.set_car_state(car, s);
    arena.set_car_controls(car, controls_3000());
    arena.step_tick();
    let after = *arena.get_car_state(car);
    assert!(
        (after.phys.vel.y - 233.625076).abs() < 0.25,
        "auto-roll linear must apply (vel.y near 233.63), got {}",
        after.phys.vel.y
    );
    assert!(
        (after.phys.ang_vel.x + 4.907515).abs() < 0.2,
        "auto-roll torque must apply (ang.x near -4.9075), got {}",
        after.phys.ang_vel.x
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
    s.controls = controls_3001();
    s.prev_controls = controls_3001();
    arena.set_car_state(car, s);
    arena.set_car_controls(car, controls_3001());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(
        has_car_world(&events, car),
        "fresh wall contact at pose3000 must report an event"
    );

    // Retained-only leg at pose3001.
    let mut s = pose_3001();
    s.controls = controls_3002();
    s.prev_controls = controls_3002();
    arena.set_car_state(car, s);
    arena.set_car_controls(car, controls_3002());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(
        has_car_world(&events, car),
        "retained wall contact at pose3001 must report an event (processed callback)"
    );
    assert!(
        arena.get_car_state(car).world_contact_normal.is_some(),
        "retained wall contact must set world state"
    );

    // Repair leg at pose3002 with carried world.
    let mut s = pose_3002();
    s.world_contact_normal = arena.get_car_state(car).world_contact_normal;
    s.controls = controls_3003();
    s.prev_controls = controls_3003();
    arena.set_car_state(car, s);
    arena.set_car_controls(car, controls_3003());
    arena.step_tick();
    let after = *arena.get_car_state(car);
    assert!(
        (after.phys.vel.y + 10.112940).abs() < 0.25,
        "auto-roll linear must apply (vel.y near -10.11), got {}",
        after.phys.vel.y
    );
    assert!(
        (after.phys.ang_vel.x + 6.126513).abs() < 0.2,
        "auto-roll torque must apply (ang.x near -6.1265), got {}",
        after.phys.ang_vel.x
    );
}

/// Non-activation: far from any wall, no event and no world pre and postpatch.
#[test]
fn retained_report_stays_quiet_far_from_wall() {
    init_meshes();
    let (mut arena, car) = new_soccar_one_octane();
    let mut s = pose_2998();
    s.phys.pos -= Vec3A::new(0.0, 60.0, 0.0);
    s.controls = controls_2999();
    s.prev_controls = controls_2999();
    arena.set_car_state(car, s);
    arena.set_car_controls(car, controls_2999());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(
        !has_car_world(&events, car),
        "far pose (105 UU from wall) must report no contact"
    );
    assert!(
        arena.get_car_state(car).world_contact_normal.is_none(),
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
    s.controls = controls_2999();
    s.prev_controls = controls_2999();
    arena.set_car_state(car, s);
    arena.set_car_controls(car, controls_2999());
    let events: Vec<ArenaEvent> = arena.step_tick().to_vec();
    assert!(has_car_world(&events, car), "warmup contact must exist");

    // Teleport far: refresh must cull everything, no report either way.
    let mut s = pose_2998();
    s.phys.pos -= Vec3A::new(0.0, 60.0, 0.0);
    s.controls = controls_2999();
    s.prev_controls = controls_2999();
    arena.set_car_state(car, s);
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
        arena.get_car_state(car).world_contact_normal.is_none(),
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
    let mut sa = CarState::DEFAULT;
    sa.phys.pos = Vec3A::new(1400.0, 4000.0, 500.0);
    sa.phys.vel = Vec3A::ZERO;
    sa.phys.ang_vel = Vec3A::ZERO;
    sa.is_on_ground = false;
    let mut sb = CarState::DEFAULT;
    sb.phys.pos = Vec3A::new(1490.5, 4000.0, 500.0);
    sb.phys.vel = Vec3A::ZERO;
    sb.phys.ang_vel = Vec3A::ZERO;
    sb.is_on_ground = false;
    arena.set_car_state(a, sa);
    arena.set_car_state(b, sb);
    let neutral = CarControls::DEFAULT;
    arena.set_car_controls(a, neutral);
    arena.set_car_controls(b, neutral);

    let first: Vec<ArenaEvent> = arena.step_tick().to_vec();
    for car in [a, b] {
        assert!(!has_car_world(&first, car));
        assert!(arena.get_car_state(car).world_contact_normal.is_none());
    }
    // Check that the solver processed the overlapping pair.
    let pa = arena.get_car_state(a).phys.pos;
    let pb = arena.get_car_state(b).phys.pos;
    assert!(
        (pa - pb).length() > 105.0,
        "solver rows must have separated the overlap, distance {}",
        (pa - pb).length()
    );
}
