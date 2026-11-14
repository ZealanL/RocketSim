use std::sync::Once;

use glam::{Mat3A, Quat, Vec3A};
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarState, GameMode, Team};

const NATIVE_STICKY_DELTA_BT_PER_SECOND: f32 = 0.054_167;
const UU_TO_BT: f32 = 0.02;
const NATIVE_BASE_SCALE: f32 = 0.5;
const NATIVE_GRAVITY_REFERENCE: f32 = 650.0;
const RESIDUAL_TOLERANCE: f32 = 1e-4;
const GRAVITIES: [f32; 6] = [-650.0, -325.0, -0.0, 0.0, 325.0, 650.0];

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

fn sticky_delta(body: CarBodyConfig, gravity_z: f32) -> (Vec3A, Vec3A, usize) {
    sticky_delta_with(body, gravity_z, -8.81, 0.0)
}

fn sticky_delta_with(
    body: CarBodyConfig,
    gravity_z: f32,
    tilt_deg: f32,
    throttle: f32,
) -> (Vec3A, Vec3A, usize) {
    ensure_init();
    let mut config = ArenaConfig::new(GameMode::Soccar);
    config.mutators.gravity = Vec3A::new(0.0, 0.0, gravity_z);
    let mut arena = Arena::new_with_config(config);
    let car = arena.add_car(Team::Blue, body);

    let mut state = CarState::DEFAULT;
    state.phys.pos = Vec3A::new(-1125.4, 5010.0, 20.1);
    state.phys.rot_mat = Mat3A::from_quat(Quat::from_rotation_x(tilt_deg.to_radians()));
    state.controls.throttle = throttle;
    arena.set_car_state_full(car, state);
    arena.step_tick();

    let warmed = *arena.get_car_state(car);
    let contacts: Vec<Vec3A> = warmed
        .wheels_with_contact()
        .iter()
        .filter_map(|contact| contact.map(|hit| hit.hit_normal))
        .collect();
    let contact_count = contacts.len();
    let normal_sum = contacts.into_iter().sum::<Vec3A>();
    let normal_dir = normal_sum.normalize();

    let mut sticky = arena.clone();
    let mut gate_cleared = arena;
    gate_cleared.reset_car_transient_contacts(car);

    sticky.step_tick();
    gate_cleared.step_tick();

    let active_minus_cleared = sticky.get_car_state(car).vel - gate_cleared.get_car_state(car).vel;
    (active_minus_cleared, normal_dir, contact_count)
}

#[test]
fn sticky_vertical_gravity_sign_matches_native_algebra() {
    // The ecbc sticky scalar is -abs(g) for stable finite nonzero getter reads.
    // Its helper keeps the summed wheel-normal orientation. This Arena test checks
    // that the sticky delta is opposite that normal for either gravity sign.
    // The magnitude reference is the recorded 0.054167 BT/s at scale 0.5.
    // It proves only finite vertical sign behavior; it does not identify getter units.
    let reference_delta_uu_per_tick = NATIVE_STICKY_DELTA_BT_PER_SECOND / UU_TO_BT;
    let mut maximum_error = 0.0_f32;

    for (body_name, body) in [
        ("Octane", CarBodyConfig::OCTANE),
        ("Psyclops", CarBodyConfig::PSYCLOPS),
    ] {
        let scale = if body.three_wheels {
            0.0
        } else {
            NATIVE_BASE_SCALE
        };

        for gravity_z in GRAVITIES {
            let (actual, normal_dir, contact_count) = sticky_delta(body, gravity_z);
            let expected = -normal_dir
                * reference_delta_uu_per_tick
                * (gravity_z.abs() / NATIVE_GRAVITY_REFERENCE)
                * (scale / NATIVE_BASE_SCALE);
            let error = (actual - expected).length();
            maximum_error = maximum_error.max(error);

            assert!(
                contact_count > 0,
                "{body_name} gravity_z={gravity_z} produced no prior wheel contacts"
            );
            assert!(
                normal_dir.z.abs() < 0.995,
                "{body_name} gravity_z={gravity_z} did not produce tilted wheel normals: {normal_dir:?}"
            );
            assert!(
                error < RESIDUAL_TOLERANCE,
                "{body_name} gravity_z={gravity_z} active-minus-cleared delta={actual:?}, expected={expected:?}, error={error}"
            );
        }
    }

    println!("maximum sticky-delta vector error: {maximum_error:.9} uu/s");
}

/// Warm one tick without gravity, then apply gravity and one measured tick.
///
/// Both runs start from the same pose and forward speed above the full-stick
/// threshold. The warm tick at zero gravity produces identical contact state
/// for the three-wheel and four-wheel configs, so the measured velocity
/// difference isolates the sticky scale difference.
fn full_stick_velocity(body: CarBodyConfig, gravity_z: f32) -> (Vec3A, Vec3A) {
    ensure_init();
    let mut config = ArenaConfig::new(GameMode::Soccar);
    config.mutators.gravity = Vec3A::ZERO;
    let mut arena = Arena::new_with_config(config);
    let car = arena.add_car(Team::Blue, body);

    let mut state = CarState::DEFAULT;
    state.phys.pos = Vec3A::new(-1125.4, 5010.0, 20.1);
    // Pitch tilt. The contact normals tilt in X, so a pure forward speed
    // keeps the lateral wheel velocity at zero and the three-wheel lateral
    // friction curve cannot affect the comparison.
    state.phys.rot_mat = Mat3A::from_quat(Quat::from_rotation_y(15.0_f32.to_radians()));
    state.phys.vel = Vec3A::new(100.0, 0.0, 0.0);
    arena.set_car_state_full(car, state);
    arena.step_tick();

    // Keep the two configs on the same linear and angular state. The
    // contact cache is preserved.
    arena.edit_car_state_full(car, |state| {
        let forward = state.get_forward_dir();
        state.phys.vel = forward * 100.0;
        state.phys.ang_vel = Vec3A::ZERO;
    });

    let warmed = *arena.get_car_state(car);
    let normal_dir = warmed
        .wheels_with_contact()
        .iter()
        .filter_map(|contact| contact.map(|hit| hit.hit_normal))
        .sum::<Vec3A>()
        .normalize();

    let mut mutators = ArenaConfig::new(GameMode::Soccar).mutators;
    mutators.gravity = Vec3A::new(0.0, 0.0, gravity_z);
    arena.set_mutator_config(mutators);
    arena.step_tick();

    (arena.get_car_state(car).vel, normal_dir)
}

/// Full stick uses the target's folded scale.
///
/// Three-wheel cars use `1.5 * (1 - |up.z|)`; four-wheel cars use
/// `1.5 - |up.z|`. The same Psyclops body with only `three_wheels` changed
/// gives a measured difference that isolates the folded expression.
#[test]
fn three_wheel_full_stick_uses_one_point_five_scale() {
    const FULL_STICK_TOLERANCE: f32 = 0.05;
    let mut four_wheel = CarBodyConfig::PSYCLOPS;
    four_wheel.three_wheels = false;
    let three_wheel = CarBodyConfig::PSYCLOPS;
    let mut maximum_error = 0.0_f32;

    for gravity_z in [-650.0, -325.0, 325.0, 650.0] {
        let (three_vel, three_normal) = full_stick_velocity(three_wheel, gravity_z);
        let (four_vel, four_normal) = full_stick_velocity(four_wheel, gravity_z);

        assert!(
            (three_normal - four_normal).length() < 1e-6,
            "gravity_z={gravity_z} campaign normals diverged: {three_normal:?} vs {four_normal:?}"
        );
        assert!(
            three_normal.z.abs() < 0.995,
            "gravity_z={gravity_z} did not produce tilted wheel normals: {three_normal:?}"
        );

        let z = three_normal.z.abs();
        let scale_three = 1.5 * (1.0 - z);
        let scale_four = 1.5 - z;
        // Compare the component along the contact normal. The small
        // perpendicular difference is wheel-friction coupling and is not
        // the sticky scale under test.
        let expected = -(NATIVE_STICKY_DELTA_BT_PER_SECOND / UU_TO_BT)
            * (gravity_z.abs() / NATIVE_GRAVITY_REFERENCE)
            * ((scale_three - scale_four) / NATIVE_BASE_SCALE);
        let actual = (three_vel - four_vel).dot(three_normal);
        let error = (actual - expected).abs();
        maximum_error = maximum_error.max(error);

        assert!(
            error < FULL_STICK_TOLERANCE,
            "gravity_z={gravity_z} three-minus-four delta={actual:?}, expected={expected:?}, error={error}, normal={three_normal:?}"
        );
    }

    println!("maximum three-minus-four full-stick delta error: {maximum_error:.9} uu/s");
}

/// The full-stick gate reads the quantized throttle, not the raw axis.
///
/// `0.002` quantizes to zero, so it must produce the same sticky force as a
/// zero throttle at the same speed. A raw-input gate would open full stick.
#[test]
fn tiny_throttle_does_not_open_the_full_stick_gate() {
    let zero = sticky_delta_with(CarBodyConfig::OCTANE, -650.0, -8.81, 0.0);
    let tiny = sticky_delta_with(CarBodyConfig::OCTANE, -650.0, -8.81, 0.002);

    assert!(zero.2 > 0, "tilted pose produced no wheel contacts");
    assert!(
        zero.1.z.abs() < 0.995,
        "tilted pose did not produce a usable normal: {:?}",
        zero.1
    );
    assert!(
        (zero.0 - tiny.0).length() < 1e-3,
        "tiny throttle changed the sticky force: zero={:?}, tiny={:?}",
        zero.0,
        tiny.0
    );
}
