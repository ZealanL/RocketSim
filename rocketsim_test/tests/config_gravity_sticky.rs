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
    ensure_init();
    let mut config = ArenaConfig::new(GameMode::Soccar);
    config.mutators.gravity = Vec3A::new(0.0, 0.0, gravity_z);
    let mut arena = Arena::new_with_config(config);
    let car = arena.add_car(Team::Blue, body);

    let mut state = CarState::DEFAULT;
    state.phys.pos = Vec3A::new(-1125.4, 5010.0, 20.1);
    state.phys.rot_mat = Mat3A::from_quat(Quat::from_rotation_x(-8.81_f32.to_radians()));
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
