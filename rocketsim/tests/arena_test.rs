use std::sync::Once;

use glam::Vec3A;
use rocketsim::{Arena, CarBodyConfig, GameMode, Team};

static INIT: Once = Once::new();

fn init() {
    INIT.call_once(|| {
        rocketsim::init_from_default(true).unwrap();
    });
}

#[test]
fn test_ball_the_void_always_inside() {
    init();
    let arena = Arena::new(GameMode::TheVoid);
    let ball_state = arena.get_ball_state();

    // Ball at rest should be inside TheVoid
    assert!(arena.is_inside(ball_state.phys, None));

    // Ball way outside should also be inside TheVoid
    let mut phys = ball_state.phys;
    phys.pos = Vec3A::new(100000.0, 100000.0, 100000.0);
    assert!(arena.is_inside(phys, None));
}

#[test]
fn test_ball_on_floor_not_inside() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    let ball_state = arena.get_ball_state();

    // Ball at rest on floor is touching the floor static plane
    assert!(!arena.is_inside(ball_state.phys, None));
}

#[test]
fn test_ball_in_air_inside() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    let mut phys = arena.get_ball_state().phys;

    // Center of arena, well above the floor
    phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    assert!(arena.is_inside(phys, None));

    // Near the wall but not touching
    phys.pos = Vec3A::new(3500.0, 0.0, 1000.0);
    assert!(arena.is_inside(phys, None));

    // Near the ceiling but not touching
    phys.pos = Vec3A::new(0.0, 0.0, 1900.0);
    assert!(arena.is_inside(phys, None));
}

#[test]
fn test_ball_outside_arena() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    let mut phys = arena.get_ball_state().phys;

    // Beyond the wall
    phys.pos = Vec3A::new(5000.0, 0.0, 1000.0);
    assert!(!arena.is_inside(phys, None));

    // Beyond the ceiling
    phys.pos = Vec3A::new(0.0, 0.0, 2500.0);
    assert!(!arena.is_inside(phys, None));

    // Below the floor
    phys.pos = Vec3A::new(0.0, 0.0, -100.0);
    assert!(!arena.is_inside(phys, None));
}

#[test]
fn test_ball_touching_wall_not_inside() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    let mut phys = arena.get_ball_state().phys;

    // Ball touching the side wall (arena max_x is 4096, ball radius is ~92.75)
    // Ball center should be at 4096 - ball_radius to be exactly touching
    let ball_radius = arena.get_config().mutators.ball_radius;
    phys.pos = Vec3A::new(4096.0 - ball_radius, 0.0, 1000.0);
    assert!(!arena.is_inside(phys, None));

    // Ball touching the ceiling
    phys.pos = Vec3A::new(0.0, 0.0, 2048.0 - ball_radius);
    assert!(!arena.is_inside(phys, None));
}

#[test]
fn test_car_in_air_inside() {
    init();
    let mut arena = Arena::new(GameMode::Soccar);
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let car_state = *arena.get_car_state(car_idx);
    let mut phys = car_state.phys;

    // Car in the air at center of arena
    phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    assert!(arena.is_inside(phys, Some(CarBodyConfig::OCTANE)));

    // Car near the wall but not touching
    phys.pos = Vec3A::new(3000.0, 0.0, 1000.0);
    assert!(arena.is_inside(phys, Some(CarBodyConfig::OCTANE)));
}

#[test]
fn test_car_on_floor_inside() {
    init();
    let mut arena = Arena::new(GameMode::Soccar);
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let car_state = *arena.get_car_state(car_idx);

    // Car at rest on floor - hitbox is above the floor, wheels touch floor but are not tested
    assert!(arena.is_inside(car_state.phys, Some(CarBodyConfig::OCTANE)));
}

#[test]
fn test_car_touching_wall_not_inside() {
    init();
    let mut arena = Arena::new(GameMode::Soccar);
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let car_state = *arena.get_car_state(car_idx);
    let mut phys = car_state.phys;

    // Position car well past the wall at x=4096
    // The car hitbox front extends ~73.86 UU from COM, so x=4090 is well past the wall
    phys.pos = Vec3A::new(4090.0, 0.0, 1000.0);
    assert!(!arena.is_inside(phys, Some(CarBodyConfig::OCTANE)));
}

#[test]
fn test_car_outside_arena() {
    init();
    let mut arena = Arena::new(GameMode::Soccar);
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let car_state = *arena.get_car_state(car_idx);
    let mut phys = car_state.phys;

    // Car beyond the wall
    phys.pos = Vec3A::new(5000.0, 0.0, 1000.0);
    assert!(!arena.is_inside(phys, Some(CarBodyConfig::OCTANE)));

    // Car above the ceiling
    phys.pos = Vec3A::new(0.0, 0.0, 2500.0);
    assert!(!arena.is_inside(phys, Some(CarBodyConfig::OCTANE)));
}

#[test]
fn test_car_different_hitboxes() {
    init();
    let mut arena = Arena::new(GameMode::Soccar);
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::DOMINUS);
    let car_state = *arena.get_car_state(car_idx);
    let mut phys = car_state.phys;

    // Car in the air
    phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    assert!(arena.is_inside(phys, Some(CarBodyConfig::DOMINUS)));

    // Car on floor - hitbox is above the floor
    phys.pos = car_state.phys.pos;
    assert!(arena.is_inside(phys, Some(CarBodyConfig::DOMINUS)));
}

#[test]
fn test_ball_hoops_mode() {
    init();
    let arena = Arena::new(GameMode::Hoops);
    let mut phys = arena.get_ball_state().phys;

    // Ball in the air in hoops arena
    phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    assert!(arena.is_inside(phys, None));

    // Ball touching the floor
    let ball_radius = arena.get_config().mutators.ball_radius;
    phys.pos = Vec3A::new(0.0, 0.0, ball_radius);
    assert!(!arena.is_inside(phys, None));
}

#[test]
fn test_car_hoops_mode() {
    init();
    let mut arena = Arena::new(GameMode::Hoops);
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let car_state = *arena.get_car_state(car_idx);
    let mut phys = car_state.phys;

    // Car in the air
    phys.pos = Vec3A::new(0.0, 0.0, 1000.0);
    assert!(arena.is_inside(phys, Some(CarBodyConfig::OCTANE)));

    // Car on floor - hitbox is above the floor
    phys.pos = car_state.phys.pos;
    assert!(arena.is_inside(phys, Some(CarBodyConfig::OCTANE)));
}

// Arena corner geometry:
// - Side wall: x = ±4096
// - Back wall: y = ±5120
// - Corner wall at 45°: x + y = 8064 (positive quadrant), x + y = -8064 (negative), etc.
// - Corner wall length: 1629.174

fn ball_phys_at(pos: Vec3A) -> rocketsim::PhysState {
    rocketsim::PhysState {
        pos,
        rot_mat: glam::Mat3A::IDENTITY,
        vel: Vec3A::ZERO,
        ang_vel: Vec3A::ZERO,
    }
}

#[test]
fn test_ball_corner_pos_pos_inside() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    // Well inside the (+x, +y) corner wall (x + y = 8064)
    assert!(arena.is_inside(ball_phys_at(Vec3A::new(3800.0, 3800.0, 500.0)), None));
}

#[test]
fn test_ball_corner_pos_pos_touching() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    // Ball touching the (+x, +y) corner wall.
    // Ball radius ≈ 91.25. Distance to wall = 91.25 ⇒ x + y = 8064 − 91.25·√2 ≈ 7935.
    assert!(!arena.is_inside(ball_phys_at(Vec3A::new(3967.5, 3967.5, 500.0)), None));
}

#[test]
fn test_ball_corner_pos_pos_outside() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    // Past the corner wall but within side/back wall bounds.
    assert!(!arena.is_inside(ball_phys_at(Vec3A::new(4000.0, 4000.0, 500.0)), None));
}

#[test]
fn test_ball_corner_neg_pos_inside() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    // Well inside the (−x, +y) corner wall (y − x = 8064)
    assert!(arena.is_inside(ball_phys_at(Vec3A::new(-3800.0, 3800.0, 500.0)), None));
}

#[test]
fn test_ball_corner_neg_pos_touching() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    // Ball touching the (−x, +y) corner wall: y − x = 7935.
    assert!(!arena.is_inside(ball_phys_at(Vec3A::new(-3967.5, 3967.5, 500.0)), None));
}

#[test]
fn test_ball_corner_neg_pos_outside() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    assert!(!arena.is_inside(ball_phys_at(Vec3A::new(-4000.0, 4000.0, 500.0)), None));
}

#[test]
fn test_ball_corner_pos_neg_inside() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    // Well inside the (+x, −y) corner wall (x − y = 8064)
    assert!(arena.is_inside(ball_phys_at(Vec3A::new(3800.0, -3800.0, 500.0)), None));
}

#[test]
fn test_ball_corner_pos_neg_touching() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    // Ball touching the (+x, −y) corner wall: x − y = 7935.
    assert!(!arena.is_inside(ball_phys_at(Vec3A::new(3967.5, -3967.5, 500.0)), None));
}

#[test]
fn test_ball_corner_pos_neg_outside() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    assert!(!arena.is_inside(ball_phys_at(Vec3A::new(4000.0, -4000.0, 500.0)), None));
}

#[test]
fn test_ball_corner_neg_neg_inside() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    // Well inside the (−x, −y) corner wall (x + y = −8064)
    assert!(arena.is_inside(ball_phys_at(Vec3A::new(-3800.0, -3800.0, 500.0)), None));
}

#[test]
fn test_ball_corner_neg_neg_touching() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    // Ball touching the (−x, −y) corner wall: x + y = −7935.
    assert!(!arena.is_inside(ball_phys_at(Vec3A::new(-3967.5, -3967.5, 500.0)), None));
}

#[test]
fn test_ball_corner_neg_neg_outside() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    assert!(!arena.is_inside(ball_phys_at(Vec3A::new(-4000.0, -4000.0, 500.0)), None));
}

#[test]
fn test_ball_corner_low_z_inside() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    // Ball near the corner but above the wall bottom ramp (ramp radius ≈ 256 uu)
    assert!(arena.is_inside(ball_phys_at(Vec3A::new(3800.0, 3800.0, 300.0)), None));
}

#[test]
fn test_ball_behind_back_wall_not_in_net() {
    init();
    let arena = Arena::new(GameMode::Soccar);
    // Behind the back wall (y = 5120) but not in the goal net.
    // The goal is centered at x=0 with width ~1785.5 (center-to-post 892.755).
    // x=2000 is outside the goal opening, so the back wall mesh blocks it.
    assert!(!arena.is_inside(ball_phys_at(Vec3A::new(2000.0, 5500.0, 500.0)), None));
}
