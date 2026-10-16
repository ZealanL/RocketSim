//! Deterministic scene for screenshot review and determinism gates.
//!
//! Fixed seed, cars, and controls: stepping N ticks always yields the same
//! state. The photo example streams/prints this scene.

use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};

/// Builds the fixed scene: Soccar, seed 0, blue OCTANE + orange DOMINUS.
pub fn build_arena() -> Arena {
    let mut arena = Arena::new_with_config(ArenaConfig {
        rng_seed: Some(0),
        ..ArenaConfig::new(GameMode::Soccar)
    });
    arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.add_car(Team::Orange, CarBodyConfig::DOMINUS);
    arena.reset_to_random_kickoff(Some(0));
    arena
}

/// Initializes collision meshes from `ROCKETSIM_MESHES_DIR` if set, else the
/// `./collision_meshes/` of the workspace root or the crate dir (tests run
/// with the crate dir as CWD, examples with the workspace root; the net
/// crate also probes one level up).
pub fn init_meshes() {
    if let Some(dir) = std::env::var_os("ROCKETSIM_MESHES_DIR") {
        rocketsim::init(dir, true).unwrap();
        return;
    }
    for cand in [
        "./collision_meshes/",
        "../collision_meshes/",
        "../../collision_meshes/",
    ] {
        if std::path::Path::new(cand).join("soccar").exists() {
            rocketsim::init(cand, true).unwrap();
            return;
        }
    }
    rocketsim::init_from_default(true).unwrap();
}
/// Steps `ticks` times with fixed controls (car 0 drives, car 1 idles).
/// Pure sim (no sleeps), so unit tests stay fast.
pub fn step_ticks(arena: &mut Arena, ticks: usize) {
    let drive = CarControls {
        throttle: 1.0,
        ..CarControls::default()
    };
    for _ in 0..ticks {
        arena.set_car_controls(0, drive);
        arena.step_tick();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state_after(ticks: usize) -> (u64, [f32; 3], [f32; 3]) {
        init_meshes();
        let mut arena = build_arena();
        step_ticks(&mut arena, ticks);
        let ball = arena.get_ball_state().phys.pos;
        let car = arena.get_car_state(0).phys.pos;
        (
            arena.tick_count(),
            [ball.x, ball.y, ball.z],
            [car.x, car.y, car.z],
        )
    }

    #[test]
    fn photo_scene_contract() {
        let a = state_after(120);
        let b = state_after(120);
        assert_eq!(a.0, 120);
        assert_eq!(a, b, "same seed + same ticks must give same state");
        // The scene must move: car 0 drives from its spawn, ball sits still.
        init_meshes();
        let mut arena = build_arena();
        let y0 = arena.get_car_state(0).phys.pos.y;
        step_ticks(&mut arena, 120);
        let ball = arena.get_ball_state().phys.pos;
        let car = arena.get_car_state(0).phys.pos;
        assert!(ball.x.abs() < 1.0 && ball.y.abs() < 1.0, "ball={ball:?}");
        assert!(car.y - y0 > 100.0, "car0 never moved? y0={y0} car={car:?}");
    }
}
