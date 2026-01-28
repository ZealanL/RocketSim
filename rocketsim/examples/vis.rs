use device_query::{DeviceQuery, DeviceState, Keycode};
use rocketsim::{
    Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team, init_from_default,
};

fn determine_controls(device: &DeviceState) -> CarControls {
    let keys = device.get_keys();
    let mouse_state = device.get_mouse();
    let mut controls = CarControls::default();

    if keys.contains(&Keycode::A) {
        controls.steer -= 1.0;
    }
    if keys.contains(&Keycode::D) {
        controls.steer += 1.0;
    }
    if keys.contains(&Keycode::S) {
        controls.throttle -= 1.0;
    }
    if keys.contains(&Keycode::W) {
        controls.throttle += 1.0;
    }

    if keys.contains(&Keycode::Q) {
        controls.roll -= 1.0;
    }
    if keys.contains(&Keycode::E) {
        controls.roll += 1.0;
    }

    controls.handbrake = keys.contains(&Keycode::LShift);

    controls.jump = mouse_state.button_pressed[1];
    controls.boost = mouse_state.button_pressed[3];

    controls.yaw = controls.steer;
    controls.pitch = -controls.throttle;

    if controls.handbrake {
        controls.roll = controls.yaw;
        controls.yaw = 0.0;
    }

    controls
}

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new_with_config(
        GameMode::Soccar,
        ArenaConfig {
            rng_seed: Some(0),
            ..Default::default()
        },
    );

    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::BREAKOUT);

    arena.set_vis_enabled(true);

    let device_state = DeviceState::new();
    loop {
        let keys = device_state.get_keys();
        let controls = determine_controls(&device_state);

        // Reset arena
        if keys.contains(&Keycode::Backspace) || arena.tick_count() == 0 {
            arena.reset_to_random_kickoff();
        }

        arena.set_car_controls(car_idx, controls);

        arena.step_tick();

        std::thread::sleep(std::time::Duration::from_millis(8));
    }
}
