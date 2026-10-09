//! Interactive driveable demo: WASD + mouse, Q/E roll, Shift handbrake,
//! optional gamepad. The detached viewer shows it (C/Space/D/A over there).
//!
//! * `Backspace` resets to kickoff, `2` dribbles the ball, `4` launches it.
//! * Boost pins to full (demo behavior, like the old vis).
//!
//! Run with: `cargo run -p rocketsim_vis --example drive`
//! (then point `vis_viewer` at it — or let this spawn one).

use std::time::Duration;

use device_query::{DeviceQuery, DeviceState, Keycode};
use gilrs::{Axis, Button, Gilrs};
use glam::Vec3A;
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};
use rocketsim_vis::{ArenaVisExt, photo::init_meshes};

const STICK_DEADZONE: f32 = 0.1;
const TRIGGER_DEADZONE: f32 = 0.05;
const TRIGGER_SENSITIVITY: f32 = 1.25;

fn determine_keyboard_controls(device: &DeviceState, mut controls: CarControls) -> CarControls {
    let keys = device.get_keys();
    let mouse_state = device.get_mouse();

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
    controls.boost = mouse_state.button_pressed[2];

    controls.yaw = controls.steer;
    controls.pitch = -controls.throttle;

    if controls.handbrake {
        controls.roll = controls.yaw;
        controls.yaw = 0.0;
    }

    controls
}

fn determine_controller_controls(gilrs: &mut Gilrs, mut controls: CarControls) -> CarControls {
    while gilrs.next_event().is_some() {}

    for (_, gamepad) in gilrs.gamepads() {
        controls.jump |= gamepad.is_pressed(Button::South);
        controls.boost |= gamepad.is_pressed(Button::East);
        controls.handbrake |= gamepad.is_pressed(Button::West);

        let left_stick_x = gamepad.value(Axis::LeftStickX);
        if left_stick_x.abs() > STICK_DEADZONE {
            controls.steer = left_stick_x;
            controls.yaw = left_stick_x;
        }

        let left_stick_y = gamepad.value(Axis::LeftStickY);
        if left_stick_y.abs() > STICK_DEADZONE {
            controls.pitch = -left_stick_y;
        }

        let left_trigger = gamepad
            .button_data(Button::LeftTrigger2)
            .map_or(0.0, |trigger| trigger.value());
        let right_trigger = gamepad
            .button_data(Button::RightTrigger2)
            .map_or(0.0, |trigger| trigger.value());

        if right_trigger > TRIGGER_DEADZONE {
            controls.throttle = right_trigger * TRIGGER_SENSITIVITY;
        }
        if left_trigger > TRIGGER_DEADZONE {
            controls.throttle = -left_trigger * TRIGGER_SENSITIVITY;
        }

        if gamepad.is_pressed(Button::RightTrigger) {
            controls.roll += 1.0;
        }
        if gamepad.is_pressed(Button::LeftTrigger) {
            controls.roll -= 1.0;
        }
    }

    controls
}

fn main() {
    init_meshes();
    let mut arena = Arena::new_with_config(ArenaConfig {
        rng_seed: Some(0),
        ..ArenaConfig::new(GameMode::Soccar)
    });

    let mut gilrs = Gilrs::new().expect("failed to init gamepad input");

    for (_, gamepad) in gilrs.gamepads() {
        println!("Detected gamepad: {}", gamepad.name());
    }

    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);

    // Old base API: launches a viewer if none listens, streams every tick.
    // Camera keys (C/Space/D/A) live in the viewer window, which needs focus;
    // driving keys below are read globally and work from anywhere.
    arena.set_vis_enabled(true);
    println!("Drive with WASD + mouse (Q/E roll, Shift handbrake).");
    println!("Backspace = kickoff, 2 = dribble, 4 = launch ball.");

    let device_state = DeviceState::new();
    let mut prev_keys = Vec::new();
    loop {
        let held_keys = device_state.get_keys();

        let pressed_keys: Vec<Keycode> = held_keys
            .iter()
            .filter(|key| !prev_keys.contains(*key))
            .copied()
            .collect();

        let mut controls = determine_keyboard_controls(&device_state, CarControls::default());

        // Blend in gamepad input when a controller is connected.
        // (The old North-button ball-cam lived in the in-process vis; the
        // detached viewer owns cameras now — see its C/Space keys.)
        if gilrs.gamepads().next().is_some() {
            controls = determine_controller_controls(&mut gilrs, controls);
        }

        // Reset arena
        if pressed_keys.contains(&Keycode::Backspace) || arena.tick_count() == 0 {
            arena.reset_to_random_kickoff(None);
        }

        let mut car_state = *arena.get_car_state(car_idx);
        car_state.boost = 100.0;
        arena.set_car_state(car_idx, car_state);

        if pressed_keys.contains(&Keycode::Key2) {
            // Teleport ball to dribble position
            let car_state = arena.get_car_state(car_idx);

            let mut ball_state = *arena.get_ball_state();
            ball_state.phys.pos = car_state.phys.pos + Vec3A::Z * 150.0;
            ball_state.phys.vel = car_state.phys.vel;
            arena.set_ball_state(ball_state);
        } else if pressed_keys.contains(&Keycode::Key4) {
            // Launch ball
            let mut ball_state = *arena.get_ball_state();
            ball_state.phys.vel += Vec3A::Z * 1000.0;
            arena.set_ball_state(ball_state);
        }

        arena.set_car_controls(car_idx, controls);

        arena.step_tick();

        std::thread::sleep(Duration::from_millis(8));

        prev_keys = held_keys;
    }
}
