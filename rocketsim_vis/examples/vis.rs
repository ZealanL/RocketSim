//! Interactive driveable demo: WASD + mouse, Q/E roll, Shift handbrake.
//!
//! * `Backspace` resets to kickoff, `2` dribbles the ball, `4` launches it.
//! * Visualizer controls: `C` cycles cameras, `Space` toggles ball-cam.
//!
//! Run with: `cargo run -p rocketsim_vis --example vis`

use device_query::{DeviceQuery, DeviceState, Keycode};
use glam::Vec3A;
use rocketsim::{
    Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team, init_from_default,
};
use rocketsim_vis::ArenaVisExt;
use gilrs::{Gilrs, Button, Axis};

fn determine_keyboard_controls(device: &DeviceState, controls: CarControls) -> CarControls {
    let keys = device.get_keys();
    let mouse_state = device.get_mouse();
    let mut output_controls = controls.clone();

    if keys.contains(&Keycode::A) {
        output_controls.steer -= 1.0;
    }
    if keys.contains(&Keycode::D) {
        output_controls.steer += 1.0;
    }
    if keys.contains(&Keycode::S) {
        output_controls.throttle -= 1.0;
    }
    if keys.contains(&Keycode::W) {
        output_controls.throttle += 1.0;
    }

    if keys.contains(&Keycode::Q) {
        output_controls.roll -= 1.0;
    }
    if keys.contains(&Keycode::E) {
        output_controls.roll += 1.0;
    }

    output_controls.handbrake = keys.contains(&Keycode::LShift);

    output_controls.jump = mouse_state.button_pressed[1];
    output_controls.boost = mouse_state.button_pressed[2];

    output_controls.yaw = output_controls.steer;
    output_controls.pitch = -output_controls.throttle;

    if controls.handbrake {
        output_controls.roll = output_controls.yaw;
        output_controls.yaw = 0.0;
    }

    output_controls
}

fn determine_controller_controls(gilrs: &mut Gilrs, controls: CarControls) -> (CarControls, bool) {
    while let Some(_) = gilrs.next_event() {}

    let mut output_controls = controls.clone();

    let mut ball_cam = false;

    let deadzone = 0.1;
    let trigger_deadzone = 0.05;

    let input_sensitivity = 1.25;

    for (_id, gamepad) in gilrs.gamepads() {
        ball_cam = gamepad.is_pressed(Button::North);

        output_controls.jump = gamepad.is_pressed(Button::South); // Xbox - A, PS - Cross

        output_controls.boost = gamepad.is_pressed(Button::East); // Xbox - B, PS - Circle

        output_controls.handbrake = gamepad.is_pressed(Button::West); // Xbox - X, PS - Square

        let left_stick_x = gamepad.value(Axis::LeftStickX);
        let left_stick_y = gamepad.value(Axis::LeftStickY);

        if left_stick_x.abs() > deadzone {
            output_controls.steer = left_stick_x;
            output_controls.yaw = left_stick_x;
        }

        if left_stick_y.abs() > deadzone {
            output_controls.pitch = -left_stick_y;
        }

        let mut left_trigger = 0.0;
        let mut right_trigger = 0.0;

        let left_trigger_data = gamepad.button_data(Button::LeftTrigger2);
        if left_trigger_data.is_some() {
            left_trigger = left_trigger_data.unwrap().value();
        }

        let right_trigger_data = gamepad.button_data(Button::RightTrigger2);
        if right_trigger_data.is_some() {
            right_trigger = right_trigger_data.unwrap().value();
        }

        if right_trigger.abs() > trigger_deadzone {
            output_controls.throttle = right_trigger * input_sensitivity;
        }

        if left_trigger.abs() > trigger_deadzone {
            output_controls.throttle = -left_trigger * input_sensitivity;
        }

        let air_roll_right = gamepad.is_pressed(Button::RightTrigger);
        let air_roll_left = gamepad.is_pressed(Button::LeftTrigger);

        if air_roll_right {
            output_controls.roll += 1.0;
        }

        if air_roll_left {
            output_controls.roll -= 1.0;
        }
    }

    (output_controls, ball_cam)
}

// fn print_controls(controls: CarControls) {
//     println!("Throttle:  {}", controls.throttle);
//     println!("Steer:     {}", controls.steer);
//     println!("Pitch:     {}", controls.pitch);
//     println!("Yaw:       {}", controls.yaw);
//     println!("Roll:      {}", controls.roll);
//     println!("Jump:      {}", controls.jump);
//     println!("Boost:     {}", controls.boost);
//     println!("Handbrake: {}", controls.handbrake);
// }

fn main() {
    init_from_default(true).unwrap();
    let mut arena = Arena::new_with_config(ArenaConfig {
        rng_seed: Some(0),
        ..ArenaConfig::new(GameMode::Soccar)
    });

    let mut gilrs = Gilrs::new().unwrap();

    for (_id, gamepad) in gilrs.gamepads() {
        println!("Detected gamepad: {}", gamepad.name());
    }

    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);

    arena.set_vis_enabled(true);

    let device_state = DeviceState::new();
    let mut prev_keys = Vec::new();
    loop {
        let held_keys = device_state.get_keys();

        let pressed_keys: Vec<Keycode> = held_keys
            .iter()
            .filter(|&key| !prev_keys.contains(key))
            .copied()
            .collect();

        let car_controls = CarControls::default();

        let mut controls = determine_keyboard_controls(&device_state, car_controls);

        // check if we have any controllers plugged in
        let connected_controllers = gilrs.gamepads().next().is_some();

        let mut _ball_cam = false;

        if connected_controllers {
            (controls, _ball_cam) = determine_controller_controls(&mut gilrs, controls);
        }

        // print_controls(controls);

        // Reset arena
        if pressed_keys.contains(&Keycode::Backspace) || arena.tick_count() == 0 {
            arena.reset_to_random_kickoff(None);
        }

        let mut car_state = *arena.get_car_state(car_idx);
        car_state.boost = 100.0;
        // println!("Car state has jump: {}", car_state.has_flip_or_jump());
        arena.set_car_state(car_idx, car_state);

        if pressed_keys.contains(&Keycode::Key2) {
            // Teleport ball to dribble position
            let car_state = arena.get_car_state(car_idx);

            let mut ball_state = *arena.get_ball_state();
            ball_state.phys.pos = car_state.phys.pos + Vec3A::new(0.0, 0.0, 150.0);
            ball_state.phys.vel = car_state.phys.vel;
            arena.set_ball_state(ball_state);
        } else if pressed_keys.contains(&Keycode::Key4) {
            // Launch ball
            let mut ball_state = *arena.get_ball_state();
            ball_state.phys.vel += Vec3A::new(0.0, 0.0, 1000.0);
            arena.set_ball_state(ball_state);
        }

        arena.set_car_controls(car_idx, controls);

        arena.step_tick();

        std::thread::sleep(std::time::Duration::from_millis(8));

        prev_keys = held_keys;
    }
}
