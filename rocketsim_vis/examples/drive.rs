//! Interactive driveable demo: the detached `vis_viewer` shows it AND drives
//! it (viewer window owns input focus; the sim polls a control channel).
//!
//! * Keyboard/mouse come from the VIEWER window (egui input, works on
//!   Wayland): WASD + mouse buttons drive, Q/E roll, Shift handbrake,
//!   `Backspace` resets kickoff, `F2` dribbles, `F4` launches. The viewer
//!   streams [`ControlMsg`](rocketsim_vis::proto::ControlMsg) frames to
//!   this sim's control port (`127.0.0.1:39202`, see
//!   [`DEFAULT_CONTROL_PORT`](rocketsim_vis::proto::DEFAULT_CONTROL_PORT)).
//!   Global hotkey reads via `device_query` were removed on purpose:
//!   Wayland blocks global reads outside the focused window, so they
//!   silently did nothing there.
//! * Optional local gamepad (gilrs, USB — no window focus needed) still
//!   blends in when connected, same mapping as before.
//! * Ticks are wall-clock-disciplined at 120Hz by
//!   [`TickPacer`](rocketsim_vis::pace::TickPacer) (catch-up capped, excess
//!   dropped — no `sleep(8ms)` drift).
//! * Stale channel failsafe: if no fresh viewer input arrives within
//!   `STALE_AFTER`, controls fall back to neutral (prevents a stuck throttle
//!   when the viewer disconnects).
//! * Boost pins to full (demo behavior, like the old vis).
//!
//! Run with: `cargo run -p rocketsim_vis --example drive`
//! (then point `vis_viewer` at it — or let this spawn one).

use std::time::{Duration, Instant};

use gilrs::{Axis, Button, Gilrs};
use glam::Vec3A;
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};
use rocketsim_vis::{
    ArenaVisExt, net_vis::ControlServer, pace::TickPacer, photo::init_meshes,
    proto::DEFAULT_CONTROL_PORT,
};

const STICK_DEADZONE: f32 = 0.1;
const TRIGGER_DEADZONE: f32 = 0.05;
const TRIGGER_SENSITIVITY: f32 = 1.25;

/// No fresh viewer input for this long → drive neutral (viewer gone/stalled).
const STALE_AFTER: Duration = Duration::from_millis(250);

/// How often the pacer counters are logged (ticks at 120Hz).
const LOG_EVERY_TICKS: u64 = 1200;

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

    // Local gamepad is optional (headless CI has none): warn, don't abort.
    let mut gilrs = match Gilrs::new() {
        Ok(g) => Some(g),
        Err(e) => {
            eprintln!("drive: no gamepad input ({e}); viewer channel only");
            None
        }
    };
    if let Some(gilrs) = gilrs.as_mut() {
        for (_, gamepad) in gilrs.gamepads() {
            println!("Detected gamepad: {}", gamepad.name());
        }
    }

    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);

    // Old base API: launches a viewer if none listens, streams every tick.
    arena.set_vis_enabled(true);

    // Viewer→sim driving inputs (viewer dials this once its send side lands;
    // until then latest() is None and the car idles at neutral).
    let control_addr = format!("127.0.0.1:{DEFAULT_CONTROL_PORT}");
    let controls_rx = ControlServer::bind(&control_addr);
    eprintln!("drive: control channel on {control_addr} (input needs viewer focus)");

    println!("Drive from the vis_viewer window (viewer send side pending).");
    println!("Local gamepad works when connected. Boost pinned to full.");

    let mut pacer = TickPacer::at_120hz();
    let mut last_seq: Option<u64> = None;
    let mut channel = CarControls::default();
    let mut channel_at = Instant::now();
    let mut prev_buttons: u8 = 0;
    let mut total_ticks: u64 = 0;

    loop {
        let steps = pacer.wait();
        for _ in 0..steps {
            total_ticks += 1;

            if arena.tick_count() == 0 {
                arena.reset_to_random_kickoff(None);
            }

            // Newest viewer input wins; stale channel (or none yet) → neutral.
            // One lock-guarded read per tick (ControlMsg is Copy).
            let msg = controls_rx.latest();
            if let Some(m) = msg
                && last_seq != Some(m.seq)
            {
                last_seq = Some(m.seq);
                channel = m.controls;
                channel_at = Instant::now();
            }
            let mut controls = if channel_at.elapsed() > STALE_AFTER {
                CarControls::default()
            } else {
                channel
            };

            // Blend in local gamepad input when a controller is connected.
            // (The old North-button ball-cam lived in the in-process vis; the
            // detached viewer owns cameras now — see its C/Space keys.)
            if let Some(gilrs) = gilrs.as_mut()
                && gilrs.gamepads().next().is_some()
            {
                controls = determine_controller_controls(gilrs, controls);
            }

            let mut car_state = *arena.get_car_state(car_idx);
            car_state.boost = 100.0;
            arena.set_car_state(car_idx, car_state);

            // Honor the viewer's car selection when it names a live car.
            let target = msg
                .map(|m| m.car_index as usize)
                .filter(|&i| i < arena.num_cars())
                .unwrap_or(car_idx);

            // Viewer action buttons fire on their rising edge (level state
            // on the wire; Backspace/2/4 equivalents from the viewer keys).
            let buttons = msg.map(|m| m.buttons).unwrap_or(0);
            let pressed = buttons & !prev_buttons;
            prev_buttons = buttons;
            if pressed & 0b001 != 0 {
                arena.reset_to_random_kickoff(None);
            }
            if pressed & 0b010 != 0 {
                // Teleport ball to dribble position over the driven car.
                let car_state = arena.get_car_state(target);
                let mut ball_state = *arena.get_ball_state();
                ball_state.phys.pos = car_state.phys.pos + Vec3A::Z * 150.0;
                ball_state.phys.vel = car_state.phys.vel;
                arena.set_ball_state(ball_state);
            } else if pressed & 0b100 != 0 {
                // Launch ball upward.
                let mut ball_state = *arena.get_ball_state();
                ball_state.phys.vel += Vec3A::Z * 1000.0;
                arena.set_ball_state(ball_state);
            }
            arena.set_car_controls(target, controls);

            arena.step_tick();

            if total_ticks.is_multiple_of(LOG_EVERY_TICKS) {
                let stats = pacer.stats();
                let cp = arena.get_car_state(car_idx).phys.pos;
                eprintln!(
                    "drive: tick={} paced_ticks={} dropped={} max_backlog={} control_rx={} car=({:.0},{:.0},{:.0})",
                    arena.tick_count(),
                    stats.ticks,
                    stats.dropped,
                    stats.max_backlog,
                    controls_rx.received(),
                    cp.x,
                    cp.y,
                    cp.z,
                );
            }
        }
    }
}
