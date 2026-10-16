//! Interactive driveable demo: `rocketsim-viewer` shows it AND drives it.
//!
//! * All inputs come from the VIEWER window (viewer is the single owner):
//!   WASD + mouse buttons drive, Q/E roll, Shift handbrake, gamepad
//!   sticks/triggers/buttons blend in vis-side. `Backspace` resets
//!   kickoff, `F2` dribbles, `F4` launches. The viewer streams
//!   [`ControlMsg`](rocketsim_vis::proto::ControlMsg) frames to this sim's
//!   control port (`127.0.0.1:39202`).
//! * This sim just takes the channel: newest input wins, stale or missing
//!   channel falls back to neutral (no stuck throttle on disconnect).
//! * Ticks run at wall-clock 120Hz via
//!   [`TickPacer`](rocketsim_vis::pace::TickPacer) (catch-up capped,
//!   excess dropped).
//! * Live mutators (unlimited boost, regen, flips) toggle in the viewer
//!   menu and apply per tick; all off = unmodded RL behavior.
//!
//! Run with: `cargo run -p rocketsim_vis --example drive`

use std::time::{Duration, Instant};

use glam::Vec3A;
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};
use rocketsim_vis::{
    ArenaVisExt,
    mutators::Mutators,
    net_vis::ControlServer,
    pace::TickPacer,
    photo::init_meshes,
    proto::{DEFAULT_CONTROL_PORT, MUT_KNOWN_MASK},
};

/// No fresh viewer input for this long → drive neutral (viewer gone/stalled).
const STALE_AFTER: Duration = Duration::from_millis(250);

fn main() {
    init_meshes();
    let mut arena = Arena::new_with_config(ArenaConfig {
        rng_seed: Some(0),
        ..ArenaConfig::new(GameMode::Soccar)
    });

    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);

    // Detached-viewer API: launches a viewer if none listens, streams every tick.
    arena.set_vis_enabled(true);

    // Viewer→sim driving inputs (viewer dials this once its send side lands;
    // until then latest() is None and the car idles at neutral).
    let control_addr = format!("127.0.0.1:{DEFAULT_CONTROL_PORT}");
    let controls_rx = ControlServer::bind(&control_addr);
    eprintln!("drive: control channel on {control_addr} (input needs viewer focus)");

    println!("Drive from the rocketsim-viewer window (viewer send side pending).");
    println!("Gamepad works when connected (read vis-side, bindings from settings.toml).");
    println!("Mutators (unlimited boost etc.) toggle live in the viewer menu.");

    let mut pacer = TickPacer::at_120hz();
    let mut last_seq: Option<u64> = None;
    let mut channel = CarControls::default();
    // Mutator level state: frozen at last-received on link loss (controls
    // above still failsafe to neutral — mutators must not silently switch
    // off mid-flight on a dropped packet).
    let mut channel_mutators: u8 = 0;
    let mut channel_at = Instant::now();
    let mut prev_buttons: u8 = 0;

    loop {
        let steps = pacer.wait();
        for _ in 0..steps {
            if arena.tick_count() == 0 {
                arena.reset_to_random_kickoff(None);
            }

            // Newest viewer input wins; stale or missing channel → neutral.
            let msg = controls_rx.latest();
            if let Some(m) = msg
                && last_seq != Some(m.seq)
            {
                last_seq = Some(m.seq);
                channel = m.controls;
                channel_mutators = m.mutators & MUT_KNOWN_MASK;
                channel_at = Instant::now();
            }
            let controls = if channel_at.elapsed() > STALE_AFTER {
                CarControls::default()
            } else {
                channel
            };

            // Honor the viewer's car selection when it names a live car.
            let target = msg
                .map(|m| m.car_index as usize)
                .filter(|&i| i < arena.num_cars())
                .unwrap_or(car_idx);

            // Viewer buttons fire on the rising edge (level state on wire).
            let buttons = msg.map(|m| m.buttons).unwrap_or(0);
            let pressed = buttons & !prev_buttons;
            prev_buttons = buttons;
            if pressed & 0b001 != 0 {
                arena.reset_to_random_kickoff(None);
            }
            if pressed & 0b010 != 0 {
                // Dribble: ball over the driven car, matched velocity.
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
            // Live mutators, applied to the driven car before the tick.
            let mut car_state = *arena.get_car_state(target);
            Mutators::from_mask(channel_mutators).apply_pre_tick(&mut car_state);
            arena.set_car_state(target, car_state);
            arena.set_car_controls(target, controls);

            arena.step_tick();
        }
    }
}
