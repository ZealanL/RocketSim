use device_query::{DeviceQuery, DeviceState, Keycode};
use serde::Serialize;
use std::net::UdpSocket;
use std::time::Duration;

use rocketsim::{Arena, CarBodyConfig, CarControls, GameMode, PhysState, Team, init_from_default};

// TODO: Pretty sure this is bad practice
#[path = "../tests/comparison_test/state_convert.rs"]
mod state_convert;

use crate::state_convert::{
    OldArena, OldArenaConfig, OldCarConfig, OldGameMode, OldTeam, conv_to_new_car_state,
    conv_to_old_ball_state, conv_to_old_car_controls, conv_to_old_car_state,
};

/////////////////

const ROCKET_SIM_VIS_ADDRESS: &'static str = "127.0.0.1:9273";

#[derive(Serialize)]
struct PhysStateView {
    pos: [f32; 3],
    vel: [f32; 3],
    ang_vel: [f32; 3],
    forward: Option<[f32; 3]>,
    up: Option<[f32; 3]>,
}

impl PhysStateView {
    fn from_phys(phys: &PhysState, include_rot: bool) -> Self {
        Self {
            pos: phys.pos.into(),
            vel: phys.vel.into(),
            ang_vel: phys.ang_vel.into(),
            forward: include_rot.then(|| phys.get_forward_dir().into()),
            up: include_rot.then(|| phys.get_up_dir().into()),
        }
    }
}

#[derive(Serialize)]
struct CarView {
    phys: PhysStateView,
    team_num: i8,
    boost_amount: f32,
    on_ground: bool,
    is_demoed: bool,
}

#[derive(Serialize)]
struct ArenaView {
    ball_phys: PhysStateView,
    cars: Vec<CarView>,
    boost_pad_states: Vec<bool>,
}

fn determine_controls(device: &DeviceState) -> CarControls {
    let keys = device.get_keys();
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

    controls.handbrake = keys.contains(&Keycode::LShift);
    controls.jump = keys.contains(&Keycode::U);
    controls.boost = keys.contains(&Keycode::I);

    controls.yaw = controls.steer;
    controls.pitch = -controls.throttle;

    if controls.handbrake {
        controls.roll = controls.yaw;
        controls.yaw = 0.0;
    }

    controls
}

fn main() -> std::io::Result<()> {
    init_from_default(true)?;
    rocketsim_rs::init(Some("./collision_meshes"), false);

    let mut arena = Arena::new(GameMode::Soccar);
    let mut old_arena = OldArena::new(OldGameMode::Soccar, OldArenaConfig::default(), 120);

    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    let old_car_idx = old_arena
        .pin_mut()
        .add_car(OldTeam::Orange, OldCarConfig::octane());

    let socket = UdpSocket::bind("0.0.0.0:0")?;
    let device_state = DeviceState::new();

    println!("Starting RocketSimVis transmission...");

    loop {
        let keys = device_state.get_keys();
        let controls = determine_controls(&device_state);

        // Special resync-physics keybind
        if keys.contains(&Keycode::T) || arena.tick_count() == 0 {
            old_arena
                .pin_mut()
                .set_car(
                    old_car_idx,
                    conv_to_old_car_state(arena.get_car_state(car_idx)),
                )
                .unwrap();
            old_arena
                .pin_mut()
                .set_ball(conv_to_old_ball_state(arena.get_ball_state()));
        }

        arena.set_car_controls(car_idx, controls);
        old_arena
            .pin_mut()
            .set_car_controls(old_car_idx, conv_to_old_car_controls(controls))
            .unwrap();

        for _ in 0..2 {
            arena.step_tick();
        }
        old_arena.pin_mut().step(2);

        let old_car_state_conv = conv_to_new_car_state(
            &old_arena.pin_mut().get_car(old_car_idx),
            CarControls::default(),
        );

        let arena_view = ArenaView {
            ball_phys: PhysStateView::from_phys(arena.get_ball_state(), false),
            cars: (0..arena.num_cars())
                .map(|i| CarView {
                    phys: PhysStateView::from_phys(arena.get_car_state(i), true),
                    team_num: arena.get_car_info(i).team as i8,
                    boost_amount: arena.get_car_state(i).boost,
                    on_ground: arena.get_car_state(i).is_on_ground,
                    is_demoed: arena.get_car_state(i).is_demoed,
                })
                .chain(std::iter::once(CarView {
                    phys: PhysStateView::from_phys(&old_car_state_conv.phys, true),
                    team_num: Team::Orange as i8,
                    boost_amount: old_car_state_conv.boost,
                    on_ground: old_car_state_conv.is_on_ground,
                    is_demoed: old_car_state_conv.is_demoed,
                }))
                .collect(),
            boost_pad_states: arena
                .get_all_boost_pad_states()
                .iter()
                .map(|s| s.is_active())
                .collect(),
        };

        if let Ok(json) = serde_json::to_string(&arena_view) {
            socket.send_to(json.as_bytes(), ROCKET_SIM_VIS_ADDRESS)?;
        }

        std::thread::sleep(Duration::from_millis(14));
    }
}
