use rocketsim::{
    init_from_default, Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team,
};
use rocketsim_vis::ArenaVisExt;
use std::{
    env,
    io::Write,
    thread,
    time::{Duration, Instant},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let addr = args.get(1).map(String::as_str).unwrap_or("127.0.0.1:39201");
    let mode_name = args.get(2).map(String::as_str).unwrap_or("soccar");
    let mode = rocketsim_vis::parse_game_mode(mode_name).ok_or("mode must be soccar or thevoid")?;
    if !matches!(mode, GameMode::Soccar | GameMode::TheVoid) {
        return Err("mode must be soccar or thevoid".into());
    }
    let seconds: u64 = args.get(3).map(|s| s.parse()).transpose()?.unwrap_or(45);
    if seconds == 0 || seconds > 120 {
        return Err("duration must be 1..120 seconds".into());
    }

    init_from_default(true)?;
    let mut arena = Arena::new_with_config(ArenaConfig {
        rng_seed: Some(0),
        ..ArenaConfig::new(mode)
    });
    let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff(None);
    arena.try_set_net_vis(Some(addr))?;
    println!("{{\"kind\":\"ready\",\"address\":\"{addr}\",\"mode\":\"{mode_name}\",\"duration_s\":{seconds}}}");
    std::io::stdout().flush()?;

    let start = Instant::now();
    let mut tick = 0_u64;
    let mut previous = String::new();
    while start.elapsed() < Duration::from_secs(seconds) {
        let message = arena.latest_vis_controls();
        let actions = arena.take_vis_actions();
        if let Some(msg) = message {
            let target = (msg.car_index as usize).min(arena.num_cars().saturating_sub(1));
            if target != car {
                arena.set_car_controls(car, CarControls::default());
            }
            arena.set_car_controls(target, msg.controls);
            let signature = format!(
                "{:?}/{}/{}/{}",
                msg.controls, msg.buttons, msg.mutators, actions
            );
            if signature != previous || msg.buttons != 0 || actions != 0 {
                let c = msg.controls;
                println!("{{\"kind\":\"control\",\"tick\":{tick},\"seq\":{},\"car_index\":{target},\"throttle\":{},\"steer\":{},\"pitch\":{},\"yaw\":{},\"roll\":{},\"jump\":{},\"boost\":{},\"handbrake\":{},\"buttons\":{},\"actions\":{actions},\"mutators\":{}}}",
                    msg.seq, c.throttle, c.steer, c.pitch, c.yaw, c.roll, c.jump, c.boost, c.handbrake, msg.buttons, msg.mutators);
                std::io::stdout().flush()?;
                previous = signature;
            }
        } else {
            arena.set_car_controls(car, CarControls::default());
            if previous != "neutral" {
                println!("{{\"kind\":\"neutral\",\"tick\":{tick},\"throttle\":0,\"steer\":0,\"pitch\":0,\"yaw\":0,\"roll\":0,\"jump\":false,\"boost\":false,\"handbrake\":false}}");
                std::io::stdout().flush()?;
                previous = "neutral".into();
            }
        }

        arena.step_tick();
        tick += 1;
        if tick.is_multiple_of(120) {
            let state = arena.get_car_state(car);
            let p = state.phys.pos;
            let v = state.phys.vel;
            println!("{{\"kind\":\"state\",\"tick\":{tick},\"x\":{},\"y\":{},\"z\":{},\"vx\":{},\"vy\":{},\"vz\":{}}}",
                p.x, p.y, p.z, v.x, v.y, v.z);
            std::io::stdout().flush()?;
        }
        thread::sleep(Duration::from_millis(8));
    }
    arena.set_vis_enabled(false);
    println!("{{\"kind\":\"done\",\"ticks\":{tick}}}");
    std::io::stdout().flush()?;
    Ok(())
}
