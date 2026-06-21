use crate::rl_comparison_test::recording::Recording;
use crate::rl_comparison_test::recording::cpp_records::TickRecord;
use rocketsim::{Arena, CarBodyConfig, CarControls, CarState, GameMode, PhysState, Team};

mod compare;
mod recording;

pub fn set_state_to_record_tick(
    arena: &mut Arena,
    car_idx: usize,
    tick: &TickRecord,
    car_controls: &CarControls,
) {
    let mut cs = *arena.get_car_state(car_idx);
    let rep_cs: CarState = tick.car_record.into();
    cs.phys = rep_cs.phys;
    cs.is_jumping = rep_cs.is_jumping;
    cs.is_flipping = rep_cs.is_flipping;
    cs.jump_time = rep_cs.jump_time;
    cs.flip_time = rep_cs.flip_time;
    cs.controls = *car_controls;

    arena.set_car_state(car_idx, cs);

    let rep_bs_phys: PhysState = tick.ball_record.into();
    let mut bs = *arena.get_ball_state();
    bs.phys = rep_bs_phys;
    arena.set_ball_state(bs);
}

fn test_recording(recording: &Recording) {
    let mut arena = Arena::new(GameMode::Soccar);
    let car_idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE); // TODO: Support other hitboxes

    for i in 0..(recording.ticks.len() - 1) {
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + 1];
        let controls_during: CarControls = to_tick.car_record.prev_controls.into();

        set_state_to_record_tick(&mut arena, car_idx, from_tick, &controls_during);
        arena.step_tick();

        let ball_state = arena.get_ball_state();
        let car_state = arena.get_car_state(car_idx);

        let comparison = compare::compare_states_to_tick(car_state, ball_state, to_tick);
        let norm_error = comparison.calc_norm_error();
        if norm_error > 1.0 {
            let recording_name = &recording.name;
            let mut lines: Vec<String> = vec![
                "=".repeat(50),
                format!(
                    "COMPARISON TEST \"{recording_name}\" FAILED at i={i}, norm_error={norm_error}"
                ),
            ];

            lines.push(format!(
                "Inaccurate state fields (of {}):",
                comparison.map().len()
            ));
            for (k, v) in comparison.map() {
                let rel_error = v.rel_error();
                if rel_error > 0.5 {
                    lines.push(format!(" > (e={rel_error}) \"{k}\": {v:#?}"));
                }
            }
            panic!("{}", lines.join("\n"));
        }
    }
}

// This will be called from each test file
fn run_comparison_test(name: &str, recording_bytes: &[u8]) {
    if !rocketsim::is_initialized() {
        rocketsim::init_from_default(true).unwrap();
    }
    let recording = Recording::from_bytes(name, recording_bytes).unwrap();
    test_recording(&recording);
}
include!(concat!(env!("OUT_DIR"), "/gen_comparison_tests.rs"));
