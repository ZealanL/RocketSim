use rocketsim::{Arena, CarBodyConfig, CarControls, CarState, GameMode, PhysState, Team};

use crate::rl_comparison_test::recording::Recording;
use crate::rl_comparison_test::recording::tick_record::TickRecord;

mod compare;
mod recording;

pub fn set_state_to_record_tick(
    arena: &mut Arena,
    car_idcs: &[usize],
    tick: &TickRecord,
    car_controls: &[CarControls],
) {
    for (i, &car_idx) in car_idcs.iter().enumerate() {
        let mut cs = *arena.get_car_state(car_idx);
        let rep_cs: CarState = tick.car_records[i].into();
        cs.phys = rep_cs.phys;
        cs.is_jumping = rep_cs.is_jumping;
        cs.is_flipping = rep_cs.is_flipping;
        cs.jump_time = rep_cs.jump_time;
        cs.flip_time = rep_cs.flip_time;
        cs.has_jumped = rep_cs.has_jumped;

        if cs.has_flip_or_jump() {
            cs.has_double_jumped = rep_cs.has_double_jumped;
        }

        cs.controls = car_controls[i];

        arena.set_car_state(car_idx, cs);
    }

    let rep_bs_phys: PhysState = tick.ball_record.into();
    let mut bs = *arena.get_ball_state();
    bs.phys = rep_bs_phys;
    arena.set_ball_state(bs);
}

fn test_recording(recording: &Recording) {
    let mut arena = Arena::new(GameMode::Soccar);
    let num_cars = recording.info.num_cars as usize;
    let car_idcs: Vec<usize> = (0..num_cars)
        .map(|i| {
            let team = if (i % 2) == 0 {
                Team::Blue
            } else {
                Team::Orange
            };
            arena.add_car(team, CarBodyConfig::OCTANE)
        })
        .collect();

    for i in 0..(recording.ticks.len() - 1) {
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + 1];
        let controls_during: Vec<CarControls> = to_tick
            .car_records
            .iter()
            .map(|car_record| car_record.prev_controls.into())
            .collect();
        set_state_to_record_tick(&mut arena, &car_idcs, from_tick, &controls_during);
        arena.step_tick();

        let ball_state = arena.get_ball_state();
        let car_states: Vec<CarState> = car_idcs
            .iter()
            .map(|&car_idx| *arena.get_car_state(car_idx))
            .collect();

        let comparison = compare::compare_states_to_tick(&car_states, ball_state, to_tick);
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
                if rel_error >= 0.25 {
                    lines.push(format!(" > (e={rel_error}) \"{k}\": {v:#?}"));
                }
            }
            panic!("{}", lines.join("\n"));
        }
    }
}

// This will be called from each test file
#[allow(unused)]
fn run_comparison_test(name: &str, recording_bytes: &[u8]) {
    if !rocketsim::is_initialized() {
        rocketsim::init_from_default(true).unwrap();
    }
    let recording = Recording::from_bytes(name, recording_bytes).unwrap();
    test_recording(&recording);
}
include!(concat!(env!("OUT_DIR"), "/gen_comparison_tests.rs"));
