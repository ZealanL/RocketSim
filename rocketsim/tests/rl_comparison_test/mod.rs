use crate::rl_comparison_test::recording::Recording;
use crate::rl_comparison_test::recording::tick_record::TickRecord;
use glam::Vec3A;
use rocketsim::consts::BT_TO_UU;
use rocketsim::{Arena, CarBodyConfig, CarControls, CarState, GameMode, PhysState, Team};

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
        cs.jump_ticks = rep_cs.jump_ticks;
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

    // Warm-up: step once to establish contact constraints after teleport
    let warmup_controls: Vec<CarControls> = recording.ticks[0]
        .car_records
        .iter()
        .map(|cr| cr.prev_controls.into())
        .collect();
    set_state_to_record_tick(&mut arena, &car_idcs, &recording.ticks[0], &warmup_controls);

    // Seed boost tanks from the recording's first tick: RLPR carries the
    // initial boost state, and scenarios that need a non-default tank
    // (e.g. boost_run's full 100) silently diverge once accounting
    // differs. Later ticks are NOT forced - drain-rate differences stay
    // observable.
    for (i, &car_idx) in car_idcs.iter().enumerate() {
        let mut cs = *arena.get_car_state(car_idx);
        cs.boost = recording.ticks[0].car_records[i].boost_amount;
        arena.set_car_state(car_idx, cs);
    }

    arena.step_tick();

    for i in 0..(recording.ticks.len() - 1) {
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + 1];

        // The recorder occasionally drops a physics frame (game hitch), so a
        // recorded tick can be the result of TWO sim steps. Comparing one RS
        // step against a double step can never match; skip that transition.
        let frame_delta =
            to_tick.car_records[0].phys.physics_frame - from_tick.car_records[0].phys.physics_frame;
        if frame_delta != 1 {
            continue;
        }

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

        if std::env::var("RS_TRACE").is_ok() {
            let cs = &car_states[0];
            let rt = &to_tick.car_records[0];
            println!(
                "i={i} PRED wy={:+8.4} vz={:+9.3} flipT={:.3} | REAL(t{}) wy={:+8.4} vz={:+9.3}",
                cs.phys.ang_vel.y, cs.phys.vel.z, cs.flip_time,
                i + 1, rt.phys.ang_vel[1], rt.phys.lin_vel[2],
            );
        }

        if norm_error >= 1.0 {
            let recording_name = &recording.name;
            let mut lines: Vec<String> = vec![
                "=".repeat(50),
                format!(
                    "COMPARISON TEST \"{recording_name}\" FAILED at i={i}, norm_error={norm_error}"
                ),
            ];

            lines.push(format!(
                "Notable state fields (of {}):",
                comparison.map().len()
            ));
            for (k, v) in comparison.map() {
                let rel_error = v.rel_error();
                if rel_error >= 0.25 {
                    lines.push(format!(" > (e={rel_error}) \"{k}\": {v:#?}"));
                }
            }
            lines.push("CAR CONTROLS DURING TICK:".to_string());
            for (j, car_state) in car_states.iter().enumerate().take(num_cars) {
                lines.push(format!(" > Car [{j}]: {:?}", car_state.controls));
            }
            lines.push("CAR IMPULSES DURING TICK:".to_string());
            for j in 0..num_cars {
                lines.push(format!(" > Car [{j}]:"));
                let pred_impulses = arena.get_car_impulse_history(j);
                let real_impulses = to_tick.car_records[j].phys.impulse_records();

                if pred_impulses.len() + real_impulses.len() > 0 {
                    for ((name, is_accum), (lin_impulse, ang_impulse)) in pred_impulses {
                        let lin_impulse = lin_impulse * BT_TO_UU;
                        let ang_impulse = ang_impulse * BT_TO_UU;
                        lines.push(format!(
                            "\tPRED: {{ \
                            name: {name}, lin_impulse: {lin_impulse}, ang_impulse: {ang_impulse}, is_accum: {is_accum} \
                            }}"
                        ));
                    }
                    lines.push(String::new());
                    for real_impulse in real_impulses {
                        let impulse_type = real_impulse.impulse_type;
                        let lin_impulse: Vec3A = real_impulse.lin_impulse.into();
                        let ang_impulse: Vec3A = real_impulse.ang_impulse.into();
                        let lin_impulse = lin_impulse * BT_TO_UU;
                        let ang_impulse = ang_impulse * BT_TO_UU;
                        let is_accum = real_impulse.is_accum;
                        lines.push(format!(
                            "\tREAL: {{ \
                            name: {impulse_type:?}, lin_impulse: {lin_impulse}, ang_impulse: {ang_impulse}, is_accum: {is_accum} \
                            }}"
                        ));
                    }
                } else {
                    lines.push("\t(None)".to_string());
                }
            }

            panic!("{}", lines.join("\n"));
        }
    }
}

// This will be called from each test file
#[allow(unused)]
fn run_comparison_test(name: &str, recording_bytes: &[u8]) {
    // Tests run in parallel threads; racing the global init corrupts
    // lookup tables for latecomers (nondeterministic physics failures).
    static INIT: std::sync::Once = std::sync::Once::new();
    INIT.call_once(|| {
        rocketsim::init_from_default(true).unwrap();
    });
    let recording = Recording::from_bytes(name, recording_bytes).unwrap();
    test_recording(&recording);
}
include!(concat!(env!("OUT_DIR"), "/gen_comparison_tests.rs"));
