#![allow(unused)]

mod comparison_test;

use std::sync::OnceLock;

use rustc_hash::{FxBuildHasher, FxHashMap};
use test_case::test_matrix;

use crate::comparison_test::*;

const FAIL_ERROR_THRESH: f32 = 0.5;

fn init_for_test() -> &'static FxHashMap<String, TestCase> {
    static INIT: OnceLock<()> = OnceLock::new();
    static CASES: OnceLock<FxHashMap<String, TestCase>> = OnceLock::new();

    INIT.get_or_init(|| {
        rocketsim::init("../collision_meshes", false).unwrap();
        rocketsim_rs::init(Some("../collision_meshes"), false);
    });

    CASES.get_or_init(|| {
        all_test_cases::make_all_cases()
            .into_iter()
            .map(|case| (case.name.clone(), case))
            .collect()
    })
}

fn assert_test_result(test_case: &TestCase, test_result: &TestResult) {
    for (tick_idx, state) in test_result.ticks.iter().enumerate() {
        let comparison = &state.comparison;
        for (value_name, value_err) in &comparison.combine_all_err_sets() {
            if *value_err >= FAIL_ERROR_THRESH {
                let value_err_stat = test_result.val_err_stats.get(value_name).unwrap();
                let case_name = test_case.name.as_str();
                let tick_number = tick_idx + 1;
                let mean_err = value_err_stat.mean();

                let prev_state = &test_result.ticks[tick_idx.max(1) - 1];
                let cur_state = &test_result.ticks[tick_idx];

                panic!(
                    "Test comparison case \"{case_name}\" failed at tick #{tick_number} with value \"{value_name}\" (scaled error = {value_err})\
                    \n\tError mean for \"{value_name}\" during test case \"{case_name}\" = {mean_err}\
                    \nPrev state: {prev_state}\
                    \nResulting state: {cur_state}"
                );
            }
        }
    }
}

#[test_matrix([
    "ball_bounce_ground",
    "ball_bounce_ground_ang_vel",
    "ball_bounce_crazy",
    "ball_roll_up_slope",
    "ball_bounce_off_slope",
    "ball_into_goal",
    "ball_crossbar_down",
    "ball_post_out",
    "car_drive_forward",
    "car_drive_forward_boost",
    "car_drive_forward_turn",
    "car_drive_forward_turn_handbrake",
    "car_drive_up_slope",
    "car_jump_long_stationary",
    "car_jump_short_stationary",
    "car_land_ground_simple",
    "car_land_ground_complex",
    "car_land_ground_powerslide",
    "car_pogo",
    "car_double_jump",
    "car_flip_simple",
    "car_flip_complex",
    "car_auto_roll",
    "car_ball_basic_air_hit",
    "car_ball_complex_air_hit",
    "car_car_basic_bump",
    "car_car_basic_demo",
])]
fn run_case(case_name: &str) {
    let test_cases = init_for_test();

    let test_case = test_cases
        .get(case_name)
        .unwrap_or_else(|| panic!("Unknown test case name: {case_name}"));

    let test_result = test_case.run();
    assert_test_result(test_case, &test_result);
}
