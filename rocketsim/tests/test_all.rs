mod rl_comparison_test;

#[test]
fn compare_to_rl() {
    rocketsim::init_from_default(true).unwrap();
    rl_comparison_test::run_all();
}