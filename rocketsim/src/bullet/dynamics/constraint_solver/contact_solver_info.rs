pub const NUM_ITERATIONS: usize = 10;
pub const SOR: f32 = 1.0;
/// Bullet default is 0.2, but the game's vehicle-wheel pushback resolver
/// was measured (offline fit vs RLPR recordings) to use ERP = 0.1.
/// See tests/rl_comparison_test/LANDING_NOTES.md.
pub const WHEEL_PUSHBACK_ERP: f32 = 0.1;
pub const ERP_2: f32 = 0.8;
pub const SPLIT_IMPULSE_PENETRATION_THRESHOLD: f32 = 1e30;
pub const SPLIT_IMPULSE_TURN_ERP: f32 = 0.1;
pub const WARMSTARTING_FACTOR: f32 = 0.85;
pub const RESTITUTION_VELOCITY_THRESHOLD: f32 = 0.2;
