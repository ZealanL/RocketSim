pub const NUM_ITERATIONS: usize = 10;
pub const SOR: f32 = 1.0;
pub const WHEEL_PUSHBACK_ERP: f32 = 0.1;
pub const ERP_2: f32 = 0.8;
pub const SPLIT_IMPULSE_PENETRATION_THRESHOLD: f32 = 1e30;
pub const SPLIT_IMPULSE_TURN_ERP: f32 = 0.1;
pub const WARMSTARTING_FACTOR: f32 = 0.85;
/// Use a low threshold for shallow ball-world impacts.
/// Keep restitution zero for resting contacts.
pub const SPECIAL_RESTITUTION_VELOCITY_THRESHOLD: f32 = 0.05;
