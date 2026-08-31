pub const NUM_ITERATIONS: usize = 10;
pub const SOR: f32 = 1.0;
// Bullet's btContactSolverInfo uses m_erp = 0.2 for wheel raycast pushback.
pub const WHEEL_PUSHBACK_ERP: f32 = 0.2;
// RocketSim v2 overrides Bullet's contact ERP for legacy behavior.
pub const ERP_2: f32 = 0.8;
// RocketSim v2 also overrides the split-impulse threshold, causing all
// contacts to use the split-penetration path.
pub const SPLIT_IMPULSE_PENETRATION_THRESHOLD: f32 = 1e30;
pub const SPLIT_IMPULSE_TURN_ERP: f32 = 0.1;
pub const WARMSTARTING_FACTOR: f32 = 0.85;
pub const RESTITUTION_VELOCITY_THRESHOLD: f32 = 0.2;
