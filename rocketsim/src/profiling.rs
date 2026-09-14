//! Optional subsystem timing for profiling builds.

#[cfg(feature = "profile")]
use std::sync::atomic::{AtomicU64, Ordering};

#[cfg(feature = "profile")]
const NAMES: [&str; 9] = ["gravity", "predict", "aabb", "broadphase", "narrowphase", "solver", "integrate", "activation", "vehicle_rays"];

#[cfg(feature = "profile")]
static TOTAL_NS: [AtomicU64; 9] = [
    AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0),
    AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0),
    AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0),
];
#[cfg(feature = "profile")]
static CALLS: AtomicU64 = AtomicU64::new(0);

#[cfg(feature = "profile")]
pub fn record(stage: usize, elapsed: std::time::Duration) {
    TOTAL_NS[stage].fetch_add(elapsed.as_nanos() as u64, Ordering::Relaxed);
}
#[cfg(feature = "profile")]
pub fn tick() { CALLS.fetch_add(1, Ordering::Relaxed); }
#[cfg(feature = "profile")]
pub fn report() -> String {
    let calls = CALLS.load(Ordering::Relaxed).max(1);
    let mut out = format!("profile ticks={calls}\n");
    for (i, name) in NAMES.iter().enumerate() {
        let ns = TOTAL_NS[i].load(Ordering::Relaxed);
        out.push_str(&format!("{name}: total_ms={:.3} avg_us={:.3}\n", ns as f64 / 1e6, ns as f64 / calls as f64 / 1e3));
    }
    out
}
#[cfg(not(feature = "profile"))]
pub fn report() -> String { String::new() }
