//! Wall-clock-disciplined tick pacer (real-time sim loops).
//!
//! [`TickPacer`] anchors tick deadlines to a timeline: early ticks sleep
//! the remainder, late ticks run capped catch-up steps, worse lateness
//! counts as dropped and re-syncs. Used by `examples/drive.rs`.

use std::time::{Duration, Instant};

/// Sim tick rate for real-time driving (matches `TICK_TIME`, 1/120s).
pub const TICK_HZ: f64 = 120.0;

/// Nominal tick interval (8_333_333ns ≈ 1/120s).
pub const TICK_INTERVAL: Duration = Duration::from_nanos(8_333_333);

/// Tick dt handed to the sim (`TICK_TIME` as `f32`).
pub const TICK_DT: f32 = 1.0 / 120.0;

/// Max extra ticks run when behind; further lateness is dropped
/// and the clock re-syncs. 5 keeps a ~40ms hitch recoverable.
pub const MAX_CATCH_UP: u32 = 5;

/// Cumulative pacer counters (overlay / log diagnostics).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PaceStats {
    /// Ticks actually executed (includes catch-up steps).
    pub ticks: u64,
    /// Ticks skipped after [`MAX_CATCH_UP`] was exhausted.
    pub dropped: u64,
    /// Largest single catch-up burst observed (0 = never late).
    pub max_backlog: u32,
}

/// Real-time tick pacer: [`TickPacer::wait`] blocks to the next deadline and
/// returns how many ticks to run now (`1` on time, more when catching up).
///
/// The next deadline is anchored to a timeline: an early tick sleeps the
/// remainder, a late tick runs catch-up steps (capped by `max_catch_up`);
/// further lateness counts as dropped and the clock re-syncs, so a stall
/// never spirals into a backlog.
///
/// ```rust
/// use std::time::Duration;
///
/// use rocketsim_vis::pace::TickPacer;
///
/// // Drive the sim at 120 Hz: wait, then step that many ticks.
/// let mut pacer = TickPacer::at_120hz();
/// let steps = pacer.wait();
/// assert!(steps >= 1);
/// assert!(pacer.stats().ticks >= 1);
/// ```
pub struct TickPacer {
    interval: Duration,
    max_catch_up: u32,
    next: Instant,
    stats: PaceStats,
}

impl TickPacer {
    /// Pacer for `interval` ticks starting now (first [`TickPacer::wait`]
    /// sleeps one full interval).
    pub fn new(interval: Duration, max_catch_up: u32) -> Self {
        Self {
            interval,
            max_catch_up,
            next: Instant::now() + interval,
            stats: PaceStats::default(),
        }
    }

    /// 120Hz pacer ([`TICK_INTERVAL`], [`MAX_CATCH_UP`]) for sim driving.
    pub fn at_120hz() -> Self {
        Self::new(TICK_INTERVAL, MAX_CATCH_UP)
    }

    /// Tick interval this pacer enforces.
    pub fn interval(&self) -> Duration {
        self.interval
    }

    /// Counters so far (copy; cheap).
    pub fn stats(&self) -> PaceStats {
        self.stats
    }

    /// Sleeps to the next deadline, then returns the ticks to run now.
    /// Never 0; capped at `1 + max_catch_up`, then the clock re-syncs.
    pub fn wait(&mut self) -> u32 {
        let now = Instant::now();
        if now < self.next {
            std::thread::sleep(self.next - now);
        }
        let (steps, dropped) =
            steps_for_lateness(Instant::now(), self.next, self.interval, self.max_catch_up);
        self.next += self.interval * steps;
        if Instant::now() >= self.next {
            // Multi-second stall: re-sync to now, count the rest as dropped.
            let missed = steps_for_lateness_missed(Instant::now(), self.next, self.interval);
            self.stats.dropped += missed as u64;
            self.next = Instant::now() + self.interval;
        }
        self.stats.ticks += u64::from(steps);
        self.stats.dropped += u64::from(dropped);
        let backlog = steps.saturating_sub(1);
        self.stats.max_backlog = self.stats.max_backlog.max(backlog);
        steps
    }
}

/// Pure catch-up math (core of [`TickPacer::wait`]): returns
/// `(steps, dropped)` with steps clamped to `1 + max_catch_up`.
pub fn steps_for_lateness(
    now: Instant,
    next: Instant,
    interval: Duration,
    max_catch_up: u32,
) -> (u32, u32) {
    if now < next {
        return (1, 0);
    }
    let missed = steps_for_lateness_missed(now, next, interval);
    let catch_up = missed.min(max_catch_up);
    (1 + catch_up, missed - catch_up)
}

/// Whole extra intervals elapsed between `next` and `now` (0 if early).
/// Saturates instead of overflowing on hour-long stalls.
fn steps_for_lateness_missed(now: Instant, next: Instant, interval: Duration) -> u32 {
    if now < next || interval.is_zero() {
        return 0;
    }
    let late_ns = (now - next).as_nanos();
    let step_ns = interval.as_nanos().max(1);
    u32::try_from(late_ns / step_ns).unwrap_or(u32::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t0() -> Instant {
        Instant::now()
    }

    #[test]
    fn catch_up_math() {
        let base = t0();
        // Early and exactly-on-deadline both yield one step, zero drops.
        assert_eq!(
            steps_for_lateness(base, base + Duration::from_millis(8), TICK_INTERVAL, 5),
            (1, 0)
        );
        assert_eq!(steps_for_lateness(base, base, TICK_INTERVAL, 5), (1, 0));
        // 2 intervals late -> this tick + 2 catch-up, nothing dropped.
        let now = base + 2 * TICK_INTERVAL + Duration::from_micros(100);
        assert_eq!(
            steps_for_lateness(now, base, TICK_INTERVAL, MAX_CATCH_UP),
            (3, 0)
        );
        // 10 intervals late with cap 5 -> 6 steps, 5 dropped.
        let now = base + 10 * TICK_INTERVAL + Duration::from_micros(100);
        assert_eq!(
            steps_for_lateness(now, base, TICK_INTERVAL, MAX_CATCH_UP),
            (1 + MAX_CATCH_UP, 10 - MAX_CATCH_UP)
        );
    }

    #[test]
    fn stats_accumulate_over_waits() {
        // Generous interval: under parallel `cargo test` load a 1ms sleep
        // can overshoot into catch-up, so assert accounting (not exact
        // steps) — determinism of the step math is covered above.
        let mut pacer = TickPacer::new(Duration::from_millis(20), 5);
        let mut total = 0u64;
        let mut max_bl = 0u32;
        for _ in 0..2 {
            let steps = pacer.wait();
            assert!(steps >= 1, "wait must always run at least one tick");
            total += u64::from(steps);
            max_bl = max_bl.max(steps - 1);
        }
        let stats = pacer.stats();
        assert_eq!(stats.ticks, total);
        assert_eq!(stats.max_backlog, max_bl);
    }

    #[test]
    fn stall_resyncs_without_spiral() {
        // 1ms interval, sleep 50ms: capped steps + drops, clock re-synced so
        // the NEXT wait is prompt (single tick, no inherited backlog).
        let mut pacer = TickPacer::new(Duration::from_millis(1), 2);
        std::thread::sleep(Duration::from_millis(50));
        let steps = pacer.wait();
        assert_eq!(steps, 3, "capped at 1 + max_catch_up");
        assert!(pacer.stats().dropped > 0, "excess must count as dropped");
        let t = Instant::now();
        assert_eq!(pacer.wait(), 1, "clock must re-sync after a stall");
        assert!(
            t.elapsed() < Duration::from_millis(50),
            "no spiral after resync"
        );
    }
}
